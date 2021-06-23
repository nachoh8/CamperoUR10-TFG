#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <ros/ros.h>

#include <tf/tf.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <campero_ur10_msgs/ArucoMarker.h>
#include <campero_ur10_msgs/ArucoMarkerArray.h>
#include <campero_ur10_msgs/ArucoMarkersImg.h>

#include <campero_ur10_msgs/ImgPoint.h>
#include <campero_ur10_msgs/ImgTrace.h>
#include <campero_ur10_msgs/ImageDraw.h>

#define K_EXIT 27 // ESC -> exit board
#define K_SEND 115 // s -> send draw
#define K_CLEAR 99 // c -> clear draw

class ImageBoard {
private:
    campero_ur10_msgs::ImageDraw img_res_msg;

    cv::Mat img_original, img_correct, img_original_show, img_debug_show;
    std::vector< std::vector<cv::Point> > contours;
    int total_pts = 0;
    
    cv::Mat rotationMatrix, tvec;
    
    tf::Transform transform_base;
    
    double error;
    
    std::vector<campero_ur10_msgs::ArucoMarker> markers;

    /// Image Procesing
    std::shared_ptr<ImgProcessParams> img_params;

    int TOP_LEFT_IDX, TOP_RIGHT_IDX, BOTTOM_RIGHT_IDX, BOTTOM_LEFT_IDX; // indice de la marca top left, top right, bottom left, bottom right
    std::vector<cv::Point2f> marker_center_pts; // position on warped img
    std::vector<cv::Point3f> marker_pose_pts; // pose in camera coordinates

    cv::Mat H, H_inv; // homografy matrix (original image -> warped image)

    // plano formado por las marcas aruco
    double z_markers_const; // distancia de la camara al plano, la camara es perpendicular al plano
    cv::Point3f normal_plane, plane_o; // normal y punto origen del plano
    
    // board params
    cv::Point origin, origin_orig;
    cv::Mat img_board;

    /// Internal fucntions

    cv::Point H_to_Orig(cv::Point pt) {
        cv::Point3d p_src(pt.x, pt.y, 1);
        cv::Point3d p_dst(cv::Mat(H_inv * cv::Mat(p_src)));
        p_dst /= p_dst.z; // normalize

        return cv::Point(p_dst.x, p_dst.y);
    }

    cv::Point Orig_to_H(cv::Point pt) {
        cv::Point3d p_src(pt.x, pt.y, 1);
        cv::Point3d p_dst(cv::Mat(H * cv::Mat(p_src)));
        p_dst /= p_dst.z; // normalize

        return cv::Point(p_dst.x, p_dst.y);
    }

    cv::Point3f pose2Pt3f(const geometry_msgs::Pose& pose) {
        geometry_msgs::Point p = pose.position;

        return cv::Point3f(p.x, p.y, p.z);
    }

    cv::Point2f getMarkerCenter(const campero_ur10_msgs::ArucoMarker& marker) {
        cv::Point2f cent(0,0);
        for (int i = 0; i < marker.img_points.size(); i++) {
            cent.x += marker.img_points[i].x;
            cent.y += marker.img_points[i].y;
        }
        cent.x /= float(marker.img_points.size());
        cent.y /= float(marker.img_points.size());

        return cent;
    }
    
    void orderMarkers() {
        const int len = markers.size();

        std::vector< std::pair<double, int> > sum_v(len);
        std::vector< std::pair<double, int> > diff_v(len);
        for (int i = 0; i < len; i++) {
            sum_v[i] = std::make_pair(markers[i].pose.position.x + markers[i].pose.position.y, i);
            diff_v[i] = std::make_pair(markers[i].pose.position.y - markers[i].pose.position.x, i);
        }

        std::sort(sum_v.begin(), sum_v.end());
        std::sort(diff_v.begin(), diff_v.end());
        
        // top-left point min sum
        TOP_LEFT_IDX = sum_v[0].second;
        // top-right point min diff
        TOP_RIGHT_IDX = diff_v[0].second;
        // bottom-right point max sum
        BOTTOM_RIGHT_IDX = sum_v[sum_v.size()-1].second;
        // bottom-left max diff
        BOTTOM_LEFT_IDX = diff_v[diff_v.size()-1].second;
    }

    void getMarkersParameters() {
        
        orderMarkers();
        
        const int len = markers.size();

        for (int i = 0; i < len; i++) {
            cv::Point3f p = pose2Pt3f(markers[i].pose);
            // std::cout << i << " -> " << p << std::endl;
            marker_pose_pts.push_back(p);
        }
        
        // Plane parameters
        plane_o = marker_pose_pts[TOP_LEFT_IDX];
        cv::Point3f u = marker_pose_pts[TOP_RIGHT_IDX] - plane_o;
        cv::Point3f v = marker_pose_pts[BOTTOM_LEFT_IDX] - plane_o;
        
        normal_plane = u.cross(v);
        VectorEigen n;
        n << normal_plane.x, normal_plane.y, normal_plane.z;
        n.normalize();
        normal_plane = cv::Point3f(n(0), n(1), n(2));
        

        /*std::cout << "Calc Matrix" << std::endl;
        std::cout << u << "x" << v << " = " << normal_plane << std::endl;
        std::cout << plane_o << std::endl;*/
    }

    void filterRectPoints(std::vector<cv::Point2f>& pts, std::vector<cv::Point2f>& res) {
        const int len = pts.size();

        std::vector< std::pair<double, int> > sum_v(len);
        std::vector< std::pair<double, int> > diff_v(len);
        for (int i = 0; i < len; i++) {
            sum_v[i] = std::make_pair(pts[i].x + pts[i].y, i);
            diff_v[i] = std::make_pair(pts[i].y - pts[i].x, i);
        }

        std::sort(sum_v.begin(), sum_v.end());
        std::sort(diff_v.begin(), diff_v.end());

        // top-left point min sum -> idx 0
        // bottom-right point max sum -> idx 2
        // top-right point min diff -> idx 1
        // bottom-left max diff -> idx 3
        res.push_back(pts[sum_v[0].second]);
        res.push_back(pts[diff_v[0].second]);
        res.push_back(pts[sum_v[sum_v.size()-1].second]);
        res.push_back(pts[diff_v[diff_v.size()-1].second]);
    }

    void correctImage() {
        
        // Cast to cv::Point/2f type
        //std::vector< std::vector<cv::Point> > pts(markers.size());
        std::vector<cv::Point2f> pts_f;
        for (int i = 0; i < markers.size(); i++) {
            for (int j = 0; j < markers[i].img_points.size(); j++) {
                //pts[i].push_back(cv::Point(markers[i].img_points[j].x, markers[i].img_points[j].y));
                pts_f.push_back(cv::Point2f(markers[i].img_points[j].x, markers[i].img_points[j].y));
            }
        }
        
        // Delete Markers
        // cv::drawContours(image, pts, -1, cv::Scalar(255,255,255), -1);

        // Get Corner Source points
        std::vector<cv::Point2f> src_pts;
        filterRectPoints(pts_f, src_pts);

        cv::Point2f tl = src_pts[0];
        cv::Point2f tr = src_pts[1];
        cv::Point2f br = src_pts[2];
        cv::Point2f bl = src_pts[3];

        // Get Warped Destination Points
        double width_top = cv::norm(tr - tl);
        double width_bottom = cv::norm(br - bl);
        double max_W = width_top > width_bottom ? width_top : width_bottom;

        double height_left = cv::norm(bl - tl);
        double height_right = cv::norm(br - tr);
        double max_H = height_left > height_right ? height_left : height_right;

        std::vector<cv::Point2f> dst_pts;
        dst_pts.push_back(cv::Point2f(0, 0));
        dst_pts.push_back(cv::Point2f(max_W-1, 0));
        dst_pts.push_back(cv::Point2f(max_W-1, max_H-1));
        dst_pts.push_back(cv::Point2f(0, max_H-1));
        
        // Transform image
        H = cv::getPerspectiveTransform(src_pts, dst_pts);
        H_inv = H.inv();
        cv::warpPerspective(img_original, img_correct, H, cv::Size(max_W, max_H));
        
        // Get Marker Centers on warped img
        for (int i = 0; i < markers.size(); i++) {
            cv::Point center = Orig_to_H(getMarkerCenter(markers[i]));
            marker_center_pts.push_back(center);
            // cv::circle( image, center, 5, cv::Scalar(0,0,255), 3, cv::LINE_AA);
        }
    }
    
    void calculateTranformParam() {
                
        if (img_params->valid_matrix_pnp) { // si son validas tvec y rvec sirven como solución inicial
            cv::solvePnP(marker_pose_pts, marker_center_pts,
                        img_params->cameraMatrix, img_params->distortionEffect, img_params->rvec, img_params->tvec, true);
        } else {
            cv::solvePnP(marker_pose_pts, marker_center_pts,
                        img_params->cameraMatrix, img_params->distortionEffect, img_params->rvec, img_params->tvec);
        }
        cv::Rodrigues(img_params->rvec, rotationMatrix);
        
        const double max_dist_error = img_params->max_dist_error;
        error = 0.0;
        z_markers_const = 0.0;
        for (int i = 0; i < marker_pose_pts.size(); i++) {

            cv::Mat pt_c = (cv::Mat_<double>(3,1) << marker_pose_pts[i].x, marker_pose_pts[i].y, marker_pose_pts[i].z);

            cv::Mat1f uv_m = img_params->cameraMatrix * (rotationMatrix * pt_c + img_params->tvec);
            
            double d = uv_m(2);
            z_markers_const += d;
            cv::Point2f uv(uv_m(0) / d, uv_m(1) / d);

            double _error = cv::norm(uv - cv::Point2f(marker_center_pts[i]));
            if (_error > max_dist_error) {
                img_params->valid_matrix_pnp = false;
                return;
            }
            error += _error;
        }
        z_markers_const /= marker_pose_pts.size();
        error /= marker_pose_pts.size();

        tvec = img_params->tvec.clone();
        
        img_params->valid_matrix_pnp = true;
        return;
    }

    std::vector< std::vector<cv::Point3f> > img2Camera() {
        cv::Mat leftSideMat  = rotationMatrix.inv() * img_params->cameraMatrix.inv() * z_markers_const;
        cv::Mat rightSideMat = rotationMatrix.inv() * tvec;
        
        std::vector< std::vector<cv::Point3f> > res(contours.size());
        for (int i = 0; i < contours.size(); i++) {
            for (int j = 0; j < contours[i].size(); j++) {
                cv::Point pt = contours[i][j];
            
                cv::Mat uvPt = (cv::Mat_<double>(3,1) << pt.x, pt.y, 1);
                cv::Mat1f pt_m  = leftSideMat * uvPt - rightSideMat;
                cv::Point3f pt_w(pt_m(0), pt_m(1), pt_m(2));

                res[i].push_back(pt_w);
            }
        }

        return res;
    }

public:
    ImageBoard() {}
    
    ImageBoard(cv::Mat& _img, std::vector<campero_ur10_msgs::ArucoMarker> _markers, std::shared_ptr<ImgProcessParams> _img_params) {
        img_original = _img.clone();
        img_correct = _img.clone();
        markers = _markers;
        img_res_msg.W = -1;
        img_res_msg.H = -1;
        img_params = _img_params;
    }

    void process() {
        getMarkersParameters();
        correctImage();

        calculateTranformParam();
    }

    double getError() const {
        return error;
    }

    double getZ() const {
        return z_markers_const;
    }

    int numberCountours() const {
        return contours.size();
    }
    
    int numTotalPoints() const {
        /*int c = 0;
        for (int i = 0; i < contours.size(); i++) {
			c += contours[i].size();
		}
		*/
		return total_pts;
    }

    cv::Mat getImgOriginal() const {
        return img_original_show;
    }

    cv::Mat getImgDebug() const {
        return img_board;
    }

    campero_ur10_msgs::ImageDraw getImageDrawMsg()  const {
        return img_res_msg;
    }

    campero_ur10_msgs::ArucoMarker getArucoReference() const {
        return markers[TOP_LEFT_IDX];
    }

    void setTransform(tf::Transform _transform_base) {
       transform_base = _transform_base;
    }

    std::string getPrintInfo() {
        std::string msg = "Min error: " + std::to_string(error) + "\n";
        msg += "Num Contours: " + std::to_string(contours.size()) + "\n";
        msg += "Total points: " + std::to_string(total_pts) + "\n";
        msg += "Z const: " + std::to_string(z_markers_const) + "\n";

        return msg;
    }

    void img2World() {
        geometry_msgs::Pose p = markers[TOP_LEFT_IDX].pose;
        img_res_msg.traces.clear();

        std::vector< std::vector<cv::Point3f> > pts_cam = img2Camera();
        
        for (int i = 0; i < pts_cam.size(); i++) {
			campero_ur10_msgs::ImgTrace trace;
			for (auto &pt : pts_cam[i]) {
				p.position.x = pt.x;
				p.position.y = pt.y;
				p.position.z = pt.z;
				tf::Transform transform;
				
	 
				tf::poseMsgToTF(p, transform);
				transform = transform_base * transform;
				geometry_msgs::Pose pose;
				tf::poseTFToMsg(transform, pose);
				// poses_res.poses.push_back(pose);
				
				campero_ur10_msgs::ImgPoint pt_msg;
				pt_msg.x = pose.position.x;
				pt_msg.y = pose.position.y;
				pt_msg.z = pose.position.z;
				trace.points.push_back(pt_msg);
			}

			img_res_msg.traces.push_back(trace);
		}
    }

    /// Draw Board Methods
    void onMouse(int event, int x, int y)
    {
        switch (event)
        {
        case CV_EVENT_LBUTTONDOWN:
        {
            origin = cv::Point(x,y);
            std::vector<cv::Point> trace(1, origin);
            total_pts++;
            contours.push_back(trace);
            origin_orig = H_to_Orig(origin);
            break;
        }

        case CV_EVENT_MOUSEMOVE:
            if (origin.x != -1) {
                cv::Point dst(x,y);
                cv::Point dst_o = H_to_Orig(dst);
                cv::line(img_board, origin, dst, cv::Scalar(0,0,255), 1);
                cv::line(img_original_show, origin_orig, dst_o, cv::Scalar(0,0,255), 1);
                origin = dst;
                origin_orig = dst_o;
                contours[contours.size() - 1].push_back(origin);
                total_pts++;
            }
            break;

        case CV_EVENT_LBUTTONUP:
            cv::Point dst(x,y);
            cv::line(img_board, origin, dst, cv::Scalar(0,0,255), 1);
            cv::line(img_board, origin_orig, H_to_Orig(dst), cv::Scalar(0,0,255), 1);
            origin = cv::Point(-1,-1);
            break;
        }
    }

    static void onMouse(int event, int x, int y, int, void* userdata) {
        ImageBoard* res = reinterpret_cast<ImageBoard*>(userdata);
        if (res != nullptr) {
            res->onMouse(event, x, y);
        } else {
            std::cout << "null\n";
        }
    }

    void reset_draw() {
        img_board = img_correct.clone();
        img_original_show = img_original.clone();
        origin = cv::Point(-1,-1);
        origin_orig = cv::Point(-1,-1);
        contours.clear();
        total_pts = 0;
    }

    void board_main() {
        const std::string win_name = "board";
        cv::namedWindow(win_name);
        cv::setMouseCallback(win_name, onMouse, this);

        reset_draw();
        while (true) {
            cv::imshow(win_name, img_board);

            int key = cv::waitKey(1);
            if (key == K_EXIT) {
                std::cout << "EXIT\n";
                break;
            } else if (key == K_SEND) {
                std::cout << "SEND DRAW\n";
            } else if (key == K_CLEAR) {
                std::cout << "CLEAR DRAW\n";
                reset_draw();
            }
        }

        cv::destroyWindow(win_name);

        //img_original_show = img_original.clone();
        /*if (total_pts > 0) {
            std::vector< std::vector<cv::Point> > contours_orig;
        
            for(int i = 0; i < contours.size(); i++) {
                std::vector<cv::Point> pts;
                for(int j = 0; j < contours[i].size(); j++) {
                    std::cout << contours[i][j] << std::endl;
                    pts.push_back(H_to_Orig(contours[i][j]));
                }
                contours_orig.push_back(pts);
            }
            
            cv::drawContours( img_original_show, contours_orig, -1, cv::Scalar(0,0,255), 1 );
            
        }*/

    }
};
