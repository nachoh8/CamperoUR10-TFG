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

//#include <pcl_ros/point_cloud.h>
//#include <pcl/surface/concave_hull.h>
//#include <pcl_ros/surface/convex_hull.h> 
//#include <pcl/pcl_base.h>
//#include <pcl/point_types.h>
//#include <pcl/surface/concave_hull.h>

#include <campero_ur10_msgs/ArucoMarker.h>
#include <campero_ur10_msgs/ArucoMarkerArray.h>
#include <campero_ur10_msgs/ArucoMarkersImg.h>

#include <campero_ur10_msgs/ImgPoint.h>
#include <campero_ur10_msgs/ImgTrace.h>
#include <campero_ur10_msgs/ImageDraw.h>

#include "concaveman.h"

typedef double T;
typedef std::array<T, 2> point_type;

class ImageRes {
private:
    campero_ur10_msgs::ImageDraw img_res_msg;

    cv::Mat img_original, img_correct, img_original_show, img_debug_show;
    std::vector< std::vector<cv::Point> > contours;
    std::vector< std::vector<cv::Point2d> > contoursD;
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
    
    /// Internal fucntions

    cv::Point H_to_Orig(cv::Point pt) {
        cv::Point3d p_src(pt.x, pt.y, 1);
        cv::Point3d p_dst(cv::Mat(H_inv * cv::Mat(p_src)));
        p_dst /= p_dst.z; // normalize

        return cv::Point(p_dst.x, p_dst.y);
    }
    
    cv::Point2d H_to_Orig(cv::Point2d pt) {
        cv::Point3d p_src(pt.x, pt.y, 1);
        cv::Point3d p_dst(cv::Mat(H_inv * cv::Mat(p_src)));
        p_dst /= p_dst.z; // normalize

        return cv::Point2d(p_dst.x, p_dst.y);
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
	
	std::vector< std::vector<cv::Point3f> > img2CameraD() {
        cv::Mat leftSideMat  = rotationMatrix.inv() * img_params->cameraMatrix.inv() * z_markers_const;
        cv::Mat rightSideMat = rotationMatrix.inv() * tvec;
        std::cout << "img2CameraD\n";
        std::vector< std::vector<cv::Point3f> > res(contoursD.size());
        for (int i = 0; i < contoursD.size(); i++) {
            for (int j = 0; j < contoursD[i].size(); j++) {
                cv::Point2d pt = contoursD[i][j];
            
                cv::Mat uvPt = (cv::Mat_<double>(3,1) << pt.x, pt.y, 1.0);
                cv::Mat pt_m (cv::Size(3,1), CV_64F);
                pt_m = leftSideMat * uvPt - rightSideMat;
                cv::Point3f pt_w(pt_m.at<double>(0), pt_m.at<double>(1), pt_m.at<double>(2));
                /*if (j < 10) {
					std::cout << j << std::endl;
					std::cout << "pt: " << pt << std::endl;
					std::cout << "uvPt: " << uvPt << std::endl;
					double rX = leftSideMat.at<double>(0,0) * uvPt.at<double>(0) + leftSideMat.at<double>(0,1) * uvPt.at<double>(1) + leftSideMat.at<double>(0,2) * uvPt.at<double>(2);
					double rY = leftSideMat.at<double>(1,0) * uvPt.at<double>(0) + leftSideMat.at<double>(1,1) * uvPt.at<double>(1) + leftSideMat.at<double>(1,2) * uvPt.at<double>(2);
					double rZ = leftSideMat.at<double>(2,0) * uvPt.at<double>(0) + leftSideMat.at<double>(2,1) * uvPt.at<double>(1) + leftSideMat.at<double>(2,2) * uvPt.at<double>(2);
					
					rX = rX - rightSideMat.at<double>(0);
					rY = rY - rightSideMat.at<double>(1);
					rZ = rZ - rightSideMat.at<double>(2);
					
					std::cout << "Rx: " << rX << " Ry: " << rY << " Rz: " << rZ << std::endl;
					
					//std::cout << leftSideMat.at<double>(1,1) << "x" << uvPt.at<double>(1) << " = " << leftSideMat.at<double>(1,1) * uvPt.at<double>(1) <<std::endl;
					std::cout << "pt_m: " << pt_m << std::endl;
					std::cout << "pt_w: " << pt_w << std::endl;
				}*/
                res[i].push_back(pt_w);
            }
        }

        return res;
    }

public:
    ImageRes() {}
    
    ImageRes(cv::Mat& _img, std::vector<campero_ur10_msgs::ArucoMarker> _markers, std::shared_ptr<ImgProcessParams> _img_params) {
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
        return img_debug_show;
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
		
		std::vector< std::vector<cv::Point3f> > pts_cam;
		if (img_params->apply_smooth_path) {
			//std::vector<std::vector<cv::Point2d> > contoursD;
			for (int i = 0; i < contours.size(); i++) {
				std::vector<double> xx;
				std::vector<double> yy;
				for (auto &pt : contours[i]) {
					double x = double(pt.x);
					double y = double(pt.y);
					xx.push_back(x);
					yy.push_back(y);
				}
				std::vector<cv::Point2d> cD;
				smoothPath(xx, yy, cD);
				contoursD.push_back(cD);
			}
			
			pts_cam = img2CameraD();
			fill_images_showD();
			
		} else {
			pts_cam = img2Camera();
			fill_images_show();
		}
         
        
        for (int i = 0; i < pts_cam.size(); i++) {
			campero_ur10_msgs::ImgTrace trace;
			int j = 0;
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
				/*if (j < 10) {
					cv::Point2d pt = contoursD[i][j];
					std::cout << "Pt img: " << pt.x << " " << pt.y << std::endl;
					std::cout << "Ptcam: " << pt << std::endl;
					std::cout << "Ptw: " << pose.position << std::endl;
				}*/
				
				campero_ur10_msgs::ImgPoint pt_msg;
				pt_msg.x = pose.position.x;
				pt_msg.y = pose.position.y;
				pt_msg.z = pose.position.z;
				trace.points.push_back(pt_msg);
				j++;
			}

			img_res_msg.traces.push_back(trace);
		}
    }
    
    /// Common operations

    void delete_markers(cv::Mat& img) {
        // Delete Markers
        std::vector< std::vector<cv::Point> > pts(markers.size());
        for (int i = 0; i < markers.size(); i++) {
			std::vector<cv::Point2f> pts_f;
            for (int j = 0; j < markers[i].img_points.size(); j++) {
				cv::Point p = Orig_to_H(cv::Point(markers[i].img_points[j].x, markers[i].img_points[j].y));
                pts_f.push_back(cv::Point2f(p.x, p.y));
            }
            // agrandamos la marca
            std::vector<cv::Point2f> order_pts;
			filterRectPoints(pts_f, order_pts);
			cv::Point2f tl = order_pts[0];
			cv::Point2f tr = order_pts[1];
			cv::Point2f br = order_pts[2];
			cv::Point2f bl = order_pts[3];
			
			pts[i].push_back(cv::Point(tl.x - 5, tl.y - 5));
			pts[i].push_back(cv::Point(tr.x + 5, tr.y - 5));
			pts[i].push_back(cv::Point(br.x + 5, br.y + 5));
			pts[i].push_back(cv::Point(bl.x - 5, bl.y + 5));
        }
        
        cv::drawContours(img, pts, -1, cv::Scalar(0), -1);
    }

    void fill_mats(const int numImg, std::vector<cv::Vec3b>& colors, std::vector<cv::Mat>& imgs,
                    const cv::Size& size, const int imgType) {
        // Fill objects
        for(int i = 0; i < numImg; i++ )
        {
            int b = cv::theRNG().uniform(0, 255);
            int g = cv::theRNG().uniform(0, 255);
            int r = cv::theRNG().uniform(0, 255);
            colors.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));

            imgs.push_back(cv::Mat::zeros(size, imgType));
        }
    }
    
    void find_contours(const std::vector<cv::Mat> objetosImagen) {
        // Find contours on image objets
        for (int i = 0; i < objetosImagen.size(); i++) {
            std::vector< std::vector<cv::Point> > contours_local;
            cv::findContours( objetosImagen[i], contours_local, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );
            if (contours_local.size() == 0) {
                continue;
            }

            double maxLen = -1;
            int indiceMax = -1;
            for( size_t i = 0; i < contours_local.size(); i++ )
            {
                double len = cv::arcLength( contours_local[i], true );

                if (len > maxLen) {
                    maxLen = len;
                    indiceMax = i;
                }
            }

            if (indiceMax >= 0 && maxLen > img_params->min_contour_size) {
                if (img_params->apply_concaveman) {
                    std::vector<point_type> points;
                    std::vector<int> hull;
                    cv2concave(objetosImagen[i], points, hull);

                    auto concave = concaveman<T, 16>(points, hull, img_params->concaveman_alpha, 1);
                    
                    std::vector<cv::Point> shape;
                    cv::Mat aux = concave2cv(concave, img_correct.cols, img_correct.rows, shape);
                    
                    contours.push_back(shape);
                    total_pts += shape.size();
                } else {
                    contours.push_back(contours_local[indiceMax]);
                    total_pts += contours_local[indiceMax].size();
                }
            }
        }
    }

    void fill_images_show() {
        img_debug_show = img_correct.clone();
        img_original_show = img_original.clone();
        
        for (int i = 0; i < contours.size(); i++) {
            int b = cv::theRNG().uniform(0, 255);
            int g = cv::theRNG().uniform(0, 255);
            int rr = cv::theRNG().uniform(0, 255);
            cv::Vec3b c((uchar)b, (uchar)g, (uchar)rr);
            for(int j = 0; j < contours[i].size(); j++) {
                cv::circle(img_debug_show, contours[i][j], 0, c, -1);

                cv::Point p = H_to_Orig(contours[i][j]);
                cv::circle(img_original_show, p, 0, c, -1);
            }
        }
    }
    
    void fill_images_showD() {
        img_debug_show = img_correct.clone();
        img_original_show = img_original.clone();
        
        for (int i = 0; i < contoursD.size(); i++) {
            int b = cv::theRNG().uniform(0, 255);
            int g = cv::theRNG().uniform(0, 255);
            int rr = cv::theRNG().uniform(0, 255);
            cv::Vec3b c((uchar)b, (uchar)g, (uchar)rr);
            for(int j = 0; j < contoursD[i].size(); j++) {
                cv::circle(img_debug_show, contoursD[i][j], 0, c, -1);
				
                cv::Point2d p = H_to_Orig(contoursD[i][j]);
                if (j < 10) std::cout << "Pt img: " << p.x << " " << p.y << std::endl;

                cv::circle(img_original_show, p, 0, c, -1);
            }
        }
        
        /*cv::imshow("debug", img_debug_show);
		cv::imshow("original", img_original_show);
		cv::waitKey(0);*/
		cv::destroyAllWindows();
    }
    
    /// Img Processing Methods
    void find_pts() {
        contours.clear();
        contoursD.clear();
        total_pts = 0;
        
        switch(img_params->contour_method) {
            case 1:
                proc_watershed();
                break;
            default:
                proc_basic();
        }
    }

    void proc_basic() {
        // 1.To gray scale
        cv::Mat gray;
        if (img_params->apply_sharp) { // sharped image
            cv::Mat kernel = (cv::Mat_<float>(3,3) <<
                    1,  1, 1,
                    1, -8, 1,
                    1,  1, 1); // an approximation of second derivative, a quite strong kernel

            cv::Mat imgLaplacian;
            cv::filter2D(img_correct, imgLaplacian, CV_32F, kernel);
            
            cv::Mat sharp;
            img_correct.convertTo(sharp, CV_32F);
            cv::Mat imgResult = sharp - imgLaplacian;
            
            // convert back to 8bits gray scale
            imgResult.convertTo(imgResult, CV_8UC3);

            cv::cvtColor(imgResult, gray, cv::COLOR_BGR2GRAY);
        } else {
            cv::cvtColor(img_correct, gray, cv::COLOR_BGR2GRAY);
        }
        if (img_params->blur_ksize > 0) {
            cv::blur(gray, gray, cv::Size(img_params->blur_ksize, img_params->blur_ksize));
        }

        // 2.Binarizar
        cv::Mat img_otsu;
        cv::threshold(gray, img_otsu, img_params->binary_thresh, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        
        // 3.Delete Markers
        delete_markers(img_otsu);

        // 4.Calculo componentes conexas
        cv::Mat objetos;
        int nLabels = cv::connectedComponents(img_otsu, objetos, img_params->conectivity_way, CV_32S);

        // 5.Fill objects
        std::vector<cv::Vec3b> colorTab;
        std::vector<cv::Mat> objetosImagen;
        fill_mats(nLabels, colorTab, objetosImagen, img_correct.size(), CV_8UC1);
        
        // 6.Extraer los diferentes objetos y asignarlos a su imagen correspondiente
        for(int r = 0; r < objetos.rows; r++) {
            for(int c = 0; c < objetos.cols; c++) {
                int label = objetos.at<int>(r, c);
                if (label != 0) {
                    unsigned char& v = objetosImagen[label-1].at<unsigned char>(r, c);
                    v = 255;
                }
            }
        }
        
        // 7.Encontrar contornos
        find_contours(objetosImagen);

        // 8.Fill image ouput
        //fill_images_show();

    }

    void proc_watershed() {
        // 1.Binarizar
        cv::Mat gray;
        cv::cvtColor(img_correct, gray, cv::COLOR_BGR2GRAY);
        if (img_params->blur_ksize > 0) {
            cv::blur(gray, gray, cv::Size(img_params->blur_ksize, img_params->blur_ksize));
        }

        cv::Mat thresh;
        cv::threshold(gray, thresh, img_params->binary_thresh, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        
        // 2.Delete Markers
        delete_markers(thresh);

        // 3.Improve Contours
        cv::Mat kernel = (cv::Mat_<int>(3,3) <<
                    1,  1, 1,
                    1, 1, 1,
                    1,  1, 1); // an approximation of second derivative, a quite strong kernel

        cv::Mat opening;
        if (img_params->number_iterations > 0) {
            cv::morphologyEx(thresh, opening, cv::MORPH_OPEN, kernel, cv::Point(-1,-1), img_params->number_iterations);
        } else {
            opening = thresh.clone();
        }
		//show_img(opening, "opening");
        
        const int dilate_type = img_params->getDilateType();
        cv::Mat sure_bg;
        if (dilate_type != -1) {
            cv::Mat element = cv::getStructuringElement( dilate_type,
                       cv::Size( 2 * img_params->dilate_size + 1, 2 * img_params->dilate_size + 1 ),
                       cv::Point( img_params->dilate_size, img_params->dilate_size ) );
            cv::dilate(opening, sure_bg, element );
        } else {
            cv::dilate(opening, sure_bg, kernel, cv::Point(-1,-1), 3);
        }
        //show_img(sure_bg);
        const int erode_type = img_params->getErodeType();
        if (erode_type != -1) {
            cv::Mat element = cv::getStructuringElement( erode_type,
                       cv::Size( 2 * img_params->erode_type + 1, 2 * img_params->erode_type + 1 ),
                       cv::Point( img_params->erode_type, img_params->erode_type ) );
            cv::erode(sure_bg, sure_bg, element );
        }
        cv::cvtColor(sure_bg, img_debug_show, cv::COLOR_GRAY2RGB);
        //show_img(sure_bg);
        //show_img(sure_bg, "sure_bg_1");
        
        // Finding sure foreground area
        cv::Mat dist_transform;
        cv::distanceTransform(opening, dist_transform, cv::DIST_L2, 5);
        cv::Mat sure_fg;
        
        double min, max;
        cv::minMaxLoc(dist_transform, &min, &max);
        cv::threshold(dist_transform, sure_fg, 0.7*max, 255, 0);
        //show_img(sure_fg, "sure_fg_1");
        
        // Finding unknown region
        sure_fg.convertTo(sure_fg, CV_8UC3);
        cv::Mat unknown;
        cv::subtract(sure_bg, sure_fg, unknown);
		//show_img(sure_fg, "sure_fg_2");
        
        // 4.Marker labelling
        cv::Mat objects;
        int nLabels = cv::connectedComponents(sure_fg, objects, img_params->conectivity_way, CV_32S);

        // Add one to all labels so that sure background is not 0, but 1
        objects = objects+1;
        
        // 5.Now, mark the region of unknown with zero
        for(int r = 0; r < objects.rows; r++) {
            for(int c = 0; c < objects.cols; c++) {
                unsigned char v = unknown.at<unsigned char>(r, c);
                if (v == 255) {
                    int &pixel = objects.at<int>(r, c);
                    pixel = 0;
                }
            }
        }

        // 6.Watershed
        cv::watershed(img_correct, objects);
        
        // 7.Fill objects
        std::vector<cv::Vec3b> colorTab;
        std::vector<cv::Mat> objetosImagen;
        fill_mats(nLabels, colorTab, objetosImagen, img_correct.size(), CV_8UC1);
        //img_debug_show = cv::Mat::zeros(objects.size(), CV_8UC3); // img_correct.clone();

        // 8.Extraer los diferentes objetos y asignarlos a su imagen correspondiente
        for(int r = 0; r < objects.rows; r++) {
            for(int c = 0; c < objects.cols; c++) {
                int idx = objects.at<int>(r, c) - 1;
                if (idx > 0 && idx < nLabels) {
                    unsigned char& v = objetosImagen[idx - 1].at<unsigned char>(r, c);
                    v = 255;
                }
                /*cv::Vec3b &pixel = img_debug_show.at<cv::Vec3b>(r, c);
				if (idx == -1) {
                    pixel = cv::Vec3b(255, 255, 255);
                } else if (idx <= 0 || idx > nLabels) {
                    pixel = cv::Vec3b(0, 0, 0);
                } else {
					if (idx - 1 == 0) continue;
                    pixel = colorTab[idx - 1];
                    unsigned char& v = objetosImagen[idx - 1].at<unsigned char>(r, c);
                    v = 255;
                }*/
            }
        }
        // 9.Encontrar contornos
        find_contours(objetosImagen);

        // 10.Fill image ouput
        //fill_images_show();
    }

    inline cv::Mat concave2cv(std::vector<point_type>& points, const int w, const int h, std::vector<cv::Point>& shape) {
        cv::Mat res = cv::Mat::zeros(cv::Size(w, h), CV_8UC1);
        for (auto &pt : points) {
            cv::Point p(pt[0], pt[1]);
            shape.push_back(p);
            unsigned char& v = res.at<unsigned char>(p);
            v = 255;
        }

        return res;
    }

    inline void cv2concave(const cv::Mat& img, std::vector<point_type>& points, std::vector<int>& hull) {
        std::vector<cv::Point> pts_o;
        cv::findNonZero(img, pts_o);

        cv::convexHull( pts_o, hull);

        for (auto& pt: pts_o) {
            points.push_back({(double)pt.x, (double)pt.y});
        }

    }
	
	/// smoothPath
	void smoothPath(std::vector<double>& xx, std::vector<double>& yy, std::vector<cv::Point2d>& res) {
		const int n = xx.size();
		const int scale = img_params->smooth_path_kernel;
		//std::cout << "len x: " << n << " len y: " << yy.size() << std::endl;

		// 1.gaussian kernel
		cv::Mat gaussianKernel = cv::getGaussianKernel(scale, double(scale)/sqrt(2.0*M_PI), CV_64F);
		cv::mulTransposed(gaussianKernel,gaussianKernel,false);
		//std::cout << "gaussianKernel: " << gaussianKernel << std::endl;
		//std::cout << "gaussianKernel: " << gaussianKernel.size() << std::endl;

		int pos_middle = int((scale/2.0) + 0.5);
		//std::cout << "pos_middle: " << pos_middle << std::endl;
		cv::Mat H = gaussianKernel(cv::Range(pos_middle-1, pos_middle), cv::Range(0, gaussianKernel.cols));
		//std::cout << "Row " << pos_middle << ":" << H << std::endl;
		double weight = 0.0;
		for (int i = 0; i < H.cols; i++) {
			weight += H.at<double>(1, i);
		}
		//std::cout << "weight: " << weight << std::endl;

		// 2.convolution

		std::vector<double> _xExt(xx);
		_xExt.insert(_xExt.end(), xx.begin(), xx.end()); 
		_xExt.insert(_xExt.end(), xx.begin(), xx.end());
		//std::cout << "len xExt: " << _xExt.size() << std::endl;
		cv::Mat xExt(1, _xExt.size(), CV_64F, _xExt.data());
		//std::cout << "Mat_xExt: " << xExt.rows << " " << xExt.cols << " | type: " << xExt.type() << std::endl;

		cv::Mat xExtFilt;
		cv::filter2D(xExt, xExtFilt, CV_64F, H);
		xExtFilt = xExtFilt * weight;
		//std::cout << "Mat_xExtFilt: " << xExtFilt.rows << " " << xExtFilt.cols << " | type: " << xExtFilt.type() << std::endl;

		std::vector<double> _yExt(yy);
		_yExt.insert(_yExt.end(), yy.begin(), yy.end()); 
		_yExt.insert(_yExt.end(), yy.begin(), yy.end());
		//std::cout << "len yExt: " << _yExt.size() << std::endl;
		cv::Mat yExt(1, _yExt.size(), CV_64F, _yExt.data());;
		//std::cout << "Mat_yExt: " << yExt.rows << " " << yExt.cols << " | type: " << yExt.type() << std::endl;

		cv::Mat yExtFilt;
		cv::filter2D(yExt, yExtFilt, CV_64F, H);
		yExtFilt = yExtFilt * weight;
		//std::cout << "Mat_yExtFilt: " << yExtFilt.rows << " " << yExtFilt.cols << " | type: " << yExtFilt.type() << std::endl;

		//xExt.release();
		//yExt.release();

		// 3.obtein values in the middle

		cv::Mat xFilt = xExtFilt(cv::Range::all(), cv::Range(n, 2*n));
		cv::Mat yFilt = yExtFilt(cv::Range::all(), cv::Range(n, 2*n));

		//std::cout << "Mat_xFilt: " << xFilt.rows << " " << xFilt.cols << " | type: " << xFilt.type() << std::endl;
		//std::cout << "Mat_yFilt: " << yFilt.rows << " " << yFilt.cols << " | type: " << yFilt.type() << std::endl;

		// 4.rescale

		/*double min, maxY_Filt;
		cv::minMaxLoc(yFilt, &min, &maxY_Filt);
		double maxY = yy[0];
		for (int i = 1; i < n; i++) {
			if (yy[i] > maxY) {
				maxY = yy[i];
			}
		}
		double factor = std::abs(maxY) / std::abs(maxY_Filt) + 0.000001;
		std::cout << "maxY: " << maxY << " maxY_Filt: " << maxY_Filt << std::endl;*/
		double min, maxX_Filt;
		cv::minMaxLoc(xFilt, &min, &maxX_Filt);
		double maxX = xx[0];
		for (int i = 1; i < n; i++) {
			if (xx[i] > maxX) {
				maxX = xx[i];
			}
		}
		double factor = std::abs(maxX) / std::abs(maxX_Filt) + 0.000001;
		std::cout << "maxX: " << maxX << " maxY_Filt: " << maxX_Filt << std::endl;
		std::cout << "Factor: " << factor << std::endl;
		xFilt = xFilt * factor;
		yFilt = yFilt * factor;

		//std::cout << "Mat_xFilt: " << xFilt.rows << " " << xFilt.cols << " | type: " << xFilt.type() << std::endl;
		//std::cout << "Mat_yFilt: " << yFilt.rows << " " << yFilt.cols << " | type: " << yFilt.type() << std::endl;

		for (int i = 0; i < n; i++) {
			double x = xFilt.at<double>(i);
			double y = yFilt.at<double>(i);
			//std::cout << "x: " << x << " y: " << y << std::endl;
			res.push_back(cv::Point2d(x,y));
		}
	}
	
};
