#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <ros/ros.h> 
#include "std_msgs/String.h"

#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/PoseArray.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <image_transport/image_transport.h> // permite suscribirse/publicar en canales de imagen
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <geometry_msgs/Pose.h>

#include <cv_bridge/cv_bridge.h> // puente entre OpenCV y sensor_msgs/Image

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <campero_ur10_msgs/ArucoMarker.h>
#include <campero_ur10_msgs/ArucoMarkerArray.h>
#include <campero_ur10_msgs/ArucoMarkersImg.h>

#include <campero_ur10_msgs/ImgPoint.h>
#include <campero_ur10_msgs/ImgTrace.h>
#include <campero_ur10_msgs/ImageDraw.h>

#define DEAULT_MAX_DIST_ERROR 7

static const std::string NODE_NAME = "image_inpainting";
static const std::string TOPIC_NAME_IMG_DRAW = "image_points";

typedef Eigen::Matrix<float, 3, 1> VectorEigen;

void show_img(const cv::Mat& img, const std::string& title = "img") {
    cv::imshow(title, img);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

class ImgProcessParams {
    std::string getErodeDilateStrType(int type) {
        std::string str_type;
        switch(type) {
            case 1:
                str_type = "MORPH_RECT";
                break;
            case 2:
                str_type = "MORPH_CROSS";
                break;
            case 3:
                str_type = "MORPH_ELLIPSE";
                break;
            default:
                str_type = "NOT_APPLY";
                break;
        }

        return str_type;
    }

    int getErodeDilateType(int type) {
        switch(type) {
            case 1:
                return cv::MORPH_RECT;
            case 2:
                return cv::MORPH_CROSS;
            case 3:
                return cv::MORPH_ELLIPSE;
            default:
                return -1;
        }
    }

public:
    int max_dist_error;

    bool valid_matrix_pnp = false;
    cv::Mat cameraMatrix, // 3x3
            distortionEffect, // 4x1
            rvec, tvec; // 1x3
    cv::Size camSize;

    int canny_threshold1, canny_threshold2;
    int blur_ksize;
    int erode_type, erode_size, dilate_type, dilate_size;

    ImgProcessParams() {}

    ImgProcessParams(int _max_dist_error, int _canny_threshold1, int _canny_threshold2, int _blur_ksize,
                    int _erode_type, int _erode_size, int _dilate_type, int _dilate_size)
    {
        max_dist_error = _max_dist_error;
        canny_threshold1 = _canny_threshold1;
        canny_threshold2 = _canny_threshold2;
        blur_ksize = _blur_ksize;
        erode_type = _erode_type;
        erode_size = _erode_size;
        dilate_type = _dilate_type;
        dilate_size = _dilate_size;
    }

    void print_config() {
        ROS_INFO("--Image Processing Parameters--");
        ROS_INFO("Max Distance Error: %d px", max_dist_error);
        ROS_INFO("Canny Threshold 1: %d ", canny_threshold1);
        ROS_INFO("Canny Threshold 2: %d ", canny_threshold2);
        ROS_INFO("Blur Kernel Size: %dx%d ", blur_ksize, blur_ksize);
        ROS_INFO("Erode type: %s ", getErodeDilateStrType(erode_type).c_str());
        ROS_INFO("Erode Size: %d ", erode_size);
        ROS_INFO("Dilate type: %s ", getErodeDilateStrType(dilate_type).c_str());
        ROS_INFO("Dilate Size: %d ", dilate_size);
    }

    int getErodeType() {
        return getErodeDilateType(erode_type);
    }

    int getDilateType() {
        return getErodeDilateType(dilate_type);
    }
};

class ImageRes {
private:
    campero_ur10_msgs::ImageDraw img_res_msg;

    cv::Mat img_original, img_correct, img_original_show, img_debug_show;
    std::vector< std::vector<cv::Point> > contours;
    
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
        int c = 0;
        for (int i = 0; i < contours.size(); i++) {
			c += contours[i].size();
		}
		
		return c;
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

    void find_pts() {
        proc_img_v1();
    }

    void findContours() {
        cv::Mat src = img_correct.clone();
        cv::Mat mask;
        cv::inRange(src, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), mask);
        src.setTo(cv::Scalar(0, 0, 0), mask);

        cv::Mat kernel = (cv::Mat_<float>(3,3) <<
                1,  1, 1,
                1, -8, 1,
                1,  1, 1);
        
        cv::Mat imgLaplacian;
        cv::filter2D(src, imgLaplacian, CV_32F, kernel);
        cv::Mat sharp;
        src.convertTo(sharp, CV_32F);
        cv::Mat imgResult = sharp - imgLaplacian;

        // convert back to 8bits gray scale
        imgResult.convertTo(imgResult, CV_8UC3);
        imgLaplacian.convertTo(imgLaplacian, CV_8UC3);


        // Create binary image from source image
        cv::Mat bw;
        cv::cvtColor(imgResult, bw, cv::COLOR_BGR2GRAY);
        cv::threshold(bw, bw, 40, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        
        // Perform the distance transform algorithm
        cv::Mat dist;
        cv::distanceTransform(bw, dist, cv::DIST_L2, 3);

        // Normalize the distance image for range = {0.0, 1.0}
        // so we can visualize and threshold it
        cv::normalize(dist, dist, 0, 1.0, cv::NORM_MINMAX);

        // Threshold to obtain the peaks
        // This will be the markers for the foreground objects
        cv::threshold(dist, dist, 0.4, 1.0, cv::THRESH_BINARY);
        // Dilate a bit the dist image
        cv::Mat kernel1 = cv::Mat::ones(3, 3, CV_8U);
        cv::dilate(dist, dist, kernel1);

        // Create the CV_8U version of the distance image
        // It is needed for findContours()
        cv::Mat dist_8u;
        dist.convertTo(dist_8u, CV_8U);
        // Find total markers
        //std::vector< std::vector<cv::Point> > contours;
        cv::findContours(dist_8u, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // Create the marker image for the watershed algorithm
        cv::Mat markers = cv::Mat::zeros(dist.size(), CV_32S);
        // Draw the foreground markers
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::drawContours(markers, contours, static_cast<int>(i), cv::Scalar(static_cast<int>(i)+1), -1);
        }
        // Draw the background marker
        cv::circle(markers, cv::Point(5,5), 3, cv::Scalar(255), -1);
        cv::Mat markers8u;
        markers.convertTo(markers8u, CV_8U, 10);

        // Perform the watershed algorithm
        cv::watershed(imgResult, markers);
        cv::Mat mark;
        markers.convertTo(mark, CV_8U);
        cv::bitwise_not(mark, mark);
        //    imshow("Markers_v2", mark); // uncomment this if you want to see how the mark
        // image looks like at that point
        // Generate random colors
        std::vector<cv::Vec3b> colors;
        for (size_t i = 0; i < contours.size(); i++)
        {
            int b = cv::theRNG().uniform(0, 256);
            int g = cv::theRNG().uniform(0, 256);
            int r = cv::theRNG().uniform(0, 256);
            colors.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
        }
        // Create the result image
        cv::Mat dst = cv::Mat::zeros(markers.size(), CV_8UC3);
        // Fill labeled objects with random colors
        for (int i = 0; i < markers.rows; i++)
        {
            for (int j = 0; j < markers.cols; j++)
            {
                int index = markers.at<int>(i,j);
                if (index > 0 && index <= static_cast<int>(contours.size()))
                {
                    dst.at<cv::Vec3b>(i,j) = colors[index-1];
                }
            }
        }

        img_original_show = img_correct.clone();
        img_debug_show = dst.clone();
    }
    
    void proc_img_v1() {
        cv::Mat gray;
        cv::cvtColor(img_correct, gray, cv::COLOR_BGR2GRAY);

        cv::Mat thresh;
        cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        
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
        
        cv::drawContours(thresh, pts, -1, cv::Scalar(0), -1);

        // 
        cv::Mat kernel = (cv::Mat_<int>(3,3) <<
                    1,  1, 1,
                    1, 1, 1,
                    1,  1, 1); // an approximation of second derivative, a quite strong kernel

        cv::Mat opening;
        cv::morphologyEx(thresh, opening, cv::MORPH_OPEN, kernel, cv::Point(-1,-1), 2);
		show_img(opening, "opening");
        cv::Mat sure_bg;
        cv::dilate(opening, sure_bg, kernel, cv::Point(-1,-1), 3);
        show_img(sure_bg, "sure_bg_1");
        // Finding sure foreground area
        cv::Mat dist_transform;
        cv::distanceTransform(opening, dist_transform, cv::DIST_L2, 5);
        cv::Mat sure_fg;
        
        double min, max;
        cv::minMaxLoc(dist_transform, &min, &max);
        cv::threshold(dist_transform, sure_fg, 0.7*max, 255, 0);
        show_img(sure_fg, "sure_fg_1");
        // Finding unknown region
        sure_fg.convertTo(sure_fg, CV_8UC3);
        cv::Mat unknown;
        cv::subtract(sure_bg, sure_fg, unknown);
		show_img(sure_fg, "sure_fg_2");
        // Marker labelling
        cv::Mat objects;
        int nLabels = cv::connectedComponents(sure_fg, objects);
        std::cout << "Num Componentes Conexas: " << nLabels << std::endl;

        // Add one to all labels so that sure background is not 0, but 1
        objects = objects+1;
        
        // Now, mark the region of unknown with zero
        for(int r = 0; r < objects.rows; r++) {
            for(int c = 0; c < objects.cols; c++) {
                unsigned char v = unknown.at<unsigned char>(r, c);
                if (v == 255) {
                    int &pixel = objects.at<int>(r, c);
                    pixel = 0;
                }
            }
        }

        cv::watershed(img_correct, objects);
        img_debug_show = cv::Mat::zeros(objects.size(), CV_8UC3); // img_correct.clone();

        std::vector<cv::Vec3b> colorTab;
        std::vector<cv::Mat> objetosImagen;
        for(int i = 0; i < nLabels; i++ )
        {
            int b = cv::theRNG().uniform(0, 255);
            int g = cv::theRNG().uniform(0, 255);
            int r = cv::theRNG().uniform(0, 255);
            colorTab.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));

            objetosImagen.push_back(cv::Mat::zeros(img_correct.size(), CV_8UC1));
        }

        for(int r = 0; r < objects.rows; r++) {
            for(int c = 0; c < objects.cols; c++) {
                int idx = objects.at<int>(r, c);
                cv::Vec3b &pixel = img_debug_show.at<cv::Vec3b>(r, c);
				if (idx == -1) {
                    pixel = cv::Vec3b(255, 255, 255);
                } else if (idx <= 0 || idx > nLabels) {
                    pixel = cv::Vec3b(0, 0, 0);
                } else {
					if (idx -1 == 0) continue;
                    pixel = colorTab[idx - 1];
                    unsigned char& v = objetosImagen[idx - 1].at<unsigned char>(r, c);
                    v = 255;
                }
            }
        }
		
		std::vector< std::vector<cv::Point> > contours_orig;
        img_debug_show = img_correct.clone();
        img_original_show = img_original.clone();
        for (int i = 0; i < objetosImagen.size(); i++) {
            std::vector< std::vector<cv::Point> > contours_local;
            cv::findContours( objetosImagen[i], contours_local, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

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

            if (indiceMax >= 0) {
				std::vector<cv::Point> pts;
				for(int r = 0; r < contours_local[indiceMax].size(); r++) {
					pts.push_back(H_to_Orig(contours_local[indiceMax][r]));
				}
				contours_orig.push_back(pts);
                cv::drawContours( img_debug_show, contours_local, indiceMax, cv::Scalar(0,0,255), 1 );

                contours.push_back(contours_local[indiceMax]);
            }
        }
        cv::drawContours( img_original_show, contours_orig, -1, cv::Scalar(0,0,255), 1 );
        cv::cvtColor(thresh, img_debug_show, cv::COLOR_GRAY2RGB);
    }

    void descriptores(cv::Mat img, cv::Mat& img_show, int i){
        const int thresh = 100;
        //cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        
        // 1.Calculo de los contornos
        std::vector< std::vector<cv::Point> > contours_local;
        cv::findContours( img, contours_local, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
        
        //std::cout << "-------------------------------------------" << std::endl;
        //std::cout << "Descriptores para el objeto: " << i << std::endl;

        // 2.Escogemos el contorno con el perimetro más grande
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

        //std::cout << " * Perimetro: " << maxLen << std::endl;

        // 6.Imagen resultante con el contorno
        if (indiceMax >= 0) {
            std::cout << maxLen << std::endl; 
            const cv::Scalar color = cv::Scalar((rand()&255), (rand()&255), (rand()&255));
            cv::Mat drawing = cv::Mat::zeros( img.size(), CV_8UC3 ); 
            cv::drawContours( drawing, contours_local, (int)indiceMax, color, 3 );
            show_img(drawing);
            //cv::drawContours( img_show, contours_local, (int)indiceMax, color, 1 );
        }
    }

    void proc_img_v2() {
        img_original_show = img_correct.clone();
        img_debug_show = img_correct.clone();

        cv::Mat kernel = (cv::Mat_<float>(3,3) <<
                    1,  1, 1,
                    1, -8, 1,
                    1,  1, 1); // an approximation of second derivative, a quite strong kernel

        cv::Mat imgLaplacian;
        cv::filter2D(img_debug_show, imgLaplacian, CV_32F, kernel);
        
        cv::Mat sharp;
        img_debug_show.convertTo(sharp, CV_32F);
        cv::Mat imgResult = sharp - imgLaplacian;
        
        // convert back to 8bits gray scale
        imgResult.convertTo(imgResult, CV_8UC3);
        imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
        // imshow( "Laplace Filtered Image", imgLaplacian );
        //cv::imshow( "New Sharped Image", imgResult );

        cv::Mat gray;
        cv::cvtColor(imgResult, gray, cv::COLOR_BGR2GRAY);

        cv::Mat img_otsu;
        cv::threshold(gray, img_otsu, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        
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
        
        cv::drawContours(img_otsu, pts, -1, cv::Scalar(0), -1);

        // 2.Calculo componentes conexas
        cv::Mat objetos;
        int nLabels = cv::connectedComponents(img_otsu, objetos, 8, CV_32S);
        //std::cout << "Num Componentes Conexas: " << nLabels << std::endl;

        // especifica un color para cada etiqueta
        std::vector<cv::Vec3b> colors(nLabels);
        colors[0] = cv::Vec3b(0, 0, 0); // background -> black
        for (int label = 1; label < nLabels; ++label) {
            colors[label] = cv::Vec3b( (rand()&255), (rand()&255), (rand()&255) );
        }
        
        std::vector<cv::Mat> objetosImagen;
        for (int i = 0; i < nLabels; i++) {
            cv::Mat im(img_otsu.size(), img_otsu.type());
            objetosImagen.push_back(im);
        }

        cv::Mat img_res(img_otsu.size(), img_debug_show.type());
        // extraer los diferentes objetos y asignarlos a su imagen correspondiente
        for(int r = 0; r < objetos.rows; r++) {
            for(int c = 0; c < objetos.cols; c++) {
                int label = objetos.at<int>(r, c);
                if (label != 0) {
                    cv::Vec3b &pixel = img_debug_show.at<cv::Vec3b>(r, c);
                    pixel = colors[label];
                    unsigned char& v = objetosImagen[label-1].at<unsigned char>(r, c);
                    v = 255;
                }
            }
        }
        
        for(int i = 0; i < nLabels - 1; i++){
            descriptores(objetosImagen[i], img_original_show, i);
        }
        std::vector<cv::Point> tt;
        tt.push_back(cv::Point(0,0));
        contours.push_back(tt);
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
};

class ImageInpainting
{
private:
    int num_errors = 0, num_it = 0;

    ros::Subscriber process_cmd_sub;
    bool update_image = true;
    
    /// Ros
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    tf::TransformListener _tfListener;

    std::string camera_frame , cam_ref_frame, robot_frame;
	
    
    /// aruco markers and image
    ros::Subscriber markers_img_sub;

    /// Camera Info
    ros::Subscriber cam_info_sub;

    bool cam_info_received = false;

    tf::StampedTransform rightToLeft;

    /// Image Procesing
    image_transport::Subscriber image_sub;


    std::shared_ptr<ImgProcessParams> img_params = std::make_shared<ImgProcessParams>();

    /// Result
    ros::Publisher img_pts_pub;
    image_transport::Publisher image_res_pub, image_debug_pub;

    std::unique_ptr<ImageRes> best_img;

public:
    ImageInpainting() : it(nh) {
        markers_img_sub = nh.subscribe("/aruco_detector/markers_img", 1, &ImageInpainting::markers_img_callback, this);
        cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, &ImageInpainting::cam_info_callback, this);
        process_cmd_sub = nh.subscribe("/" + NODE_NAME + "/cmd", 1, &ImageInpainting::process_cmd_callback, this);
        //image_sub = it.subscribe("/camera/color/image_rect_color", 1, &ImageInpainting::image_callback, this);

        image_debug_pub = it.advertise("/" + NODE_NAME + "/image_debug", 1);
        image_res_pub = it.advertise("/" + NODE_NAME + "/image_res", 1);
        img_pts_pub = nh.advertise<campero_ur10_msgs::ImageDraw>(TOPIC_NAME_IMG_DRAW, 1);

        nh.param<std::string>("/" + NODE_NAME + "/cam_ref_frame", cam_ref_frame, "");
        nh.param<std::string>("/" + NODE_NAME + "/camera_frame", camera_frame, "");
        nh.param<std::string>("/" + NODE_NAME + "/robot_frame", robot_frame, "");

        nh.param<int>("/" + NODE_NAME + "/max_dist_error", img_params->max_dist_error, DEAULT_MAX_DIST_ERROR);
        nh.param<int>("/" + NODE_NAME + "/canny_th1", img_params->canny_threshold1, 100);
        nh.param<int>("/" + NODE_NAME + "/canny_th2", img_params->canny_threshold2, 300);
        nh.param<int>("/" + NODE_NAME + "/blur_ksize", img_params->blur_ksize, 3);
        nh.param<int>("/" + NODE_NAME + "/erode_type", img_params->erode_type, 0);
        nh.param<int>("/" + NODE_NAME + "/erode_size", img_params->erode_size, 3);
        nh.param<int>("/" + NODE_NAME + "/dilate_type", img_params->dilate_type, 0);
        nh.param<int>("/" + NODE_NAME + "/dilate_size", img_params->dilate_size, 3);
        
        ROS_INFO("Robot frame: %s | Camera Reference frame: %s | Camera frame: %s ", robot_frame.c_str(), cam_ref_frame.c_str(), camera_frame.c_str());
        img_params->print_config();
    }
    
    bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform) {
        std::string errMsg;

        if ( !_tfListener.waitForTransform(refFrame,
                                        childFrame,
                                        ros::Time(0),
                                        ros::Duration(0.5),
                                        ros::Duration(0.01),
                                        &errMsg)
            )
        {
            ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
            return false;
        }
        else
        {
            try
            {
                _tfListener.lookupTransform( refFrame, childFrame,
                                            ros::Time(0),                  //get latest available
                                            transform);
            }
            catch ( const tf::TransformException& e)
            {
                ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
                return false;
            }
        }

        return true;
    }

    void publishBestImg() {
        if (best_img == nullptr) {
            ROS_INFO("No hay imagen para mostrar");
            return;
        }

        // Publish Result Image
        if (image_res_pub.getNumSubscribers() > 0) {
            cv_bridge::CvImage out_msg;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image = best_img->getImgOriginal();
            image_res_pub.publish(out_msg.toImageMsg());
        }

        // Publish Debug Image
        if (image_debug_pub.getNumSubscribers() > 0) {
            cv_bridge::CvImage out_msg;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image = best_img->getImgDebug();
            image_debug_pub.publish(out_msg.toImageMsg());
        }
    }

    void publish_markers_img() {
        static tf::TransformBroadcaster br;

        if (best_img == nullptr) {
            ROS_INFO("No hay imagen para mostrar");
            return;
        }

        campero_ur10_msgs::ImageDraw im = best_img->getImageDrawMsg();
        geometry_msgs::Pose pose_ref = best_img->getArucoReference().pose;

        ros::Time t_current = ros::Time::now();
        int i = -1;
        for (auto &trace : im.traces) {
            for (auto &pt : trace.points) {
				i++;
				if (i%6 != 0) continue;
					
                pose_ref.position.x = pt.x;
				pose_ref.position.y = pt.y;
				pose_ref.position.z = pt.z;
                tf::Transform transform;
                tf::poseMsgToTF(pose_ref, transform);
                
                tf::StampedTransform stampedTransform(transform, t_current,
                                                robot_frame, "pt_board_" + std::to_string(i));
                br.sendTransform(stampedTransform);
            }
        }

    }
    
    void markers_img_callback(const campero_ur10_msgs::ArucoMarkersImg& msg) {
        if (!update_image) return;

        if (!cam_info_received) {
            ROS_WARN("Camera info not received");
            return;
        }
        
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg.img, sensor_msgs::image_encodings::BGR8);
            
            ImageRes* img_p = new ImageRes(cv_ptr->image, msg.markers.markers, img_params);
            
            //cv::Mat _rvec, _tvec;
            img_p->process();

            num_it++;
            if (!img_params->valid_matrix_pnp) {
                ROS_WARN("Error(%d/%d) calculando parametros -> error supera el limite max", (++num_errors), num_it);
                delete img_p;
                return;
            }
            //rvec = _rvec;
            //tvec = _tvec;

            if (best_img != nullptr && img_p->getError() > best_img->getError()) return;

            // Optical cam to cam ref
            tf::StampedTransform Cam_optToCam_ref;
            Cam_optToCam_ref.setIdentity();
            if ( cam_ref_frame != camera_frame ) {
                if (!getTransform(cam_ref_frame, camera_frame, Cam_optToCam_ref)) {
                    return;
                }
            }

            // ref cam to robot ref
            tf::StampedTransform Cam_refToRobot_ref;
            Cam_refToRobot_ref.setIdentity();
            if ( cam_ref_frame != robot_frame ) {
                if (!getTransform(cam_ref_frame, robot_frame, Cam_refToRobot_ref)) {
                    return;
                }
            }

            tf::Transform transform_base = static_cast<tf::Transform>(Cam_optToCam_ref)
                                      * static_cast<tf::Transform>(Cam_refToRobot_ref) 
                                      * static_cast<tf::Transform>(rightToLeft);
            
            img_p->setTransform(transform_base);
            img_p->find_pts();

            if (img_p->numberCountours() == 0) {
                delete img_p;
                return;
            }

            best_img.reset(img_p);
            
            best_img->img2World();

            ROS_INFO("--New best image--");
			ROS_INFO("Min error: %f ", best_img->getError());
			ROS_INFO("Number of contours to send: %d ", best_img->numberCountours());
			ROS_INFO("Number of total points to send: %d ", best_img->numTotalPoints());
			ROS_INFO("Z const: %f ", best_img->getZ());

            publishBestImg();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void cam_info_callback(const sensor_msgs::CameraInfo &cam_info) {
        ROS_INFO("Camera info received");

        img_params->cameraMatrix = cv::Mat(3, 3, CV_64FC1);
        img_params->distortionEffect = cv::Mat(4, 1, CV_64FC1);
        img_params->camSize = cv::Size(cam_info.height, cam_info.width);

        img_params->cameraMatrix.setTo(0);
        img_params->cameraMatrix.at<double>(0,0) = cam_info.P[0];   img_params->cameraMatrix.at<double>(0,1) = cam_info.P[1];   img_params->cameraMatrix.at<double>(0,2) = cam_info.P[2];
        img_params->cameraMatrix.at<double>(1,0) = cam_info.P[4];   img_params->cameraMatrix.at<double>(1,1) = cam_info.P[5];   img_params->cameraMatrix.at<double>(1,2) = cam_info.P[6];
        img_params->cameraMatrix.at<double>(2,0) = cam_info.P[8];   img_params->cameraMatrix.at<double>(2,1) = cam_info.P[9];   img_params->cameraMatrix.at<double>(2,2) = cam_info.P[10];

        for(int i=0; i<4; ++i)
            img_params->distortionEffect.at<double>(i, 0) = 0;

        rightToLeft.setIdentity();
        rightToLeft.setOrigin(
            tf::Vector3(
                -cam_info.P[3]/cam_info.P[0],
                -cam_info.P[7]/cam_info.P[5],
                0.0));

        cam_info_received = true;
        cam_info_sub.shutdown();
    }

    void process_cmd_callback(const std_msgs::String::ConstPtr& msg) {
        
        std::string cmd = msg->data;

        if (cmd == "reset") {
            ROS_WARN("--Command: Image Reset--");
            best_img.reset(nullptr);
        } else if (cmd == "send") {
            ROS_INFO("--Command: Send Image Points--");
            if (best_img != nullptr) {
                publish_markers_img();
                img_pts_pub.publish(best_img->getImageDrawMsg());
            } else {
                ROS_WARN("No hay ninguna imagen para enviar");
            }
        } else if (cmd == "update_on"){
            ROS_WARN("--Command: Update Image Enable--");
            update_image = true;
        } else if (cmd == "update_off"){
            ROS_WARN("--Command: Update Image Disable--");
            update_image = false;
        } else if (cmd == "update_image"){
            ROS_INFO("--Command: Update Image--");
            nh.param<int>("/" + NODE_NAME + "/max_dist_error", img_params->max_dist_error, DEAULT_MAX_DIST_ERROR);
            nh.param<int>("/" + NODE_NAME + "/canny_th1", img_params->canny_threshold1, 100);
            nh.param<int>("/" + NODE_NAME + "/canny_th2", img_params->canny_threshold2, 300);
            nh.param<int>("/" + NODE_NAME + "/blur_ksize", img_params->blur_ksize, 3);
            nh.param<int>("/" + NODE_NAME + "/erode_type", img_params->erode_type, 0);
            nh.param<int>("/" + NODE_NAME + "/erode_size", img_params->erode_size, 3);
            nh.param<int>("/" + NODE_NAME + "/dilate_type", img_params->dilate_type, 0);
            nh.param<int>("/" + NODE_NAME + "/dilate_size", img_params->dilate_size, 3);
            
            img_params->print_config();

            
            best_img->find_pts();
            
            best_img->img2World();

            ROS_INFO("--New image--");
			ROS_INFO("Min error: %f ", best_img->getError());
			ROS_INFO("Number of contours to send: %d ", best_img->numberCountours());
			ROS_INFO("Z const: %f ", best_img->getZ());

            publishBestImg();
        } else {
            ROS_WARN("--Command: not known--");
        }
        
        
    }

    void image_callback(const sensor_msgs::ImageConstPtr& msg) {
        if (!update_image) return;
        if (!cam_info_received) {
            ROS_WARN("Camera info not received");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            cv::Mat src = cv_ptr->image;
            proc_img_1(src);

            // Publish Result Image
            if (image_res_pub.getNumSubscribers() > 0) {
                cv_bridge::CvImage out_msg;
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = src;
                image_res_pub.publish(out_msg.toImageMsg());
            }
        } catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void image_test_1() {
        cv::Mat src = cv::imread("/home/nacho8/ROS_workspaces/campero_ur10_ws/test/cartas-de-poker.jpg");

        /*cv::Mat mask;
        cv::inRange(src, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), mask);
        src.setTo(cv::Scalar(0, 0, 0), mask);
        // Show output image
        cv::imshow("Black Background Image", src);*/

        cv::Mat kernel = (cv::Mat_<float>(3,3) <<
                        1,  1, 1,
                        1, -8, 1,
                        1,  1, 1); // an approximation of second derivative, a quite strong kernel
        
        // do the laplacian filtering as it is
        // well, we need to convert everything in something more deeper then CV_8U
        // because the kernel has some negative values,
        // and we can expect in general to have a Laplacian image with negative values
        // BUT a 8bits unsigned int (the one we are working with) can contain values from 0 to 255
        // so the possible negative number will be truncated
        cv::Mat imgLaplacian;
        cv::filter2D(src, imgLaplacian, CV_32F, kernel);
        
        cv::Mat sharp;
        src.convertTo(sharp, CV_32F);
        cv::Mat imgResult = sharp - imgLaplacian;
        
        // convert back to 8bits gray scale
        imgResult.convertTo(imgResult, CV_8UC3);
        imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
        // imshow( "Laplace Filtered Image", imgLaplacian );
        cv::imshow( "New Sharped Image", imgResult );
        
        cv::Mat gray;
        cv::cvtColor(imgResult, gray, cv::COLOR_BGR2GRAY);

        cv::Mat bw;
        cv::threshold(gray, bw, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

        cv::imshow("Binary Image", bw);

        

        // Perform the distance transform algorithm
        cv::Mat dist;
        cv::distanceTransform(bw, dist, cv::DIST_L2, 3);
        // Normalize the distance image for range = {0.0, 1.0}
        // so we can visualize and threshold it
        cv::normalize(dist, dist, 0, 1.0, cv::NORM_MINMAX);
        cv::imshow("Distance Transform Image", dist);
        
        // Threshold to obtain the peaks
        // This will be the markers for the foreground objects
        cv::threshold(dist, dist, 0.4, 1.0, cv::THRESH_BINARY);
        // Dilate a bit the dist image
        cv::Mat kernel1 = cv::Mat::ones(3, 3, CV_8U);
        cv::dilate(dist, dist, kernel1);
        cv::imshow("Peaks", dist);
        // Create the CV_8U version of the distance image
        // It is needed for findContours()
        cv::Mat dist_8u;
        dist.convertTo(dist_8u, CV_8U);
        // Find total markers
        std::vector< std::vector<cv::Point> > contours;
        cv::findContours(dist_8u, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // Create the marker image for the watershed algorithm
        cv::Mat markers = cv::Mat::zeros(dist.size(), CV_32S);
        // Draw the foreground markers
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::drawContours(markers, contours, static_cast<int>(i), cv::Scalar(static_cast<int>(i)+1), -1);
        }
        // Draw the background marker
        cv::circle(markers, cv::Point(5,5), 3, cv::Scalar(255), -1);
        cv::Mat markers8u;
        markers.convertTo(markers8u, CV_8U, 10);
        cv::imshow("Markers", markers8u);
        // Perform the watershed algorithm
        cv::watershed(imgResult, markers);
        cv::Mat mark;
        markers.convertTo(mark, CV_8U);
        cv::bitwise_not(mark, mark);
        //    imshow("Markers_v2", mark); // uncomment this if you want to see how the mark
        // image looks like at that point
        // Generate random colors
        std::vector<cv::Vec3b> colors;
        for (size_t i = 0; i < contours.size(); i++)
        {
            int b = cv::theRNG().uniform(0, 256);
            int g = cv::theRNG().uniform(0, 256);
            int r = cv::theRNG().uniform(0, 256);
            colors.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
        }
        // Create the result image
        cv::Mat dst = cv::Mat::zeros(markers.size(), CV_8UC3);
        // Fill labeled objects with random colors
        for (int i = 0; i < markers.rows; i++)
        {
            for (int j = 0; j < markers.cols; j++)
            {
                int index = markers.at<int>(i,j);
                if (index > 0 && index <= static_cast<int>(contours.size()))
                {
                    dst.at<cv::Vec3b>(i,j) = colors[index-1];
                }
            }
        }
        
        // Visualize the final image
        cv::imshow("Final Result", dst);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    
    void proc_img_1(cv::Mat& src) {
        cv::Mat kernel = (cv::Mat_<float>(3,3) <<
                    1,  1, 1,
                    1, -8, 1,
                    1,  1, 1); // an approximation of second derivative, a quite strong kernel

        cv::Mat imgLaplacian;
        cv::filter2D(src, imgLaplacian, CV_32F, kernel);
        
        cv::Mat sharp;
        src.convertTo(sharp, CV_32F);
        cv::Mat imgResult = sharp - imgLaplacian;
        
        // convert back to 8bits gray scale
        imgResult.convertTo(imgResult, CV_8UC3);
        imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
        // imshow( "Laplace Filtered Image", imgLaplacian );
        //cv::imshow( "New Sharped Image", imgResult );

        cv::Mat gray;
        cv::cvtColor(imgResult, gray, cv::COLOR_BGR2GRAY);

        cv::Mat img_otsu;
        cv::threshold(gray, img_otsu, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        //cv::imshow("Otsu",img_otsu);

         // 2.Calculo componentes conexas
        cv::Mat objetos;
        int nLabels = cv::connectedComponents(img_otsu, objetos, 8, CV_32S);
        std::cout << "Num Componentes Conexas: " << nLabels << std::endl;

        // especifica un color para cada etiqueta
        std::vector<cv::Vec3b> colors(nLabels);
        colors[0] = cv::Vec3b(0, 0, 0); // background -> black
        for (int label = 1; label < nLabels; ++label) {
            colors[label] = cv::Vec3b( (rand()&255), (rand()&255), (rand()&255) );
        }
        
        // inicializa las imagenes de los objetos
        std::vector<cv::Mat> objetosImagen;
        for (int i = 0; i < nLabels; i++) {
            cv::Mat im(img_otsu.size(), CV_8UC3);
            objetosImagen.push_back(im);
        }
        
        cv::Mat img_res(img_otsu.size(), src.type());
        // extraer los diferentes objetos y asignarlos a su imagen correspondiente
        for(int r = 0; r < objetos.rows; r++) {
            for(int c = 0; c < objetos.cols; c++) {
                int label = objetos.at<int>(r, c);
                if (label != 0) {
                    cv::Vec3b &pixel = src.at<cv::Vec3b>(r, c);
                    pixel = colors[label];
                }
            }
        }
        ///cv::imshow("img_Res", img_res);

        // mostrar objetos
        /*for(int i = 0; i < nLabels - 1; i++) {
            std::string st = "Objeto "+ std::to_string(i);
            cv::imshow(st, objetosImagen[i]);
        }*/

        //cv::waitKey(0);
        //cv::destroyAllWindows();
    }

    void proc_img_2(cv::Mat& src) {
        //cv::Mat src = cv::imread("/home/nacho8/ROS_workspaces/campero_ur10_ws/test/water_coins.jpg");

        cv::Mat gray;
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

        cv::Mat thresh;
        cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        //cv::imshow("Otsu",thresh);

        cv::Mat kernel = (cv::Mat_<int>(3,3) <<
                    1,  1, 1,
                    1, 1, 1,
                    1,  1, 1); // an approximation of second derivative, a quite strong kernel

        cv::Mat opening;
        cv::morphologyEx(thresh, opening, cv::MORPH_OPEN, kernel, cv::Point(-1,-1), 2);

        cv::Mat sure_bg;
        cv::dilate(opening, sure_bg, kernel, cv::Point(-1,-1), 3);
        
        // Finding sure foreground area
        cv::Mat dist_transform;
        cv::distanceTransform(opening, dist_transform, cv::DIST_L2, 5);
        cv::Mat sure_fg;
        
        double min, max;
        cv::minMaxLoc(dist_transform, &min, &max);
        cv::threshold(dist_transform, sure_fg, 0.7*max, 255, 0);
        //cv::imshow("sure_bg",sure_bg);
        //cv::imshow("sure_fg",sure_fg);
        
        // Finding unknown region
        sure_fg.convertTo(sure_fg, CV_8UC3);
        cv::Mat unknown;
        cv::subtract(sure_bg, sure_fg, unknown);
        //cv::imshow("unknown", unknown);

        // Marker labelling
        cv::Mat markers;
        int nLabels = cv::connectedComponents(sure_fg, markers);
        std::cout << "Num Componentes Conexas: " << nLabels << std::endl;
        
        // Add one to all labels so that sure background is not 0, but 1
        markers = markers+1;

        // Now, mark the region of unknown with zero
        for(int r = 0; r < markers.rows; r++) {
            for(int c = 0; c < markers.cols; c++) {
                unsigned char v = unknown.at<unsigned char>(r, c);
                if (v == 255) {
                    int &pixel = markers.at<int>(r, c);
                    pixel = 0;
                }
            }
        }

        cv::watershed(src, markers);
        
        for(int r = 0; r < markers.rows; r++) {
            for(int c = 0; c < markers.cols; c++) {
                int v = markers.at<int>(r, c);
                if (v == -1) {
                    cv::Vec3b &pixel = src.at<cv::Vec3b>(r, c);
                    pixel = cv::Vec3b(0, 0, 255);
                }
            }
        }

        // cv::imshow("res", src);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
    }
    
    void main() {
        //cv::Mat _img = cv::Mat::zeros(cv::Size(640, 480), CV_32SC3);

        cv::namedWindow("res", cv::WINDOW_AUTOSIZE);

        ros::Rate loop_rate(1);
        while (ros::ok()) {
            ros::spinOnce();
            //if (best_img != nullptr) cv::imshow("res", best_img->getImgCorrect());
            //cv::destroyAllWindows();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
    
    ImageInpainting im;
    
    /*ros::AsyncSpinner spinner(1);
    spinner.start();
    im.main();*/
    ros::spin();

    return 0;
}
