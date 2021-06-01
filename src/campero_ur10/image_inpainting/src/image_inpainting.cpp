#include <iostream>
#include <string>
#include <vector>

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

class ImageInpainting
{
private:
    int num_errors = 0, num_it = 0;

    /// Ros
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    tf::TransformListener _tfListener;

    std::string camera_frame , cam_ref_frame, robot_frame;
	
    // aruco markers and image
    ros::Subscriber markers_img_sub;
    std::vector<campero_ur10_msgs::ArucoMarker> markers;

    // Camera Info
    bool cam_info_received = false;
    ros::Subscriber cam_info_sub;
    cv::Mat cameraMatrix, // 3x3
            distortionEffect; // 4x1
    cv::Size camSize;

    tf::StampedTransform rightToLeft;

    /// Image Procesing
    int TOP_LEFT_IDX, TOP_RIGHT_IDX, BOTTOM_RIGHT_IDX, BOTTOM_LEFT_IDX; // indice de la marca top left, top right, bottom left, bottom right
    std::vector<cv::Point2f> marker_center_pts; // position on warped img
    std::vector<cv::Point3f> marker_pose_pts; // pose in camera coordinates

    cv::Mat H; // homografy matrix (original image -> warped image)

    int max_dist_error;
    bool valid_matrix_pnp = false; // indica si rvec y tvec son matrices validas dadas por solvePnP
    cv::Mat rvec, // 1x3 
            tvec, // 1x3
            rotationMatrix; // 3x3

    // plano formado por las marcas aruco
    double z_markers_const; // distancia de la camara al plano, la camara es perpendicular al plano
    cv::Point3f normal_plane, plane_o; // normal y punto origen del plano

    // Result
    double min_dist_error; // imagen procesada con el menor error
    geometry_msgs::PoseArray poses_res; // posicion de los puntos reconocidos en el mundo
    campero_ur10_msgs::ImageDraw img_res_msg;

    ros::Subscriber process_cmd_sub;
    bool update_image = true;
    ros::Publisher img_pts_pub;
    image_transport::Publisher image_res_pub, image_debug_pub;

public:
    ImageInpainting() : it(nh) {
        markers_img_sub = nh.subscribe("/aruco_detector/markers_img", 1, &ImageInpainting::markers_img_callback, this);
        cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, &ImageInpainting::cam_info_callback, this);
        process_cmd_sub = nh.subscribe("/" + NODE_NAME + "/cmd", 1, &ImageInpainting::process_cmd_callback, this);

        image_debug_pub = it.advertise("/" + NODE_NAME + "/image_debug", 1);
        image_res_pub = it.advertise("/" + NODE_NAME + "/image_res", 1);
        // img_pts_pub = nh.advertise<geometry_msgs::PoseArray>("/" + NODE_NAME + "/" + TOPIC_NAME_IMG_DRAW, 1);
        img_pts_pub = nh.advertise<campero_ur10_msgs::ImageDraw>(TOPIC_NAME_IMG_DRAW, 1);

        nh.param<std::string>("/" + NODE_NAME + "/cam_ref_frame", cam_ref_frame, "");
        nh.param<std::string>("/" + NODE_NAME + "/camera_frame", camera_frame, "");
        nh.param<std::string>("/" + NODE_NAME + "/robot_frame", robot_frame, "");
        nh.param<int>("/" + NODE_NAME + "/max_dist_error", max_dist_error, DEAULT_MAX_DIST_ERROR);

        min_dist_error = max_dist_error + 1.0f;

        img_res_msg.W = -1;
        img_res_msg.H = -1;

        poses_res.header.seq = 0;
        poses_res.header.frame_id = robot_frame;

        ROS_INFO("Max Distance Error: %d px", max_dist_error);
        ROS_INFO("Robot frame: %s | Camera Reference frame: %s | Camera frame: %s ", robot_frame.c_str(), cam_ref_frame.c_str(), camera_frame.c_str());
    }

    cv::Point H_to_Orig(cv::Point pt) {
        cv::Point3d p_src(pt.x, pt.y, 1);
        cv::Point3d p_dst(cv::Mat(H.inv() * cv::Mat(p_src)));
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

    void correctImage(cv::Mat& image) {
        
        // Cast to cv::Point/2f type
        std::vector< std::vector<cv::Point> > pts(markers.size());
        std::vector<cv::Point2f> pts_f;
        for (int i = 0; i < markers.size(); i++) {
            for (int j = 0; j < markers[i].img_points.size(); j++) {
                pts[i].push_back(cv::Point(markers[i].img_points[j].x, markers[i].img_points[j].y));
                pts_f.push_back(cv::Point2f(markers[i].img_points[j].x, markers[i].img_points[j].y));
            }
        }
        
        // Delete Markers
        cv::drawContours(image, pts, -1, cv::Scalar(255,255,255), -1);

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
        cv::warpPerspective(image, image, H, cv::Size(max_W, max_H));

        // Get Marker Centers on warped img
        for (int i = 0; i < markers.size(); i++) {
            cv::Point center = Orig_to_H(getMarkerCenter(markers[i]));
            marker_center_pts.push_back(center);
            // cv::circle( image, center, 5, cv::Scalar(0,0,255), 3, cv::LINE_AA);
        }

        /*cv::Mat gray, blurImage, edge1, edge2, cedge;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY );
        cv::blur(gray, blurImage, cv::Size(3,3));

        cv::Canny( blurImage, edge1, 100, 300, 3);

        cedge = cv::Scalar::all(0);
        image.copyTo(cedge, edge1);

        cv::Mat dx,dy;
        cv::Scharr(blurImage,dx,CV_16S,1,0);
        cv::Scharr(blurImage,dy,CV_16S,0,1);
        cv::Canny( dx,dy, edge2, 400, 1200 );
        cedge = cv::Scalar::all(0);
        image.copyTo(cedge, edge2);
        
        // Delete Markers
        //cv::fillPoly(image, pts, cv::Scalar(255,255,255));
        //cv::drawContours(image, pts, -1, cv::Scalar(255,255,255), -1);
        for (int i = 0; i < pts.size(); i++) {
            for (int j = 0; j < pts[i].size(); j++) {
                cv::Point pt = Orig_to_H(pts[i][j]);
                pts[i][j] = pt;
            }
        }
        cv::drawContours(cedge, pts, -1, cv::Scalar(0), -1, cv::LINE_AA);

        cedge.convertTo(image, CV_8UC3);*/
        /*cv::imshow("r", cedge);
        cv::waitKey(0);
        cv::destroyAllWindows();*/
    }
    
    std::vector<cv::Point> test(cv::Mat& img, cv::Mat& img_correct) {
        cv::Mat gray;
        cv::cvtColor(img_correct, gray, cv::COLOR_BGR2GRAY);

        // Noise Reduction
        cv::medianBlur(gray, gray, 5);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows/16, 100, 30, 1, 30);

        if (circles.empty()) {
			std::vector<cv::Point> res;
			return res;
		 }
         
        // cogemos el primero
        cv::Vec3i c = circles[0];

        // Draw on Warped Image
        cv::Point center = cv::Point(c[0], c[1]);
        // draw marker on center
        cv::drawMarker(img_correct, center, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 3);
        // draw circle
        int radius = c[2];
        cv::circle( img_correct, center, radius, cv::Scalar(0,0,255), 3, cv::LINE_AA);

        // Transform to original image
        cv::Point pt_r = H_to_Orig(cv::Point(center.x + radius, center.y)); // punto en la circunferencia del circulo
        cv::Point center_o = H_to_Orig(center);
        float r = cv::norm(pt_r - center_o);
        
        // Draw on Original Image
        cv::drawMarker(img, center_o, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 3);
        cv::circle( img, center_o, r, cv::Scalar(0,0,255), 3, cv::LINE_AA);

        std::vector<cv::Point> res;
        res.push_back(center);

        for (int i = 0; i < marker_center_pts.size(); i++) {
            // Warped image
            cv::Point pt(marker_center_pts[i].x, marker_center_pts[i].y);
            cv::drawMarker(img_correct, pt, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 3);
            //res.push_back(pt);
            
            // Original image
            center_o = H_to_Orig(pt);
            cv::drawMarker(img, center_o, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 3);
        }

        return res;
    }

    std::vector<cv::Point> test_pts_circulo(cv::Mat& img, cv::Mat& img_correct) {
        cv::Mat gray;
        cv::cvtColor(img_correct, gray, cv::COLOR_BGR2GRAY);

        // Noise Reduction
        cv::medianBlur(gray, gray, 5);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows/16, 100, 30, 1, 200);

        std::vector<cv::Point> res;
        if (circles.empty()) return res;
         
        // cogemos el primero
        cv::Vec3i c = circles[0];

        // Draw on Warped Image
        cv::Point center = cv::Point(c[0], c[1]);
        int radius = c[2];

        // draw center
        cv::drawMarker(img_correct, center, cv::Scalar(0,0,255), cv::MARKER_CROSS, 15, 2);
        
        // draw circle
        for (double th = 0; th < 2*M_PI; th += 0.1) {
            cv::Point pt(center.x + radius * cos(th), center.y + radius * sin(th));
            cv::drawMarker(img_correct, pt, cv::Scalar(0,0,255), cv::MARKER_CROSS, 5, 1);

            cv::Point pt_o = H_to_Orig(pt);
            cv::drawMarker(img, pt_o, cv::Scalar(0,0,255), cv::MARKER_CROSS, 5, 1);

            res.push_back(pt);
        }

        // Transform to original image
        cv::Point center_o = H_to_Orig(center);
        
        // Draw on Original Image
        cv::drawMarker(img, center_o, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 3);

        for (int i = 0; i < marker_center_pts.size(); i++) {
            // Warped image
            cv::Point pt(marker_center_pts[i].x, marker_center_pts[i].y);
            cv::drawMarker(img_correct, pt, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 3);
            
            // Original image
            center_o = H_to_Orig(pt);
            cv::drawMarker(img, center_o, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 3);
        }

        return res;
    }

    std::vector< std::vector<cv::Point> > test_pts_contours(cv::Mat& img, cv::Mat& img_correct) {
        cv::Mat gray, blurImage, res_img;
		cv::cvtColor(img_correct, gray, cv::COLOR_BGR2GRAY );
		cv::blur(gray, blurImage, cv::Size(3,3));

		cv::Canny( blurImage, res_img, 100, 300, 3);
        
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
        
        cv::drawContours(res_img, pts, -1, cv::Scalar(0), -1);
		
		// Find contours
		std::vector< std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> h;
		cv::findContours(res_img, contours, h, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		
		std::cout << "Num Contours: " << contours.size() << std::endl;
		cv::imshow("canny", res_img);
		cv::waitKey(0);
		cv::destroyAllWindows();
		// Process Result
        //std::vector<cv::Point> res;
		
		cv::drawContours(img_correct, contours, -1, cv::Scalar(0,0,255), 1);

        for (int i = 0; i < contours.size(); i++) {
			for (int j = 0; j < contours[i].size(); j++) {
				// Warped image
				cv::Point pt(contours[i][j].x, contours[i][j].y);
				cv::drawMarker(img_correct, pt, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 1);
				//res.push_back(pt);
				// Original image
				pt = H_to_Orig(pt);
				cv::drawMarker(img, pt, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 1);
			}
        }

        for (int i = 0; i < marker_center_pts.size(); i++) {
            // Warped image
            cv::Point pt(marker_center_pts[i].x, marker_center_pts[i].y);
            cv::drawMarker(img_correct, pt, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 2);
            //res.push_back(pt);
            // Original image
            pt = H_to_Orig(pt);
            cv::drawMarker(img, pt, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 2);
        }
        
        //res_img.convertTo(img_correct, img_correct.type());

        return contours;
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

    double calculateTranformParam() {
        if (valid_matrix_pnp) { // si son validas tvec y rvec sirven como solución inicial
            cv::solvePnP(marker_pose_pts, marker_center_pts, cameraMatrix, distortionEffect, rvec, tvec, true);
        } else {
            cv::solvePnP(marker_pose_pts, marker_center_pts, cameraMatrix, distortionEffect, rvec, tvec);
        }
        cv::Rodrigues(rvec, rotationMatrix);
        
        double _error_mean = 0.0;
        z_markers_const = 0.0;
        for (int i = 0; i < marker_pose_pts.size(); i++) {

            cv::Mat pt_c = (cv::Mat_<double>(3,1) << marker_pose_pts[i].x, marker_pose_pts[i].y, marker_pose_pts[i].z);

            cv::Mat1f uv_m = cameraMatrix * (rotationMatrix * pt_c + tvec);
            
            double d = uv_m(2);
            z_markers_const += d;
            cv::Point2f uv(uv_m(0) / d, uv_m(1) / d);

            double _error = cv::norm(uv - cv::Point2f(marker_center_pts[i]));
            if (_error > max_dist_error) {
                /*std::cout << "uv: " << uv << " | true_ground: " << marker_center_pts[i] << std::endl;
                std::cout << "Error: " <<  _error << std::endl;*/
                valid_matrix_pnp = false;
                return max_dist_error;
            }
            _error_mean += _error;
        }
        z_markers_const /= marker_pose_pts.size();
        _error_mean /= marker_pose_pts.size();

        valid_matrix_pnp = true;

        return _error_mean;
    }

    std::vector< std::vector<cv::Point3f> > pts2Camera(std::vector< std::vector<cv::Point> >& pts) {
        cv::Mat leftSideMat  = rotationMatrix.inv() * cameraMatrix.inv() * z_markers_const;
        cv::Mat rightSideMat = rotationMatrix.inv() * tvec;
        
        std::vector< std::vector<cv::Point3f> > res(pts.size());
        for (int i = 0; i < pts.size(); i++) {
			for (int j = 0; j < pts[i].size(); j++) {
				cv::Point pt = pts[i][j];
            
				cv::Mat uvPt = (cv::Mat_<double>(3,1) << pt.x, pt.y, 1);
				cv::Mat1f pt_m  = leftSideMat * uvPt - rightSideMat;
				cv::Point3f pt_w(pt_m(0), pt_m(1), pt_m(2));

				res[i].push_back(pt_w);
			}
        }

        return res;
    }

    void ptsCam2World(const std::vector< std::vector<cv::Point3f> >& pts_cam, const geometry_msgs::Pose& pose_base, const tf::Transform& transform_base) {
        geometry_msgs::Pose p = pose_base;
        poses_res.poses.clear();
        img_res_msg.traces.clear();

        
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
				poses_res.poses.push_back(pose);
				
				campero_ur10_msgs::ImgPoint pt_msg;
				pt_msg.x = pose.position.x;
				pt_msg.y = pose.position.y;
				pt_msg.z = pose.position.z;
				trace.points.push_back(pt_msg);
			}

			img_res_msg.traces.push_back(trace);
		}
    }

    void clearMarkers() {
		markers.clear();
        marker_center_pts.clear();
        marker_pose_pts.clear();
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

    void markers_img_callback(const campero_ur10_msgs::ArucoMarkersImg& msg) {
        if (!update_image) return;

        if (!cam_info_received) {
            ROS_WARN("Camera info not received");
            return;
        }
        
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg.img, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = cv_ptr->image;

            clearMarkers();

            markers = msg.markers.markers;

            getMarkersParameters();
            
            cv::Mat img_correct = img.clone();
            correctImage(img_correct);
            
            
            double _error = calculateTranformParam();

            num_it++;
            if (!valid_matrix_pnp) {
                ROS_WARN("Error calculando parametros(%d/%d)", (++num_errors), num_it);
                return;
            }
            
            if (_error > min_dist_error && poses_res.poses.size() > 0) return;

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
            
            std::vector< std::vector<cv::Point> > pts_img = test_pts_contours(img, img_correct);
            
            if (pts_img.empty()) return;
            
            std::vector< std::vector<cv::Point3f> > pts_cam = pts2Camera(pts_img);
			
            ptsCam2World(pts_cam, markers[TOP_LEFT_IDX].pose, transform_base);
            
			min_dist_error = _error;
			
			ROS_INFO("--New best image--");
			ROS_INFO("Min error: %f ", min_dist_error);
			ROS_INFO("Number of points to send: %d ", poses_res.poses.size());
			ROS_INFO("Z const: %f ", z_markers_const);

            // Publish Result Image
            if (image_res_pub.getNumSubscribers() > 0) {
                cv_bridge::CvImage out_msg;
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = img;
                image_res_pub.publish(out_msg.toImageMsg());
            }

            // Publish Debug Image
            if (image_debug_pub.getNumSubscribers() > 0) {
                cv_bridge::CvImage out_msg;
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = img_correct;
                image_debug_pub.publish(out_msg.toImageMsg());
            }
            
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void cam_info_callback(const sensor_msgs::CameraInfo &cam_info) {
        ROS_INFO("Camera info received");

        cameraMatrix = cv::Mat(3, 3, CV_64FC1);
        distortionEffect = cv::Mat(4, 1, CV_64FC1);
        camSize = cv::Size(cam_info.height, cam_info.width);

        cameraMatrix.setTo(0);
        cameraMatrix.at<double>(0,0) = cam_info.P[0];   cameraMatrix.at<double>(0,1) = cam_info.P[1];   cameraMatrix.at<double>(0,2) = cam_info.P[2];
        cameraMatrix.at<double>(1,0) = cam_info.P[4];   cameraMatrix.at<double>(1,1) = cam_info.P[5];   cameraMatrix.at<double>(1,2) = cam_info.P[6];
        cameraMatrix.at<double>(2,0) = cam_info.P[8];   cameraMatrix.at<double>(2,1) = cam_info.P[9];   cameraMatrix.at<double>(2,2) = cam_info.P[10];

        for(int i=0; i<4; ++i)
            distortionEffect.at<double>(i, 0) = 0;

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
            ROS_INFO("--Command: Image Reset--");
            poses_res.poses.clear();
            img_res_msg.traces.clear();
            min_dist_error = max_dist_error + 0.1;
        } else if (cmd == "send") {
            ROS_INFO("--Command: Send Image Points--");

            if (min_dist_error >= max_dist_error) {
                ROS_WARN("Error_Mean_IMAGE(%f) >= MAX_ERROR(%d)", min_dist_error, max_dist_error);
                return;
            }

            if (poses_res.poses.empty()) {
                ROS_WARN("No hay puntos para enviar");
                return;
            }

            if (img_pts_pub.getNumSubscribers() > 0) {

                img_pts_pub.publish(img_res_msg);

                ROS_INFO("Image Send Correct");
            } else {
                ROS_WARN("No subscribers");
            }
        } else if (cmd == "update_on"){
            ROS_WARN("--Command: Update Image Enable--");
            update_image = true;
        } else if (cmd == "update_off"){
            ROS_WARN("--Command: Update Image Disable--");
            update_image = false;
        } else {
            ROS_WARN("--Command: not known--");
        }
        
    }

    void main() {
        ros::Rate loop_rate(10);

        static tf::TransformBroadcaster br;
        // wait
        while(ros::ok()) {
            ros::spinOnce();

            if (poses_res.poses.size() > 0) { // show points in rviz
                ros::Time t_current = ros::Time::now();
                int i = 0;
                for (auto &pt : poses_res.poses) {
                    tf::Transform transform;
                    tf::poseMsgToTF(pt, transform);
                    
                    tf::StampedTransform stampedTransform(transform, t_current,
                                                    robot_frame, "pt_board_" + std::to_string(i));
                    br.sendTransform(stampedTransform);
                    i++;
                }
            }

            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
    
    ImageInpainting im;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ros::spin();

    im.main();

    return 0;
}
