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

void show_img(const cv::Mat& img) {
    cv::imshow("img", img);
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

    cv::Mat getImgOriginal() const {
        return img_original_show;
    }

    cv::Mat getImgDebug() const {
        return img_debug_show;
    }

    campero_ur10_msgs::ImageDraw getImageDrawMsg()  const {
        return img_res_msg;
    }

    void setTransform(tf::Transform _transform_base) {
       transform_base = _transform_base;
    }

    /*void findContours_old(const double canny_threshold1, const double canny_threshold2, const int blur_ksize) {
        cv::Mat gray, blurImage, res_img;
		cv::cvtColor(img_correct, gray, cv::COLOR_BGR2GRAY );
		cv::blur(gray, blurImage, cv::Size(blur_ksize, blur_ksize));

		cv::Canny( blurImage, res_img, canny_threshold1, canny_threshold2, 3);
        
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
		std::vector<cv::Vec4i> h;
        contours.clear();
		cv::findContours(res_img, contours, h, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		// show_img(res_img);

        // Process Result
        img_debug_show = img_correct.clone();
        cv::RNG rng(12345);
        for (int i = 0; i < contours.size(); i++ ) {
            cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
            cv::drawContours( img_correct_show, contours, (int)i, color, 2, cv::LINE_8, h, 0 );
        }

        for (int i = 0; i < marker_center_pts.size(); i++) {
            // Warped image
            cv::Point pt(marker_center_pts[i].x, marker_center_pts[i].y);
            //cv::drawMarker(img_correct_show, pt, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 2);
            //res.push_back(pt);
            // Original image
            pt = H_to_Orig(pt);
            cv::drawMarker(img_original, pt, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 2);
        }
    }*/

    void findContours() {
        cv::Mat gray, blurImage, res_img;
		cv::cvtColor(img_correct, gray, cv::COLOR_BGR2GRAY );
		cv::blur(gray, blurImage, cv::Size(img_params->blur_ksize, img_params->blur_ksize));

		cv::Canny( blurImage, res_img, img_params->canny_threshold1, img_params->canny_threshold2, 3);
        
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
		
        // Apply Morphological Operations 
        const int dilate_type = img_params->getDilateType();
        if (dilate_type != -1) {
            cv::Mat element = cv::getStructuringElement( dilate_type,
                       cv::Size( 2 * img_params->dilate_size + 1, 2 * img_params->dilate_size + 1 ),
                       cv::Point( img_params->dilate_size, img_params->dilate_size ) );
            cv::dilate(res_img, res_img, element );
        }

        const int erode_type = img_params->getErodeType();
        if (erode_type != -1) {
            cv::Mat element = cv::getStructuringElement( erode_type,
                       cv::Size( 2 * img_params->erode_size + 1, 2 * img_params->erode_size + 1 ),
                       cv::Point( img_params->erode_size, img_params->erode_size ) );
            cv::erode(res_img, res_img, element );
        }

		// Find contours
		std::vector<cv::Vec4i> h;
        contours.clear();
		cv::findContours(res_img, contours, h, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        cv::cvtColor(res_img, img_debug_show, CV_GRAY2RGB);
        
        
        // Process Result
        img_original_show = cv::Mat(img_debug_show.size(), img_debug_show.type(), cv::Scalar(0,0,0));

        std::vector< std::vector<cv::Point> > contours_orig(contours.size());
        //img_original_show = img_original.clone();
        
        cv::RNG rng(12345);
        for (int i = 0; i < contours.size(); i++ ) {
            
            /*for (int j = 0; j < contours[j].size(); j++) {
                contours_orig[i].push_back(H_to_Orig(contours[i][j]));
            }*/


            cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
            //cv::drawContours( img_original_show, contours_orig, i, color, 1, cv::LINE_8, cv::Mat(), 0 );
            cv::drawContours( img_original_show, contours, i, color, 1, cv::LINE_8, h, 0 );
        }
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

            if (best_img != nullptr && img_p->getError() > best_img->getError() && img_p->numberCountours() < best_img->numberCountours()) return;

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
            img_p->findContours();

            if (img_p->numberCountours() == 0) {
                return;
            }

            best_img.reset(img_p);
            
            best_img->img2World();

            ROS_INFO("--New best image--");
			ROS_INFO("Min error: %f ", best_img->getError());
			ROS_INFO("Number of contours to send: %d ", best_img->numberCountours());
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

            
            best_img->findContours();
            
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
