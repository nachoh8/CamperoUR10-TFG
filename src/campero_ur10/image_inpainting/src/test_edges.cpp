#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <image_transport/image_transport.h> // permite suscribirse/publicar en canales de imagen
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <campero_ur10_msgs/ArucoMarker.h>
#include <campero_ur10_msgs/ArucoMarkerArray.h>
#include <campero_ur10_msgs/ArucoMarkersImg.h>

typedef Eigen::Matrix<float, 3, 1> VectorEigen;

bool cam_info_received = false;

static const std::string camera_frame = "camera_color_optical_frame";
static const std::string cam_ref_frame = "base_link";
static const std::string robot_frame = "base_link";

/// Image Procesing
int TOP_LEFT_IDX, TOP_RIGHT_IDX, BOTTOM_RIGHT_IDX, BOTTOM_LEFT_IDX; // indice de la marca top left, top right, bottom left, bottom right
std::vector<cv::Point2f> marker_center_pts; // position on warped img
std::vector<cv::Point3f> marker_pose_pts; // pose in camera coordinates

cv::Mat H; // homografy matrix (original image -> warped image)

double z_markers_const; // distancia de la camara al plano, la camara es perpendicular al plano
cv::Point3f normal_plane, plane_o;

cv::Mat cameraMatrix, // 3x3
        distortionEffect; // 4x1
cv::Size camSize;
tf::StampedTransform rightToLeft;

ros::Subscriber cam_info_sub;

std::vector<campero_ur10_msgs::ArucoMarker> markers;
cv::Mat res, img_correct;
int canny_threshold = 10;

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
    if (len == 4) return;

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

bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform) {
    std::string errMsg;
    static tf::TransformListener _tfListener;
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

void apply_edge() {
    cv::Mat gray, blurImage;
    cv::cvtColor(img_correct, gray, cv::COLOR_BGR2GRAY );
    cv::blur(gray, blurImage, cv::Size(3,3));

    cv::Canny( blurImage, res, canny_threshold, canny_threshold * 3, 3);

    // Delete Markers
    /*
    std::vector< std::vector<cv::Point> > pts(markers.size());
    for (int i = 0; i < markers.size(); i++) {
        for (int j = 0; j < markers[i].img_points.size(); j++) {
            cv::Point pt(markers[i].img_points[j].x, markers[i].img_points[j].y);
            pts[i].push_back(Orig_to_H(pt));
        }
    }

    cv::drawContours(res, pts, -1, cv::Scalar(0), -1, cv::LINE_AA);*/

    cv::imshow("res", res);
}

static void on_trackbar( int, void* ) {
    apply_edge();
}

void markers_img_callback(const campero_ur10_msgs::ArucoMarkersImg& msg) {
    if (!cam_info_received) return;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg.img, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;
        
        markers.clear();
        marker_center_pts.clear();
        marker_pose_pts.clear();

        markers = msg.markers.markers;

        getMarkersParameters();
        
        img_correct = img.clone();
        correctImage(img_correct);
        
        char TrackbarContrast_gain[50];
        const int contrast_max_slider = 100;

        cv::namedWindow("res", cv::WINDOW_AUTOSIZE);
        sprintf( TrackbarContrast_gain, "Canny Threshold %d", 100 );
        cv::createTrackbar( TrackbarContrast_gain, "res", &canny_threshold, 100, on_trackbar );
        
        apply_edge();

        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
    if (!cam_info_received) return;
    
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        cv::Mat img = cv_ptr->image.clone(); // image with markers

        img_correct = img.clone();

        char TrackbarContrast_gain[50];
        const int contrast_max_slider = 100;

        cv::namedWindow("res", cv::WINDOW_AUTOSIZE);
        sprintf( TrackbarContrast_gain, "Canny Threshold %d", 100 );
        cv::createTrackbar( TrackbarContrast_gain, "res", &canny_threshold, 100, on_trackbar );
        
        apply_edge();

        cv::waitKey(0);
        cv::destroyAllWindows();
    
    } catch (cv_bridge::Exception& e) {

        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_inpainting_test");
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    res = cv::Mat(cv::Size(640,480), CV_8UC3);
    
    ros::NodeHandle nh;
    cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, &cam_info_callback);
    image_transport::ImageTransport it(nh);
    //image_transport::Subscriber image_sub = it.subscribe("/camera/color/image_rect_color", 1, &image_callback);
    ros::Subscriber sub = nh.subscribe("/aruco_detector/markers_img", 1, &markers_img_callback);

    
    ros::Rate rate(1);

    
    while (ros::ok()) {
        rate.sleep();
    }
    
}