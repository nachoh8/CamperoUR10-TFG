#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h> 

#include <eigen3/Eigen/Geometry>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

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

typedef Eigen::Matrix<float, 3, 1> VectorEigen;

class ImageInpainting
{
private:
    int num_errors = 0, num_it = 0;

    /// Ros
    ros::NodeHandle nh;
    image_transport::ImageTransport it;

    // aruco markers and image
    ros::Subscriber markers_img_sub;

    // Camera Info
    ros::Subscriber cam_info_sub;
    bool cam_info_received = false;

    // Result Image
    image_transport::Publisher image_res_pub;

    // Debug Image
    image_transport::Publisher image_debug_pub;

    /// Image Procesing
    int TOP_LEFT_IDX, TOP_RIGHT_IDX, BOTTOM_RIGHT_IDX, BOTTOM_LEFT_IDX; // indice de la marca top left, top right, bottom left, bottom right
    std::vector<cv::Point2f> marker_center_pts; // position on warped img
    std::vector<cv::Point3f> marker_pose_pts; // pose in camera coordinates

    cv::Mat H; // homografy matrix (original image -> warped image)

    cv::Mat cameraMatrix, // 3x3
            distortionEffect; // 4x1
    cv::Size camSize;

    bool valid_matrix_pnp = false; // indica si rvec y tvec son matrices validas dadas por solvePnP
    cv::Mat rvec, // 1x3 
            tvec, // 1x3
            rotationMatrix; // 3x3

    // plano formado por las marcas aruco
    double z_markers_const; // distancia de la camara al plano, la camara es perpendicular al plano
    cv::Point3f normal_plane, plane_o; // normal y punto origen del plano

public:
    ImageInpainting() : it(nh) {
        image_debug_pub = it.advertise("/image_inpainting/image_debug", 1);
        image_res_pub = it.advertise("/image_inpainting/image_res", 1);
        markers_img_sub = nh.subscribe("/aruco_detector/markers_img", 1, &ImageInpainting::markers_img_callback, this);
        cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, &ImageInpainting::cam_info_callback, this);
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
    
    void orderMarkers(const std::vector<campero_ur10_msgs::ArucoMarker>& markers) {
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

    void correctImage(const std::vector<campero_ur10_msgs::ArucoMarker>& markers, cv::Mat& image) {
        
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
        cv::fillPoly(image, pts, cv::Scalar(255,255,255));
        
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
    
    std::vector<cv::Point> test(cv::Mat& img, cv::Mat& img_correct) {
        cv::Mat gray;
        cv::cvtColor(img_correct, gray, cv::COLOR_BGR2GRAY);

        // Noise Reduction
        cv::medianBlur(gray, gray, 5);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows/16, 100, 30, 1, 30);

        //if (circles.empty()) return cv::Point(-1,-1);
         
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
            res.push_back(pt);
            
            // Original image
            center_o = H_to_Orig(pt);
            cv::drawMarker(img, center_o, cv::Scalar(0,0,255), cv::MARKER_CROSS, 20, 3);
        }

        return res;
    }

    void getMarkersParameters(const std::vector<campero_ur10_msgs::ArucoMarker>& markers) {
        
        orderMarkers(markers);
        
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

    bool calculateTranformParam() {
        if (valid_matrix_pnp) { // si son validas tvec y rvec sirven como solución inicial
            cv::solvePnP(marker_pose_pts, marker_center_pts, cameraMatrix, distortionEffect, rvec, tvec, true);
        } else {
            cv::solvePnP(marker_pose_pts, marker_center_pts, cameraMatrix, distortionEffect, rvec, tvec);
        }
        cv::Rodrigues(rvec, rotationMatrix);
        
        z_markers_const = 0.0;
        for (int i = 0; i < marker_pose_pts.size(); i++) {
            //std::cout << "----ID " << i << "----" << std::endl;
            //std::cout << "world pose " << marker_pose_pts[i] << std::endl;

            cv::Mat pt_c = (cv::Mat_<double>(3,1) << marker_pose_pts[i].x, marker_pose_pts[i].y, marker_pose_pts[i].z);
            //std::cout << "pt_c " << pt_c << std::endl;

            cv::Mat1f uv_m = cameraMatrix * (rotationMatrix * pt_c + tvec);
            //std::cout << "uv_m " << uv_m << std::endl;
            
            //std::cout << "true center " << marker_center_pts[i] << std::endl;
            double d = uv_m(2);
            z_markers_const += d;
            //std::cout << "d " << d << std::endl;
            cv::Point2f uv(uv_m(0) / d, uv_m(1) / d);
            //std::cout << "uv " << uv << std::endl;
            // cv::circle( img_correct, cv::Point(uv.x, uv.y), 10, cv::Scalar(255,0,0), 3, cv::LINE_AA);

            if (cv::norm(uv - marker_center_pts[i]) > 5) {
                return false;
            }
        }

        z_markers_const /= marker_pose_pts.size();

        return true;
    }

    std::vector<cv::Point3f> pts2Camera(std::vector<cv::Point>& pts) {
        cv::Mat leftSideMat  = rotationMatrix.inv() * cameraMatrix.inv() * z_markers_const;
        cv::Mat rightSideMat = rotationMatrix.inv() * tvec;
        
        std::vector<cv::Point3f> res;
        for (int i = 0; i < pts.size(); i++) {
            cv::Point pt = pts[i];
            
            cv::Mat uvPt = (cv::Mat_<double>(3,1) << pt.x, pt.y, 1);
            cv::Mat1f pt_m  = leftSideMat * uvPt - rightSideMat;
            cv::Point3f pt_w(pt_m(0), pt_m(1), pt_m(2));

            res.push_back(pt_w);
        }

        return res;
    }

    void clearMarkers() {
        marker_center_pts.clear();
        marker_pose_pts.clear();
    }

    void markers_img_callback(const campero_ur10_msgs::ArucoMarkersImg& msg) {
        if (!cam_info_received) {
            ROS_WARN("Camera info not received");
            return;
        }
        
        static tf::TransformBroadcaster br;
        ros::Time t_current = ros::Time::now();
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg.img, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = cv_ptr->image;

            clearMarkers();

            std::vector<campero_ur10_msgs::ArucoMarker> markers = msg.markers.markers;

            getMarkersParameters(markers);
            
            cv::Mat img_correct = img.clone();
            correctImage(markers, img_correct);
            
            num_it++;
            valid_matrix_pnp = calculateTranformParam();
            if (!valid_matrix_pnp) {
                ROS_WARN("Error calculando parametros(%d/%d)", (++num_errors), num_it);
                return;
            }
            
            std::vector<cv::Point> pts = test(img, img_correct);
            std::vector<cv::Point3f> pts_w = pts2Camera(pts);

            geometry_msgs::Pose p = markers[0].pose;
            int i = 0;
            for (auto &pt_w : pts_w) {
                p.position.x = pt_w.x;
                p.position.y = pt_w.y;
                p.position.z = pt_w.z;
                tf::Transform transform;
                tf::poseMsgToTF(p, transform);
                tf::StampedTransform stampedTransform(transform, t_current,
                                                "camera_color_optical_frame", "pt_board_" + std::to_string(i));
                br.sendTransform(stampedTransform);
                i++;
            }

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

        cam_info_received = true;
        cam_info_sub.shutdown();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_inpainting");
    
    ImageInpainting im;
    
    ros::spin();

    return 0;
}