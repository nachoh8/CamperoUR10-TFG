#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h> 

#include <tf/tf.h>

#include <image_transport/image_transport.h> // permite suscribirse/publicar en canales de imagen
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/Pose.h>

#include <cv_bridge/cv_bridge.h> // puente entre OpenCV y sensor_msgs/Image

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <campero_ur10_msgs/ArucoMarker.h>
#include <campero_ur10_msgs/ArucoMarkerArray.h>
#include <campero_ur10_msgs/ArucoMarkersImg.h>

class ImageInpainting
{
private:
    /// Ros

    ros::NodeHandle nh;
    image_transport::ImageTransport it;

    // aruco markers and image
    ros::Subscriber markers_img_sub;

    // Result Image
    image_transport::Publisher image_res_pub;

    // Debug Image
    image_transport::Publisher image_debug_pub;

    /// Image Procesing
    cv::Mat H; // homografy matrix
    

public:
    ImageInpainting() : it(nh) {
        image_debug_pub = it.advertise("/image_inpainting/image_debug", 1);
        image_res_pub = it.advertise("/image_inpainting/image_res", 1);
        markers_img_sub = nh.subscribe("/aruco_detector/markers_img", 1, &ImageInpainting::markers_img_callback, this);
    }

    ~ImageInpainting(){}

    cv::Point ptTfPerspective(cv::Point pt) {
        cv::Point3d p_src(pt.x, pt.y, 1);
        cv::Point3d p_dst(cv::Mat(H.inv() * cv::Mat(p_src)));
        p_dst /= p_dst.z; // normalize

        return cv::Point(p_dst.x, p_dst.y);
    }

    geometry_msgs::Pose pt2Pose(cv::Point pt) {

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
    
    void correctImage(std::vector<campero_ur10_msgs::ArucoMarker>& markers, cv::Mat& image) {
        
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
    }
    
    cv::Point test(cv::Mat& img, cv::Mat& img_correct) {
        cv::Mat gray;
        cv::cvtColor(img_correct, gray, cv::COLOR_BGR2GRAY);

        // Noise Reduction
        cv::medianBlur(gray, gray, 5);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows/16, 100, 30, 1, 30);

        if (circles.empty()) return cv::Point(-1,-1);
         
        // cogemos el primero
        cv::Vec3i c = circles[0];

        // Draw on Warped Image
        cv::Point center = cv::Point(c[0], c[1]);
        // draw marker on center
        cv::drawMarker(img_correct, center, cv::Scalar(0,0,255), cv::MARKER_CROSS);
        // draw circle
        int radius = c[2];
        cv::circle( img_correct, center, radius, cv::Scalar(0,0,255), 3, cv::LINE_AA);

        // Transform to original image
        cv::Point pt_r = ptTfPerspective(cv::Point(center.x + radius, center.y)); // punto en la circunferencia del circulo
        cv::Point center_o = ptTfPerspective(center);
        float r = cv::norm(pt_r - center_o);
        
        // Draw on Original Image
        cv::drawMarker(img, center_o, cv::Scalar(0,0,255), cv::MARKER_CROSS);
        cv::circle( img, center_o, r, cv::Scalar(0,0,255), 3, cv::LINE_AA);

        return center_o;
    }

    void markers_img_callback(const campero_ur10_msgs::ArucoMarkersImg& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg.img, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = cv_ptr->image;

            //campero_ur10_msgs::ArucoMarkerArray markers = msg.markers;
            std::vector<campero_ur10_msgs::ArucoMarker> markers = msg.markers.markers;
            
            cv::Mat img_correct = img.clone();
            correctImage(markers, img_correct);

            cv::Point pt = test(img, img_correct);
            if (pt.x == -1) {
                ROS_WARN("Test failed");
            } else {
                // 1.Obtener 4 puntos esquinas imagen
                // 2.Obtener 4 puntos esquinas imagen
                // 3.obtener de la camara -> camaraMatrix y distMatrix
                
                /*geometry_msgs::Pose p = markers[0].pose;
                tf::Transform world_ref;
                tf::poseMsgToTF(p, world_ref);
                
                geometry_msgs::Vector3 v;
                tf::vector3TFToMsg(world_ref.getOrigin(),v);
                std::cout << v << std::endl;
                std::cout << p << std::endl;

                
                tf::Vector3 t = world_ref * world_ref.getOrigin();
                tf::vector3TFToMsg(t,v);
                tf::Quaternion q = world_ref.getRotation();
                tf::Matrix3x3 m(q);

                double r, p, y;
                m.getRPY(r,p,y);

                cv::Mat rvec = (cv::Mat_<double>(1,3))
                std::cout << v << std::endl;*/
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_inpainting");
    
    ImageInpainting im;
    
    ros::spin();

    return 0;
}