#include <ros/ros.h>
#include <image_transport/image_transport.h> // permite suscribirse/publicar en canales de imagen
#include <cv_bridge/cv_bridge.h> // puente entre OpenCV y sensor_msgs/Image
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageInpainting
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;

    // aruco markers
    ros::Subscriber aruco_sub;

    // Image Raw
    image_transport::Subscriber image_sub;

    // Result Image
    image_transport::Publisher image_res_pub;

public:
    ImageInpainting() : it(nh) {
        image_sub = it.subscribe("/aruco_detector/image_raw", 1, &ImageInpainting::image_callback, this);
        image_res_pub = it.advertise("/image_inpainting/image_res", 1);
        //aruco_sub = nh.subscribe("/camera_info", 1, &ArucoDetector::cam_info_callback, this);
    }

    ~ImageInpainting(){}

    void image_callback(const sensor_msgs::ImageConstPtr& msg) {

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_inpainting");
    
    ImageInpainting im;
    
    ros::spin();

    return 0;
}