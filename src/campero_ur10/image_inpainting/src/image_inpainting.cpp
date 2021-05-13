#include <ros/ros.h>
#include <image_transport/image_transport.h> // permite suscribirse/publicar en canales de imagen
#include <cv_bridge/cv_bridge.h> // puente entre OpenCV y sensor_msgs/Image
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter()
    : it_(nh_)
    {
    // Subscrive to input video feed and publish output video feed
    // /campero/front_ptz_camera/image_raw
    // /campero/front_ptz_camera/image_rect_color
    // /campero/front_rgbd_camera/color/image_raw

    // /campero/rear_rgbd_camera/ -> ¿es la camara real sense del brazo?
    // Topics:
    // /campero/rear_rgbd_camera/color/image_raw -> 
    // /campero/rear_rgbd_camera/depth/image_rect_raw -> 
    // /campero/rear_rgbd_camera/infra1/image_rect_raw ->
    // /campero/rear_rgbd_camera/infra2/image_rect_raw ->
    // /campero/rear_rgbd_camera/realsense2_camera_manager/bond ->
    image_sub_ = it_.subscribe("/campero/rear_rgbd_camera/color/image_raw", 1,
        &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
    cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
    
    // Metodo 1: copiar la imagen y convertir
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    /*
    // Metodo 2: apunta a la misma imagen y convierte
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        if (enc::isColor(msg->encoding)) // si la imagen se puede convertir a color
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        else // si es monocromatica
            cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
    } catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    */

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}