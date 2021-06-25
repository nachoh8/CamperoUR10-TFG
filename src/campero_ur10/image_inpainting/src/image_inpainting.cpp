#include "../include/image_inpainting/utils.h"
#include "../include/image_inpainting/img_params.hpp"
#include "../include/image_inpainting/img_res.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <ros/ros.h> 
#include "std_msgs/String.h"

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

#define TEST 0

static const std::string NODE_NAME = "image_inpainting";
static const std::string TOPIC_NAME_IMG_DRAW = "image_points";

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
    #if TEST
    image_transport::Subscriber image_test_sub;
    #endif

    /// Result
    ros::Publisher img_pts_pub;
    image_transport::Publisher image_res_pub, image_debug_pub;

    std::unique_ptr<ImageRes> best_img;

    void load_img_params() {
        nh.param<int>("/" + NODE_NAME + "/max_dist_error", img_params->max_dist_error, DEAULT_MAX_DIST_ERROR);
        nh.param<int>("/" + NODE_NAME + "/contour_method", img_params->contour_method, 0);
        nh.param<int>("/" + NODE_NAME + "/min_contour_size", img_params->min_contour_size, 30);
        nh.param<bool>("/" + NODE_NAME + "/apply_concaveman", img_params->apply_concaveman, true);
        nh.param<double>("/" + NODE_NAME + "/concaveman_alpha", img_params->concaveman_alpha, 1.0);
        nh.param<bool>("/" + NODE_NAME + "/apply_sharp", img_params->apply_sharp, false);
        nh.param<int>("/" + NODE_NAME + "/binary_thresh", img_params->binary_thresh, 0);
        nh.param<int>("/" + NODE_NAME + "/conectivity_way", img_params->conectivity_way, 8);
        nh.param<int>("/" + NODE_NAME + "/number_iterations", img_params->number_iterations, 9);
        nh.param<int>("/" + NODE_NAME + "/blur_ksize", img_params->blur_ksize, 3);
        nh.param<int>("/" + NODE_NAME + "/erode_type", img_params->erode_type, 0);
        nh.param<int>("/" + NODE_NAME + "/erode_size", img_params->erode_size, 3);
        nh.param<int>("/" + NODE_NAME + "/dilate_type", img_params->dilate_type, 0);
        nh.param<int>("/" + NODE_NAME + "/dilate_size", img_params->dilate_size, 3);

        nh.param<int>("/" + NODE_NAME + "/canny_th1", img_params->canny_threshold1, 100);
        nh.param<int>("/" + NODE_NAME + "/canny_th2", img_params->canny_threshold2, 300);
    }
    
public:
    ImageInpainting() : it(nh) {
        markers_img_sub = nh.subscribe("/aruco_detector/markers_img", 1, &ImageInpainting::markers_img_callback, this);
        cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, &ImageInpainting::cam_info_callback, this);
        process_cmd_sub = nh.subscribe("/" + NODE_NAME + "/cmd", 1, &ImageInpainting::process_cmd_callback, this);
        
        #if TEST
        image_test_sub = it.subscribe("/camera/color/image_rect_color", 1, &ImageInpainting::image_callback, this);
        #endif

        image_debug_pub = it.advertise("/" + NODE_NAME + "/image_debug", 1);
        image_res_pub = it.advertise("/" + NODE_NAME + "/image_res", 1);
        img_pts_pub = nh.advertise<campero_ur10_msgs::ImageDraw>(TOPIC_NAME_IMG_DRAW, 1);

        nh.param<std::string>("/" + NODE_NAME + "/cam_ref_frame", cam_ref_frame, "");
        nh.param<std::string>("/" + NODE_NAME + "/camera_frame", camera_frame, "");
        nh.param<std::string>("/" + NODE_NAME + "/robot_frame", robot_frame, "");
        
        load_img_params();

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
			ROS_INFO(" %s ", best_img->getPrintInfo().c_str());

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
            load_img_params();
            
            img_params->print_config();

            if (best_img == nullptr) {
                ROS_WARN("No hay ninguna imagen para procesar");
            } else {
                best_img->find_pts();
            
                best_img->img2World();

                ROS_INFO("--New best image--");
			    ROS_INFO(" %s ", best_img->getPrintInfo().c_str());

                publishBestImg();
            }
        } else {
            ROS_WARN("--Command: not known--");
        }
        
        
    }

    #if TEST
    void image_callback(const sensor_msgs::ImageConstPtr& msg) {
        if (!cam_info_received) {
            ROS_WARN("Camera info not received");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            cv::Mat img = cv_ptr->image.clone(); // image with markers

            cv::Mat res;
            if (img_params->contour_method == 0) {
                    proc_img_v1(img, res);
            } else if (img_params->contour_method == 1) {
                    proc_img_v2(img, res);
            }
            if (image_debug_pub.getNumSubscribers() > 0) {
                cv_bridge::CvImage out_msg;
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = res;
                image_debug_pub.publish(out_msg.toImageMsg());
            }
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
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
    
    void find_contours(const std::vector<cv::Mat> objetosImagen, cv::Mat& res) {
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

            if (indiceMax >= 0 && maxLen > img_params->min_contour_size) {
                cv::drawContours( res, contours_local, indiceMax, cv::Scalar(0,0,255), 1 );
            }
        }
        
    }

    void proc_img_v1(const cv::Mat& src, cv::Mat& res) {
        // 1.To gray scale
        cv::Mat gray;
        if (img_params->apply_sharp) { // sharped image
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

            cv::cvtColor(imgResult, gray, cv::COLOR_BGR2GRAY);
        } else {
            cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        }
        
        // 2.Binarizar
        cv::Mat img_otsu;
        cv::threshold(gray, img_otsu, img_params->binary_thresh, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        
        // 3.Delete Markers
        //delete_markers(img_otsu);
        
        // 4.Calculo componentes conexas
        cv::Mat objetos;
        int nLabels = cv::connectedComponents(img_otsu, objetos, img_params->conectivity_way, CV_32S);

        // 5.Fill objects
        std::vector<cv::Vec3b> colorTab;
        std::vector<cv::Mat> objetosImagen;
        fill_mats(nLabels, colorTab, objetosImagen, src.size(), CV_8UC1);
        
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
        res = src.clone();
        find_contours(objetosImagen, res);
    }

    void proc_img_v2(const cv::Mat& src, cv::Mat& res) {
        // 1.Binarizar
        cv::Mat gray;
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

        cv::Mat thresh;
        cv::threshold(gray, thresh, img_params->binary_thresh, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        
        // 2.Delete Markers
        //delete_markers(thresh);

        // 3.Improve Contours
        cv::Mat kernel = (cv::Mat_<int>(3,3) <<
                    1,  1, 1,
                    1, 1, 1,
                    1,  1, 1); // an approximation of second derivative, a quite strong kernel

        cv::Mat opening;
        cv::morphologyEx(thresh, opening, cv::MORPH_OPEN, kernel, cv::Point(-1,-1), 9);
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
        cv::watershed(src, objects);
        
        // 7.Fill objects
        std::vector<cv::Vec3b> colorTab;
        std::vector<cv::Mat> objetosImagen;
        fill_mats(nLabels, colorTab, objetosImagen, src.size(), CV_8UC1);

        // 8.Extraer los diferentes objetos y asignarlos a su imagen correspondiente
        for(int r = 0; r < objects.rows; r++) {
            for(int c = 0; c < objects.cols; c++) {
                int idx = objects.at<int>(r, c) - 1;
                if (idx > 0 && idx < nLabels) {
                    unsigned char& v = objetosImagen[idx - 1].at<unsigned char>(r, c);
                    v = 255;
                }
            }
        }
		
        // 9.Find contours on image objets
        res = src.clone();
        find_contours(objetosImagen, res);
    }
    #endif
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
