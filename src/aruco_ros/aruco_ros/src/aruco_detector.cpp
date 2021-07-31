/*****************************
 * Autor: Ignacio Herrera Seara, nachohseara@gmail.com
 * Based on: https://github.com/pal-robotics/aruco_ros/blob/melodic-devel/aruco_ros/src/simple_single.cpp
********************************/

#include <iostream>
#include <string>
#include <vector>
#include <utility>

#include <ros/ros.h>
#include "std_msgs/String.h"

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <aruco_ros/aruco_ros_utils.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>

#include <campero_ur10_msgs/ArucoMarker.h>
#include <campero_ur10_msgs/ArucoMarkerArray.h>
#include <campero_ur10_msgs/ArucoMarkersImg.h>


using namespace aruco;

class ArucoDetector
{
private:
  ros::Subscriber process_cmd_sub;

  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  MarkerDetector mDetector;
  
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  
  image_transport::Publisher image_detector_pub;
  ros::Publisher markers_pose_pub;
  
  // Cuano encuentra 4 marcas publica las marcas y la imagen original
  // image_transport::Publisher image_res_pub;
  ros::Publisher markers_pub;
  
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;

  double marker_size;
  bool rotate_marker_axis_;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  tf::TransformListener _tfListener;

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;
  
  const int frame_send = 5;
  int frame_count = 0;
  
  void load_detector_config() {
    std::string aux;
    nh.param<std::string>("corner_refinement", aux, "LINES");
    if ( aux == "SUBPIX" ) {
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::SUBPIX);
    } else if ( aux == "HARRIS" ) {
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::HARRIS);
    } else if ( aux == "NONE" ) {
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::NONE); 
    } else {
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
    }

    
    nh.param<std::string>("threshold_method", aux, "ADPT_THRES");
    if ( aux == "FIXED_THRES" ) {
      mDetector.mySetThresholdMethod(0);
    } else if ( aux == "CANNY" ) {
      mDetector.mySetThresholdMethod(2);
    } else {
      mDetector.mySetThresholdMethod(1);
    }
    
    double th1, th2;
    nh.param<double>("threshold_1", th1, 7);
    nh.param<double>("threshold_2", th2, 7);
    mDetector.setThresholdParams(th1, th2);

  }

  void print_detector_info() {
    std::string aux;
    switch (mDetector.getCornerRefinementMethod()) {
      case aruco::MarkerDetector::SUBPIX:
        aux = "SUBPIX";
        break;
      case aruco::MarkerDetector::HARRIS:
        aux = "HARRIS";
        break;
      case aruco::MarkerDetector::NONE:
        aux = "NONE";
        break;
      default:
        aux = "LINES";
    }
    ROS_INFO("Corner refinement method: %s ", aux.c_str());
    
    switch (mDetector.myGetThresholdMethod()) {
      case 0:
        aux = "FIXED_THRES";
        break;
      case 2:
        aux = "CANNY";
        break;
      default:
        aux = "ADPT_THRES";
    }
    ROS_INFO("Threshold method: %s ", aux.c_str());

    double th1, th2;
    mDetector.getThresholdParams(th1, th2);
    ROS_INFO_STREAM("Threshold method: " << " th1: " << th1 << " th2: " << th2);

    float mins, maxs;
    mDetector.getMinMaxSize(mins, maxs);
    ROS_INFO_STREAM("Marker size min: " << mins << "  max: " << maxs);

    ROS_INFO_STREAM("Desired speed: " << mDetector.getDesiredSpeed());
  }
  
  int idExists(const int id, const campero_ur10_msgs::ArucoMarkerArray& markers) {
    int count = 0;
    for (int i = 0; i < markers.markers.size(); i++) {
      if (markers.markers[i].id == id) {
        count++;
      }
    }

    return count;
  }

public:
  ArucoDetector()
    : cam_info_received(false),
      nh("~"),
      it(nh)
  {

    load_detector_config();
    print_detector_info();    

    process_cmd_sub = nh.subscribe("/aruco_detector/cmd", 1, &ArucoDetector::process_cmd_callback, this);
    image_sub = it.subscribe("/image", 1, &ArucoDetector::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoDetector::cam_info_callback, this);

    image_detector_pub = it.advertise("detector", 1);
    markers_pub = nh.advertise<campero_ur10_msgs::ArucoMarkersImg>("markers_img", 1);
    markers_pose_pub = nh.advertise<campero_ur10_msgs::ArucoMarkerArray>("markers_pose", 1);

    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<std::string>("reference_frame", reference_frame, "");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<std::string>("marker_frame", marker_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);
    nh.param<bool>("rotate_marker_axis", rotate_marker_axis_, true);

    ROS_ASSERT(camera_frame != "" && marker_frame != "");

    if ( reference_frame.empty() )
      reference_frame = camera_frame;

    ROS_INFO("Aruco node started with marker size of %f m", marker_size);
    ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
             reference_frame.c_str(), marker_frame.c_str());

    dyn_rec_server.setCallback(boost::bind(&ArucoDetector::reconf_callback, this, _1, _2));
  }

  bool getTransform(const std::string& refFrame,
                    const std::string& childFrame,
                    tf::StampedTransform& transform)
  {
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


  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    if (!cam_info_received) {
      ROS_WARN("Camera info not received");
      return;
    }
    if ((image_detector_pub.getNumSubscribers() == 0) &&
        (markers_pub.getNumSubscribers() == 0) &&
        (markers_pose_pub.getNumSubscribers() == 0))
    {
      // ROS_INFO("No subscribers, not looking for aruco markers");
      return;
    }

    static tf::TransformBroadcaster br;
    ros::Time curr_stamp = msg->header.stamp;
    
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      cv::Mat image_detector = cv_ptr->image.clone(); // image with markers

      // Detect Markers
      std::vector<Marker> markers;
      mDetector.detect(image_detector, markers, camParam, marker_size, false);

      // Init ArucoMarkersImg msg
      campero_ur10_msgs::ArucoMarkersImg marker_array;

      // Point Set of Marker Points
      std::vector< std::vector<cv::Point2f> > marker_pts;

      // for each marker, draw info and its boundaries in the image
      for (size_t i = 0; i < markers.size(); ++i) {
        if (!markers[i].isValid()) continue;
        
        // 1.Get marker transform
        tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i], rotate_marker_axis_);
        // tf::StampedTransform cameraToReference;
        // cameraToReference.setIdentity();

        /*if ( reference_frame != camera_frame )
        {
          getTransform(reference_frame,
                        camera_frame,
                        cameraToReference);
        }*/

        /*transform = 
          //static_cast<tf::Transform>(cameraToReference) 
            static_cast<tf::Transform>(rightToLeft) 
          * transform;*/
        
        std::string marker_frame_final = marker_frame + "_" + std::to_string(markers[i].id);
        int c = idExists(markers[i].id, marker_array.markers);
        if (c > 0) {
          marker_frame_final = marker_frame_final + "_" + std::to_string(c);
        }
        tf::StampedTransform stampedTransform(transform, curr_stamp,
                                              reference_frame, marker_frame_final);
        br.sendTransform(stampedTransform);
        
        // 2.Build ArucoMarker msg and Add to ArucoMarkerArray msg
        campero_ur10_msgs::ArucoMarker marker_msg;
        tf::poseTFToMsg(transform, marker_msg.pose);

        /*std::cout << markers[i].id << std::endl;
        std::cout << marker_msg.pose << std::endl;*/
        
        for (int j = 0; j < markers[i].size(); j++) {
          geometry_msgs::Point32 pt;
          pt.x = markers[i][j].x;
          pt.y = markers[i][j].y;
          marker_msg.img_points.push_back(pt);
        }
        marker_msg.id = markers[i].id;

		frame_count++;
		if (frame_count == frame_send) {
			marker_array.markers.markers.push_back(marker_msg);
			frame_count = 0;
		}

        // 3.Draw Marker on image
        markers[i].draw(image_detector,cv::Scalar(0,0,255),2);

        // 4.Draw a 3d cube if there is 3d info
        if (camParam.isValid() && marker_size > 0) {
          CvDrawingUtils::draw3dAxis(image_detector, markers[i], camParam);
        }

        // 5.Add Marker Image Points to Point Set
        marker_pts.push_back(markers[i]);
      }

      markers_pose_pub.publish(marker_array.markers);
      
      // Num Markers is correct
      if (markers.size() == 4 && markers_pub.getNumSubscribers() > 0) {
        /*cv::Mat image_res = cv_ptr->image;
        correctImage(marker_pts, image_res);*/

        // Publish ArucoMarkerArray with Original Image
        marker_array.img = *(cv_ptr->toImageMsg());
        markers_pub.publish(marker_array);

        // Publish Original Image
        /*cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = cv_ptr->image;
        image_res_pub.publish(out_msg.toImageMsg());*/
      }

      // Publish Image
      if (image_detector_pub.getNumSubscribers() > 0) {
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = image_detector;
        image_detector_pub.publish(out_msg.toImageMsg());
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CamaraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(
        tf::Vector3(
            -msg.P[3]/msg.P[0],
            -msg.P[7]/msg.P[5],
            0.0));

    cam_info_received = true;
    cam_info_sub.shutdown();

    ROS_INFO("Camera Info received");
  }

  void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
  {
    mDetector.setThresholdParams(config.param1,config.param2);
    if (config.normalizeImage)
    {
      ROS_WARN("normalizeImageIllumination is unimplemented!");
    }
  }

  void process_cmd_callback(const std_msgs::String::ConstPtr& msg) {
    std::string cmd = msg->data;

    if (cmd == "update_config") {
      load_detector_config();
      print_detector_info();
    }
  }
};


int main(int argc,char **argv)
{
  ros::init(argc, argv, "aruco_detector");

  ArucoDetector node;

  ros::spin();
}
