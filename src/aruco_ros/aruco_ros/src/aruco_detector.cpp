/*****************************
 * Autor: Ignacio Herrera Seara, nachohseara@gmail.com
 * Based on: https://github.com/pal-robotics/aruco_ros/blob/melodic-devel/aruco_ros/src/simple_single.cpp
********************************/

#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <aruco_ros/aruco_ros_utils.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>

using namespace aruco;

class ArucoSimple
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  MarkerDetector mDetector;
  
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  
  image_transport::Publisher image_pub;
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

public:
  ArucoSimple()
    : cam_info_received(false),
      nh("~"),
      it(nh)
  {

    std::string refinementMethod;
    nh.param<std::string>("corner_refinement", refinementMethod, "LINES");
    if ( refinementMethod == "SUBPIX" )
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::SUBPIX);
    else if ( refinementMethod == "HARRIS" )
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::HARRIS);
    else if ( refinementMethod == "NONE" )
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::NONE); 
    else      
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES); 

    //Print parameters of aruco marker detector:
    ROS_INFO_STREAM("Corner refinement method: " << mDetector.getCornerRefinementMethod());
    ROS_INFO_STREAM("Threshold method: " << mDetector.getThresholdMethod());
    double th1, th2;
    mDetector.getThresholdParams(th1, th2);
    ROS_INFO_STREAM("Threshold method: " << " th1: " << th1 << " th2: " << th2);
    float mins, maxs;
    mDetector.getMinMaxSize(mins, maxs);
    ROS_INFO_STREAM("Marker size min: " << mins << "  max: " << maxs);
    ROS_INFO_STREAM("Desired speed: " << mDetector.getDesiredSpeed());
    


    image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    markers_pub = nh.advertise<aruco_msgs::MarkerArray>("markers", 100);

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

    dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));
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
    if ((image_pub.getNumSubscribers() == 0) &&
        (markers_pub.getNumSubscribers() == 0))
    {
      ROS_DEBUG("No subscribers, not looking for aruco markers");
      return;
    }

    static tf::TransformBroadcaster br;
    if(cam_info_received)
    {
      ros::Time curr_stamp = msg->header.stamp;
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        // Detect Markers
        std::vector<Marker> markers;
        mDetector.detect(inImage, markers, camParam, marker_size, false);

        // Init MarkerArray msg
        aruco_msgs::MarkerArray marker_array;
        marker_array.header.frame_id = reference_frame;
        marker_array.header.stamp = curr_stamp;

        // Point Set of Marker Points
        std::vector<cv::Point> marker_pts;

        // for each marker, draw info and its boundaries in the image
        for (size_t i = 0; i < markers.size(); ++i) {
          // 1.Build Marker msg
          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i], rotate_marker_axis_);
          tf::StampedTransform cameraToReference;
          cameraToReference.setIdentity();

          if ( reference_frame != camera_frame )
          {
            getTransform(reference_frame,
                          camera_frame,
                          cameraToReference);
          }

          transform = 
            static_cast<tf::Transform>(cameraToReference) 
            * static_cast<tf::Transform>(rightToLeft) 
            * transform;

          tf::StampedTransform stampedTransform(transform, curr_stamp,
                                                reference_frame, marker_frame);
          br.sendTransform(stampedTransform);
          
          aruco_msgs::Marker marker_msg;
          tf::poseTFToMsg(transform, marker_msg.pose.pose);
          
          marker_msg.header = marker_array.header;
          marker_msg.id = markers[i].id;

          // TODO: ¿que hay que poner en confidence y en pose.covariance?
          marker_msg.confidence = 0.98f;

          // Add to MarkerArray
          marker_array.markers.push_back(marker_msg);

          // 2.Draw Marker on image
          markers[i].draw(inImage,cv::Scalar(0,0,255),2);

          // 3.Draw a 3d cube if there is 3d info
          if (camParam.isValid() && marker_size > 0) {
            CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }

          // 4.Add Marker Points to Point Set
          for (int j = 0; j < markers[i].size(); j++) {
            marker_pts.push_back(cv::Point(markers[i][j].x,markers[i][j].y));
          }
        }

        // Publish MarkerArray
        markers_pub.publish(marker_array);
        
        // Get Convex Hull
        if (markers.size() == 4) {
          
          std::vector< std::vector<cv::Point> > hull(1);
          cv::convexHull(cv::Mat(marker_pts), hull[0], false);

          // Draw Convex Shape
          cv::drawContours( inImage, hull, 0, cv::Scalar(0,255,0));

          // Draw convex points
          for (int i = 0; i < hull[0].size(); i++) {
            cv::circle(inImage, hull[0][i], 10, cv::Scalar(0,0,0), -1);
          }
        }

        // Publish Image
        if(image_pub.getNumSubscribers() > 0)
        {
          //show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if (markers.size() == 4) {
          cv::imshow("Res", inImage);
          cv::waitKey(0);
          cv::destroyWindow("Res");
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
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
  }


  void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
  {
    mDetector.setThresholdParams(config.param1,config.param2);
    if (config.normalizeImage)
    {
      ROS_WARN("normalizeImageIllumination is unimplemented!");
    }
  }
};


int main(int argc,char **argv)
{
  ros::init(argc, argv, "aruco_simple");

  ArucoSimple node;

  ros::spin();
}
