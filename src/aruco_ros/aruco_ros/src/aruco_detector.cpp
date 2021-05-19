/*****************************
 * Autor: Ignacio Herrera Seara, nachohseara@gmail.com
 * Based on: https://github.com/pal-robotics/aruco_ros/blob/melodic-devel/aruco_ros/src/simple_single.cpp
********************************/

#include <iostream>
#include <string>
#include <vector>
#include <utility>

#include <ros/ros.h>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <aruco_ros/aruco_ros_utils.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

/*#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>*/

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>

#include <campero_ur10_msgs/ArucoMarker.h>
#include <campero_ur10_msgs/ArucoMarkerArray.h>

using namespace aruco;

class ArucoDetector
{
private:
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  MarkerDetector mDetector;
  
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  
  image_transport::Publisher image_detector_pub;
  image_transport::Publisher image_res_pub;
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
  ArucoDetector()
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
    


    image_sub = it.subscribe("/image", 1, &ArucoDetector::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoDetector::cam_info_callback, this);

    image_detector_pub = it.advertise("detector", 1);
    image_res_pub = it.advertise("result", 1);
    markers_pub = nh.advertise<campero_ur10_msgs::ArucoMarkerArray>("markers", 100);

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


  /*void filterRectPoints(std::vector<cv::Point2f>& pts, std::vector<cv::Point2f>& res) {
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
  }*/
  
  /*void correctImage(std::vector<cv::Point2f>& marker_pts, cv::Mat& image) {
    // WARNING: si se hace con hull los puntos deben ser Point (tipo entero)
    //std::vector< std::vector<cv::Point> > hull(1);
    //cv::convexHull(cv::Mat(marker_pts), hull[0], false);
    
    // Get Corner points
    // filterRectPoints(hull[0]);
    std::vector<cv::Point2f> src_pts;
    filterRectPoints(marker_pts, src_pts);

    cv::Point2f tl = src_pts[0];//hull[0][0];
    cv::Point2f tr = src_pts[1];//hull[0][1];
    cv::Point2f br = src_pts[2];//hull[0][2];
    cv::Point2f bl = src_pts[3];//hull[0][3];

    // Draw Convex Shape
    // cv::drawContours( inImage, hull, 0, cv::Scalar(0,255,0));
    /*cv::line(image, tl, tr, cv::Scalar(255,0,0), 2);
    cv::line(image, tl, bl, cv::Scalar(255,0,0), 2);
    cv::line(image, tr, br, cv::Scalar(255,0,0), 2);
    cv::line(image, bl, br, cv::Scalar(255,0,0), 2);
  
    // Draw convex points
    /*for (int i = 0; i < hull[0].size(); i++) {
      std::cout << hull[0][i] << std::endl;
      cv::circle(image, hull[0][i], 10, cv::Scalar(0,0,0), -1);
    }

    // Get Warped Points
    double width_top = cv::norm(tr - tl);
    double width_bottom = cv::norm(br - bl);
    double max_W = width_top > width_bottom ? width_top : width_bottom;
    // std::cout << width_top << " " << width_bottom << std::endl;

    double height_left = cv::norm(bl - tl);
    double height_right = cv::norm(br - tr);
    double max_H = height_left > height_right ? height_left : height_right;
    // std::cout << height_left << " " << height_right << std::endl;

    std::vector<cv::Point2f> dst_pts;
    dst_pts.push_back(cv::Point2f(0, 0));
    dst_pts.push_back(cv::Point2f(max_W-1, 0));
    dst_pts.push_back(cv::Point2f(max_W-1, max_H-1));
    dst_pts.push_back(cv::Point2f(0, max_H-1));
    
    // Transform image
    cv::Mat H = cv::getPerspectiveTransform(src_pts, dst_pts);
    cv::warpPerspective(image, image, H, cv::Size(max_W, max_H));
  }*/

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
  
  void correctImage(std::vector< std::vector<cv::Point2f> >& marker_pts, cv::Mat& image) {
    
    std::vector< std::vector<cv::Point> > pts(marker_pts.size());
    std::vector<cv::Point2f> pts_f;
    for (int i = 0; i < marker_pts.size(); i++) {
      for (int j = 0; j < marker_pts[i].size(); j++) {
        pts[i].push_back(cv::Point(marker_pts[i][j].x, marker_pts[i][j].y));
        pts_f.push_back(marker_pts[i][j]);
      }
    }
    // Get Corner points
    std::vector<cv::Point2f> src_pts;
    filterRectPoints(pts_f, src_pts);

    cv::Point2f tl = src_pts[0];//hull[0][0];
    cv::Point2f tr = src_pts[1];//hull[0][1];
    cv::Point2f br = src_pts[2];//hull[0][2];
    cv::Point2f bl = src_pts[3];//hull[0][3];

    // Draw Convex Shape
    //cv::drawContours( image, pts, -1, cv::Scalar(255,255,255), 1, cv::FILLED);
    cv::fillPoly(image, pts, cv::Scalar(255,255,255));

    // Get Warped Points
    double width_top = cv::norm(tr - tl);
    double width_bottom = cv::norm(br - bl);
    double max_W = width_top > width_bottom ? width_top : width_bottom;
    // std::cout << width_top << " " << width_bottom << std::endl;

    double height_left = cv::norm(bl - tl);
    double height_right = cv::norm(br - tr);
    double max_H = height_left > height_right ? height_left : height_right;
    // std::cout << height_left << " " << height_right << std::endl;

    std::vector<cv::Point2f> dst_pts;
    dst_pts.push_back(cv::Point2f(0, 0));
    dst_pts.push_back(cv::Point2f(max_W-1, 0));
    dst_pts.push_back(cv::Point2f(max_W-1, max_H-1));
    dst_pts.push_back(cv::Point2f(0, max_H-1));
    
    // Transform image
    cv::Mat H = cv::getPerspectiveTransform(src_pts, dst_pts);
    cv::warpPerspective(image, image, H, cv::Size(max_W, max_H));
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
    if ((image_detector_pub.getNumSubscribers() == 0) &&
        (markers_pub.getNumSubscribers() == 0))
    {
      ROS_INFO("No subscribers, not looking for aruco markers");
      return;
    }

    static tf::TransformBroadcaster br;
    if(cam_info_received)
    {
      ros::Time curr_stamp = msg->header.stamp;
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        cv::Mat image_detector = cv_ptr->image.clone(); // image with markers

        // Detect Markers
        std::vector<Marker> markers;
        mDetector.detect(image_detector, markers, camParam, marker_size, false);

        // Init MarkerArray msg
        campero_ur10_msgs::ArucoMarkerArray marker_array;

        // Point Set of Marker Points
        std::vector< std::vector<cv::Point2f> > marker_pts;

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

          std::string marker_frame_final = marker_frame + "_" + std::to_string(markers[i].id);
          tf::StampedTransform stampedTransform(transform, curr_stamp,
                                                reference_frame, marker_frame_final);
          br.sendTransform(stampedTransform);
          
          campero_ur10_msgs::ArucoMarker marker_msg;
          tf::poseTFToMsg(transform, marker_msg.pose);
          //tf::poseTFToMsg(transform, marker_msg.pose.pose);
          
          // marker_msg.header = marker_array.header;
          /*marker_msg.header.frame_id = marker_frame_final;
          marker_msg.header.stamp = curr_stamp;*/
          for (int j = 0; j < markers[i].size(); j++) {
            geometry_msgs::Point32 pt;
            pt.x = markers[i][j].x;
            pt.y = markers[i][j].y;
            marker_msg.img_points.push_back(pt);
          }
          marker_msg.id = markers[i].id;

          // Add to MarkerArray
          marker_array.markers.push_back(marker_msg);

          // 2.Draw Marker on image
          markers[i].draw(image_detector,cv::Scalar(0,0,255),2);

          // 3.Draw a 3d cube if there is 3d info
          if (camParam.isValid() && marker_size > 0) {
            CvDrawingUtils::draw3dAxis(image_detector, markers[i], camParam);
          }

          // 4.Add Marker Image Points to Point Set
          marker_pts.push_back(markers[i]);
        }

        // Publish MarkerArray
        markers_pub.publish(marker_array);
        
        // Get Convex Hull
        if (markers.size() == 4) {
          cv::Mat image_res = cv_ptr->image;
          correctImage(marker_pts, image_res);

          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = image_res;
          image_res_pub.publish(out_msg.toImageMsg());
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

  ArucoDetector node;

  ros::spin();
}
