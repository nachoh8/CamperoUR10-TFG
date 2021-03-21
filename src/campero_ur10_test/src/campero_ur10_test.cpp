#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene/planning_scene.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "../include/c_ur10_def/c_ur10_utils.h"
#include "../include/c_ur10_def/campero_ur10.h"

using namespace std;

void callback(geometry_msgs::PoseArray points) {
  ROS_INFO("Points received -> Parse");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_group_interface_tutorial_ur10");
  ros::NodeHandle node_handle;
  //ros::Rate loop_rate(10);

  //ros::Subscriber sub = node_handle.subscribe(TOPIC_NAME, QUEUE_SIZE, &CamperoUR10::callbackDraw, &c_ur10);
  //ros::Subscriber sub = node_handle.subscribe(TOPIC_NAME, QUEUE_SIZE, callback);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  CamperoUR10 c_ur10;
  c_ur10.main();


  return 0;

}
/*int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial_ur10");
  ros::NodeHandle node_handle;

  //if (!c_ur10.goReadyDraw()) return;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  //CamperoUR10 c_ur10;
  
  c_ur10.main();

  /*ROS_INFO("end demo");
  ros::shutdown();

  ros::Subscriber sub = node_handle.subscribe(TOPIC_NAME, QUEUE_SIZE, callbackDraw);

  ros::spin();

  return 0;
}*/
