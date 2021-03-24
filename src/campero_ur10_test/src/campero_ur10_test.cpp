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

#include "../include/c_ur10_def/campero_ur10.h"

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_group_interface_tutorial_ur10");
  ros::NodeHandle node_handle;
  
  // Load args
  string mode = "";
  for (int i = 0; i < argc; i++) {
    if (!strcmp("-m", argv[i])) {
      mode = argv[++i];
    }
  }

  C_UR10_Mode _mode;

  if (mode == "teleop") {
    _mode = C_UR10_Mode::TELEOP;
  } else if (mode == "draw") {
    _mode = C_UR10_Mode::DRAW_IMAGE;
  } else {
    ROS_INFO("Mode not valid");
    return -1;
  }
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  CamperoUR10 c_ur10(_mode);
  c_ur10.main();


  return 0;

}
