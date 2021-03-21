#include <iostream>
#include <fstream>
#include <string>

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

//#include <rviz_visual_tools/rviz_visual_tools.h>

#include "../include/c_ur10_def/c_ur10_utils.h"
#include "../include/c_ur10_def/campero_ur10.h"

using namespace std;

void parseWS(string& file, moveit::planning_interface::MoveGroupInterface& move_group,
            std::vector<moveit_msgs::CollisionObject>& walls) {
  if (file == "") {
    cout << "No se ha especificado ningun archivo de workspace\n";
    exit(1);
  }

  ifstream infile(file.c_str());

  if (!infile.is_open()) {
    ROS_ERROR("Error opening ws file");
    ros::shutdown;
    exit(1);
  }
  //vector<OperatorTurtlebot*> ops;
  string line;
  ROS_INFO("Parsing ws file");

  while (getline(infile, line)) {
    if (line == "WALL") {
      int n;
      infile >> n;
      for (int i = 0; i < n; i++) {
        // Get wall
        string id;
        double sx,sy,sz,px,py,pz,ox,oy,oz,ow;
        infile >> id >> sx >> sy >> sz >> px >> py >> pz >> ox >> oy >> oz >> ow;
        string msg = "LOAD: " + id + " " + to_string(sx) + to_string(sy) + " " + to_string(sz)
            + " " + to_string(px) + " " + to_string(py) + " " + to_string(pz)
            + " " + to_string(ox) + " " + to_string(oy) + " " + to_string(oz) + + " " + to_string(ow) +"\n";
        cout << msg;
        
        moveit_msgs::CollisionObject w_object;
        w_object.header.frame_id = move_group.getPlanningFrame();
        w_object.id = id;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = sx;
        primitive.dimensions[primitive.BOX_Y] = sy;
        primitive.dimensions[primitive.BOX_Z] = sz;

        geometry_msgs::Pose w_pose;
        w_pose.position.x = px;
        w_pose.position.y = py;
        w_pose.position.z = pz;
        w_pose.orientation.x = ox;
        w_pose.orientation.y = oy;
        w_pose.orientation.z = oz;
        w_pose.orientation.w = ow;

        w_object.primitives.push_back(primitive);
        w_object.primitive_poses.push_back(w_pose);
        w_object.operation = w_object.ADD;

        walls.push_back(w_object);
      }
    } 
  }

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial_ur10");
  ros::NodeHandle node_handle;
  
  // Load args
  /*string fileWS = "";
  for (int i = 0; i < argc; i++) {
    if (!strcmp("-fws", argv[i])) {
      fileWS = argv[++i];
    }
  }*/

  CamperoUR10 c_ur10;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup
  // ^^^^^

  moveit::planning_interface::MoveGroupInterface move_group(C_UR10_PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(C_UR10_PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  
  moveit_visual_tools::MoveItVisualTools visual_tools(C_UR10_BASE_LINK);
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  // Workspace
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Hay que definir paredes invisibles puesto que no funciona definir un workspace como tal

  // Add Walls
  //std::vector<moveit_msgs::CollisionObject> collision_objects;
  //parseWS(fileWS, move_group, collision_objects);

  /*geometry_msgs::Pose p_pose;
  visual_tools.publishXYPlane(p_pose, rvt::RED);
  visual_tools.publishXZPlane(p_pose, rvt::GREEN);
  p_pose.position.x = 0.75;
  visual_tools.publishYZPlane(p_pose, rvt::BLUE);
  visual_tools.trigger();*/
  
  //planning_scene_interface.addCollisionObjects(collision_objects);
  
  /*geometry_msgs::Pose w_pose;
  w_pose.position.x = 0;
  w_pose.position.y = 0;
  w_pose.position.z = 0;
  w_pose.orientation.x = 0;
  w_pose.orientation.y = 0;
  w_pose.orientation.z = 0;
  w_pose.orientation.w = 1;
    
  std_msgs::ColorRGBA color;
  color.r = 0;
  color.g = 0;
  color.b = 1;
  color.r = 1;


  visual_tools.publishCollisionBlock(w_pose, "w_front");
  visual_tools.trigger();

  visual_tools.publishText(text_pose, "Add Walls", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  //visual_tools.
  ROS_INFO_NAMED("tutorial", "Walls added");

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to remove Walls");

  // Remove Walls
  ROS_INFO_NAMED("tutorial", "Remove Walls");
  /*std::vector<std::string> object_ids;
  for (int i = 0; i < collision_objects.size(); i++) {
    object_ids.push_back(collision_objects[i].id);
  }
  planning_scene_interface.removeCollisionObjects(object_ids);


  ros::shutdown();
  return 0;*/

  // Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.

  geometry_msgs::PoseStamped home_pose = move_group.getCurrentPose(); // get wrist3_link pose
  cout << "home_pose: " << home_pose << endl;

  tf2::Quaternion orientation;
  orientation.setRPY(-1*M_PI, 0, 0);
  orientation = orientation.normalize();

  move_group.setPlanningTime(10.0);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window set init pose");

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  //move_group.setNamedTarget("ready_draw");

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute plan");

  if (success) {
    cout << "Executing...\n";
    move_group.execute(my_plan);
  }

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute plan");

  geometry_msgs::Pose target_pose1 = move_group.getCurrentPose().pose;
  target_pose1.orientation = tf2::toMsg(orientation);
  target_pose1.position.x += 0.1;
  target_pose1.position.z -= 0.2;
  move_group.setPoseTarget(target_pose1);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute plan");

  if (success) {
    cout << "Executing...\n";
    move_group.execute(my_plan);
  }

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute plan");

  move_group.setStartStateToCurrentState();
  
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = C_UR10_W3_LINK;
  ocm.header.frame_id = C_UR10_BASE_LINK;
  ocm.weight = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.orientation = tf2::toMsg(orientation);

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);
  
  robot_state::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::Pose start_pose2 = move_group.getCurrentPose().pose;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  cout << "Constrainsts: " << move_group.getPathConstraints() << endl;

  /*visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to try move to home");
  move_group.setPoseTarget(home_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing home goal (constraints) %s", success ? "" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(home_pose.pose, "goal");
  visual_tools.publishText(text_pose, "Constrained Invalid Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to home");

  if (success) {
    ROS_INFO("Moving...");
    move_group.execute(my_plan);
  }*/

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan valid goal");

  /*geometry_msgs::Pose target_pose_const = move_group.getCurrentPose().pose;
  target_pose_const.position.z -= 0.5;
  //target_pose_const.position.y += 0.;
  target_pose_const.position.x += 0.3;*/

  //double zz = move_group.getCurrentPose().pose.position.z;
  //double xx = move_group.getCurrentPose().pose.position.x;
  //double yy = move_group.getCurrentPose().pose.position.y;

  
  double r =  0.2;

  vector<geometry_msgs::Pose> waypoints;

  /*geometry_msgs::Pose pos_final;
  pos_final.orientation = tf2::toMsg(orientation);
  pos_final.position.x = -0.14;
  pos_final.position.y = 0.0;
  pos_final.position.z = 0.7;*/

  for (double th = 0; th < 2*M_PI; th += 0.1) {
    geometry_msgs::Pose target_pose_const = move_group.getCurrentPose().pose;
    target_pose_const.position.x += r * cos(th);
    target_pose_const.position.y += r * sin(th);
    waypoints.push_back(target_pose_const);
  }

  /*for (int i = 0; i < 5; i++) {
    geometry_msgs::Pose target_pose_const = move_group.getCurrentPose().pose;
    //zz -= 0.1;
    //target_pose_const.position.z = zz;
    target_pose_const.position.x = xx;
    target_pose_const.position.y = yy;
    waypoints.push_back(target_pose_const);
  }*/


  move_group.setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  //move_group.setPoseTarget(target_pose_const);
  /*

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing valid goal (constraints) %s", success ? "" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose_const, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();*/

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to valid goal");

  if (success) {
    ROS_INFO("Moving...");
    move_group.execute(my_plan);
  }

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move start position and end the demo");

  start_state = *move_group.getCurrentState();
  start_pose2 = move_group.getCurrentPose().pose;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  move_group.setPoseTarget(target_pose1);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Go to start position %s", success ? "" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Home Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  if (success) {
    ROS_INFO("Moving...");
    move_group.execute(my_plan);
  }


  // END_TUTORIAL

  move_group.clearPathConstraints();

  cout << "Constrainsts: " << move_group.getPathConstraints() << endl;

  move_group.setStartStateToCurrentState();

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();


  ROS_INFO("end demo");
  ros::shutdown();

  return 0;
}
