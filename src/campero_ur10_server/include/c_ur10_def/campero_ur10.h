#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <campero_ur10_msgs/MoveOp.h>
#include <campero_ur10_msgs/ImageDraw.h>
#include <campero_ur10_msgs/ImgTrace.h>

#include "c_ur10_utils.h"

enum class C_UR10_Mode {DRAW_IMAGE, TELEOP};

class CamperoUR10 {
private:
    moveit::planning_interface::MoveGroupInterface move_group = 
        moveit::planning_interface::MoveGroupInterface(C_UR10_PLANNING_GROUP);
    
    const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(C_UR10_PLANNING_GROUP);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit_visual_tools::MoveItVisualTools visual_tools = 
        moveit_visual_tools::MoveItVisualTools(C_UR10_BASE_LINK);

    // Teleop parameters
    bool add_ori;

    // Draw Board parameters
    double z_pen_down = -1.0, z_pen_up = -1.0, board_min_x = -1.0, board_min_y = -1.0, board_size = -1.0;

    // eef_step: max distancia entre dos puntos consecutivos en la trayectoria cartesiana resultante
    // jump_threshold poner 0 para evitar saltos en las soluciones con IK
    const double jump_threshold = 0.0, eef_step = 0.01;

    geometry_msgs::Quaternion ori_constraint;

    ros::NodeHandle nh;
    ros::Subscriber sub;

    C_UR10_Mode mode;

    void prompt(std::string msg);

    void showPlan();

    void init(C_UR10_Mode _mode);

    void loadDrawConfig(std::string& file);

public:
    CamperoUR10(C_UR10_Mode _mode, std::string& config_file, bool _add_ori);

    bool plan();
    
    double planCarthesian(std::vector<geometry_msgs::Pose>& waypoints);

    bool execute();

    bool plan_execute();

    bool plan_exec_Carthesian(std::vector<geometry_msgs::Pose>& waypoints);

    bool moveJoint(const int joint, const double value);

    bool moveJoint(const std::string& joint, const double value);

    bool goReadyDraw();

    bool addOriConstraint();

    void main();

    void goHome();

    void processTrace(const campero_ur10_msgs::ImgTrace trace, const double div, std::vector<geometry_msgs::Pose>& waypoints);

    void callbackDraw(const campero_ur10_msgs::ImageDraw image);

    void callbackMoveOp(const campero_ur10_msgs::MoveOp operation);

    moveit::planning_interface::MoveGroupInterface& getMoveGroup();

    //void drawCircle(const double x, const double y, const double r);

    //void drawSin(const double x, const double y, const double maxX, const double maxY);
};

