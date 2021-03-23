#include "../include/c_ur10_def/campero_ur10.h"

#include <iostream>
#include <cmath>

#include <ros/ros.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_interface.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define C_UR10_INFO_NAMED "CamperoUR10"
#define NODE_NAME "campero_ur10"
#define TOPIC_NAME_MOVE "campero_ur10_move"
#define TOPIC_NAME_BOARD "image_points"
#define QUEUE_SIZE 10

CamperoUR10::CamperoUR10() {
    // Create Orientation Constraint Quaternion
    tf2::Quaternion ori;
    ori.setRPY(-1*M_PI, 0, 0);

    ori_constraint = tf2::toMsg(ori.normalize());

    move_group.setPlanningTime(10);

    move_group.setMaxVelocityScalingFactor(0.1);

    move_group.setStartStateToCurrentState();


    //sub_image = nh.subscribe(TOPIC_NAME, QUEUE_SIZE, &CamperoUR10::callbackDraw, this);

    //kinematic_state = move_group.getCurrentState();

    // Load Visual Tools
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    visual_tools.trigger();

    // Show Info
    ROS_INFO("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
}

void CamperoUR10::prompt(std::string msg) {
    visual_tools.prompt(msg);
}

void CamperoUR10::showPlan() {
    visual_tools.deleteAllMarkers();
    
    for (std::size_t i = 0; i < move_group.getPoseTargets().size(); ++i) {
        visual_tools.publishAxisLabeled(move_group.getPoseTargets()[i].pose, "pt" + std::to_string(i), rviz_visual_tools::SMALL);
    }

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    
    visual_tools.trigger();
}

bool CamperoUR10::plan() {
    ROS_INFO("Planning...");
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        ROS_INFO("Plan Ok");
        //showPlan();
        return true;
    }

    ROS_INFO("Plan FAILED");

    return false;
}

double CamperoUR10::planCarthesian(std::vector<geometry_msgs::Pose>& waypoints) {
    ROS_INFO("Planning...");

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    ROS_INFO("Cartesian plan path (%.2f%% acheived)", fraction * 100.0);

    // Show plan
    visual_tools.deleteAllMarkers();
    visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
    /*for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
    */visual_tools.trigger();


    // Apply Plan
    moveit::core::RobotStatePtr kinematic_state = move_group.getCurrentState();

    robot_trajectory::RobotTrajectory rt(kinematic_state->getRobotModel(), C_UR10_PLANNING_GROUP);
    rt.setRobotTrajectoryMsg(*kinematic_state, trajectory);

    
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);

    my_plan.trajectory_ = trajectory;

    return fraction;
}

bool CamperoUR10::execute() {
    ROS_INFO("Moving...");
    bool success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        //kinematic_state = move_group.getCurrentState();
        move_group.setStartStateToCurrentState();

        return true;
    }
    return false;
}

bool CamperoUR10::plan_execute() {
    if (plan()) {
        //prompt("Press 'next' to execute plan");
        return execute();
    }

    return false;
}

bool CamperoUR10::plan_exec_Carthesian(std::vector<geometry_msgs::Pose>& waypoints) {
    planCarthesian(waypoints);
    
    //prompt("Press 'next' to execute plan");

    return execute();
}

bool CamperoUR10::moveJoint(const int joint, const double value) {
    std::vector<double> v = move_group.getCurrentJointValues();
    v[joint] = value;

    if (move_group.setJointValueTarget(v)) {
        bool success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group.setStartStateToCurrentState();
            return true;
        }
    }
    
    return false;
}

bool CamperoUR10::moveJoint(const std::string& joint, const double value) {
    return moveJoint(jointName2Idx(joint), value);
}

bool CamperoUR10::goReadyDraw() {
    ROS_INFO("1.Set correct orientation");

    geometry_msgs::Pose target_ori = move_group.getCurrentPose().pose;
    target_ori.orientation = ori_constraint;
    move_group.setPoseTarget(target_ori);

    if (plan_execute()) {
        ROS_INFO("2.Add orientation constraint");
        
        addOriConstraint();
        std::cout << move_group.getPathConstraints() << std::endl;

        ROS_INFO("3.Go to ready draw position");

        move_group.setNamedTarget(C_UR10_POSE_READY_DRAW);

        /*geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
        target_pose.position.x += 0.25;
        target_pose.position.z -= 0.55;
        target_pose.position.y += 0.15;
        move_group.setPoseTarget(target_pose);*/

        return plan_execute();
    }

    return false;
}

void CamperoUR10::addOriConstraint() {
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = C_UR10_W3_LINK;
    ocm.header.frame_id = C_UR10_BASE_LINK;
    ocm.weight = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.orientation = ori_constraint;

    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(constraints);
    
    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose2 = move_group.getCurrentPose().pose;
    start_state.setFromIK(joint_model_group, start_pose2);
    move_group.setStartState(start_state);
}

/*void CamperoUR10::drawCircle(const double x, const double y, const double r) {
    std::cout << "Drawing circle -> center (" << x << ", " << y  << ")"
                        << " r: " << r << std::endl;
    
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;

    for (double th = 0; th < 2*M_PI; th += 0.1) {
        target_pose.position.x = x + r * cos(th);
        target_pose.position.y = y + r * sin(th);
        waypoints.push_back(target_pose);
    }

    plan_exec_Carthesian(waypoints);
}

void CamperoUR10::drawSin(const double x, const double y, const double maxX, const double maxY) {
    const double plane_len = 4*M_PI;
    const double step = 0.1;
    const double real_len = maxX - x < 0 ? (maxX - x)*-1 : maxX - x;
    const double real_step = step*(real_len/plane_len);

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    
    double xx = x;

    for (double th = -2*M_PI; th < 2*M_PI; th += step) {
        xx += real_step;
        target_pose.position.y = xx;
        target_pose.position.x = y + sin(th)*maxY;
        waypoints.push_back(target_pose);
    }

    plan_exec_Carthesian(waypoints);
}*/

moveit::planning_interface::MoveGroupInterface& CamperoUR10::getMoveGroup() {
    return move_group;
}

void CamperoUR10::callbackDraw(geometry_msgs::PoseArray points) {
    ROS_INFO("Points received -> Parse");
    
    double div = 0.5/50;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target = move_group.getCurrentPose().pose;
    std::cout << "N_points: " << points.poses.size() << std::endl;
    for (int i = 0; i < points.poses.size(); i++) {
        target.position.y = -0.25 + points.poses[i].position.x * div;
        target.position.x = -0.45 + points.poses[i].position.y * div;
        waypoints.push_back(target);
        /*std::cout << "Pt: (" << points.poses[i].position.x << ", " << points.poses[i].position.y << ") -> ("
                    << target.position.y << ", " << target.position.x << ")\n";*/
    }

    plan_exec_Carthesian(waypoints);
    ROS_INFO("Callback end");
}

void CamperoUR10::callbackMoveOp(const campero_ur10_msgs::MoveOp operation) {
    ROS_INFO("Operation received");
    const int type = operation.type, id = operation.id;
    const double v = operation.value;
    
    if (id < 0 || id > 5) {
        ROS_INFO("Id %d not valid", id);
    } else {
        if (operation.type == campero_ur10_msgs::MoveOp::MOVE_JOINT) {
            ROS_INFO("Moving joint: %d", id);
            double f_v = v + move_group.getCurrentJointValues()[id];
            ROS_INFO("New value target: %.2f rads", f_v);
            moveJoint(id, f_v);
        } else if (operation.type == campero_ur10_msgs::MoveOp::MOVE_CARTHESIAN) {
            ROS_INFO("Moving carthesiand: %d", id);
            geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
            switch (id)
            {
            case campero_ur10_msgs::MoveOp::X_AXIS:
                target_pose.position.x += v;
                break;
            
            case campero_ur10_msgs::MoveOp::Y_AXIS:
                target_pose.position.y += v;
                break;
            case campero_ur10_msgs::MoveOp::Z_AXIS:
                target_pose.position.z += v;
                break;
            default:
                break;
            }
            move_group.setPoseTarget(target_pose);
            plan_execute();
        }
    }

    ROS_INFO("Callback end");
}

void CamperoUR10::goHome() {
    move_group.clearPathConstraints();
    move_group.setNamedTarget(C_UR10_POSE_UP);
    plan_execute();
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
}

void CamperoUR10::main() {
    /*std::vector<double> v = move_group.getCurrentJointValues();
    for (int i = 0; i < v.size(); i++) {
        std::cout << v[i] << std::endl;
    }
    moveJoint(C_UR10_SHOULDER_PAN_JOINT, 0.1);
    v.clear();
    v = move_group.getCurrentJointValues();
    for (int i = 0; i < v.size(); i++) {
        std::cout << v[i] << std::endl;
    }
    return;*/
    //if (!goReadyDraw()) return;

    std::cout << "READY\n";
    ros::Rate loop_rate(10);
    //sub_image = nh.subscribe(TOPIC_NAME_BOARD, QUEUE_SIZE, &CamperoUR10::callbackDraw, this);
    sub_move = nh.subscribe(TOPIC_NAME_MOVE, QUEUE_SIZE, &CamperoUR10::callbackMoveOp, this);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    //ros::spin();
}

/*void CamperoUR10::main() {
    // go ready draw position
    if (!goReadyDraw()) return;
    
    // draw circle
    prompt("Press Next to draw");

    geometry_msgs::Pose init_pos = move_group.getCurrentPose().pose;

    double ox = init_pos.position.y - 0.2;
    double oy = init_pos.position.x - 0.1;
    double maxX = ox + 0.5;
    double maxY = 0.1;
    drawSin(oy, ox, maxX, maxY);

    // end
    prompt("Press Next to End");
    
}*/
