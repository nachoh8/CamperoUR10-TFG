#include "../include/c_ur10_def/campero_ur10.h"

#include <iostream>
#include <cmath>
#include <fstream>

#include <ros/ros.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_interface.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rvt = rviz_visual_tools;

//#define C_UR10_INFO_NAMED "CamperoUR10"  // roslog name

#define NODE_NAME "campero_ur10"

#define TOPIC_NAME_TELEOP "campero_ur10_move"
#define TOPIC_NAME_IMG_DRAW "image_points"
#define TOPIC_NAME_CONFIG "update_config"

#define QUEUE_SIZE_TELEOP 1 // teleop topic queue size
#define QUEUE_SIZE_IMG_DRAW 1 // img_draw topic queue size

#define FIELD_W_BOARD "#W_BOARD"
#define FIELD_H_BOARD "#H_BOARD"
#define FIELD_BOARD_X "#BOARD_X"
#define FIELD_BOARD_Y "#BOARD_Y"
#define FIELD_BOARD_Z "#BOARD_Z"
#define FIELD_Z_PEN_DOWN "#Z_PEN_DOWN"
#define FIELD_Z_PEN_UP "#Z_PEN_UP"
#define FIELD_CORRECT_X "#CORRECT_X"
#define FIELD_CORRECT_Y "#CORRECT_Y"
#define FIELD_CORRECT_Z "#CORRECT_Z"

CamperoUR10::CamperoUR10(C_UR10_Mode _mode, std::string& config_file, bool _add_ori) {
    init(_mode);
    
    add_ori = _add_ori;

    loadConfig(config_file);
}

void CamperoUR10::init(C_UR10_Mode _mode) {
    mode = _mode;

    // Config Topic
    config_sub = nh.subscribe(TOPIC_NAME_CONFIG, 1, &CamperoUR10::updateConfigCallback, this);

    // Create Orientation Constraint Quaternion
    tf2::Quaternion ori;
    ori.setRPY(M_PI, 0, 0);

    ori_constraint = tf2::toMsg(ori.normalize());

    // Config move_group
    move_group.setPlanningTime(10);

    move_group.setMaxVelocityScalingFactor(0.1);

    // move_group.setMaxAccelerationScalingFactor(1);

    move_group.setStartStateToCurrentState();

    // Load Visual Tools
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    visual_tools.trigger();

    // Show Info
    ROS_INFO("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
}

bool CamperoUR10::loadDrawConfig(std::string& file) {
    std::fstream fin;
    fin.open(file, std::ios::in);

    double _z_pen_down = -1.0, _z_pen_up = -1.0;
    double _correct_x = -1.0, _correct_y = -1.0, _correct_z = -1.0;
    double _board_x = -1.0, _board_y = -1.0, _board_z = BOARD_Z_DEFAULT;
    double _w_board = -1.0, _h_board = -1.0;
    
    int n_param = 0;
    std::string line;
    while (!fin.eof()) {
        getline(fin, line);
        
        if (line.compare(FIELD_W_BOARD) == 0) {
            fin >> _w_board;
            n_param++;
            
        } else if (line.compare(FIELD_H_BOARD) == 0) {
            fin >> _h_board;
            n_param++;
            
        } else if (line.compare(FIELD_BOARD_X) == 0) {
            fin >> _board_x;
            n_param++;
            
        } else if (line.compare(FIELD_BOARD_Y) == 0) {
            fin >> _board_y;
            n_param++;

        } else if (line.compare(FIELD_BOARD_Z) == 0) { // opcional
            fin >> _board_z;
            n_param++;

        } else if (line.compare(FIELD_Z_PEN_DOWN) == 0) {
            fin >> _z_pen_down;
            n_param++;

        } else if (line.compare(FIELD_Z_PEN_UP) == 0) {
            fin >> _z_pen_up;
            n_param++;
            
        } else if (line.compare(FIELD_CORRECT_X) == 0) {
            fin >> _correct_x;
            n_param++;
            
        } else if (line.compare(FIELD_CORRECT_Y) == 0) {
            fin >> _correct_y;
            n_param++;
            
        } else if (line.compare(FIELD_CORRECT_Z) == 0) {
            fin >> _correct_z;
            n_param++;
            
        }
    }

    fin.close();

    bool _draw_okey = !(_z_pen_down == -1 || _z_pen_up == -1 ||
                        _correct_y == -1 || _correct_x == -1 ||_correct_z == -1 ||
                        _w_board <= 0 || _h_board <= 0 ||
                        _board_x == -1 || _board_y == -1 || _board_z == -1);

    if (_draw_okey) {
        z_pen_down = _z_pen_down;
        z_pen_up = _z_pen_up;
        correct_x = _correct_x;
        correct_y = _correct_y;
        correct_z = _correct_z;
        
        board_min_x = _board_x;
        board_min_y = _board_y;
        board_z = _board_z;
        w_board = _w_board;
        h_board = _h_board;

        draw_okey = _draw_okey;

        deleteMarkers(); // remove previuos board and draw new board
    }

    return _draw_okey;
}

void CamperoUR10::printDrawConfig() {
    ROS_INFO("----Draw Config----");
    if (!draw_okey) {
        ROS_WARN("Error: configuracion no valida");
    }

    ROS_INFO("--Pen:");
    std::string s = "---Z_PEN_DOWN: " + std::to_string(z_pen_down);
    ROS_INFO(s.c_str());
    s = "---Z_PEN_UP: " + std::to_string(z_pen_up);
    ROS_INFO(s.c_str());
    s = "---Correcion de posicion(x,y): (" + std::to_string(correct_x) + ", " + std::to_string(correct_y) + ")";
    ROS_INFO(s.c_str());

    ROS_INFO("--Board:");
    s = "---Size: " + std::to_string(w_board) + "x" + std::to_string(h_board);
    ROS_INFO(s.c_str());
    s = "---Position: (" + std::to_string(board_min_x) + ", " + std::to_string(board_min_y) + ", " + std::to_string(board_z) + ")";
    ROS_INFO(s.c_str());
}

void CamperoUR10::updateConfigCallback(const std_msgs::String::ConstPtr& file) {
    std::string s = file->data;
    loadConfig(s);
}

void CamperoUR10::loadConfig(std::string& file) {
    if (file.empty()) {
        return;
    }
    
    if (mode == C_UR10_Mode::DRAW_IMAGE) {
        ROS_INFO("----Actualizando Configuracion Dibujar: %s----", file.c_str());
        if (loadDrawConfig(file)) {
            printDrawConfig();
        } else {
            ROS_INFO("Error: no se ha aplicado la configuracion, faltan parametros");
        }
    }
}

void CamperoUR10::deleteMarkers(bool keep_visual_refs) {
    visual_tools.deleteAllMarkers();
    if (keep_visual_refs) {
        if (mode == C_UR10_Mode::DRAW_IMAGE) showBoard();
    }
}

void CamperoUR10::prompt(std::string msg) {
    visual_tools.prompt(msg);
}

void CamperoUR10::showPlan() {
    deleteMarkers();
    
    /*for (std::size_t i = 0; i < move_group.getPoseTargets().size(); ++i) {
        visual_tools.publishAxisLabeled(move_group.getPoseTargets()[i].pose, "pt" + std::to_string(i), rviz_visual_tools::SMALL);
    }*/

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    
    visual_tools.trigger();
}

bool CamperoUR10::plan() {
    ROS_INFO("Planning...");
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        ROS_INFO("Plan Ok");
        showPlan();
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
    deleteMarkers();
    visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
    visual_tools.trigger();


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
    
        
    if (!addOriConstraint()) return false;

    //ROS_INFO("Go to ready draw position");
    //move_group.setNamedTarget(C_UR10_POSE_READY_DRAW_PEN);

    return true;//plan_execute();
}

bool CamperoUR10::addOriConstraint() {
    ROS_INFO("1.Set correct orientation");

    geometry_msgs::Pose target_ori = move_group.getCurrentPose().pose;
    target_ori.orientation = ori_constraint;
    move_group.setPoseTarget(target_ori);
    if (!plan_execute()) return false;

    ROS_INFO("2.Add orientation constraint");

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = C_UR10_W3_LINK;
    ocm.header.frame_id = C_UR10_BASE_LINK;
    ocm.weight = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 2*M_PI;
    ocm.orientation = ori_constraint;

    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(constraints);
    
    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose2 = move_group.getCurrentPose().pose;
    start_state.setFromIK(joint_model_group, start_pose2);
    move_group.setStartState(start_state);

    std::cout << move_group.getPathConstraints() << std::endl;

    return true;
}

moveit::planning_interface::MoveGroupInterface& CamperoUR10::getMoveGroup() {
    return move_group;
}

void CamperoUR10::processRealTrace(const campero_ur10_msgs::ImgTrace trace, std::vector<geometry_msgs::Pose>& waypoints) {
    const int numPts = trace.points.size();
    if (numPts <= 0) return;

    ROS_INFO("Trace: %d points", numPts);
    
	geometry_msgs::Pose target = move_group.getCurrentPose().pose;
    
    // go to first point with pen up
    target.position.y = trace.points[0].y + correct_y;
    target.position.x = trace.points[0].x + correct_x;
    target.position.z = z_pen_up;
    waypoints.push_back(target);
	
	target.position.z = board_z + correct_z;
    for (int i = 0; i < numPts; i++) {
        target.position.y = trace.points[i].y + correct_y;
		target.position.x = trace.points[i].x + correct_x;
        waypoints.push_back(target);
    }
    
    target.position.z = z_pen_up;
    waypoints.push_back(target);
}

void CamperoUR10::processTrace(const campero_ur10_msgs::ImgTrace trace, const double w_div, const double h_div, std::vector<geometry_msgs::Pose>& waypoints) {
    const int numPts = trace.points.size();
    if (numPts <= 0) return;

    ROS_INFO("Trace: %d points", numPts);

    geometry_msgs::Pose target = move_group.getCurrentPose().pose;
    
    // go to first point with pen up
    target.position.y = board_min_y + trace.points[0].x * w_div + correct_y;
    target.position.x = board_min_x + trace.points[0].y * h_div + correct_x;
    target.position.z = z_pen_up;
    waypoints.push_back(target);

    // add all points
    target.position.z = z_pen_down;
    for (int i = 0; i < numPts; i++) {
        target.position.y = board_min_y + trace.points[i].x * w_div + correct_y;
        target.position.x = board_min_x + trace.points[i].y * h_div + correct_x;
        waypoints.push_back(target);
    }

    // pen up
    target.position.z = z_pen_up;
    waypoints.push_back(target);
}

void CamperoUR10::callbackDraw(const campero_ur10_msgs::ImageDraw image) {
    if (!draw_okey) {
        ROS_WARN("WARNING: configuracion dibujar incompleta");
        return;
    }

    ROS_INFO("Image received: %d traces", image.traces.size());
    
    bool _real_pts = image.W == -1 && image.H == -1;

    std::vector<geometry_msgs::Pose> waypoints;

    if (_real_pts) {
        ROS_INFO("Process Real Points");
        
        for (int i = 0; i < image.traces.size(); i++) {
            processRealTrace(image.traces[i], waypoints);
        }

    } else {
        ROS_INFO("Process Virtual Board Points");
        if (image.W <= 0 || image.H <= 0) {
            ROS_WARN("Virtual Board Size invalid: %dx%d", image.W, image.H);
            return;
        }

        const double w_div = w_board / image.W;
        const double h_div = h_board / image.H;

        for (int i = 0; i < image.traces.size(); i++) {
            processTrace(image.traces[i], w_div, h_div, waypoints);
        }
    }
    
    // plan & execute
    plan_exec_Carthesian(waypoints);
    /*if (plan_exec_Carthesian(waypoints)) {
        // go to ready draw posiyion
        ROS_INFO("Move to ready position");

        //move_group.setNamedTarget(C_UR10_POSE_READY_DRAW_PEN);

        //plan_execute();
    }*/
    
    ROS_INFO("Callback end");
}

void CamperoUR10::callbackMoveOp(const campero_ur10_msgs::MoveOp operation) {
    ROS_INFO("Operation received");

    const int type = operation.type, id = operation.id;
    const double v = operation.value;
    
    if (id < 0 || id > 5) { // id not valid
        ROS_INFO("Id %d not valid", id);
    } else {
        if (operation.type == campero_ur10_msgs::MoveOp::MOVE_JOINT) {
            ROS_INFO("Moving joint: %d", id);

            double f_v = v + move_group.getCurrentJointValues()[id]; // update joint id value
            ROS_INFO("New value target: %.2f rads", f_v);
            
            moveJoint(id, f_v);

        } else if (operation.type == campero_ur10_msgs::MoveOp::MOVE_CARTHESIAN) {
            ROS_INFO("Moving carthesian: %d", id);

            geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
            if (id < campero_ur10_msgs::MoveOp::RX_AXIS) { // XYZ
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

                ROS_INFO("Position target: (%.2f, %.2f, %2.f)", target_pose.position.x, target_pose.position.y, target_pose.position.z);

            }  else { // RPY
                tf2::Quaternion q;
                tf2::fromMsg(target_pose.orientation, q);
                tf2::Matrix3x3 m(q);

                double r, p, y;
                m.getRPY(r, p, y);
                
                switch (id)
                {
                case campero_ur10_msgs::MoveOp::RX_AXIS:
                    r += v;
                    break;
                case campero_ur10_msgs::MoveOp::RY_AXIS:
                    p += v;
                    break;
                case campero_ur10_msgs::MoveOp::RZ_AXIS:
                    y += v;
                    break;
                default:
                    break;
                }
                q.setRPY(r,p,y);
                target_pose.orientation = tf2::toMsg(q.normalize());

                ROS_INFO("RPY target: (%.2f, %.2f, %2.f)", r, p, y);
            }
            
            // plan & eceute POSE/RPY
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
    deleteMarkers(false);
}

void CamperoUR10::showBoard() {
    if (!draw_okey) return;

    geometry_msgs::Polygon pl;
    geometry_msgs::Point32 pt;

    pt.x = board_min_x;
    pt.y = board_min_y;
    pt.z = board_z;
    pl.points.push_back(pt);
    
    pt.x = board_min_x;
    pt.y = board_min_y + w_board;
    pl.points.push_back(pt);

    pt.x = board_min_x + h_board;
    pt.y = board_min_y + w_board;
    pl.points.push_back(pt);

    pt.x = board_min_x + h_board;
    pt.y = board_min_y;
    pl.points.push_back(pt);
    
    visual_tools.publishPolygon(pl);

    visual_tools.trigger();
}

void CamperoUR10::main() {
    
    switch (mode)
    {
    case C_UR10_Mode::DRAW_IMAGE:
        {
            ROS_INFO("Mode: Draw Image");
            
            //printDrawConfig();
            
            // go ready draw position
            if (!goReadyDraw()) return;
			
			sub = nh.subscribe(TOPIC_NAME_IMG_DRAW, QUEUE_SIZE_IMG_DRAW, &CamperoUR10::callbackDraw, this);
        }
        break;

    case C_UR10_Mode::TELEOP:
        ROS_INFO("Mode: Teleop");
        
        if (add_ori) { // add orientation constraint
            if (!addOriConstraint()) return;
        }

        sub = nh.subscribe(TOPIC_NAME_TELEOP, QUEUE_SIZE_TELEOP, &CamperoUR10::callbackMoveOp, this);
        break;

    default:
        ROS_INFO("Mode not valid -> exit");
        return;
    }

    ROS_INFO("Campero_UR10 READY\n");
    ros::Rate loop_rate(10);

    // wait
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}
