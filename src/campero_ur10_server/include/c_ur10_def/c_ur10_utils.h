#ifndef __C_UR10_DEF__
#define __C_UR10_DEF__

#include <string>

//#include <moveit/move_group_interface/move_group_interface.h>

extern const std::string C_UR10_PLANNING_GROUP; // = "manipulator";
extern const std::string C_UR10_PLANNING_GROUP_EE; // = "endeffector";

extern const std::string C_UR10_BASE_LINK; // = "campero_base_footprint";//"campero_ur10_base_link";
extern const std::string C_UR10_SHOULDER_LINK; // = "campero_ur10_shoulder_link";
extern const std::string C_UR10_UPPER_ARM_LINK; // = "campero_ur10_upper_arm_link";
extern const std::string C_UR10_FOREARM_LINK; // = "campero_ur10_forearm_link";
extern const std::string C_UR10_W1_LINK; // = "campero_ur10_wrist_1_link";
extern const std::string C_UR10_W2_LINK; // = "campero_ur10_wrist_2_link";
extern const std::string C_UR10_W3_LINK; // = "campero_ur10_wrist_3_link";

extern const std::string C_UR10_EE_LINK; // = "campero_ur10_ee_link";

extern const std::string C_UR10_SHOULDER_PAN_JOINT; // = "campero_ur10_shoulder_pan_joint";
extern const std::string C_UR10_SHOULDER_LIFT_JOINT; // = "campero_ur10_shoulder_lift_joint";
extern const std::string C_UR10_ELBOW_JOINT; // = "campero_ur10_elbow_joint";
extern const std::string C_UR10_W1_JOINT; // = "campero_ur10_wrist_1_joint";
extern const std::string C_UR10_W2_JOINT; // = "campero_ur10_wrist_2_joint";
extern const std::string C_UR10_W3_JOINT; // = "campero_ur10_wrist_3_joint";

extern const int C_UR10_SHOULDER_PAN_JOINT_IDX;
extern const int C_UR10_SHOULDER_LIFT_JOINT_IDX;
extern const int C_UR10_ELBOW_JOINT_IDX;
extern const int C_UR10_W1_JOINT_IDX;
extern const int C_UR10_W2_JOINT_IDX;
extern const int C_UR10_W3_JOINT_IDX;

int jointName2Idx(const std::string& name);

extern const std::string C_UR10_EE_JOINT; // = "campero_ur10_ee_fixed_joint";

// Poses
extern const std::string C_UR10_POSE_INIT;
extern const std::string C_UR10_POSE_UP;
extern const std::string C_UR10_POSE_READY_DRAW;
extern const std::string C_UR10_POSE_READY_DRAW_PEN;

#endif
