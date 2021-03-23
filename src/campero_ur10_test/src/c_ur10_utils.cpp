#include "../include/c_ur10_def/c_ur10_utils.h"

const std::string C_UR10_PLANNING_GROUP = "manipulator";
const std::string C_UR10_PLANNING_GROUP_EE = "endeffector";

const std::string C_UR10_BASE_LINK = "campero_base_footprint";//"campero_ur10_base_link";
const std::string C_UR10_SHOULDER_LINK = "campero_ur10_shoulder_link";
const std::string C_UR10_UPPER_ARM_LINK = "campero_ur10_upper_arm_link";
const std::string C_UR10_FOREARM_LINK = "campero_ur10_forearm_link";
const std::string C_UR10_W1_LINK = "campero_ur10_wrist_1_link";
const std::string C_UR10_W2_LINK = "campero_ur10_wrist_2_link";
const std::string C_UR10_W3_LINK = "campero_ur10_wrist_3_link";

const std::string C_UR10_EE_LINK = "campero_ur10_ee_link";

const std::string C_UR10_SHOULDER_PAN_JOINT = "campero_ur10_shoulder_pan_joint";
const std::string C_UR10_SHOULDER_LIFT_JOINT = "campero_ur10_shoulder_lift_joint";
const std::string C_UR10_ELBOW_JOINT = "campero_ur10_elbow_joint";
const std::string C_UR10_W1_JOINT = "campero_ur10_wrist_1_joint";
const std::string C_UR10_W2_JOINT = "campero_ur10_wrist_2_joint";
const std::string C_UR10_W3_JOINT = "campero_ur10_wrist_3_joint";

const int C_UR10_SHOULDER_PAN_JOINT_IDX = 0;
const int C_UR10_SHOULDER_LIFT_JOINT_IDX = 1;
const int C_UR10_ELBOW_JOINT_IDX = 2;
const int C_UR10_W1_JOINT_IDX = 3;
const int C_UR10_W2_JOINT_IDX = 4;
const int C_UR10_W3_JOINT_IDX = 5;

int jointName2Idx(const std::string& name) {
    if (name == C_UR10_SHOULDER_PAN_JOINT) {
        return C_UR10_SHOULDER_PAN_JOINT_IDX;
    } else if (name == C_UR10_SHOULDER_LIFT_JOINT) {
        return C_UR10_SHOULDER_LIFT_JOINT_IDX;
    } else if (name == C_UR10_ELBOW_JOINT) {
        return C_UR10_ELBOW_JOINT_IDX;
    } else if (name == C_UR10_W1_JOINT) {
        return C_UR10_W1_JOINT_IDX;
    } else if (name == C_UR10_W2_JOINT) {
        return C_UR10_W2_JOINT_IDX;
    } else if (name == C_UR10_W3_JOINT) {
        return C_UR10_W3_JOINT_IDX;
    }

    return -1;
}

const std::string C_UR10_EE_JOINT = "campero_ur10_ee_fixed_joint";

const std::string C_UR10_POSE_INIT = "init";
const std::string C_UR10_POSE_UP = "up";
const std::string C_UR10_POSE_READY_DRAW = "ready_draw";
