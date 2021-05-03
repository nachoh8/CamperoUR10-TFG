#include "../include/c_ur10_def/c_ur10_utils.h"

const std::string C_UR10_PLANNING_GROUP = "manipulator";
const std::string C_UR10_PLANNING_GROUP_EE = "endeffector";

const std::string C_UR10_BASE_LINK = "campero_base_footprint";

//const std::string C_UR10_PREFIX = "campero_ur10_";

#define USE_PREFIX 0

#if USE_PREFIX
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

const std::string C_UR10_EE_JOINT = "campero_ur10_ee_fixed_joint";
#else
const std::string C_UR10_SHOULDER_LINK = "shoulder_link";
const std::string C_UR10_UPPER_ARM_LINK = "upper_arm_link";
const std::string C_UR10_FOREARM_LINK = "forearm_link";
const std::string C_UR10_W1_LINK = "wrist_1_link";
const std::string C_UR10_W2_LINK = "wrist_2_link";
const std::string C_UR10_W3_LINK = "wrist_3_link";

const std::string C_UR10_EE_LINK = "ee_link";

const std::string C_UR10_SHOULDER_PAN_JOINT = "shoulder_pan_joint";
const std::string C_UR10_SHOULDER_LIFT_JOINT = "shoulder_lift_joint";
const std::string C_UR10_ELBOW_JOINT = "elbow_joint";
const std::string C_UR10_W1_JOINT = "wrist_1_joint";
const std::string C_UR10_W2_JOINT = "wrist_2_joint";
const std::string C_UR10_W3_JOINT = "wrist_3_joint";

const std::string C_UR10_EE_JOINT = "ee_fixed_joint";
#endif

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

const std::string C_UR10_POSE_INIT = "init";
const std::string C_UR10_POSE_UP = "up";
const std::string C_UR10_POSE_READY_DRAW = "ready_draw";
const std::string C_UR10_POSE_READY_DRAW_PEN = "ready_draw_pen";

