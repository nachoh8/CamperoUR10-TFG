#include "../include/c_ur10_def/c_ur10_utils.h"

const std::string C_UR10_PLANNING_GROUP = "manipulator";
const std::string C_UR10_PLANNING_GROUP_EE = "endeffector";

const std::string C_UR10_BASE_LINK = "campero_base_footprint";

#define USE_PREFIX 1

#if USE_PREFIX
const std::string C_UR10_PREFIX = "campero_ur10_";
#else
const std::string C_UR10_PREFIX = "";
#endif

const std::string C_UR10_SHOULDER_LINK = C_UR10_PREFIX + "shoulder_link";
const std::string C_UR10_UPPER_ARM_LINK = C_UR10_PREFIX + "upper_arm_link";
const std::string C_UR10_FOREARM_LINK = C_UR10_PREFIX + "forearm_link";
const std::string C_UR10_W1_LINK = C_UR10_PREFIX + "wrist_1_link";
const std::string C_UR10_W2_LINK = C_UR10_PREFIX + "wrist_2_link";
const std::string C_UR10_W3_LINK = C_UR10_PREFIX + "wrist_3_link";

const std::string C_UR10_EE_LINK = C_UR10_PREFIX + "ee_link";

const std::string C_UR10_SHOULDER_PAN_JOINT = C_UR10_PREFIX + "shoulder_pan_joint";
const std::string C_UR10_SHOULDER_LIFT_JOINT = C_UR10_PREFIX + "shoulder_lift_joint";
const std::string C_UR10_ELBOW_JOINT = C_UR10_PREFIX + "elbow_joint";
const std::string C_UR10_W1_JOINT = C_UR10_PREFIX + "wrist_1_joint";
const std::string C_UR10_W2_JOINT = C_UR10_PREFIX + "wrist_2_joint";
const std::string C_UR10_W3_JOINT = C_UR10_PREFIX + "wrist_3_joint";

const std::string C_UR10_EE_JOINT = C_UR10_PREFIX + "ee_fixed_joint";

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

