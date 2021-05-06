
C_UR10_PLANNING_GROUP = "manipulator"
C_UR10_PLANNING_GROUP_EE = "endeffector"

C_UR10_POSE_INIT = "init"
C_UR10_POSE_UP = "up"
C_UR10_POSE_READY_DRAW = "ready_draw"

C_UR10_BASE_LINK = "campero_base_footprint"
C_UR10_SHOULDER_LINK = "campero_ur10_shoulder_link"
C_UR10_UPPER_ARM_LINK = "campero_ur10_upper_arm_link"
C_UR10_FOREARM_LINK = "campero_ur10_forearm_link"
C_UR10_W1_LINK = "campero_ur10_wrist_1_link"
C_UR10_W2_LINK = "campero_ur10_wrist_2_link"
C_UR10_W3_LINK = "campero_ur10_wrist_3_link"

C_UR10_EE_LINK = "campero_ur10_ee_link"

C_UR10_SHOULDER_PAN_JOINT = "campero_ur10_shoulder_pan_joint"
C_UR10_SHOULDER_LIFT_JOINT = "campero_ur10_shoulder_lift_joint"
C_UR10_ELBOW_JOINT = "campero_ur10_elbow_joint"
C_UR10_W1_JOINT = "campero_ur10_wrist_1_joint"
C_UR10_W2_JOINT = "campero_ur10_wrist_2_joint"
C_UR10_W3_JOINT = "campero_ur10_wrist_3_joint"

C_UR10_EE_JOINT = "campero_ur10_ee_fixed_joint"

C_UR10_SHOULDER_PAN_JOINT_IDX = 0
C_UR10_SHOULDER_LIFT_JOINT_IDX = 1
C_UR10_ELBOW_JOINT_IDX = 2
C_UR10_W1_JOINT_IDX = 3
C_UR10_W2_JOINT_IDX = 4
C_UR10_W3_JOINT_IDX = 5

def jointName2Idx(name):
    """ transform joint name(str) to joint index(int) """

    if name == C_UR10_SHOULDER_PAN_JOINT:
        return C_UR10_SHOULDER_PAN_JOINT_IDX
    elif name == C_UR10_SHOULDER_LIFT_JOINT:
        return C_UR10_SHOULDER_LIFT_JOINT_IDX
    elif name == C_UR10_ELBOW_JOINT:
        return C_UR10_ELBOW_JOINT_IDX
    elif name == C_UR10_W1_JOINT:
        return C_UR10_W1_JOINT_IDX
    elif name == C_UR10_W2_JOINT:
        return C_UR10_W2_JOINT_IDX
    elif name == C_UR10_W3_JOINT:
        return C_UR10_W3_JOINT_IDX
    
    return -1
