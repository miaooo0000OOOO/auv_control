import numpy as np
from utils import compute_thruster_direction

pi = np.pi

# TODO 需要根据实际情况测量和修改

# 推进器数量
THRUSTER_NUMBER = 6

# 推进器正向最大力
THRUSTER_POS_MAX_FORCE = 10.0  # N

# 推进器反向最大力
THRUSTER_NEG_MAX_FORCE = 10.0  # N


# 推进器位置
# R: right
# L: left
# F: front
# M: middle
# B: back

THRUSTER_LF_POS = np.array([0.1746, 0.1093, -0.015])
THRUSTER_RF_POS = np.array([0.1746, -0.1093, -0.015])
THRUSTER_LM_POS = np.array([0, 0.11, 0.075])
THRUSTER_RM_POS = np.array([0, -0.11, 0.075])
THRUSTER_LB_POS = np.array([-0.1746, 0.1093, -0.015])
THRUSTER_RB_POS = np.array([-0.1746, -0.1093, -0.015])

THRUSTER_POS_LIST = np.row_stack(
    (
        THRUSTER_LF_POS,
        THRUSTER_RF_POS,
        THRUSTER_LM_POS,
        THRUSTER_RM_POS,
        THRUSTER_LB_POS,
        THRUSTER_RB_POS,
    )
)

# 推进器欧拉角
THRUSTER_LF_DIR_RPY = np.array([0, 0, pi / 4])
THRUSTER_RF_DIR_RPY = np.array([0, 0, -pi / 4])
THRUSTER_LM_DIR_RPY = np.array([0, pi / 2, 0])
THRUSTER_RM_DIR_RPY = np.array([0, pi / 2, 0])
THRUSTER_LB_DIR_RPY = np.array([0, 0, 3 * pi / 4])
THRUSTER_RB_DIR_RPY = np.array([0, 0, -3 * pi / 4])

THRUSTER_DIR_RPY_LIST = np.row_stack(
    (
        THRUSTER_LF_DIR_RPY,
        THRUSTER_RF_DIR_RPY,
        THRUSTER_LM_DIR_RPY,
        THRUSTER_RM_DIR_RPY,
        THRUSTER_LB_DIR_RPY,
        THRUSTER_RB_DIR_RPY,
    )
)

# 推进器方向向量
THRUSTER_DIR_VEC_LIST = np.array(
    [compute_thruster_direction(rpy) for rpy in THRUSTER_DIR_RPY_LIST]
)


# 浮块位置

BUOYANCY_BLOCK_LF_POS = np.array([0.135, 0.11, 0.03])
BUOYANCY_BLOCK_RF_POS = np.array([0.135, -0.11, 0.03])
BUOYANCY_BLOCK_LB_POS = np.array([-0.135, 0.11, 0.03])
BUOYANCY_BLOCK_RB_POS = np.array([-0.135, -0.11, 0.03])

BUOYANCY_BLOCK_POS_LIST = np.row_stack(
    (
        BUOYANCY_BLOCK_LF_POS,
        BUOYANCY_BLOCK_RF_POS,
        BUOYANCY_BLOCK_LB_POS,
        BUOYANCY_BLOCK_RB_POS,
    )
)

# 重心
CENTER_OF_GRAVITY = np.array([0, 0, -0.01])  # m

# 浮心
CENTER_OF_BUOYANCY = np.mean(BUOYANCY_BLOCK_POS_LIST, axis=0)  # m

# 总质量
TOTAL_MASS = 11.0  # kg

# 重力加速度
g = 9.81  # m/s^2

GRAVITY_FORCE = TOTAL_MASS * g  # N

# 浮力，在淡水中净浮力约为0
BUOYANCY_FORCE = GRAVITY_FORCE  # N
