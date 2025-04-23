import system_params_symbol as sps
import output_interface as oif
import input_interface as iif

import numpy as np
from scipy.spatial.transform import Rotation

def build_thruster_matrix(orientation: np.ndarray) -> np.ndarray:
    """
    构建给定姿态下的推进器矩阵。

    该函数根据输入的姿态角计算推进器在世界坐标系中的方向和位置，
    并生成一个6x6的推进器配置矩阵，用于分配推进器的推力。

    Args:
        orientation (np.ndarray): 一个一维数组或列表，包含以弧度为单位的滚转、俯仰和偏航角。

    Returns:
        np.ndarray: 一个6x6的矩阵，表示世界坐标系中的推进器配置。
    """
    # 根据姿态角计算旋转矩阵，将局部坐标系转换到世界坐标系
    R = Rotation.from_euler('xyz', orientation).as_matrix()
    # 初始化推进器矩阵，6x6矩阵，每列对应一个推进器的力和力矩分量
    B = np.zeros((6, 6))
    # 遍历每个推进器
    for i in range(6):
        # 计算推进器在世界坐标系中的方向（推进器推力方向）
        thruster_dir_world = R @ sps.f[i]
        # 计算推进器在世界坐标系中的位置（推进器相对于质心的位置）
        thruster_pos_world = R @ sps.r[i]
        # 推进器的力方向分量（前三行表示力）
        B[0:3, i] = thruster_dir_world
        # 推进器的力矩分量（后三行表示力矩，位置与方向的叉积）
        B[3:6, i] = np.cross(thruster_pos_world, thruster_dir_world)
    # 返回推进器矩阵
    return B

def compute_buoyancy_gravity_moment(orientation: np.ndarray) -> np.ndarray:
    """
    计算给定姿态下浮力和重力产生的力矩。

    Args:
        orientation (np.ndarray): 一个一维数组或列表，包含以弧度为单位的滚转、俯仰和偏航角。

    Returns:
        np.ndarray: 一个3D向量，表示浮力和重力产生的力矩。
    """
    # 根据姿态角计算旋转矩阵，将局部坐标系转换到世界坐标系
    R = Rotation.from_euler('xyz', orientation).as_matrix()
    # 计算浮心在世界坐标系中的位置
    cob_world = R @ sps.r_b
    # 计算重心在世界坐标系中的位置
    com_world = R @ sps.r_g
    # 定义浮力方向（沿z轴正方向）
    buoyancy_dir = np.array([0, 0, 1])
    # 定义重力方向（沿z轴负方向）
    gravity_dir = np.array([0, 0, -1])
    # 计算浮力产生的力矩（浮心位置与浮力的叉积）
    buoyancy_moment = np.cross(cob_world, buoyancy_dir * sps.F_b)
    # 计算重力产生的力矩（重心位置与重力的叉积）
    gravity_moment = np.cross(com_world, gravity_dir * sps.G)
    # 返回浮力和重力力矩的总和
    return buoyancy_moment + gravity_moment

def thrust_allocation(desired_wrench: np.ndarray, orientation: np.ndarray) -> np.ndarray:
    """
    根据期望的力和力矩分配推进器的推力。

    Args:
        desired_wrench (np.ndarray): 一个6D向量，表示期望的力和力矩 [Fx, Fy, Fz, Mx, My, Mz]。
        orientation (np.ndarray): 一个一维数组或列表，包含以弧度为单位的滚转、俯仰和偏航角。

    Returns:
        np.ndarray: 一个6D向量，表示每个推进器的推力分配。
    """
    # 根据姿态角计算旋转矩阵，将局部坐标系转换到世界坐标系
    R = Rotation.from_euler('xyz', orientation).as_matrix()
    # 构建推进器矩阵
    B = build_thruster_matrix(orientation)
    # 计算浮力和重力产生的力矩
    buoyancy_gravity_moment = compute_buoyancy_gravity_moment(orientation)
    # 计算浮力和重力产生的合力
    buoyancy_gravity_force = R @ np.array([0, 0, sps.F_b - sps.G])
    
    # 创建补偿后的期望力和力矩
    compensated_wrench = desired_wrench.copy()
    # 减去浮力和重力的影响
    compensated_wrench[0:3] -= buoyancy_gravity_force   
    # 减去浮力和重力力矩的影响
    compensated_wrench[3:6] -= buoyancy_gravity_moment
    
    # 使用伪逆矩阵计算推进器推力分配
    return np.linalg.pinv(B) @ compensated_wrench