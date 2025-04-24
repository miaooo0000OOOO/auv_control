import numpy as np

def rpy_to_rotation_matrix(rpy: np.ndarray) -> np.ndarray:
    """
    将航向角、俯仰角和横滚角转换为旋转矩阵。

    Args:
        rpy (np.ndarray): 一个包含航向角、俯仰角和横滚角（以弧度为单位）的1D数组或列表。

    Returns:
        np.ndarray: 一个3x3的旋转矩阵。
    """    
    roll, pitch, yaw = rpy
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    return R_z @ R_y @ R_x

def dir_vec_to_rotation_matrix(dir_vec: np.ndarray) -> np.ndarray:
    """
    将方向向量转换为旋转矩阵。

    Args:
        dir_vec (np.ndarray): 一个3D方向向量。

    Returns:
        np.ndarray: 一个3x3的旋转矩阵。
    """
    assert dir_vec.shape == (3,), "方向向量必须是一个3D向量。"
    
    # 归一化方向向量
    dir_vec = dir_vec / np.linalg.norm(dir_vec)
    
    # 创建旋转矩阵
    R = np.eye(3)
    R[0, 0] = dir_vec[0]
    R[1, 1] = dir_vec[1]
    R[2, 2] = dir_vec[2]
    
    return R

def compute_thruster_direction(rpy: np.ndarray) -> np.ndarray:
    """
    根据横滚角、俯仰角和航向角计算推进器的方向向量。

    Args:
        rpy (np.ndarray): 一个包含横滚角、俯仰角和航向角（以弧度为单位）的1D数组或列表。

    Returns:
        np.ndarray: 一个3D方向向量，表示推进器的方向。
    """
    from scipy.spatial.transform import Rotation
    rot = Rotation.from_euler('xyz', rpy)
    return rot.apply([1, 0, 0])