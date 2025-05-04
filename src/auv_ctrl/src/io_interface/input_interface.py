import numpy as np
from typing import Callable

get_depth: Callable[[], float] = None
get_imu: Callable[[], np.ndarray] = None
get_joystick: Callable[[], np.ndarray] = None


def default_get_depth():
    """
    获取深度传感器的值。

    Returns:
        float: 深度传感器的值，单位为米。
    """
    # 假设深度传感器返回一个随机值
    return np.random.uniform(0, 10)  # 随机深度值，范围在0到10米之间


def default_get_imu():
    """
    获取IMU的值。

    Returns:
        np.ndarray: 一个包含6个元素的数组，表示以弧度为单位的滚转、俯仰、偏航（rpy）和角速度（rad/s），
    """
    # 姿态rad 角速度rad/s
    return np.random.uniform(-np.pi, np.pi, size=6)


def default_get_joystick():
    """
    获取操纵杆的输入值。

    Returns:
        np.ndarray: 一个包含操纵杆输入的数组，表示前后、左右、上下、滚转、俯仰和偏航，
        值的范围为 -1 到 1。
    """
    # 使用随机值模拟操纵杆输入
    return np.random.uniform(-1, 1, size=6)  # 随机操纵杆值，范围在 [-1, 1] 之间


# 初始化为默认实现
get_depth = default_get_depth
get_imu = default_get_imu
get_joystick = default_get_joystick

# TODO 根据实际硬件接口重写

# 示例：如果需要重写，可以直接替换 get_depth 和 get_imu
# def custom_get_depth():
#     return 5.0  # 返回一个固定的深度值
#
# # 替换为自定义实现
# get_depth = custom_get_depth
# 这个函数最初被初始化为默认实现，返回随机值。用户可以根据需要重写这些函数，以提供自定义的传感器数据。
