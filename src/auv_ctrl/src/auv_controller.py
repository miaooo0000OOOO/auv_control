import numpy as np
from control_tools.pid import PIDController
from control_tools.pid import default_pid_controller
from control import thrust_allocation
import io_interface.input_interface as iif
import io_interface.output_interface as oif
import fcntl
from typing import Callable


class AUVController:
    """
    水下机器人控制器类。

    该类用于控制水下机器人的推进器推力分配，以实现目标状态的跟踪。

    它使用 PID 控制器来计算推进器的推力，并通过推力分配算法将力和力矩转换为推进器的推力指令。

    机身坐标系定义：
    - x 轴指向前方 front
    - y 轴指向左方 left
    - z 轴指向上方 up
    """

    def __init__(self):
        """
        初始化 AUV 控制器。
        """
        # PID 控制器初始化
        # TODO 需要调参
        self.pids = {
            "xy_speed": default_pid_controller(),
            "z_speed": default_pid_controller(),
            "angular_speed": default_pid_controller(),
            "orientation": default_pid_controller(),
            "depth": default_pid_controller(),
        }

        self.get_depth: Callable[[], float] = iif.get_depth
        self.get_imu: Callable[[], np.ndarray] = iif.get_imu
        self.get_joystick: Callable[[], np.ndarray] = iif.get_joystick

        self.send_thrust_cmd: Callable[[np.ndarray], None] = oif.send_thrust_cmd

        # 目标状态初始化 不为 None 表示有效u
        self.target = {
            "xy_speed": np.zeros(2),  # 线速度 [vx, vy] m/s
            "z_speed": 0.0,  # 深度速度 m/s
            "angular_speed": np.zeros(3),  # 角速度 [wx, wy, wz] rad/s
            "orientation": None,  # 姿态 [roll, pitch, yaw] rad
            "depth": None,  # 深度 m
        }

        # 其中depth与z_speed互斥
        # orientation与angular_speed互斥

    def set_target(self, target: dict):
        """
        设置目标状态。

        Args:
            target (dict): 目标状态字典，包含以下键：
                - "xy_speed": 线速度 [vx, vy] m/s
                - "z_speed": 深度速度 m/s
                - "angular_speed": 角速度 [wx, wy, wz] rad/s
                - "orientation": 姿态 [roll, pitch, yaw] rad
                - "depth": 深度 m
        Raises:
            AssertionError: 如果目标状态无效。
        """
        # 检查字典是否包含所有键
        required_keys = ["xy_speed", "z_speed", "angular_speed", "orientation", "depth"]
        for key in required_keys:
            if key not in target:
                raise KeyError(f"目标状态字典缺少键: {key}")

        # 判断目标状态是否有效
        assert (
            # xor
            (target.get("depth") is None)
            != (target.get("z_speed") is None)
        ), "目标状态无效：depth与z_speed互斥"
        assert (
            # xor
            (target.get("orientation") is None)
            != (target.get("angular_speed") is None)
        ), "目标状态无效：orientation与angular_speed互斥"

        # 如果值的类型是list而非numpy.ndarray，则转换为numpy.ndarray
        for key in ["xy_speed", "angular_speed", "orientation"]:
            if isinstance(target.get(key), list):
                target[key] = np.array(target[key])
        self.target = target

    def compute_control(self, dt):
        """
        计算推进器的推力分配。

        Args:
            dt (float): 时间间隔。
        """
        # 获取当前深度和姿态
        current_depth = self.get_depth()
        imu = self.get_imu()
        current_orientation = imu[:3]  # [roll, pitch, yaw]
        current_angular_speed = imu[3:6]  # [wx, wy, wz]
        current_speed = imu[6:9]  # [vx, vy, vz]

        # 初始化期望的力和力矩
        desired_wrench = np.zeros(6)  # [Fx, Fy, Fz, Mx, My, Mz]

        # 计算线速度控制力
        if self.target["xy_speed"] is not None:
            speed_error = self.target["xy_speed"] - current_speed[:2]
            xy_force = self.pids["xy_speed"].compute(speed_error, dt)
            desired_wrench[:2] = xy_force  # Fx, Fy

        # 计算深度控制力
        if self.target["depth"] is not None:
            depth_error = self.target["depth"] - current_depth
            depth_force = self.pids["depth"].compute(depth_error, dt)
            desired_wrench[2] = depth_force  # Fz
        # 计算深度速度控制力
        elif self.target["z_speed"] is not None:
            z_speed_error = self.target["z_speed"] - current_speed[2]
            z_force = self.pids["z_speed"].compute(z_speed_error, dt)
            desired_wrench[2] += z_force  # Fz
        else:
            raise RuntimeError("意外的代码路径被执行 depth与z_speed均为None")

        # 计算角速度控制力矩
        if self.target["angular_speed"] is not None:
            angular_speed_error = self.target["angular_speed"] - current_angular_speed
            angular_torque = self.pids["angular_speed"].compute(angular_speed_error, dt)
            desired_wrench[3:] = angular_torque  # Mx, My, Mz
        # 计算姿态控制力矩
        elif self.target["orientation"] is not None:
            orientation_error = self.target["orientation"] - current_orientation
            orientation_torque = self.pids["orientation"].compute(orientation_error, dt)
            desired_wrench[3:] = orientation_torque  # Mx, My, Mz
        else:
            raise RuntimeError(
                "意外的代码路径被执行 orientation与angular_speed均为None"
            )

        # 调试信息
        print("期望的力和力矩:")
        print(
            f"  力: [Fx: {desired_wrench[0]:.2f}, Fy: {desired_wrench[1]:.2f}, Fz: {desired_wrench[2]:.2f}]"
        )
        print(
            f"  力矩: [Mx: {desired_wrench[3]:.2f}, My: {desired_wrench[4]:.2f}, Mz: {desired_wrench[5]:.2f}]"
        )

        # 计算推进器推力分配
        thrusts = thrust_allocation(desired_wrench, current_orientation)

        # 发送推进器推力指令
        self.send_thrust_cmd(thrusts)

    def reset(self):
        """
        重置控制器状态。
        """
        for pid in self.pids.values():
            pid.reset()

        # 重置目标状态
        self.target = {
            "xy_speed": np.zeros(2),
            "z_speed": 0.0,
            "angular_speed": np.zeros(3),
            "orientation": None,
            "depth": None,
        }


# 示例运行
if __name__ == "__main__":
    import time
    import os

    def custom_get_joystick():
        # 定义共享内存文件路径
        SHM_FILE = "/dev/shm/IMU_DATA.npy"

        try:
            with open(SHM_FILE, "rb") as f:
                fcntl.flock(f, fcntl.LOCK_SH)  # 加读锁
                data: np.ndarray = np.load(f)
                fcntl.flock(f, fcntl.LOCK_UN)  # 释放锁
            if data.size != 6:
                print(f"共享内存文件 {SHM_FILE} 数据不完整，返回默认值")
                return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            rpy = data[:3]
            rpy_speed = data[3:6]
            print(rpy_speed.round(2))
            rpy_speed *= np.pi / 180.0  # 转换为弧度
            return np.array([0.0, 0.0, 0.0, rpy_speed[0], rpy_speed[1], rpy_speed[2]])
        except Exception as e:
            print(f"读取共享内存文件 {SHM_FILE} 时出错：{e}")
            return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def custom_get_depth():
        return 0.0

    def custom_get_imu():
        return np.zeros(9)

    # 替换默认实现
    iif.get_joystick = custom_get_joystick
    iif.get_depth = custom_get_depth
    iif.get_imu = custom_get_imu
    controller = AUVController()
    controller.reset()

    # 模拟控制循环
    try:
        while True:
            dt = 0.1  # 假设时间间隔为 0.1 秒
            controller.set_target(
                {
                    "xy_speed": np.array([0.1, 0.0]),
                    "angular_speed": None,
                    "orientation": np.array([0.0, 0.0, 0.0]),
                    "z_speed": None,
                    "depth": 1.0,
                }
            )
            controller.compute_control(dt)
            print("------------")
            time.sleep(dt)
    except KeyboardInterrupt:
        print("控制器已停止")
