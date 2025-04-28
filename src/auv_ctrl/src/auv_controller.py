import numpy as np
from control_tools.pid import PIDController
from control import thrust_allocation
import io_interface.input_interface as iif
import io_interface.output_interface as oif
import socket
import fcntl


class AUVController:
    def __init__(self):
        """
        初始化 AUV 控制器。
        """
        # PID 控制器初始化
        # TODO 需要调参
        self.depth_pid = PIDController(
            kp=2.0, ki=0.5, kd=0.1, dead_zone=0.05, integral_limit=10.0
        )
        self.velocity_pid = PIDController(
            kp=1.0, ki=0.3, kd=0.05, dead_zone=0.01, integral_limit=5.0
        )
        self.angular_velocity_pid = PIDController(
            kp=1.5, ki=0.4, kd=0.1, dead_zone=0.01, integral_limit=5.0
        )

    def compute_control(self, dt):
        """
        计算推进器的推力分配。

        Args:
            dt (float): 时间间隔。
        """
        # 获取传感器数据
        current_depth = iif.get_depth()
        current_orientation = iif.get_orientation()  # [roll, pitch, yaw]
        joystick_input = iif.get_joystick()  # [vx, vy, vz, wx, wy, wz]

        # 摇杆输入解析
        # TODO 需要重映射摇杆输入
        desired_velocity = joystick_input[:3]  # 线速度 [vx, vy, vz]
        desired_angular_velocity = joystick_input[3:]  # 角速度 [wx, wy, wz]

        # 深度控制（Z 轴方向的力）
        depth_force = self.depth_pid.compute(
            setpoint=desired_velocity[2], measurement=current_depth, dt=dt
        )

        # 线速度控制（X 和 Y 轴方向的力）
        linear_force = np.zeros(3)
        for i in range(2):  # 仅计算 X 和 Y 轴的力
            linear_force[i] = self.velocity_pid.compute(
                setpoint=desired_velocity[i], measurement=0.0, dt=dt
            )

        # 角速度控制（滚转、俯仰和偏航的力矩）
        angular_torque = np.zeros(3)
        for i in range(3):
            angular_torque[i] = self.angular_velocity_pid.compute(
                setpoint=desired_angular_velocity[i],
                measurement=current_orientation[i],
                dt=dt,
            )

        # 期望的力和力矩
        desired_wrench = np.zeros(6)
        desired_wrench[:3] = linear_force  # [Fx, Fy, Fz]
        desired_wrench[2] = depth_force  # Z 轴方向的力
        desired_wrench[3:] = angular_torque  # [Mx, My, Mz]

        # 计算推进器推力分配
        thrusts = thrust_allocation(desired_wrench, current_orientation)

        # 发送推进器推力指令
        oif.send_thrust_cmd(thrusts)

    def reset(self):
        """
        重置控制器状态。
        """
        self.depth_pid.reset()
        self.velocity_pid.reset()
        self.angular_velocity_pid.reset()


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
                data = np.load(f)
                fcntl.flock(f, fcntl.LOCK_UN)  # 释放锁
            if data.size < 3:
                print(f"共享内存文件 {SHM_FILE} 数据不完整，返回默认值")
                return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            rpy = data[:3]
            print(rpy)
            return np.array([0.0, 0.0, 0.0, rpy[0], rpy[1], rpy[2]])
        except Exception as e:
            print(f"读取共享内存文件 {SHM_FILE} 时出错：{e}")
            return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def custom_get_depth():
        return 1.0

    def custom_get_orientation():
        return np.array([0.0, 0.0, 0.0])
    
    # 替换默认实现
    iif.get_joystick = custom_get_joystick
    iif.get_depth = custom_get_depth
    iif.get_orientation = custom_get_orientation

    controller = AUVController()
    controller.reset()

    # 模拟控制循环
    try:
        while True:
            dt = 0.1  # 假设时间间隔为 0.1 秒
            controller.compute_control(dt)
            time.sleep(dt)
    except KeyboardInterrupt:
        print("控制器已停止")
