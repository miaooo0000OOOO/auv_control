from typing import Optional
import numpy as np


class PIDController:
    """
    PID 控制器类。

    该类实现了一个简单的 PID 控制器，具有死区和积分清零功能。

    可以接受多个输入和输出，适用于多变量控制系统。
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        dead_zone: Optional[float] = 0.0,
        integral_limit: Optional[float] = float("inf"),
    ):
        """
        初始化 PID 控制器。

        Args:
            kp (float): 比例增益。
            ki (float): 积分增益。
            kd (float): 微分增益。
            dead_zone (Optional[float]): 死区范围，误差小于此值时输出为零。
            integral_limit (Optional[float]): 积分上限，限制积分项的绝对值。
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dead_zone = dead_zone
        self.integral_limit = integral_limit

        self.integral = None
        self.prev_error = None

    def reset(self):
        """重置积分项和前一误差。"""
        self.integral = None
        self.prev_error = None

    def compute(self, error: np.ndarray, dt: float) -> np.ndarray:
        """
        计算 PID 控制器的输出。

        Args:
            error (np.ndarray): 误差值。
            dt (float): 时间间隔。

        Returns:
            np.ndarray: PID 控制器的输出。
        """
        # 初始化积分项和前一误差
        if self.integral is None:
            self.integral = np.zeros_like(error)
        if self.prev_error is None:
            self.prev_error = np.zeros_like(error)

        # 应用死区
        error = np.where(np.abs(error) < self.dead_zone, 0.0, error)

        # 计算积分项，限制积分上限
        self.integral += error * dt
        self.integral = np.clip(
            self.integral, -self.integral_limit, self.integral_limit
        )

        # 计算微分项
        derivative = (error - self.prev_error) / dt if dt > 0 else np.zeros_like(error)

        # 计算 PID 输出
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # 更新前一误差
        self.prev_error = error

        return output


def default_pid_controller() -> PIDController:
    """创建并返回一个默认的 PIDController 实例。

    Returns:
        PIDController: 一个具有默认参数的 PIDController（kp=1.0, ki=0.0, kd=0.0, dead_zone=0.0, integral_limit=inf）。
    """
    return PIDController(
        kp=1.0,
        ki=0.0,
        kd=0.0,
        dead_zone=0.0,
        integral_limit=float("inf"),
    )
