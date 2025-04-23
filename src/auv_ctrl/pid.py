class PIDController:
    def __init__(self, kp, ki, kd, dead_zone=0.0, integral_limit=float('inf')):
        """
        初始化 PID 控制器。

        Args:
            kp (float): 比例增益。
            ki (float): 积分增益。
            kd (float): 微分增益。
            dead_zone (float): 死区范围，误差小于此值时输出为零。
            integral_limit (float): 积分上限，限制积分项的绝对值。
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dead_zone = dead_zone
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        """重置积分项和前一误差。"""
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measurement, dt):
        """
        计算 PID 控制器的输出。

        Args:
            setpoint (float): 目标值。
            measurement (float): 当前测量值。
            dt (float): 时间间隔。

        Returns:
            float: PID 控制器的输出。
        """
        # 计算误差
        error = setpoint - measurement

        # 应用死区
        if abs(error) < self.dead_zone:
            return 0.0

        # 计算积分项，限制积分上限
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        # 计算微分项
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        # 计算 PID 输出
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # 更新前一误差
        self.prev_error = error

        return output
