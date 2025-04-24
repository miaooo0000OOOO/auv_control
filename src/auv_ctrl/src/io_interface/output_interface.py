import numpy as np
from typing import Callable

import system_params.system_params_symbol as sps

# 发送推力指令的函数
send_thrust_cmd: Callable[[np.ndarray], None] = None

def default_send_thrust_cmd(thrust: np.ndarray) -> None:
    """
    默认的设置推进器推力指令的实现。

    参数:
        thrust (np.ndarray): 包含6个元素的数组，表示每个推进器的推力指令。
    """
    # 将推力值限制在允许范围内
    thrust = np.clip(thrust, -sps.F_n_max, sps.F_p_max)
    
    # 假设这里是将推力指令发送到推进器的地方
    print("设置推力指令:", thrust)

# 初始化为默认实现
send_thrust_cmd = default_send_thrust_cmd

# TODO 根据实际硬件接口重写

# # 示例：如果需要重写，可以直接替换 send_thrust_cmd
# def custom_send_thrust_cmd(thrust: np.ndarray) -> None:
#     print("自定义推力指令:", thrust)

# # 替换为自定义实现
# send_thrust_cmd = custom_send_thrust_cmd