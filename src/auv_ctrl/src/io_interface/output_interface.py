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
    print(f"原始推力: {thrust.round(2)}")

    # 将推力映射为-100%~100%的范围
    thrust = thrust / sps.F_p_max * 100
    thrust = np.clip(thrust, -100, 100)  # 限制在-100%到100%之间

    print(
        "推力指令: {:.2f}%, {:.2f}%, {:.2f}%, {:.2f}%, {:.2f}%, {:.2f}%".format(
            thrust[0], thrust[1], thrust[2], thrust[3], thrust[4], thrust[5]
        )
    )


# 初始化为默认实现
send_thrust_cmd = default_send_thrust_cmd

# TODO 根据实际硬件接口重写

# # 示例：如果需要重写，可以直接替换 send_thrust_cmd
# def custom_send_thrust_cmd(thrust: np.ndarray) -> None:
#     print("自定义推力指令:", thrust)

# # 替换为自定义实现
# send_thrust_cmd = custom_send_thrust_cmd
