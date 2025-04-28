import numpy as np

# 将上级目录添加到路径中
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from auv_controller import AUVController

# 初始化控制器
controller = AUVController()

# 测试不同的期望角速度和线速度
test_cases = [
    {"joystick_input": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "description": "静止"},
    {"joystick_input": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], "description": "前进"},
    {"joystick_input": [0.0, 1.0, 0.0, 0.0, 0.0, 0.0], "description": "侧移"},
    {"joystick_input": [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "description": "上升"},
    {"joystick_input": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0], "description": "滚转"},
    {"joystick_input": [0.0, 0.0, 0.0, 0.0, 1.0, 0.0], "description": "俯仰"},
    {"joystick_input": [0.0, 0.0, 0.0, 0.0, 0.0, 1.0], "description": "偏航"},
    {"joystick_input": [1.0, 1.0, 0.0, 0.5, 0.5, 0.0], "description": "复杂运动"},
]


def to_func(arr):
    """
    将数组转换为函数
    """

    def func():
        return np.array(arr)

    return func


# 模拟测试
for case in test_cases:
    print(f"\n测试用例: {case['description']}")
    controller.get_joystick = to_func(case["joystick_input"])
    controller.get_depth = to_func(0.0)
    controller.get_orientation = to_func(np.array([0.0, 0.0, 0.0]))
    controller.compute_control(dt=0)  # dt = 0时，PID控制器只保留P项
