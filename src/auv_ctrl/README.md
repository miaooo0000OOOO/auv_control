# AUV Control

本项目是一个用于自主水下机器人 (AUV) 的仿真与控制系统，包含推进器推力分配、PID 控制器、传感器接口以及摇杆输入的实现。

## 功能概述

- **推进器推力分配**：根据期望的力和力矩，计算 6 个推进器的推力分配。
- **PID 控制器**：实现带有死区和积分上限的 PID 控制器，用于深度、速度和姿态控制。
- **传感器接口**：提供深度计、陀螺仪和摇杆输入的模拟接口。
- **摇杆控制**：通过摇杆输入直接控制 AUV 的速度和角速度。
- **浮力与重力补偿**：计算浮力和重力对 AUV 的影响，并在推力分配中进行补偿。

## 文件结构

```
.   
├── control.py # 推进器推力分配与浮力/重力补偿逻辑 
├── pid.py # PID 控制器实现 
├── input_interface.py # 传感器与摇杆输入接口 
├── output_interface.py # 推力指令输出接口 
├── auv_controller.py # 主控制器逻辑 
├── system_params_symbol.py # 系统参数定义 
└── README.md # 项目说明文档
```

## 安装与运行

### 环境要求

- Python 3.8 或更高版本
- 依赖库：
  - `numpy`
  - `scipy`

### 安装依赖

使用以下命令安装所需依赖：

```bash
pip install numpy scipy
```

### 运行控制器

运行主控制器脚本：

```bash
python auv_controller.py
```

## 使用说明

### 主要模块
1. 推进器推力分配 (control.py)：

   - thrust_allocation(desired_wrench, orientation)：根据期望的力和力矩分配推进器推力。

2. PID 控制器 (pid.py)：

   - PIDController：实现比例、积分、微分控制，支持死区和积分上限。

3. 传感器接口 (input_interface.py)：

    - get_depth()：获取当前深度。
    - get_orientation()：获取当前姿态（滚转、俯仰、偏航）。
    - get_joystick()：获取摇杆输入。

4. 推力指令输出 (output_interface.py)：

   - send_thrust_cmd(thrust)：发送推进器推力指令。

5. 主控制器 (auv_controller.py)：

   - 通过摇杆输入控制 AUV 的速度和角速度，结合 PID 控制和推力分配实现闭环控制。


### 自定义实现
自定义传感器接口
可以通过重写 `input_interface.py` 中的默认方法来自定义传感器输入。例如：

```python
def custom_get_depth():
    return 5.0  # 返回固定深度值

iif.get_depth = custom_get_depth
```

## 进一步开发

查找项目内的`TODO`字样，