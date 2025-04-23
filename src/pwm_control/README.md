# ROS2 PWM 控制软件包

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-9C27B0)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue)](LICENSE)

用于嵌入式Linux系统的PWM控制ROS2软件包，支持通过ROS话题实现转速-PWM占空比的线性映射。

## 功能特性

- 📡 订阅 `/set_rpm` 话题接收0-100%转速指令
- ⚡ 实时PWM控制（50Hz频率）
- 🎚️ 支持5%-10%占空比线性映射
- 🔄 自动处理PWM极性配置（支持normal/inversed）
- 🛠️ 完善的错误检测和日志系统

## 硬件要求

- Linux内核版本 ≥ 4.4（支持PWM sysfs接口）
- PWM芯片路径：`/sys/class/pwm/pwmchip0`
- 用户权限：需具备PWM设备读写权限

## 安装指南

### 依赖安装
```bash
sudo apt install ros-humble-rclcpp ros-humble-std-msgs
```

### 软件包安装
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-repo/pwm_control.git
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select pwm_control
```

## 使用方法

### 1. 准备PWM通道
```bash
# 导出PWM通道（需root权限）
sudo su
echo 0 > /sys/class/pwm/pwmchip0/export
exit

# 设置权限（推荐配置udev规则）
sudo chmod 666 /sys/class/pwm/pwmchip0/pwm0/*
```

### 2. 运行控制节点
```bash
source install/setup.bash
ros2 run pwm_control pwm_controller
```

### 3. 发送控制指令
```bash
# 设置50%转速
ros2 topic pub /set_rpm std_msgs/msg/Float32 "{data: 50.0}"

# 停止电机
ros2 topic pub /set_rpm std_msgs/msg/Float32 "{data: 0.0}"
```

## 配置说明

### 参数调整
在 `pwm_controller.cpp` 中可修改：
```cpp
// 占空比范围（单位：ns）
const int DUTY_MIN = 1000000;  // 5% (1ms)
const int DUTY_MAX = 2000000;  // 10% (2ms)

// PWM频率设置
const int PWM_PERIOD = 20000000; // 50Hz (20ms)
```

### 极性配置
若需要强制设置极性，修改初始化函数：
```cpp
write_sysfs("/sys/class/pwm/pwmchip0/pwm0/polarity", "normal");
```

## 调试工具

### 实时监控PWM状态
```bash
watch -n 0.5 'cat /sys/class/pwm/pwmchip0/pwm0/{duty_cycle,period,enable,polarity}'
```

### PWM手动测试（不启动ROS）
```bash
# 设置周期
echo 20000000 | sudo tee /sys/class/pwm/pwmchip0/pwm0/period

# 测试占空比
echo 1500000 | sudo tee /sys/class/pwm/pwmchip0/pwm0/duty_cycle
```

## 故障排查

### 常见问题
| 现象 | 解决方案 |
|------|----------|
| `Permission denied` | 执行权限配置命令或使用`sudo`运行节点 |
| PWM无输出 | 检查`enable`状态是否为1 |
| 极性不匹配 | 确保在设置周期前配置极性 |
| 响应延迟 | 检查ROS2话题发布频率 |


