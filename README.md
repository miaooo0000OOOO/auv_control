# 六轴水下机器人的相关代码

## 简介

六轴水下机器人的控制代码，主控为RK3588，系统为Ubuntu+ROS2

## 项目结构

```
auv_control
├─assets 资产
│  └─UAV.SLDASM Solidworks导出的URDF文件
├─doc 文档
└─src 源代码
    ├─auv_ctrl 控制核心代码
    ├─pca9685_thruster_speed_control PCA9685芯片 I2C转PWM推进器驱动
    ├─wheeltec_n100 轮趣N100 IMU驱动
    └─pwm_control 推进器直连PWM驱动 ROS2包
```
