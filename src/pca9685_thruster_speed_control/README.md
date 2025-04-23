# PCA9685 Thruster Speed Control

## 项目概述
该项目实现了对PCA9685 PWM驱动芯片的控制，主要用于推进器的速度控制。通过I2C接口与PCA9685通信，能够设置推进器的PWM输出，实现精确的速度调节。

此外，项目支持通过 Python 脚本调用 C 函数，方便用户在 Python 环境中控制推进器。

## 文件结构
```
pca9685_thruster_speed_control
├── src
│   ├── pca9685_thruster_speed_control.c  # 实现PCA9685推进器控制的主要功能
│   ├── pca9685_thruster_speed_control.h  # 声明与PCA9685相关的函数和常量
│   ├── pca9685_thruster_speed_control.py # Python FFI 示例代码
├── Makefile                               # 定义项目的构建规则，支持调试、发布和共享库构建
└── README.md                              # 项目的文档和使用说明
```

## 使用说明

### 编译项目
项目使用`Makefile`进行构建，支持调试、发布和共享库三种模式。

- **调试构建**: 
  使用以下命令进行调试构建：
  ```
  make debug
  ```

- **发布构建**: 
  使用以下命令进行发布构建：
  ```
  make release
  ```

- **生成共享库**: 
  使用以下命令生成共享库（供 Python 调用）：
  ```
  make shared
  ```

### 清理构建文件
要清理生成的对象文件、可执行文件和共享库，可以使用：
```
make clean
```

## 依赖
- GCC编译器
- Linux环境下的I2C支持
- Python 3.x（用于 Python FFI 示例）

## 运行程序

### 使用 C 程序
编译完成后，可以运行生成的可执行文件进行推进器控制。根据实际I2C总线和设备地址进行相应的修改。

### 使用 Python 调用共享库
1. 确保已生成共享库（`libpca9685.so`）。
2. 使用 Python 脚本调用共享库中的函数。示例代码位于 `src/pca9685_thruster_speed_control.py`。

运行 Python 示例：
```bash
python3 src/pca9685_thruster_speed_control.py
```

### 示例输出
以下是运行 Python 示例的输出：
```
PCA9685 device opened successfully
Thruster 1 speed set to 50.0%
```

## 注意事项
- 如果运行在 Linux 上，访问 I2C 总线可能需要超级用户权限，运行脚本时需要使用 `sudo`。
- 确保共享库文件（`libpca9685.so`）与 Python 脚本在同一目录，或者提供绝对路径。