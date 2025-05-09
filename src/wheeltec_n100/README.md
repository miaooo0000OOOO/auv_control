# Wheeltec N100 模块

## 简介

`wheeltec_n100` 是一个用于接收和解析串口数据的 Python 模块，支持 IMU、AHRS、INSGPS 和地理位置等多种数据类型的解析。该模块适用于需要从串口设备读取传感器数据的场景。

## 功能

- 支持多种数据类型的解析，包括：
  - IMU 数据（陀螺仪和加速度计）
  - AHRS 数据（姿态和航向）
  - INSGPS 数据（惯性导航和 GPS 数据）
  - 地理位置数据（经纬度和高度）
- 实时打印解析后的数据。
- **通过共享内存实时发送角度和角速度数据**，供可视化程序使用。

## 依赖项

在使用该模块之前，请确保已安装以下 Python 库：

- `pyserial`
- `numpy`
- `matplotlib`

可以通过以下命令安装依赖项：

```bash
pip install pyserial numpy matplotlib
```

如果不需要可视化可以不安装 `matplotlib` 库

## 使用方法

### 数据接收与共享内存写入

1. 克隆或下载项目到本地。

2. 运行脚本：

```bash
python wheeltec_n100.py
```

  默认串口名为 `/dev/ttyUSB0` ，请确保该串口名无误。Windows上的串口名例如 `COM6`。

  要在 Linux 上成功打开串口，请确保当前用户在 `dialout` 用户组中。

  脚本会自动连接到指定的串口设备，解析数据并将角度和角速度数据实时写入共享内存文件 `/dev/shm/AHRS_DATA.npy`。

### udev 规则绑定 USB

将 `99-wheeltec-n100` 放入 `/etc/udev/rules.d/` 文件夹中，会在插入陀螺仪时自动创建软链接 `/dev/wheeltec_n100`。

`idProduct` 和 `idVendor` 可以使用 `udevadm info --name=/dev/ttyUSB0 --attribute-walk` 命令查看。

`MODE="0666"` 设置设备访问权限为所有用户可读写。

### 数据可视化

1. 确保 `wheeltec_n100.py` 正在运行并持续更新共享内存文件。

2. 运行可视化脚本 `data_vis.py`：

```bash
python data_vis.py
```

3. 可视化脚本会从共享内存文件 `/dev/shm/AHRS_DATA.npy` 中读取数据，并实时绘制角度和角速度的折线图。

## 注意事项

- 请确保指定的串口号正确且设备已连接。
- 如果脚本无法找到指定的串口号，会提示错误并退出。
- 共享内存文件路径为 `/dev/shm/AHRS_DATA.npy`，请确保系统支持 `/dev/shm`。

## 可视化示例

运行 `data_vis.py` 后，您将看到如下实时更新的折线图：

- 红色：`Roll` 角度（单位：度）
- 绿色：`Pitch` 角度（单位：度）
- 蓝色：`Yaw` 角度（单位：度）

同色虚线为对应的角速度。

折线图会动态更新最近 100 个数据点。
