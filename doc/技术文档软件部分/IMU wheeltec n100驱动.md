# 2.3 软件系统介绍
## 2.3.1 IMU模块设计说明

### 模块功能
1. 通过串口接收Wheeltec N100传感器的原始数据
2. 解析多种数据帧类型（IMU/AHRS/INSGPS等）
3. 通过共享内存实时传输姿态数据
4. 提供数据可视化接口

### 关键特性
- 支持921600bps高速串口通信
- 6自由度姿态数据输出（Roll/Pitch/Yaw及角速度）
- 自动设备绑定（udev规则）

## 2.3.2 模块分层设计

### 顶层架构
```mermaid
graph TD
    subgraph 应用层
        Data_Visualization[数据可视化]
        Control_System[控制系统]
    end
    
    subgraph 服务层
        Shared_Memory[共享内存服务]
        Serial_Parser[串口解析]
    end
    
    subgraph 驱动层
        USB_UART[USB转串口驱动]
        Sensor_Fusion[传感器融合算法]
    end
    
    Data_Visualization --> Shared_Memory
    Control_System --> Shared_Memory
    Shared_Memory --> Serial_Parser
    Serial_Parser --> USB_UART
    USB_UART --> Wheeltec_N100
```

### 核心函数流程图

#### 1. 数据接收流程
```mermaid
graph TD
    A[串口数据] --> B{帧头校验?}
    B -->|FC| C[读取类型字节]
    B -->|其他| Z[丢弃]
    C --> D{类型校验?}
    D -->|40h IMU| E[读取56字节]
    D -->|41h AHRS| F[读取48字节]
    D -->|42h INSGPS| G[读取72字节]
    E --> H[解析IMU数据]
    F --> I[解析AHRS数据]
    G --> J[解析INSGPS数据]
    H --> K[更新共享内存]
    I --> K
    J --> K
```

关键参数：
```python
输入：
  串口数据流（hex格式）
输出：
  AHRS_DATA.npy:
    [roll, pitch, yaw, roll_speed, pitch_speed, yaw_speed]
```

#### 2. 共享内存更新流程
```mermaid
graph TD
    A[获取新数据] --> B[申请写入锁]
    B --> C[创建numpy数组]
    C --> D[写入共享内存]
    D --> E[释放锁]
    E --> F[返回成功状态]
```

#### 3. 数据可视化流程
```mermaid
graph TD
    A[启动动画] --> B[定时器触发]
    B --> C[读取共享内存]
    C --> D{数据有效?}
    D -->|是| E[更新曲线]
    D -->|否| F[保持显示]
    E --> G[重绘界面]
```

### 关键数据结构

#### 1. 数据帧格式
| 类型 | 长度 | 内容 |
|------|------|------|
| IMU(40h) | 56字节 | 12个float + 2个int32 |
| AHRS(41h) | 48字节 | 10个float + 2个int32 |
| INSGPS(42h) | 72字节 | 16个float + 2个int32 |

#### 2. 共享内存结构
```python
np.array([
    roll,          # 横滚角(rad)
    pitch,         # 俯仰角(rad) 
    yaw,           # 偏航角(rad)
    roll_speed,    # 横滚角速度(rad/s)
    pitch_speed,   # 俯仰角速度(rad/s)
    yaw_speed      # 偏航角速度(rad/s)
], dtype=np.float32)
```

### 错误处理机制
| 错误类型 | 处理方式 |
|----------|----------|
| 串口断开 | 自动重连机制 |
| 数据校验失败 | 丢弃当前帧 |
| 共享内存冲突 | 文件锁保护 |
| 数据超范围 | 自动归一化处理 |