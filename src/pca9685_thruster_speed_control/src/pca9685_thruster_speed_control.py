import ctypes
from ctypes import c_int, c_float, c_char_p

# 加载共享库
# Linux: libpca9685.so
pca9685 = ctypes.CDLL('./libpca9685.so')

# 定义函数原型
pca9685.pca9685_open.argtypes = [c_char_p, c_int]
pca9685.pca9685_open.restype = c_int

pca9685.pca9685_close.argtypes = []
pca9685.pca9685_close.restype = None

pca9685.set_thruster_speed.argtypes = [c_int, c_float]
pca9685.set_thruster_speed.restype = c_int

# 示例：调用函数
if __name__ == "__main__":
    i2c_bus = b"/dev/i2c-4"  # I2C 总线路径
    addr = 0x40  # PCA9685 I2C 地址

    # 打开设备
    if pca9685.pca9685_open(i2c_bus, addr) < 0:
        print("Failed to open PCA9685 device")
        return -1
    print("PCA9685 device opened successfully")

    # 设置推进器速度
    thruster_index = 1
    speed = 50.0  # 50%
    if pca9685.set_thruster_speed(thruster_index, speed) < 0:
        print(f"Failed to set speed for thruster {thruster_index}")
    else:
        print(f"Thruster {thruster_index} speed set to {speed}%")

    # 关闭设备
    pca9685.pca9685_close()