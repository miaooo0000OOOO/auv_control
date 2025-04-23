import argparse
import serial
import serial.tools.list_ports
import threading
import struct
from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE
from time import sleep


# 宏定义参数
FRAME_HEAD = str("fc")
FRAME_END = str("fd")

TYPE_IMU = str("40")
TYPE_AHRS = str("41")
TYPE_INSGPS = str("42")
TYPE_GEODETIC_POS = str("5c")
TYPE_GROUND = str("f0")

IMU_LEN = str("38")  # 56
AHRS_LEN = str("30")  # 48
INSGPS_LEN = str("48")  # 80
GEODETIC_POS_LEN = str("20")  # 32
PI = 3.141592653589793
DEG_TO_RAD = 0.017453292519943295


def parse_opt(known=False):
    """
    解析脚本的命令行选项。

    Args:
        known (bool, optional): 如果为 True，则仅解析已知参数。默认为 False。

    Returns:
        argparse.Namespace: 解析后的命令行参数。
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=str, default="com6", help="接收数据的串口号。")
    parser.add_argument("--bps", type=int, default=921600, help="串口的波特率。")
    parser.add_argument(
        "--timeout", type=int, default=20, help="串口的超时时间（秒）。"
    )

    receive_params = parser.parse_known_args()[0] if known else parser.parse_args()
    return receive_params


def try_find_serial():
    """
    尝试查找串口并打印结果。
    """
    if find_serial():
        print("find this port : " + opt.port)
    else:
        print("error:  unable to find this port : " + opt.port)
        exit(1)


def receive_data():
    """
    接收数据的线程函数。
    该函数尝试打开指定的串口，并循环读取数据。根据数据类型解析并打印IMU、AHRS、INSGPS和地理位置数据。
    """
    try_find_serial()
    # 尝试打开串口
    try:
        serial_ = serial.Serial(
            port=opt.port,
            baudrate=opt.bps,
            bytesize=EIGHTBITS,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            timeout=opt.timeout,
        )
    except:
        print("error:  unable to open port .")
        exit(1)
    print("baud rates = {}" + str(serial_.baudrate))
    # 循环读取数据
    while serial_.isOpen() and tr.is_alive():
        check_head = serial_.read().hex()
        # 校验帧头
        if check_head != FRAME_HEAD:
            continue
        head_type = serial_.read().hex()
        # 校验数据类型
        if (
            head_type != TYPE_IMU
            and head_type != TYPE_AHRS
            and head_type != TYPE_INSGPS
            and head_type != TYPE_GEODETIC_POS
            and head_type != 0x50
            and head_type != TYPE_GROUND
        ):
            continue
        check_len = serial_.read().hex()
        # 校验数据类型的长度
        if head_type == TYPE_IMU and check_len != IMU_LEN:
            continue
        elif head_type == TYPE_AHRS and check_len != AHRS_LEN:
            continue
        elif head_type == TYPE_INSGPS and check_len != INSGPS_LEN:
            continue
        elif head_type == TYPE_GEODETIC_POS and check_len != GEODETIC_POS_LEN:
            continue
        elif head_type == TYPE_GROUND or head_type == 0x50:
            continue
        # check_sn = serial_.read().hex()
        # head_crc8 = serial_.read().hex()
        # crc16_H_s = serial_.read().hex()
        # crc16_L_s = serial_.read().hex()

        # 读取并解析IMU数据
        if head_type == TYPE_IMU:
            data_s = serial_.read(int(IMU_LEN, 16))
            print("Gyroscope_X(rad/s): " + str(struct.unpack("f", data_s[0:4])[0]))
            print("Gyroscope_Y(rad/s) : " + str(struct.unpack("f", data_s[4:8])[0]))
            print("Gyroscope_Z(rad/s) : " + str(struct.unpack("f", data_s[8:12])[0]))
            print(
                "Accelerometer_X(m/s^2) : " + str(struct.unpack("f", data_s[12:16])[0])
            )
            print(
                "Accelerometer_Y(m/s^2) : " + str(struct.unpack("f", data_s[16:20])[0])
            )
            print(
                "Accelerometer_Z(m/s^2) : " + str(struct.unpack("f", data_s[20:24])[0])
            )
        # 读取并解析AHRS数据
        elif head_type == TYPE_AHRS:
            data_s = serial_.read(int(AHRS_LEN, 16))
            print("RollSpeed(rad/s): " + str(struct.unpack("f", data_s[0:4])[0]))
            print("PitchSpeed(rad/s) : " + str(struct.unpack("f", data_s[4:8])[0]))
            print("HeadingSpeed(rad) : " + str(struct.unpack("f", data_s[8:12])[0]))
            print("Roll(rad) : " + str(struct.unpack("f", data_s[12:16])[0]))
            print("Pitch(rad) : " + str(struct.unpack("f", data_s[16:20])[0]))
            print("Heading(rad) : " + str(struct.unpack("f", data_s[20:24])[0]))
            print("Q1 : " + str(struct.unpack("f", data_s[24:28])[0]))
            print("Q2 : " + str(struct.unpack("f", data_s[28:32])[0]))
            print("Q3 : " + str(struct.unpack("f", data_s[32:36])[0]))
            print("Q4 : " + str(struct.unpack("f", data_s[36:40])[0]))
            # print("Timestamp(us) : " + str(struct.unpack('ii', data_s[40:48])[0]))
        # 读取并解析INSGPS数据
        elif head_type == TYPE_INSGPS:
            data_s = serial_.read(int(INSGPS_LEN, 16))
            print("BodyVelocity_X:(m/s)" + str(struct.unpack("f", data_s[0:4])[0]))
            print("BodyVelocity_Y:(m/s)" + str(struct.unpack("f", data_s[4:8])[0]))
            print("BodyVelocity_Z:(m/s)" + str(struct.unpack("f", data_s[8:12])[0]))
            print(
                "BodyAcceleration_X:(m/s^2)" + str(struct.unpack("f", data_s[12:16])[0])
            )
            print(
                "BodyAcceleration_Y:(m/s^2)" + str(struct.unpack("f", data_s[16:20])[0])
            )
            print(
                "BodyAcceleration_Z:(m/s^2)" + str(struct.unpack("f", data_s[20:24])[0])
            )
            print("Location_North:(m)" + str(struct.unpack("f", data_s[24:28])[0]))
            print("Location_East:(m)" + str(struct.unpack("f", data_s[28:32])[0]))
            print("Location_Down:(m)" + str(struct.unpack("f", data_s[32:36])[0]))
            print("Velocity_North:(m)" + str(struct.unpack("f", data_s[36:40])[0]))
            print("Velocity_East:(m/s)" + str(struct.unpack("f", data_s[40:44])[0]))
            print("Velocity_Down:(m/s)" + str(struct.unpack("f", data_s[44:48])[0]))
        elif head_type == TYPE_GEODETIC_POS:
            data_s = serial_.read(int(GEODETIC_POS_LEN, 16))
            print(" Latitude:(rad)" + str(struct.unpack("d", data_s[0:8])[0]))
            print("Longitude:(rad)" + str(struct.unpack("d", data_s[8:16])[0]))
            print("Height:(m)" + str(struct.unpack("d", data_s[16:24])[0]))
        print("--------------------")
        sleep(0.1)


# 寻找输入的port串口
def find_serial():
    """
    寻找输入的port串口

    Returns:
        bool: 如果找到指定的串口，则返回 True；否则返回 False。
    """
    port_list = list(serial.tools.list_ports.comports())
    for port in port_list:
        if port.name.lower() == opt.port.lower():
            return True
    return False


if __name__ == "__main__":
    opt = parse_opt()
    tr = threading.Thread(target=receive_data)
    tr.daemon = True  # 设置为守护线程
    tr.start()
    while tr.is_alive():
        try:
            sleep(1)  # 主线程保持活跃
        except KeyboardInterrupt:
            print("程序被中断，退出...")
            break
