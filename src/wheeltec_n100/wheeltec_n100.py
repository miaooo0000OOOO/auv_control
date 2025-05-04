#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import struct
from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE

# import transforms3d as tfs
import numpy as np
import os
import fcntl

# 宏定义参数
FRAME_HEAD = str("fc")
FRAME_END = str("fd")
TYPE_IMU = str("40")
TYPE_AHRS = str("41")
TYPE_INSGPS = str("42")
TYPE_GEODETIC_POS = str("5c")
TYPE_GROUND = str("f0")
TYPE_SYS_STATE = str("50")
TYPE_BODY_ACCELERATION = str("62")
TYPE_ACCELERATION = str("61")
TYPE_MSG_BODY_VEL = str("60")
IMU_LEN = str("38")  # //56
AHRS_LEN = str("30")  # //48
INSGPS_LEN = str("48")  # //72
GEODETIC_POS_LEN = str("20")  # //32
SYS_STATE_LEN = str("64")  # // 100
BODY_ACCELERATION_LEN = str("10")  # // 16
ACCELERATION_LEN = str("0c")  # 12
PI = 3.141592653589793
DEG_TO_RAD = 0.017453292519943295
isrun = True

# 定义共享内存文件路径
AHRS_SHM_FILE = "/dev/shm/AHRS_DATA.npy"
AHRS_SHM_SIZE = 6  # 6个浮点数

# 确保共享内存文件存在并初始化
if not os.path.exists(AHRS_SHM_FILE):
    np.save(AHRS_SHM_FILE, np.zeros(AHRS_SHM_SIZE, dtype=np.float32))  # 初始化为 0


def receive_data(ser: serial.Serial):
    """
    接收数据的函数。
    将保存到SHM_FILE中，包含roll、pitch、yaw、roll_speed、pitch_speed、yaw_speed。
    读取串口数据并解析IMU、AHRS、INSGPS、GPS等数据类型。
    Args:
        ser (serial.Serial): 串口对象，用于接收数据。
    """

    # 循环读取数据
    if not ser.isOpen():
        return

    check_head = ser.read().hex()
    # 校验帧头
    if check_head != FRAME_HEAD:
        return
    head_type = ser.read().hex()
    # 校验数据类型
    if head_type not in {
        TYPE_IMU,
        TYPE_AHRS,
        TYPE_INSGPS,
        TYPE_GEODETIC_POS,
        0x50,
        TYPE_GROUND,
    }:
        return
    check_len = ser.read().hex()
    # 校验数据类型的长度
    if head_type == TYPE_IMU and check_len != IMU_LEN:
        return
    elif head_type == TYPE_AHRS and check_len != AHRS_LEN:
        return
    elif head_type == TYPE_INSGPS and check_len != INSGPS_LEN:
        return
    elif head_type == TYPE_GEODETIC_POS and check_len != GEODETIC_POS_LEN:
        return
    elif head_type == TYPE_SYS_STATE and check_len != SYS_STATE_LEN:
        return
    elif head_type == TYPE_GROUND or head_type == 0x50:
        return
    elif head_type == TYPE_MSG_BODY_VEL and check_len != ACCELERATION_LEN:
        print(
            "check head type "
            + str(TYPE_MSG_BODY_VEL)
            + " failed;"
            + " check_LEN:"
            + str(check_len)
        )
        return
    elif head_type == TYPE_BODY_ACCELERATION and check_len != BODY_ACCELERATION_LEN:
        print(
            "check head type "
            + str(TYPE_BODY_ACCELERATION)
            + " failed;"
            + " check_LEN:"
            + str(check_len)
        )
        return
    elif head_type == TYPE_ACCELERATION and check_len != ACCELERATION_LEN:
        print(
            "check head type "
            + str(TYPE_ACCELERATION)
            + " failed;"
            + " ckeck_LEN:"
            + str(check_len)
        )
        return
    # check_sn = serial_.read().hex()
    # head_crc8 = serial_.read().hex()
    # crc16_H_s = serial_.read().hex()
    # crc16_L_s = serial_.read().hex()

    # 读取并解析IMU数据
    if head_type == TYPE_IMU:
        data_s = ser.read(int(IMU_LEN, 16))
        IMU_DATA = struct.unpack("12f ii", data_s[0:56])
        # print(IMU_DATA)
        # print("Gyroscope_X(rad/s): " + str(IMU_DATA[0]))
        # print("Gyroscope_Y(rad/s) : " + str(IMU_DATA[1]))
        # print("Gyroscope_Z(rad/s) : " + str(IMU_DATA[2]))
        # print("Accelerometer_X(m/s^2) : " + str(IMU_DATA[3]))
        # print("Accelerometer_Y(m/s^2) : " + str(IMU_DATA[4]))
        # print("Accelerometer_Z(m/s^2) : " + str(IMU_DATA[5]))
        # print("Magnetometer_X(mG) : " + str(IMU_DATA[6]))
        # print("Magnetometer_Y(mG) : " + str(IMU_DATA[7]))
        # print("Magnetometer_Z(mG) : " + str(IMU_DATA[8]))
        # print("IMU_Temperature : " + str(IMU_DATA[9]))
        # print("Pressure : " + str(IMU_DATA[10]))
        # print("Pressure_Temperature : " + str(IMU_DATA[11]))
        # print("Timestamp(us) : " + str(IMU_DATA[12]))
    # 读取并解析AHRS数据
    elif head_type == TYPE_AHRS:
        data_s = ser.read(int(AHRS_LEN, 16))
        AHRS_DATA = struct.unpack("10f ii", data_s[0:48])
        # print(AHRS_DATA)
        # print("RollSpeed(rad/s): " + str(AHRS_DATA[0]))
        # print("PitchSpeed(rad/s) : " + str(AHRS_DATA[1]))
        # print("HeadingSpeed(rad) : " + str(AHRS_DATA[2]))
        # print("Roll(rad) : " + str(AHRS_DATA[3]))
        # print("Pitch(rad) : " + str(AHRS_DATA[4]))
        # print("Heading(rad) : " + str(AHRS_DATA[5]))
        roll = AHRS_DATA[4]
        pitch = AHRS_DATA[5]
        yaw = AHRS_DATA[6]

        # print(f"Roll(deg): {roll}, Pitch(deg): {pitch}, Heading(deg): {yaw}")

        roll_speed = AHRS_DATA[1]
        pitch_speed = AHRS_DATA[2]
        yaw_speed = AHRS_DATA[3]
        # print(f"RollSpeed(deg/s): {roll_speed}, PitchSpeed(deg/s): {pitch_speed}, HeadingSpeed(deg/s): {heading_speed}")

        # 写入共享内存文件
        with open(AHRS_SHM_FILE, "wb") as f:
            fcntl.flock(f, fcntl.LOCK_EX)  # 加写锁
            np.save(
                f,
                np.array(
                    [roll, pitch, yaw, roll_speed, pitch_speed, yaw_speed],
                    dtype=np.float32,
                ),
            )
            fcntl.flock(f, fcntl.LOCK_UN)  # 释放锁
        # Q1W=struct.unpack('f', data_s[24:28])[0]
        # Q2X=struct.unpack('f', data_s[28:32])[0]
        # Q3Y=struct.unpack('f', data_s[32:36])[0]
        # Q4Z=struct.unpack('f', data_s[36:40])[0]
        # Q2EULER=tfs.euler.quat2euler([Q1W,Q2X,Q3Y,Q4Z],"sxyz") #使用前需要安装依赖pip install transforms3d -i https://pypi.tuna.tsinghua.edu.cn/simple
        # euler=[0,1,2]
        # euler[0]=Q2EULER[0]*360/2/PI
        # euler[1]=Q2EULER[1]*360/2/PI
        # euler[2]=Q2EULER[2]*360/2/PI
        # print("euler_x: "+str(euler[0]))
        # print("euler_y: "+str(euler[1]))
        # print("euler_z: "+str(euler[2]))
        # print("Q1 : " + str(AHRS_DATA[6]))
        # print("Q2 : " + str(AHRS_DATA[7]))
        # print("Q3 : " + str(AHRS_DATA[8]))
        # print("Q4 : " + str(AHRS_DATA[9]))
        # print("Timestamp(us) : " + str(AHRS_DATA[10]))
    # 读取并解析INSGPS数据
    elif head_type == TYPE_INSGPS:
        data_s = ser.read(int(INSGPS_LEN, 16))
        INSGPS_DATA = struct.unpack("16f ii", data_s[0:72])
        # print(INSGPS_DATA)
        # print("BodyVelocity_X:(m/s)" + str(INSGPS_DATA[0]))
        # print("BodyVelocity_Y:(m/s)" + str(INSGPS_DATA[1]))
        # print("BodyVelocity_Z:(m/s)" + str(INSGPS_DATA[2]))

        # print("BodyAcceleration_X:(m/s^2)" + str(INSGPS_DATA[3]))
        # print("BodyAcceleration_Y:(m/s^2)" + str(INSGPS_DATA[4]))
        # print("BodyAcceleration_Z:(m/s^2)" + str(INSGPS_DATA[5]))
        # print("Location_North:(m)" + str(INSGPS_DATA[6]))
        # print("Location_East:(m)" + str(INSGPS_DATA[7]))
        # print("Location_Down:(m)" + str(INSGPS_DATA[8]))
        # print("Velocity_North:(m)" + str(INSGPS_DATA[9]))
        # print("Velocity_East:(m/s)" + str(INSGPS_DATA[10]))
        # print("Velocity_Down:(m/s)" + str(INSGPS_DATA[11]))
        # print("Acceleration_North:(m/s^2)" + str(INSGPS_DATA[12]))
        # print("Acceleration_East:(m/s^2)" + str(INSGPS_DATA[13]))
        # print("Acceleration_Down:(m/s^2)" + str(INSGPS_DATA[14]))
        # print("Pressure_Altitude:(m)" + str(INSGPS_DATA[15]))
        # print("Timestamp:(us)" + str(INSGPS_DATA[16]))
    # 读取并解析GPS数据
    elif head_type == TYPE_GEODETIC_POS:
        data_s = ser.read(int(GEODETIC_POS_LEN, 16))
        # print(" Latitude:(rad)" + str(struct.unpack('d', data_s[0:8])[0]))
        # print("Longitude:(rad)" + str(struct.unpack('d', data_s[8:16])[0]))
        # print("Height:(m)" + str(struct.unpack('d', data_s[16:24])[0]))
    elif head_type == TYPE_SYS_STATE:
        data_s = ser.read(int(SYS_STATE_LEN, 16))
        # print("Unix_time:" + str(struct.unpack('i', data_s[4:8])[0]))
        # print("Microseconds:" + str(struct.unpack('i', data_s[8:12])[0]))
        # print(" System_status:" + str(struct.unpack('d', data_s[0:2])[0]))
        # print("System_Z(m/s^2): " + str(struct.unpack('f', data_s[56:60])[0]))
    elif head_type == TYPE_BODY_ACCELERATION:
        data_s = ser.read(int(BODY_ACCELERATION_LEN, 16))
        # print(" System_status:" + str(struct.unpack('d', data_s[0:2])[0]))
        # print("BodyAcceleration_Z(m/s^2): " + str(struct.unpack('f', data_s[8:12])[0]))
    elif head_type == TYPE_ACCELERATION:
        data_s = ser.read(int(ACCELERATION_LEN, 16))
        # print(" System_status:" + str(struct.unpack('d', data_s[0:2])[0]))
        # print("Acceleration_Z(m/s^2): " + str(struct.unpack('f', data_s[8:12])[0]))
    elif head_type == TYPE_MSG_BODY_VEL:
        data_s = ser.read(int(ACCELERATION_LEN, 16))
        # print(" System_status:" + str(struct.unpack('d', data_s[0:2])[0]))
        # Velocity_X = struct.unpack('f', data_s[0:4])[0]   # 解析第一个双精度浮点数
        # Velocity_Y = struct.unpack('f', data_s[4:8])[0]  # 解析第二个双精度浮点数
        # Velocity_Z = struct.unpack('f', data_s[8:12])[0] # 解析第三个双精度浮点数
        # print(f"Velocity_X: {Velocity_X}, Velocity_Y: {Velocity_Y}, Velocity_Z: {Velocity_Z}")
        # print("Velocity_X(m/s): " + str(struct.unpack('f', data_s[0:4])[0]))
        # print("Velocity_Y(m/s): " + str(struct.unpack('f', data_s[4:8])[0]))
        # print("Velocity_Z(m/s): " + str(struct.unpack('f', data_s[8:12])[0]))


if __name__ == "__main__":
    ser = serial.Serial(
        port="/dev/ttyUSB0",  # windows: COM6
        baudrate=921600,
        bytesize=EIGHTBITS,
        parity=PARITY_NONE,
        stopbits=STOPBITS_ONE,
        timeout=20,
    )
    while True:
        receive_data(ser)
