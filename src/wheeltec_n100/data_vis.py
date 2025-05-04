import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import fcntl

# 读取共享内存文件路径
AHRS_SHM_FILE = "/dev/shm/AHRS_DATA.npy"

# 初始化数据存储
rolls, pitches, headings = [], [], []
roll_speeds, pitch_speeds, heading_speeds = [], [], []

# 创建图形和子图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))

# 子图1：角度
(line_roll,) = ax1.plot([], [], label="Roll (deg)", color="r")
(line_pitch,) = ax1.plot([], [], label="Pitch (deg)", color="g")
(line_heading,) = ax1.plot([], [], label="Heading (deg)", color="b")
ax1.set_xlim(0, 100)  # 显示最近100个数据点
ax1.set_ylim(-360, 360)  # 根据角度范围设置Y轴
ax1.set_title("Real-Time Roll, Pitch, and Heading")
ax1.set_xlabel("Time (arbitrary units)")
ax1.set_ylabel("Angle (degrees)")
ax1.legend()
ax1.grid()

# 子图2：角速度
(line_roll_speed,) = ax2.plot(
    [], [], label="Roll Speed (deg/s)", color="r", linestyle="--"
)
(line_pitch_speed,) = ax2.plot(
    [], [], label="Pitch Speed (deg/s)", color="g", linestyle="--"
)
(line_heading_speed,) = ax2.plot(
    [], [], label="Heading Speed (deg/s)", color="b", linestyle="--"
)
ax2.set_xlim(0, 100)
ax2.set_ylim(-360, 360)
ax2.set_title("Real-Time Roll, Pitch, and Heading Speeds")
ax2.set_xlabel("Time (arbitrary units)")
ax2.set_ylabel("Angular Speed (deg/s)")
ax2.legend()
ax2.grid()


# 更新函数
def update(frame):
    global rolls, pitches, headings, roll_speeds, pitch_speeds, heading_speeds

    try:
        # 读取 AHRS 数据
        with open(AHRS_SHM_FILE, "rb") as f:
            fcntl.flock(f, fcntl.LOCK_SH)
            ahrs_data = np.load(f)
            fcntl.flock(f, fcntl.LOCK_UN)

        # 更新角度和角速度数据
        rolls.append(ahrs_data[0] / np.pi * 180)
        pitches.append(ahrs_data[1] / np.pi * 180)
        headings.append(ahrs_data[2] / np.pi * 180)
        roll_speeds.append(ahrs_data[3] / np.pi * 180)
        pitch_speeds.append(ahrs_data[4] / np.pi * 180)
        heading_speeds.append(ahrs_data[5] / np.pi * 180)

        # 保持最近100个数据点
        if len(rolls) > 100:
            rolls.pop(0)
            pitches.pop(0)
            headings.pop(0)
            roll_speeds.pop(0)
            pitch_speeds.pop(0)
            heading_speeds.pop(0)

        # 更新子图1：角度
        line_roll.set_data(range(len(rolls)), rolls)
        line_pitch.set_data(range(len(pitches)), pitches)
        line_heading.set_data(range(len(headings)), headings)

        # 更新子图2：角速度
        line_roll_speed.set_data(range(len(roll_speeds)), roll_speeds)
        line_pitch_speed.set_data(range(len(pitch_speeds)), pitch_speeds)
        line_heading_speed.set_data(range(len(heading_speeds)), heading_speeds)

    except Exception as e:
        print(f"读取共享内存文件时出错：{e}")

    return (
        line_roll,
        line_pitch,
        line_heading,
        line_roll_speed,
        line_pitch_speed,
        line_heading_speed,
    )


# 动画
ani = FuncAnimation(fig, update, interval=100)  # 每100ms更新一次
plt.tight_layout()
plt.show()
