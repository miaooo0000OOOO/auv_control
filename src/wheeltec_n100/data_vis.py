import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import fcntl

# 读取IMU_DATA.npy文件
shm_file = "/dev/shm/IMU_DATA.npy"

# 初始化数据存储
rolls, pitches, headings = [], [], []
line_roll_speeds, line_pitch_speeds, line_heading_speeds = [], [], []

# 创建图形
fig, ax = plt.subplots(figsize=(10, 6))
(line_roll,) = ax.plot([], [], label="Roll (deg)", color="r")
(line_pitch,) = ax.plot([], [], label="Pitch (deg)", color="g")
(line_heading,) = ax.plot([], [], label="Heading (deg)", color="b")

(line_roll_speed,) = ax.plot(
    [], [], label="Roll Speed (deg/s)", color="r", linestyle="--"
)
(line_pitch_speed,) = ax.plot(
    [], [], label="Pitch Speed (deg/s)", color="g", linestyle="--"
)
(line_heading_speed,) = ax.plot(
    [], [], label="Heading Speed (deg/s)", color="b", linestyle="--"
)


ax.set_xlim(0, 100)  # 假设显示最近100个数据点
ax.set_ylim(-360, 360)  # 根据角度范围设置Y轴
ax.set_xlabel("Time (arbitrary units)")
ax.set_ylabel("Angle (degrees)")
ax.set_title("Real-Time Roll, Pitch, and Heading")
ax.legend()
ax.grid()

SHOW_ANGLE_SPEED = True  # 是否显示角速度曲线


# 更新函数
def update(frame):
    global rolls, pitches, headings, line_roll_speeds, line_pitch_speeds, line_heading_speeds

    try:
        # 使用文件锁读取共享内存文件
        with open(shm_file, "rb") as f:
            fcntl.flock(f, fcntl.LOCK_SH)  # 加读锁
            data = np.load(f)
            fcntl.flock(f, fcntl.LOCK_UN)  # 释放锁

        # 更新数据
        rolls.append(data[0])
        pitches.append(data[1])
        headings.append(data[2])

        line_roll_speeds.append(data[3])
        line_pitch_speeds.append(data[4])
        line_heading_speeds.append(data[5])

        # 保持最近100个数据点
        if len(rolls) > 100:
            rolls.pop(0)
            pitches.pop(0)
            headings.pop(0)
            line_roll_speeds.pop(0)
            line_pitch_speeds.pop(0)
            line_heading_speeds.pop(0)

        # 更新图形
        line_roll.set_data(range(len(rolls)), rolls)
        line_pitch.set_data(range(len(pitches)), pitches)
        line_heading.set_data(range(len(headings)), headings)

        if SHOW_ANGLE_SPEED:
            line_roll_speed.set_data(range(len(line_roll_speeds)), line_roll_speeds)
            line_pitch_speed.set_data(range(len(line_pitch_speeds)), line_pitch_speeds)
            line_heading_speed.set_data(
                range(len(line_heading_speeds)), line_heading_speeds
            )

    except Exception as e:
        print(f"读取共享内存文件 {shm_file} 时出错：{e}")

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
plt.show()
