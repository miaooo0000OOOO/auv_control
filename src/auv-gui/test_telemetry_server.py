from flask import Flask
from flask_sock import Sock
import random
import json
import time
import math

app = Flask(__name__)
sock = Sock(app)

# 初始化数据
current_data = {
    "depth": 5.0,  # 初始深度
    "roll": 0.0,  # 初始横滚角
    "pitch": 0.0,  # 初始俯仰角
    "yaw": 0.0,  # 初始偏航角
    "thrusters": [0.0 for _ in range(6)],  # 初始推进器转速
}

# 时间变量，用于正弦函数
time_counter = 0


def generate_next_data(data, use_sine_wave=True):
    """基于当前数据生成下一组数据"""
    global time_counter
    if use_sine_wave:
        # 使用正弦函数生成数据
        time_counter += 0.1
        data["depth"] = 5 + 5 * math.sin(time_counter)  # 深度在 0 到 10 之间变化
        data["roll"] = (180 * math.sin(time_counter)) % 360
        data["pitch"] = 90 * math.sin(time_counter / 2)
        data["yaw"] = (360 * math.sin(time_counter / 3)) % 360
        data["thrusters"] = [
            math.sin(time_counter + i) for i in range(6)
        ]
        # data["depth"] = 0  # 深度在 0 到 10 之间变化
        # data["roll"] = 350
        # data["pitch"] = 0
        # data["yaw"] = 0
        # data["thrusters"] = [0 for i in range(6)]
    else:
        # 随机连续变化
        data["depth"] = max(0, min(10, data["depth"] + random.uniform(-0.1, 0.1)))
        data["roll"] = (data["roll"] + random.uniform(-1, 1)) % 360
        data["pitch"] = max(-90, min(90, data["pitch"] + random.uniform(-1, 1)))
        data["yaw"] = (data["yaw"] + random.uniform(-1, 1)) % 360
        data["thrusters"] = [
            max(-1, min(1, t + random.uniform(-0.05, 0.05))) for t in data["thrusters"]
        ]
    return data


@sock.route("/telemetry")
def telemetry_stream(ws):
    print("Client connected")
    global current_data
    use_sine_wave = True  # 切换为正弦函数模式
    while True:
        current_data = generate_next_data(current_data, use_sine_wave=use_sine_wave)
        ws.send(json.dumps(current_data))
        time.sleep(0.1)  # 每 0.1 秒发送一次数据


if __name__ == "__main__":
    # 启动 Flask 应用
    app.run(host="0.0.0.0", port=4000, use_debugger=False, use_reloader=False)
