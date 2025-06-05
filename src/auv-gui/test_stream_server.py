import cv2
import base64
from flask import Flask
from flask_sock import Sock
from simple_websocket.ws import Server

app = Flask(__name__)
sock = Sock(app)


@sock.route("/video-stream")
def video_stream(ws: Server):
    """视频流路由"""
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("无法打开摄像头")
        return

    while True:
        success, frame = camera.read()
        if not success:
            continue

        # 将帧编码为 JPEG 格式并转换为 Base64
        _, buffer = cv2.imencode(".jpg", frame)
        frame_data = base64.b64encode(buffer).decode("utf-8")

        # # 计算帧率
        # fps = camera.get(cv2.CAP_PROP_FPS)
        # print(fps)

        # 通过 WebSocket 发送帧
        ws.send(frame_data)


if __name__ == "__main__":
    # 启动 Flask 应用
    app.run(host="0.0.0.0", port=5000, use_debugger=False, use_reloader=False)
