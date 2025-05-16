# 水下机器人上位机

## 简介

跨平台的窗口程序，实时显示下位机发来的视频流和遥测数据，可以通过摇杆控制下位机运动

## 通信

使用 WebSocket ，下位机在5000端口发送视频流，在4000端口发送遥测数据。

示例代码参考: `test_stream_server.py` 和 `test_telemetry_server.py`

用于测试的Python需要安装`flask_sock`, `flask` 和 `python-opencv`

## 技术框架

Vue3 + Yarn + Rust + Tauri2

Vue3: 前端框架

Yarn: 前端包管理器

Rust: 后端

Tauri: 桌面应用框架

## 编译打包

```bash
yarn install
yarn tauri build
```