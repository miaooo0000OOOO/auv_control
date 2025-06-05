# 水下机器人上位机

## 简介

跨平台的窗口程序，实时显示下位机发来的视频流和遥测数据，可以通过摇杆控制下位机运动

## 通信

使用WebSocket通信，上位机在3000端口/control-msg路径上发送摇杆控制信息给下位机

下位机在4000端口/telemetry路径上发送遥测数据给上位机；在5000端口/video-stream路径上发送视频流给上位机

下位机主机名，端口和路径可以通过配置文件覆写，配置文件可以参考`auv_net_config.example.json`，配置文件位于`$CONFIG_DIR/auv-gui/auv_net_config.json`，其中的`$CONFIG_DIR`在不同操作系统映射如下：

 - **Linux:** Resolves to `$XDG_CONFIG_HOME` or `$HOME/.config`.
 - **macOS:** Resolves to `$HOME/Library/Application Support`.
 - **Windows:** Resolves to `{FOLDERID_RoamingAppData}`.

示例代码参考: `test_stream_server.py`, `test_telemetry_server.py`, `test_control_msg_client.py`

用于测试的Python需要安装`flask_sock`, `flask`, `websockets` 和 `opencv-python`

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

## Windows打包的网络问题

参考 [[如何解决安装失败]  #7338](https://github.com/tauri-apps/tauri/issues/7338)