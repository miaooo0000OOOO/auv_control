import { defineStore } from 'pinia'
import { ref, watch } from 'vue'
import { invoke } from "@tauri-apps/api/core";

// 异步加载配置
async function loadAuvConfig() {
  try {
    const config = await invoke('load_auv_config')
    console.log('配置加载成功', config)
    return config as any
  } catch (e) {
    console.warn('配置加载失败，使用默认配置', e)
    return {}
  }
}

export const useSubmarineStore = defineStore('submarine', () => {
  // 姿态数据
  const roll = ref(0) // 横滚角
  const pitch = ref(0) // 俯仰角
  const yaw = ref(0) // 偏航角

  // 深度数据
  const depth = ref(0) // 当前深度

  // 摇杆数据
  const leftJoystick = ref({ x: 0, y: 0 })
  const rightJoystick = ref({ x: 0, y: 0 })

  // 推进器数据
  // -1 ~ 1
  const thrusters = ref([
    0, // 推进器 1
    0, // 推进器 2
    0, // 推进器 3
    0, // 推进器 4
    0, // 推进器 5
    0, // 推进器 6
  ])

  // 视频流
  const videoStream = ref(null)

  // 默认值
  // 水下机器人主机地址
  const auvHost = ref("192.168.1.1")

  // 视频流的端口
  const videoStreamPort = ref(1145)

  // 视频流的路径
  const videoStreamPath = ref("/video-stream")

  // 遥测数据的端口
  const telemetryPort = ref(1919)

  // 遥测数据的路径
  const telemetryPath = ref("/telemetry")

  // 控制消息的端口
  const controlMessagePort = ref(2020) 

  // 控制消息的路径
  const controlMessagePath = ref("/control")

  // 初始化配置
  async function initAuvConfig() {
    const config = await loadAuvConfig()
    auvHost.value = config.auvHost
    videoStreamPort.value = config.videoStreamPort
    videoStreamPath.value = config.videoStreamPath
    telemetryPort.value = config.telemetryPort
    telemetryPath.value = config.telemetryPath
    controlMessagePort.value = config.controlMessagePort
    controlMessagePath.value = config.controlMessagePath
  }

  // 更新时间
  const currentTime = ref(new Date())

  // 更新时间的定时器
  setInterval(() => {
    currentTime.value = new Date()
  }, 1000)

  // 发送摇杆数据到后端
  function sendJoystickData() {
    const data = {
      left: leftJoystick.value,
      right: rightJoystick.value,
    }
    invoke('send_joystick', { data: JSON.stringify(data) })
  }

  // 监听摇杆变化并发送
  watch(leftJoystick, sendJoystickData, { deep: true })
  watch(rightJoystick, sendJoystickData, { deep: true })

  return {
    roll,
    pitch,
    yaw,
    depth,
    leftJoystick,
    rightJoystick,
    thrusters,
    videoStream,
    auvHost,
    videoStreamPort,
    videoStreamPath,
    telemetryPort,
    telemetryPath,
    controlMessagePort,
    controlMessagePath,
    currentTime,
    initAuvConfig, // 导出初始化方法
    sendJoystickData,
  }
})

