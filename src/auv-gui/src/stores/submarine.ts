import { defineStore } from 'pinia'
import { ref } from 'vue'

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

  // 视频流 URL
  const videoStreamUrl = ref("ws://localhost:5000/video-stream") // 替换为实际的流媒体 URL

  // 更新时间
  const currentTime = ref(new Date())

  // 更新时间的定时器
  setInterval(() => {
    currentTime.value = new Date()
  }, 1000)

  return {
    roll,
    pitch,
    yaw,
    depth,
    leftJoystick,
    rightJoystick,
    thrusters, // 添加推进器数据
    videoStream,
    videoStreamUrl,
    currentTime,
  }
})

