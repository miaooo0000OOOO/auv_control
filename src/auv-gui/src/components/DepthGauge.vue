<script setup lang="ts">
import { ref, watch, onMounted, onBeforeUnmount } from 'vue'
import { useSubmarineStore } from '@/stores/submarine'

const store = useSubmarineStore()
const canvasRef = ref<HTMLCanvasElement | null>(null)
const width = 120
const height = 320

const majorStep = 100
const visibleMajorCount = 10
const majorSpacing = height / (visibleMajorCount - 1)

let animationFrame: number | null = null
const currentOffset = ref(0) // 当前动画偏移

function getVisibleMajorRange() {
  // 当前深度对应的主刻度索引
  const currentMajorIndex = Math.floor((store.depth * 100) / majorStep)
  const half = Math.floor(visibleMajorCount / 2)
  const start = Math.max(0, currentMajorIndex - half)
  const end = start + visibleMajorCount
  return { start, end }
}

function draw() {
  const ctx = canvasRef.value?.getContext('2d')
  if (!ctx) return
  ctx.clearRect(0, 0, width, height)

  // 渐变阴影
  const gradTop = ctx.createLinearGradient(0, 0, 0, 40)
  gradTop.addColorStop(0, '#16213e')
  gradTop.addColorStop(1, 'transparent')
  ctx.fillStyle = gradTop
  ctx.fillRect(0, 0, width, 40)

  const gradBot = ctx.createLinearGradient(0, height - 40, 0, height)
  gradBot.addColorStop(0, 'transparent')
  gradBot.addColorStop(1, '#16213e')
  ctx.fillStyle = gradBot
  ctx.fillRect(0, height - 40, width, 40)

  // 动态计算可见主刻度范围
  const { start: visibleMajorStart, end: visibleMajorEnd } = getVisibleMajorRange()

  ctx.save()
  ctx.translate(40, 0) // 不再整体平移，y轴直接计算

  for (let i = visibleMajorStart; i < visibleMajorEnd; i++) {
    // 让0刻度线在depth=0时位于指针中线
    const y = height / 2 + (i * majorSpacing - currentOffset.value)
    // 主刻度线
    ctx.strokeStyle = '#4cc9f0'
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.moveTo(0, y)
    ctx.lineTo(30, y)
    ctx.stroke()
    // 主刻度标签
    ctx.fillStyle = '#4cc9f0'
    ctx.font = '14px sans-serif'
    ctx.textBaseline = 'middle'
    ctx.fillText(`${((i * majorStep) / 100).toFixed(0)}m`, 35, y)
    // 副刻度线
    for (let j = 1; j < 4; j++) {
      const yMinor = y + (j * majorSpacing) / 4
      ctx.strokeStyle = '#4cc9f0'
      ctx.lineWidth = 1
      ctx.beginPath()
      ctx.moveTo(0, yMinor)
      ctx.lineTo(15, yMinor)
      ctx.stroke()
    }
  }
  ctx.restore()

  // 三角形指针
  ctx.save()
  ctx.translate(8, height / 2)
  ctx.beginPath()
  ctx.moveTo(0, -14)
  ctx.lineTo(22, 0)
  ctx.lineTo(0, 14)
  ctx.closePath()
  ctx.fillStyle = '#ffcc00'
  ctx.shadowColor = '#ffcc00aa'
  ctx.shadowBlur = 4
  ctx.fill()
  ctx.restore()
}

function targetOffset() {
  // 当前深度对应的像素位置
  return ((store.depth * 100) / majorStep) * majorSpacing
}

function animate() {
  const speed = 0.15
  const diff = targetOffset() - currentOffset.value
  if (Math.abs(diff) > 0.5) {
    currentOffset.value += diff * speed
    draw()
    animationFrame = requestAnimationFrame(animate)
  } else {
    currentOffset.value = targetOffset()
    draw()
    animationFrame = null
  }
}

function handleResize() {
  animate()
}

watch(
  () => store.depth,
  () => {
    if (!animationFrame) animate()
  },
)

onMounted(() => {
  currentOffset.value = targetOffset()
  draw()
  window.addEventListener('resize', handleResize)
})

onBeforeUnmount(() => {
  if (animationFrame) cancelAnimationFrame(animationFrame)
  window.removeEventListener('resize', handleResize)
})
</script>

<template>
  <div class="depth-gauge-area">
    <canvas class="depth-gauge" ref="canvasRef" :width="width" :height="height"></canvas>
    <div class="depth-value-text">深度：{{ store.depth.toFixed(2) }} 米</div>
  </div>
</template>

<style scoped>
.depth-value-text {
  color: #4cc9f0;
  font-size: 2vw;
  font-weight: bold;
  margin: 5%;
}

.depth-gauge {
  border-radius: 8px;
  background-color: #0f3460;
}

.depth-gauge-area {
  width: 100%;
  display: flex;
  justify-content: center;
  align-items: center;
}
</style>
