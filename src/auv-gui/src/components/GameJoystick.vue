<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue'
import { useSubmarineStore } from '@/stores/submarine'

const props = defineProps({
  side: {
    type: String,
    required: true,
    validator: (value: string) => ['left', 'right'].includes(value),
  },
})

const store = useSubmarineStore()
const joystickRef = ref<HTMLElement | null>(null)
const isDragging = ref(false)
const position = ref({ x: 0, y: 0 })
const joystickSize = ref(0)
const handleSize = ref(0)
const maxDistance = ref(0)

const handleStart = (e: MouseEvent | TouchEvent) => {
  isDragging.value = true
  if (joystickRef.value) {
    const rect = joystickRef.value.getBoundingClientRect()
    joystickSize.value = rect.width
    handleSize.value = joystickSize.value * 0.5
    maxDistance.value = (joystickSize.value - handleSize.value) / 1.3
  }
  updatePosition(e)
}

const handleMove = (e: MouseEvent | TouchEvent) => {
  if (!isDragging.value) return
  e.preventDefault()
  updatePosition(e)
}

const handleEnd = () => {
  isDragging.value = false
  position.value = { x: 0, y: 0 }
  if (props.side === 'left') {
    store.leftJoystick = { x: 0, y: 0 }
  } else {
    store.rightJoystick = { x: 0, y: 0 }
  }
}

const updatePosition = (e: MouseEvent | TouchEvent) => {
  if (!joystickRef.value) return

  const rect = joystickRef.value.getBoundingClientRect()
  const centerX = rect.left + rect.width / 2
  const centerY = rect.top + rect.height / 2

  let clientX, clientY
  if (e instanceof MouseEvent) {
    clientX = e.clientX
    clientY = e.clientY
  } else if (e.touches && e.touches.length > 0) {
    clientX = e.touches[0].clientX
    clientY = e.touches[0].clientY
  } else {
    return
  }

  let x = clientX - centerX
  let y = clientY - centerY

  // Calculate distance from center
  const distance = Math.sqrt(x * x + y * y)

  // Normalize if outside max distance
  if (distance > maxDistance.value) {
    x = (x / distance) * maxDistance.value
    y = (y / distance) * maxDistance.value
  }

  position.value = { x, y }

  // Normalize values between -1 and 1
  const normalizedX = x / maxDistance.value
  const normalizedY = y / maxDistance.value

  if (props.side === 'left') {
    store.leftJoystick = { x: normalizedX, y: normalizedY }
  } else {
    store.rightJoystick = { x: normalizedX, y: normalizedY }
  }
}

onMounted(() => {
  window.addEventListener('mousemove', handleMove)
  window.addEventListener('mouseup', handleEnd)
  window.addEventListener('touchmove', handleMove, { passive: false })
  window.addEventListener('touchend', handleEnd)
})

onUnmounted(() => {
  window.removeEventListener('mousemove', handleMove)
  window.removeEventListener('mouseup', handleEnd)
  window.removeEventListener('touchmove', handleMove)
  window.removeEventListener('touchend', handleEnd)
})
</script>

<template>
  <div
    class="joystick"
    ref="joystickRef"
    @mousedown="handleStart"
    @touchstart="handleStart"
    :class="[side]"
  >
    <!-- 左侧手柄方向提示 -->
    <template v-if="side === 'left'">
      <span class="joystick-label top">前</span>
      <span class="joystick-label bottom">后</span>
      <span class="joystick-label left">左</span>
      <span class="joystick-label right">右</span>
    </template>
    <!-- 右侧手柄方向提示 -->
    <template v-else>
      <span class="joystick-label top">上</span>
      <span class="joystick-label bottom">下</span>
      <span class="joystick-label left vertical">左转</span>
      <span class="joystick-label right vertical">右转</span>
    </template>
    <div
      class="joystick-handle"
      :style="{
        transform: `translate(${position.x}px, ${position.y}px)`,
      }"
    ></div>
  </div>
</template>

<style scoped>
.joystick {
  position: relative;
  width: 100%;
  height: auto;
  aspect-ratio: 1 / 1;
  border-radius: 50%;
  background-color: rgba(128, 128, 128, 0.2);
  display: flex;
  justify-content: center;
  align-items: center;
  touch-action: none;
  user-select: none;
  margin: 16px;
}

.joystick-handle {
  position: absolute;
  width: 30%;
  height: 30%;
  border-radius: 50%;
  background-color: rgba(255, 255, 255, 0.8);
  cursor: move;
  touch-action: none;
  will-change: transform;
}

.joystick-label {
  position: absolute;
  color: #4cc9f0;
  font-size: 0.9rem;
  font-weight: bold;
  pointer-events: none;
  user-select: none;
  opacity: 0.85;
  text-shadow: 0 1px 4px #222;
  letter-spacing: 0.1em;
}

.joystick-label.top {
  top: 8px;
  left: 50%;
  transform: translateX(-50%);
}
.joystick-label.bottom {
  bottom: 8px;
  left: 50%;
  transform: translateX(-50%);
}
.joystick-label.left {
  left: 8px;
  top: 50%;
  transform: translateY(-50%);
}
.joystick-label.right {
  right: 8px;
  top: 50%;
  transform: translateY(-50%);
}
.joystick-label.vertical {
  writing-mode: vertical-rl;
  text-align: center;
  line-height: 1.1;
  letter-spacing: 0.05em;
}
</style>
