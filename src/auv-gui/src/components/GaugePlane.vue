<script setup lang="ts">
import { ref, watch, onMounted, onBeforeUnmount } from "vue";
import { useSubmarineStore } from "@/stores/submarine";

const store = useSubmarineStore();
const canvasRef = ref<HTMLCanvasElement | null>(null);
const width = 320;
const height = 320;

let animationFrame: number | null = null;
const currentRoll = ref(0);
const currentPitch = ref(0);
const currentYaw = ref(0);

function draw() {
  const ctx = canvasRef.value?.getContext("2d");
  if (!ctx) return;
  ctx.clearRect(0, 0, width, height);

  // 圆形边框
  ctx.save();
  ctx.beginPath();
  ctx.arc(width / 2, height / 2, width / 2 - 2, 0, Math.PI * 2);
  ctx.closePath();
  ctx.lineWidth = 6;
  ctx.strokeStyle = "#4cc9f0";
  ctx.stroke();
  ctx.restore();

  // 内圆
  ctx.save();
  ctx.beginPath();
  ctx.arc(width / 2, height / 2, width / 2 - 16, 0, Math.PI * 2);
  ctx.closePath();
  ctx.lineWidth = 8;
  ctx.strokeStyle = "#3a0ca3";
  ctx.stroke();
  ctx.restore();

  // 姿态仪背景（天/地）
  ctx.save();
  ctx.translate(width / 2, height / 2);
  ctx.rotate((currentRoll.value * Math.PI) / 180);
  // pitch影响上下分界线
  const pitchOffset = currentPitch.value * 2;
  // 天空
  ctx.fillStyle = "#48cae4";
  ctx.fillRect(-width, -width / 2 + pitchOffset - width, width * 2, width * 3);
  // 地面
  ctx.fillStyle = "#333acd";
  ctx.fillRect(-width, 0 + pitchOffset, width * 2, width * 3);
  ctx.restore();

  // 刻度尺
  ctx.save();
  ctx.translate(width / 2, height / 2);
  for (let i = 0; i <= 23; i++) {
    ctx.save();
    ctx.rotate((i * 15 * Math.PI) / 180);
    ctx.beginPath();
    ctx.moveTo(0, -width / 2 + 24);
    ctx.lineTo(0, -width / 2 + 44);
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.font = "14px sans-serif";
    ctx.fillStyle = "#ffffff";
    ctx.textAlign = "center";
    ctx.textBaseline = "bottom";
    // 角度数字旋转处理
    const angle = i * 15;
    ctx.save();
    ctx.translate(0, -width / 2 + 12);
    if (angle > 90 && angle < 270) {
      ctx.rotate(Math.PI);
    }
    ctx.fillText(`${angle}`, 0, 7);
    ctx.restore();
    ctx.restore();
  }
  ctx.restore();

  // 遮罩层
  ctx.save();
  ctx.beginPath();
  ctx.arc(width / 2, height / 2, width / 2 - 2, 0, Math.PI * 2);
  ctx.closePath();
  ctx.fillStyle = "rgba(255,255,255,0.08)";
  ctx.fill();
  ctx.restore();

  // 指针
  ctx.save();
  ctx.translate(width / 2, height / 2);
  ctx.rotate((currentYaw.value * Math.PI) / 180);
  ctx.beginPath();
  ctx.moveTo(0, -width / 2 + 48);
  ctx.lineTo(-8, 0);
  ctx.lineTo(8, 0);
  ctx.closePath();
  ctx.fillStyle = "#ff0000";
  ctx.shadowColor = "#ff0000aa";
  ctx.shadowBlur = 4;
  ctx.fill();
  ctx.restore();

  // 指针中心圆点
  ctx.save();
  ctx.beginPath();
  ctx.arc(width / 2, height / 2, 16, 0, Math.PI * 2);
  ctx.closePath();
  ctx.fillStyle = "#ffcc00";
  ctx.shadowColor = "#ffcc00aa";
  ctx.shadowBlur = 4;
  ctx.fill();
  ctx.restore();
}

function animate() {
  const speed = 0.18;
  
  // 处理 roll 的差值，选择最短路径
  let diffRoll = store.roll - currentRoll.value;
  if (diffRoll > 180) {
    diffRoll -= 360; // 顺时针绕大圈改为逆时针
  } else if (diffRoll < -180) {
    diffRoll += 360; // 逆时针绕大圈改为顺时针
  }

  // 处理 pitch 的差值
  const diffPitch = store.pitch - currentPitch.value;

  // 处理 yaw 的差值，选择最短路径
  let diffYaw = store.yaw - currentYaw.value;
  if (diffYaw > 180) {
    diffYaw -= 360; // 顺时针绕大圈改为逆时针
  } else if (diffYaw < -180) {
    diffYaw += 360; // 逆时针绕大圈改为顺时针
  }

  let changed = false;

  if (Math.abs(diffRoll) > 0.2) {
    currentRoll.value += diffRoll * speed;
    changed = true;
  } else {
    currentRoll.value = store.roll;
  }

  if (Math.abs(diffPitch) > 0.2) {
    currentPitch.value += diffPitch * speed;
    changed = true;
  } else {
    currentPitch.value = store.pitch;
  }

  if (Math.abs(diffYaw) > 0.2) {
    currentYaw.value += diffYaw * speed;
    changed = true;
  } else {
    currentYaw.value = store.yaw;
  }

  draw();
  if (changed) {
    animationFrame = requestAnimationFrame(animate);
  } else {
    animationFrame = null;
  }
}

function handleResize() {
  animate();
}

watch(
  () => [store.roll, store.pitch, store.yaw],
  () => {
    if (!animationFrame) animate();
  }
);

onMounted(() => {
  currentRoll.value = store.roll;
  currentPitch.value = store.pitch;
  currentYaw.value = store.yaw;
  draw();
  window.addEventListener("resize", handleResize);
});

onBeforeUnmount(() => {
  if (animationFrame) cancelAnimationFrame(animationFrame);
  window.removeEventListener("resize", handleResize);
});
</script>

<template>
  <div class="gauge-plane-canvas-area" style="position: relative">
    <canvas
      ref="canvasRef"
      :width="width"
      :height="height"
      class="gauge-plane-canvas"
    ></canvas>
    <!-- 方位文字 -->
    <span class="gauge-label north">北</span>
    <span class="gauge-label south">南</span>
    <span class="gauge-label west">西</span>
    <span class="gauge-label east">东</span>
  </div>
</template>

<style scoped>
.gauge-plane-canvas-area {
  width: 100%;
  height: 100%;
  display: flex;
  justify-content: center;
  align-items: center;
  position: relative;
}
.gauge-plane-canvas {
  width: 70%;
  height: auto;
  max-width: 400px;
  max-height: 400px;
  background: #1a1a2e;
  border-radius: 50%;
  box-shadow: 0 2px 12px #0005;
  display: block;
  margin: 0 auto;
}

/* 方位文字样式 */
.gauge-label {
  position: absolute;
  color: #fff;
  font-size: 1.2rem;
  font-weight: bold;
  user-select: none;
  pointer-events: none;
  text-shadow: 0 2px 8px #000a;
  letter-spacing: 0.2em;
}
.north {
  top: 8%;
  left: 50%;
  transform: translate(-50%, -50%);
}
.south {
  bottom: 8%;
  left: 50%;
  transform: translate(-50%, 50%);
}
.west {
  left: 8%;
  top: 50%;
  transform: translate(-50%, -50%);
}
.east {
  right: 8%;
  top: 50%;
  transform: translate(50%, -50%);
}
</style>
