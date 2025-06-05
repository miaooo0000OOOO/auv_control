<script setup lang="ts">
import { ref, onMounted, onUnmounted } from "vue";
import { useSubmarineStore } from "@/stores/submarine";

const store = useSubmarineStore();

const videoSrc = ref("");
const imgWidth = ref(0);
const imgHeight = ref(0);
const containerWidth = ref(0);
const containerHeight = ref(0);
const containerRef = ref<HTMLDivElement | null>(null);

let socket: WebSocket | null = null;
let retryTimer: number | null = null;
const RETRY_INTERVAL = 2000; // 2秒重试一次

function connectSocket() {
  if (socket) {
    socket.close();
    socket = null;
  }

  const host = store.auvHost;
  const port = store.videoStreamPort;
  const path = store.videoStreamPath;

  const videoStreamUrl = `ws://${host}:${port}${path}`;

  socket = new WebSocket(videoStreamUrl);

  socket.onmessage = (event) => {
    videoSrc.value = `data:image/jpeg;base64,${event.data}`;
  };

  socket.onerror = () => {
    console.error("WebSocket to video error, will retry...");
    retryConnect();
  };

  socket.onclose = () => {
    retryConnect();
  };
}

function retryConnect() {
  if (retryTimer) clearTimeout(retryTimer);
  retryTimer = window.setTimeout(() => {
    connectSocket();
  }, RETRY_INTERVAL);
}

function cleanup() {
  if (socket) {
    socket.onclose = null;
    socket.onerror = null;
    socket.onmessage = null;
    socket.close();
    socket = null;
  }
  if (retryTimer) {
    clearTimeout(retryTimer);
    retryTimer = null;
  }
}

function updateContainerSize() {
  const maxW = containerRef.value?.parentElement?.clientWidth || 800;
  const maxH = containerRef.value?.parentElement?.clientHeight || 600;
  if (imgWidth.value && imgHeight.value) {
    const ratio = imgWidth.value / imgHeight.value;
    let w = maxW;
    let h = w / ratio;
    if (h > maxH) {
      h = maxH;
      w = h * ratio;
    }
    containerWidth.value = w;
    containerHeight.value = h;
  }
}

// 监听图片加载，获取原始宽高并调整容器
function onImgLoad(e: Event) {
  const img = e.target as HTMLImageElement;
  imgWidth.value = img.naturalWidth;
  imgHeight.value = img.naturalHeight;
  updateContainerSize();
}

onMounted(() => {
  connectSocket();
  window.addEventListener("resize", updateContainerSize);
});

onUnmounted(() => {
  cleanup();
  window.removeEventListener("resize", updateContainerSize);
});
</script>

<template>
  <div
    class="video-container"
    ref="containerRef"
    :style="{
      width: containerWidth + 'px',
      height: containerHeight + 'px',
    }"
  >
    <img
      class="video-element"
      :src="videoSrc"
      alt="Video Stream"
      @load="onImgLoad"
      draggable="false"
      style="width: 100%; height: 100%; border-radius: 16px; display: block"
    />
    <div class="time-display">
      {{ new Date().toLocaleTimeString() }}
    </div>
  </div>
</template>

<style scoped>
.video-container {
  background-color: #16213e;
  border-radius: 16px;
  position: relative;
  overflow: hidden;
  display: flex;
  align-items: center;
  justify-content: center;
  /* 不设置固定宽高，由JS动态控制 */
}

.time-display {
  position: absolute;
  top: 10px;
  right: 10px;
  background-color: rgba(0, 0, 0, 0.5);
  padding: 5px 10px;
  border-radius: 4px;
  font-size: 1rem;
  color: #ffffff;
}
</style>
