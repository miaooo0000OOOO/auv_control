import { useSubmarineStore } from "@/stores/submarine";

export function useTelemetryWebSocket() {
  const store = useSubmarineStore();
  let socket: WebSocket | null = null;
  let retryTimer: number | null = null;
  const RETRY_INTERVAL = 2000; // 2秒重试一次
  let processing = false; // 标记是否正在处理消息

  function connectSocket() {
    if (socket) {
      socket.close();
      socket = null;
    }

    socket = new WebSocket("http://localhost:4000/telemetry");

    socket.onmessage = async (event) => {
      if (processing) return; // 如果正在处理消息，直接返回
      processing = true;

      try {
        const data = JSON.parse(event.data);
        if (
          typeof data.depth === "number" &&
          typeof data.roll === "number" &&
          typeof data.pitch === "number" &&
          typeof data.yaw === "number" &&
          Array.isArray(data.thrusters)
        ) {
          store.depth = data.depth;
          store.roll = data.roll;
          store.pitch = data.pitch;
          store.yaw = data.yaw;
          store.thrusters = data.thrusters;
        }
      } catch (e) {
        console.warn("Telemetry parse error:", e);
      }

      // 等待 0.01 秒后允许处理下一条消息
      await new Promise((resolve) => setTimeout(resolve, 10));
      processing = false;
    };

    socket.onerror = () => {
      console.error("WebSocket to telemetry error, will retry...");
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

  // 初始化连接
  connectSocket();

  // 返回清理函数，供外部调用
  return { cleanup };
}
