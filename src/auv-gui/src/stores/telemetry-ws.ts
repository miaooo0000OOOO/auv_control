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

    // 从store中读取主机、端口和路径
    const host = store.auvHost;
    const port = store.telemetryPort;
    const path = store.telemetryPath;
    const wsUrl = `ws://${host}:${port}${path}`;
    socket = new WebSocket(wsUrl);

    socket.onmessage = async (event) => {
      if (processing) return;
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

  connectSocket();

  return { cleanup };
}
