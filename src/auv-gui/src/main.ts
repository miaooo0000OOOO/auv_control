import './assets/main.css'
import { createApp } from 'vue'
import { createPinia } from 'pinia'
import App from './App.vue'
import { useTelemetryWebSocket } from '@/stores/telemetry-ws'
import { useSubmarineStore } from '@/stores/submarine'

const app = createApp(App)
app.use(createPinia())

// 初始化配置后再挂载和启动WebSocket
const store = useSubmarineStore()
store.initAuvConfig().then(() => {
  app.mount('#app')
  useTelemetryWebSocket()
})