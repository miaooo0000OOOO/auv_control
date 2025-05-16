<script setup lang="ts">
import { onMounted, ref } from 'vue'
import * as THREE from 'three'
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { useSubmarineStore } from '@/stores/submarine'

const store = useSubmarineStore()
const containerRef = ref<HTMLElement | null>(null)
let controls: OrbitControls | null = null
let camera: THREE.PerspectiveCamera | null = null
let initialCameraPosition: THREE.Vector3 | null = null
let initialCameraTarget: THREE.Vector3 | null = null

function resetView() {
  if (!controls || !camera || !initialCameraPosition || !initialCameraTarget) return
  camera.position.copy(initialCameraPosition)
  controls.target.copy(initialCameraTarget)
  controls.update()
}

onMounted(() => {
  if (!containerRef.value) return

  // 初始化场景
  const scene = new THREE.Scene()
  scene.background = new THREE.Color(0x1a1a2e)

  // 初始化渲染器
  const renderer = new THREE.WebGLRenderer({ antialias: true })
  renderer.setSize(containerRef.value.clientWidth, containerRef.value.clientHeight)
  containerRef.value.appendChild(renderer.domElement)

  console.log('Renderer size:', renderer.getSize(new THREE.Vector2()))
  console.log('Container size:', containerRef.value?.clientWidth, containerRef.value?.clientHeight)

  // 初始化相机
  camera = new THREE.PerspectiveCamera(
    75,
    containerRef.value.clientWidth / containerRef.value.clientHeight,
    0.1,
    1000,
  )
  camera.position.set(0, 0, 5) // 设置初始相机位置

  // 添加轨道控制器
  controls = new OrbitControls(camera, renderer.domElement)
  controls.enableDamping = true

  // 添加环境光
  const ambientLight = new THREE.AmbientLight(0x87ceeb, 0.5) // 浅蓝色环境光
  scene.add(ambientLight)

  // 添加方向光
  const directionalLight = new THREE.DirectionalLight(0xffffff, 1)
  directionalLight.position.set(1, 1, 1)
  scene.add(directionalLight)

  // 加载OBJ模型
  const loader = new OBJLoader()
  loader.load(
    '/models/submarine.obj', // 替换为您的 .obj 文件路径
    (obj) => {
      console.log('Model loaded:', obj) // 调试信息

      // 计算模型的大小和中心
      const box = new THREE.Box3().setFromObject(obj)
      const size = box.getSize(new THREE.Vector3())
      const center = box.getCenter(new THREE.Vector3())

      console.log('Model size:', size)
      console.log('Model center:', center)

      // 根据模型大小调整相机位置
      const maxDim = Math.max(size.x, size.y, size.z)
      const fov = camera!.fov * (Math.PI / 180) // 将 FOV 转换为弧度
      const cameraDis = (maxDim / (2 * Math.tan(fov / 2))) * 0.7 // 计算相机距离
      camera!.position.set(cameraDis, cameraDis, -cameraDis) // 适当增加距离以留出边距
      camera!.lookAt(center)

      // 保存初始相机位置和目标
      initialCameraPosition = camera!.position.clone()
      initialCameraTarget = center.clone()

      // 更新轨道控制器
      controls!.target.copy(center)
      controls!.update()

      // 将模型添加到场景
      obj.traverse((child) => {
        if ((child as THREE.Mesh).isMesh) {
          ;(child as THREE.Mesh).material = new THREE.MeshStandardMaterial({ color: 0x22aa22 })
        }
      })
      scene.add(obj)

      // 添加平移和缩放变换
      obj.position.set(0, 0, 1.5) // 模型平移
      obj.scale.set(1.0, 1.0, 1.0) // 模型缩放

      console.log('Scene children:', scene.children)

      // 动画循环
      const animate = () => {
        requestAnimationFrame(animate)

        // 更新模型姿态
        obj.rotation.set(
          store.roll * (Math.PI / 180),
          store.yaw * (Math.PI / 180),
          store.pitch * (Math.PI / 180),
        )

        controls!.update()
        renderer.render(scene, camera!)
      }
      animate()
    },
    undefined,
    (error) => {
      console.error('Error loading model:', error)
    },
  )

  // 响应式调整
  window.addEventListener('resize', () => {
    if (!camera) return
    camera.aspect = containerRef.value!.clientWidth / containerRef.value!.clientHeight
    camera.updateProjectionMatrix()
    renderer.setSize(containerRef.value!.clientWidth, containerRef.value!.clientHeight)
  })
})
</script>

<template>
  <div class="model-container" ref="containerRef">
    <div class="model3d-overlay">
      <span class="model3d-title">3D姿态</span>
      <button class="model3d-reset" @click="resetView">复位</button>
    </div>
  </div>
</template>

<style scoped>
.model-container {
  width: 50%;
  height: 100%;
  background-color: #16213e;
  border-radius: 8px;
  position: relative;
  overflow: hidden;
}

.model3d-overlay {
  position: absolute;
  left: 0;
  top: 0;
  width: 100%;
  z-index: 2;
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
  pointer-events: none;
  padding: 12px 16px;
  box-sizing: border-box;
}

.model3d-title {
  color: #fff;
  font-size: 1.2rem;
  font-weight: bold;
  background: rgba(34, 34, 68, 0.7);
  border-radius: 4px;
  padding: 4px 12px;
  pointer-events: auto;
}

.model3d-reset {
  background: #4cc9f0;
  color: #16213e;
  border: none;
  border-radius: 4px;
  font-size: 1rem;
  font-weight: bold;
  padding: 4px 16px;
  cursor: pointer;
  pointer-events: auto;
  transition: background 0.2s;
}
.model3d-reset:hover {
  background: #38bdf8;
}
</style>
