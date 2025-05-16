<script setup lang="ts">
import { computed } from 'vue'
import { useSubmarineStore } from '@/stores/submarine'

const store = useSubmarineStore()

// 计算推进器的动态数据
const thrusterData = computed(() =>
  store.thrusters.map((speed: number, idx: number) => ({
    index: idx + 1,
    speed,
  })),
)
</script>

<template>
  <div class="thrusters-container">
    <div class="thruster" v-for="thruster in thrusterData" :key="thruster.index">
      <h3>推进器 {{ thruster.index }}</h3>
      <p>转速：{{ (thruster.speed * 100).toFixed(0) }} %</p>
    </div>
  </div>
</template>

<style scoped>
.thrusters-container {
  width: 100%;
  height: 100%;
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 16px;
  margin-top: 20px;
}

.thruster {
  background: rgba(64, 64, 64, 0.4);
  padding: 16px;
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  width: 100%;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  text-align: center;
}

.thruster h3 {
  font-size: clamp(1rem, 5vw, 2rem);
  margin-bottom: 8px;
  color: #ffffff;
}

.thruster p {
  font-size: clamp(0.8rem, 4vw, 1.5rem);
  margin: 4px 0;
  color: #dcdcdc;
}
</style>
