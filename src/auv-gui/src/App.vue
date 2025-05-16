<script setup lang="ts">
import { ref } from "vue";
import { invoke } from "@tauri-apps/api/core";

import GaugePlane from '@/components/GaugePlane.vue'
import DepthGauge from '@/components/DepthGauge.vue'
import GameJoystick from '@/components/GameJoystick.vue'
import VideoFeed from '@/components/VideoFeed.vue'
import Model3D from '@/components/Model3D.vue'
import Thrusters from '@/components/Thrusters.vue'
</script>

<template>
  <div class="dashboard-container">
    <div class="left-panel">
      <GaugePlane />
      <DepthGauge />
      <div class="joystick-area">
        <GameJoystick side="left" />
        <GameJoystick side="right" />
      </div>
    </div>

    <div class="right-panel">
      <div class="upper-panel">
        <VideoFeed />
      </div>

      <div class="down-panel">
        <Model3D />
        <Thrusters />
      </div>
    </div>
  </div>
</template>

<style scoped>
.dashboard-container {
  display: flex;
  height: 100vh;
  width: 100vw;
  background-color: #1a1a2e;
  color: white;
}

.left-panel {
  width: 30%;
  display: flex;
  flex-direction: column;
  padding: 1rem;
}

.right-panel {
  width: 70%;
  display: flex;
  flex-direction: column;
  padding: 1rem;
}

.upper-panel {
  width: 100%;
  height: 60%;
  display: flex;
  justify-content: center;
  align-items: center;
}

.down-panel {
  width: 100%;
  height: 40%;
  display: flex;
  flex-direction: row;
  padding: 1rem;
}

.joystick-area {
  display: flex;
  height: 30%;
  justify-content: space-between;
  margin-top: 2rem;
}
</style>
