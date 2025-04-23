#ifndef PCA9685_THRUSTER_SPEED_CONTROL_H
#define PCA9685_THRUSTER_SPEED_CONTROL_H

#include <stdint.h>

// PCA9685寄存器定义
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_LED0_ON_H 0x07
#define PCA9685_LED0_OFF_L 0x08
#define PCA9685_LED0_OFF_H 0x09

// PCA9685默认地址
#define PCA9685_I2C_ADDR 0x40

// 推进器参数
#define THRUSTER_COUNT 8
#define PWM_FREQUENCY 50 // 50Hz
#define MIN_DUTY 5.0f    // 5% 对应最小推力(1100μs)
#define MAX_DUTY 10.0f   // 10% 对应最大推力(1900μs)

// 函数声明
int pca9685_open(const char *i2c_bus, int addr);
void pca9685_close();
int pca9685_write_reg(uint8_t reg, uint8_t value);
int pca9685_read_reg(uint8_t reg, uint8_t *value);
int pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off);
int pca9685_init(int freq);
int set_thruster_speed(int thruster_index, float speed);
void init_thrusters();

#endif // PCA9685_THRUSTER_SPEED_CONTROL_H