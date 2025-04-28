#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include "pca9685_thruster_speed_control.h"

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

// 全局变量
static int i2c_fd = -1;

/// @brief 打开并初始化 PCA9685 的 I2C 设备
/// @param i2c_bus I2C 总线的设备文件路径，例如 "/dev/i2c-1"
/// @param addr PCA9685 的 I2C 地址（7 位地址）
/// @return 成功返回 0，失败返回 -1 并打印错误信息
int pca9685_open(const char *i2c_bus, int addr)
{
    if ((i2c_fd = open(i2c_bus, O_RDWR)) < 0)
    {
        fprintf(stderr, "Failed to open I2C bus %s: %s\n",
                i2c_bus, strerror(errno));
        return -1;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0)
    {
        fprintf(stderr, "Failed to set I2C address 0x%02X: %s\n",
                addr, strerror(errno));
        close(i2c_fd);
        i2c_fd = -1;
        return -1;
    }

    return 0;
}

/**
 * @brief 关闭PCA9685设备的I2C连接。
 *
 * 如果I2C文件描述符有效（大于等于0），此函数将关闭文件描述符并将其重置为-1。
 *
 * @note 在程序结束或不再需要与PCA9685通信时，应调用此函数以释放资源。
 */
/// @brief
void pca9685_close()
{
    if (i2c_fd >= 0)
    {
        close(i2c_fd);
        i2c_fd = -1;
    }
}

/// @brief 写入单个寄存器的值到PCA9685设备
/// @param reg 寄存器地址
/// @param value 要写入的值
/// @return 成功返回0，失败返回-1
/// @note 该函数通过I2C接口将指定值写入到指定寄存器中。如果写入失败，会打印错误信息到标准错误输出。
int pca9685_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};

    if (write(i2c_fd, buf, 2) != 2)
    {
        fprintf(stderr, "Failed to write to I2C device: %s\n",
                strerror(errno));
        return -1;
    }

    return 0;
}

/// @brief 读取 PCA9685 的单个寄存器值
/// @param reg 寄存器地址
/// @param value 用于存储读取值的指针
/// @return 成功返回 0，失败返回 -1
/// @note 该函数通过 I2C 接口与 PCA9685 通信，首先写入寄存器地址，然后读取寄存器值。
///       如果写入或读取失败，会打印错误信息到标准错误输出。
int pca9685_read_reg(uint8_t reg, uint8_t *value)
{
    if (write(i2c_fd, &reg, 1) != 1)
    {
        fprintf(stderr, "Failed to write to I2C device: %s\n",
                strerror(errno));
        return -1;
    }

    if (read(i2c_fd, value, 1) != 1)
    {
        fprintf(stderr, "Failed to read from I2C device: %s\n",
                strerror(errno));
        return -1;
    }

    return 0;
}

/// @brief 设置指定通道的PWM输出
/// @param channel PWM通道号（0-15）
/// @param on PWM信号高电平开始的计数值
/// @param off PWM信号高电平结束的计数值
/// @return 成功返回0，失败返回-1
int pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off)
{
    uint8_t buf[5];

    buf[0] = PCA9685_LED0_ON_L + 4 * channel;
    buf[1] = on & 0xFF;
    buf[2] = on >> 8;
    buf[3] = off & 0xFF;
    buf[4] = off >> 8;

    if (write(i2c_fd, buf, 5) != 5)
    {
        fprintf(stderr, "Failed to set PWM output: %s\n",
                strerror(errno));
        return -1;
    }

    return 0;
}

/// @brief 初始化PCA9685设备
/// @details 此函数用于初始化PCA9685 PWM驱动芯片，包括复位设备、设置PWM频率、配置模式寄存器等操作。
/// @param freq PWM频率（单位：Hz），用于设置PCA9685的输出频率。
/// @return 返回0表示初始化成功。
int pca9685_init(int freq)
{
    uint8_t prescale, oldmode, newmode;
    double prescaleval;

    // 复位设备
    pca9685_write_reg(PCA9685_MODE1, 0x00);

    // 计算预分频值
    prescaleval = 25000000.0; // 25MHz
    prescaleval /= 4096.0;    // 12-bit
    prescaleval /= freq;
    prescaleval -= 1.0;
    prescale = (uint8_t)(prescaleval + 0.5);

    // 读取当前模式
    pca9685_read_reg(PCA9685_MODE1, &oldmode);

    // 进入睡眠模式设置预分频
    newmode = (oldmode & 0x7F) | 0x10; // 睡眠模式
    pca9685_write_reg(PCA9685_MODE1, newmode);
    pca9685_write_reg(PCA9685_PRESCALE, prescale);
    pca9685_write_reg(PCA9685_MODE1, oldmode);

    // 等待振荡器稳定
    usleep(5000);

    // 启用自动递增和恢复正常模式
    pca9685_write_reg(PCA9685_MODE1, oldmode | 0xA0);

    // 设置输出驱动配置
    pca9685_write_reg(PCA9685_MODE2, 0x04);

    return 0;
}

/// @brief 设置推进器速度 (-100%~100%)
/// @param thruster_index 推进器索引（1到THRUSTER_COUNT）
/// @param speed 推进器速度百分比（-100.0f~100.0f）
/// @return 成功返回0，失败返回-1
/// @note 如果传入的速度超出范围（小于-100或大于100），将自动限制在有效范围内。
///       推进器索引从1开始计数，对应的PWM通道索引为thruster_index-1。
///       速度值会线性映射到5%-10%的占空比范围，并转换为0-4095的PWM值。
int set_thruster_speed(int thruster_index, float speed)
{
    // 参数检查
    if (thruster_index < 1 || thruster_index > THRUSTER_COUNT)
    {
        fprintf(stderr, "Invalid thruster index: %d (must be 1-%d)\n",
                thruster_index, THRUSTER_COUNT);
        return -1;
    }

    if (speed < -100.0f)
        speed = -100.0f;
    if (speed > 100.0f)
        speed = 100.0f;

    // 将-100%~100%速度线性映射到5-10%占空比
    float duty = (speed + 100.0f) * (MAX_DUTY - MIN_DUTY) / 200.0f + MIN_DUTY;

    // 计算PWM值 (占空比转换为0-4095)
    uint16_t pwm_value = (uint16_t)(duty * 4095.0f / 100.0f);

    // 设置PWM (通道索引是thruster_index-1)
    return pca9685_set_pwm(thruster_index - 1, 0, pwm_value);
}

/// @brief 初始化所有推进器速度为0
void init_thrusters()
{
    for (int i = 1; i <= THRUSTER_COUNT; i++)
    {
        set_thruster_speed(i, 0.0f);
    }
}

// 测试示例
int main(int argc, char *argv[])
{
    const char *i2c_bus = "/dev/i2c-4"; // 根据实际I2C总线修改
    int addr = PCA9685_I2C_ADDR;

    // 打开设备
    if (pca9685_open(i2c_bus, addr) < 0)
    {
        return EXIT_FAILURE;
    }

    // 初始化PCA9685 (50Hz)
    if (pca9685_init(PWM_FREQUENCY) < 0)
    {
        pca9685_close();
        return EXIT_FAILURE;
    }

    // 初始化所有推进器速度为0
    init_thrusters();

    printf("PCA9685推进器控制器初始化成功 (地址:0x%02X, 总线:%s)\n", addr, i2c_bus);

    // 示例：测试推进器1
    printf("测试推进器1...\n");
    while (1)
    {
        // 从0%到100%加速
        for (float speed = 0.0f; speed <= 100.0f; speed += 1.0f)
        {
            printf("设置推进器1速度为 %.1f%%\n", speed);
            set_thruster_speed(1, speed);
            usleep(50000); // 50ms延迟
        }   
        // 从100%到0%减速
        for (float speed = 100.0f; speed >= 0.0f; speed -= 1.0f)
        {
            printf("设置推进器1速度为 %.1f%%\n", speed);
            set_thruster_speed(1, speed);
            usleep(50000); // 50ms延迟
        }
        // 从0%到-100%加速
        for (float speed = 0.0f; speed >= -100.0f; speed -= 1.0f)
        {
            printf("设置推进器1速度为 %.1f%%\n", speed);
            set_thruster_speed(1, speed);
            usleep(50000); // 50ms延迟
        }
        // 从-100%到0%减速
        for (float speed = -100.0f; speed <= 0.0f; speed += 1.0f)
        {
            printf("设置推进器1速度为 %.1f%%\n", speed);
            set_thruster_speed(1, speed);
            usleep(50000); // 50ms延迟
        }
        // 暂停10秒
        printf("暂停10秒...\n");
        set_thruster_speed(1, 0.0f); // 停止推进器
        usleep(10000000); // 10秒延迟
    }

    // 重置所有推进器速度为0
    init_thrusters();

    // 关闭设备
    pca9685_close();

    return EXIT_SUCCESS;
}