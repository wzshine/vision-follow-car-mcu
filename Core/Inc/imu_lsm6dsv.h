#ifndef IMU_LSM6DSV_H
#define IMU_LSM6DSV_H

#include <stdint.h>

/*
 * LSM6DSVETR 6 轴 IMU 驱动（控制板 DlaBoard_A，SPI1 4 线，轮询式）。
 * 接线（原理图 gokart_v3_logic / IC 页，已与代码核对）：
 *   SCK=PA5  MISO=PA6  MOSI=PA7  CS=PA4(软件片选,空闲高,低有效)
 *   INT1=EXTI6 / INT2=EXTI5（本驱动用轮询，未用中断）
 *
 * 注意：CubeMX 的 MX_SPI1_Init 把分频设为 /2(≈42MHz)，超过 LSM6DSV 10MHz 上限；
 * IMU_Init() 会把 hspi1 分频改到 /16(≈5.25MHz)后重初始化，保证总线合规。
 *
 * 采样为原始 int16（小端寄存器）。物理量换算依配置量程：
 *   加速度 mg/LSB、角速度 mdps/LSB —— 量程在 .c 中配置并记录，最终单位与
 *   Jetson 帧格式一并敲定。
 */

typedef struct {
  int16_t  ax, ay, az;   /* 加速度原始值（int16, LSB） */
  int16_t  gx, gy, gz;   /* 角速度原始值（int16, LSB） */
  int16_t  temp;         /* 温度原始值 */
  uint8_t  ok;           /* 1=WHO_AM_I 校验通过且在采样 */
  uint8_t  whoami;       /* 实际读到的 WHO_AM_I */
  uint32_t last_ms;      /* 最近一次成功采样时戳 */
  uint32_t read_count;
} Imu_t;

extern Imu_t g_imu;

uint8_t IMU_Init(void);   /* 配置 SPI 速率 + 复位 + 量程/ODR + 校验 WHO_AM_I。返回 1=成功 */
void    IMU_Task(void);   /* 主循环调用，内部限频(约 100Hz)读取一帧到 g_imu */
uint8_t IMU_Fresh(uint32_t now_ms);   /* 最近 200ms 内有有效采样 → 1 */

#endif /* IMU_LSM6DSV_H */
