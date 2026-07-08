#ifndef BOARD_INPUTS_H
#define BOARD_INPUTS_H

#include "stm32f4xx_hal.h"

/*
 * 工作模式（手动 / 遥控 / 训练 / 跟随）：
 * 仅以串口屏变量为准——屏通过 0x05A1 地址上报键值（见 screen.h 中 SCREEN_ADDR_MODE、SCREEN_KEY_MODE_*），
 * 主循环按 key_addr 切换 g_steering_mode。无单独「模式」GPIO。
 *
 * PRND 物理档位：四路开关输入（按下接地为低），Cube 中配置为 GPIOD 上拉输入。
 */
#define GEAR_SW_GPIO_Port  GPIOD
#define GEAR_P_Pin         GPIO_PIN_0
#define GEAR_R_Pin         GPIO_PIN_1
#define GEAR_N_Pin         GPIO_PIN_3
#define GEAR_D_Pin         GPIO_PIN_4

/* Vehicle enable input: CN22 / INPUT_PD10. Active low, external pull-up. */
#define VEHICLE_ENABLE_GPIO_Port  GPIOD
#define VEHICLE_ENABLE_Pin        GPIO_PIN_10

#endif
