#ifndef BOARD_BATTERY_H
#define BOARD_BATTERY_H

#include <stdint.h>

/* 电池电量：ADC1_IN11（PC1）分压采样，返回 0~4 供串口屏（与 DlaCart 一致） */
uint8_t Battery_GetLevel(void);

#endif
