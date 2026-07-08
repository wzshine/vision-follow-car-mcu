#ifndef BOARD_RELAYS_H
#define BOARD_RELAYS_H

#include "stm32f4xx_hal.h"

/* 原车继电器逻辑占位（停转/倒车）：接线到具体 GPIO 后在此实现。
 * 刹车不在此 —— 见 brake_can.h（CAN 0x220 → 刹车板）。 */
void Board_Relay_MotorStop(GPIO_PinState level);
void Board_Relay_MotorRevert(GPIO_PinState level);

#endif
