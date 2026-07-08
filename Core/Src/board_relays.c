#include "board_relays.h"
#include "main.h"

/*
 * 原车继电器（停转/倒车等）本地 GPIO 占位。
 * 注意：刹车不在此处 —— 刹车是经 CAN 0x220 发往刹车板 DlaCart_v2.1，
 *       见 brake_can.c / brake_can.h（旧 Board_Relay_BrakeCylinder 已迁出）。
 * 行走电机现统一由 FOC2 经 CAN 0x210 执行，下列继电器接线确定后再实现。
 */

void Board_Relay_MotorStop(GPIO_PinState level)
{
  (void)level;
}

void Board_Relay_MotorRevert(GPIO_PinState level)
{
  (void)level;
}
