#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define WHEEL_MAGNETS           15
#define WHEEL_PULSES_PER_REV    WHEEL_MAGNETS

/*
 * ! 这里曾经有 WHEEL_DIAMETER_MM 260 和一个由它算出的轮周长，用来把本板
 *   磁钢拾取的脉冲换成 km/h。2026-08-25 整个删除。
 *
 *   轮周长只应该有一个家，而那个家是驱动板 —— 它才是直接驱动轮子的那一方，
 *   它把这个数存在自己的 flash 里(`drive wheel`)，换个轮子改一条命令，不用
 *   重编译任何东西。控制板的职责是下指令，它不需要知道轮胎有多大。
 *
 *   本板原来那份是从标称直径算的、比实测周长大 2.26%，而它只出现在
 *   "CAN 死掉时猜一个车速"这条几乎从不被走到的路上 ——
 *   ⚠ 一条从不被走到的退路，是一条没人知道它坏了的退路。
 *
 *   现在：车速由驱动板经 0x212 [5:6] 送来；线上没有就是没有，
 *   FocCAN_GetSpeed_01kmh() 返回 0 —— 和 GetVbus_mV/GetIbus_cA 一样的约定。
 *
 *   脉冲计数本身留着(WheelSpeed_GetRawPulses_*)：那是原始传感器读数，
 *   不需要任何轮子参数，将来要做里程计时它还在。
 */
#define WHEEL_MEASURE_INTERVAL_MS  500

void WheelSpeed_Init(void);
void WheelSpeed_CaptureCallback(TIM_HandleTypeDef *htim);
void WheelSpeed_Task(void);
uint32_t WheelSpeed_GetRawPulses_Left(void);
uint32_t WheelSpeed_GetRawPulses_Right(void);

#endif
