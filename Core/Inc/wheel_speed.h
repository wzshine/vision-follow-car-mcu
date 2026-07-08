#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define WHEEL_MAGNETS           15
#define WHEEL_PULSES_PER_REV    WHEEL_MAGNETS
#define WHEEL_DIAMETER_MM       260
#define WHEEL_CIRCUMFERENCE_MM  (314 * WHEEL_DIAMETER_MM / 100)
#define WHEEL_MEASURE_INTERVAL_MS  500

void WheelSpeed_Init(void);
void WheelSpeed_CaptureCallback(TIM_HandleTypeDef *htim);
void WheelSpeed_Task(void);
uint16_t WheelSpeed_GetSpeed_01kmh(void);
uint32_t WheelSpeed_GetRawPulses_Left(void);
uint32_t WheelSpeed_GetRawPulses_Right(void);

#endif
