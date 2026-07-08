#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"

#define MOTOR_PWM_PERIOD      1000
#define MOTOR_PWM_MIN_DUTY    0
#define MOTOR_PWM_MAX_DUTY    1000

#define RC_PWM_MIN_US         1000
#define RC_PWM_MID_US         1500
#define RC_PWM_MAX_US         2000

#define MOTOR_DEADBAND_US     20
#define MOTOR_SPEED_SCALE     120

void Motor_Control_Init(void);
void Motor_Control_Update(int16_t rc_pwm_us);
void Motor_Control_SetSpeed(int16_t duty);
void Motor_Control_Stop(void);

#endif
