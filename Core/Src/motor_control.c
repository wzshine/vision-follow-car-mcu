#include "motor_control.h"

static int16_t s_current_duty = 0;
static uint8_t s_current_direction = 1;

void Motor_Control_Init(void)
{
  s_current_duty = 0;
  s_current_direction = 1;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

void Motor_Control_Update(int16_t rc_pwm_us)
{
  int16_t duty;
  uint8_t direction;

  if (rc_pwm_us < RC_PWM_MIN_US)
  {
    rc_pwm_us = RC_PWM_MIN_US;
  }
  else if (rc_pwm_us > RC_PWM_MAX_US)
  {
    rc_pwm_us = RC_PWM_MAX_US;
  }

  if (rc_pwm_us >= (RC_PWM_MID_US - MOTOR_DEADBAND_US) && rc_pwm_us <= (RC_PWM_MID_US + MOTOR_DEADBAND_US))
  {
    duty = 0;
    direction = s_current_direction;
  }
  else
  {
    if (rc_pwm_us < RC_PWM_MID_US)
    {
      direction = 1;
      duty = (int16_t)(((int32_t)(RC_PWM_MID_US - rc_pwm_us) * MOTOR_PWM_MAX_DUTY) / (RC_PWM_MID_US - RC_PWM_MIN_US));
    }
    else
    {
      direction = 0;
      duty = (int16_t)(((int32_t)(rc_pwm_us - RC_PWM_MID_US) * MOTOR_PWM_MAX_DUTY) / (RC_PWM_MAX_US - RC_PWM_MID_US));
    }
  }

  duty = (int16_t)(((int32_t)duty * MOTOR_SPEED_SCALE) / 100);
  if (duty < MOTOR_PWM_MIN_DUTY)
  {
    duty = MOTOR_PWM_MIN_DUTY;
  }
  else if (duty > MOTOR_PWM_MAX_DUTY)
  {
    duty = MOTOR_PWM_MAX_DUTY;
  }

  s_current_duty = duty;
  s_current_direction = direction;

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)duty);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)duty);
}

void Motor_Control_SetSpeed(int16_t duty)
{
  if (duty < MOTOR_PWM_MIN_DUTY)
  {
    duty = MOTOR_PWM_MIN_DUTY;
  }
  else if (duty > MOTOR_PWM_MAX_DUTY)
  {
    duty = MOTOR_PWM_MAX_DUTY;
  }
  s_current_duty = duty;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)duty);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)duty);
}

void Motor_Control_Stop(void)
{
  s_current_duty = 0;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}
