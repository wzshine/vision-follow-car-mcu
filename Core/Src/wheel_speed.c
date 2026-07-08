#include "wheel_speed.h"
#include "main.h"

/*
 * 轮速：FG 脉冲输入捕获
 *   CN20 -> R58(100Ω) -> PA2 / TIM2_CH3，R61(10k) 下拉到 GND  (右轮 / left? 由实车标定)
 *   CN19 -> R57(100Ω) -> PA3 / TIM2_CH4，R60(10k) 下拉到 GND
 * 信号空闲电平为 0V（10k 强下拉），传感器有效边沿走上升沿。
 * 通道滤波由 Board_TuneTimersForCart() 中重新配置为 ICFilter=8，避免短促毛刺误触发。
 */

static volatile uint32_t s_pulse_left = 0;
static volatile uint32_t s_pulse_right = 0;
static uint32_t s_pulse_left_snap = 0;
static uint32_t s_pulse_right_snap = 0;
static uint32_t s_last_calc_ms = 0;
static uint16_t s_speed_01kmh = 0;

void WheelSpeed_Init(void)
{
  (void)HAL_TIM_Base_Start(&htim2);
  (void)HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  (void)HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  s_pulse_left = 0;
  s_pulse_right = 0;
  s_last_calc_ms = HAL_GetTick();
  s_speed_01kmh = 0;
}

void WheelSpeed_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM2)
  {
    return;
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    if (s_pulse_left < 500U)
    {
      s_pulse_left++;
    }
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    if (s_pulse_right < 500U)
    {
      s_pulse_right++;
    }
  }
}

void WheelSpeed_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed_ms = now - s_last_calc_ms;
  if (elapsed_ms < WHEEL_MEASURE_INTERVAL_MS)
  {
    return;
  }

  s_last_calc_ms = now;

  uint32_t pl = s_pulse_left;
  uint32_t pr = s_pulse_right;
  s_pulse_left = 0;
  s_pulse_right = 0;
  s_pulse_left_snap = pl;
  s_pulse_right_snap = pr;

  uint32_t pulses = (pl > 0U && pr > 0U) ? (pl + pr) / 2U : (pl + pr);

  uint32_t speed_01kmh = pulses * WHEEL_CIRCUMFERENCE_MM * 36U
                         / ((uint32_t)WHEEL_PULSES_PER_REV * elapsed_ms);

  s_speed_01kmh = (uint16_t)(speed_01kmh > 9999U ? 9999U : speed_01kmh);
}

uint16_t WheelSpeed_GetSpeed_01kmh(void)
{
  return s_speed_01kmh;
}

uint32_t WheelSpeed_GetRawPulses_Left(void)
{
  return s_pulse_left_snap;
}

uint32_t WheelSpeed_GetRawPulses_Right(void)
{
  return s_pulse_right_snap;
}
