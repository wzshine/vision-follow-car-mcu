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

  /* 只留快照，不再换算成 km/h —— 那需要一个轮周长，而轮周长只有驱动板有。
   *
   * ! 顺带带走了一个和别处一模一样的错误：原来这里写
   *       pulses = (pl > 0 && pr > 0) ? (pl + pr) / 2 : (pl + pr);
   *   —— 一个轮子读到 0 就取和而不是取平均。同一个形状在
   *   FocCAN_GetSpeed_01kmh 的 rpm 合成里、在 0x211 的 foc2 槽守卫里各出现
   *   过一次，2026-08-25 同一天全部修掉。**三处不是三个 bug，是一个习惯**：
   *   把"读到 0"当成"没有数据"。 */
  (void)elapsed_ms;
}

uint32_t WheelSpeed_GetRawPulses_Left(void)
{
  return s_pulse_left_snap;
}

uint32_t WheelSpeed_GetRawPulses_Right(void)
{
  return s_pulse_right_snap;
}
