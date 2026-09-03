#include "rc_input.h"
#include "cart_param.h"
#include "stm32f4xx_hal.h"

static volatile uint32_t s_pulse_width_us = RC_MID_US;
static volatile uint32_t s_pwm_update_count = 0;
static volatile uint32_t s_rising_edge_time = 0;
static volatile uint8_t s_capture_state = 0;
static volatile uint32_t s_callback_count = 0;
static volatile uint32_t s_last_update_ms = 0;

static volatile uint32_t s_pulse_width_ch2_us = RC_MID_US;
static volatile uint32_t s_pwm_update_count_ch2 = 0;
static volatile uint32_t s_rising_edge_time_ch2 = 0;
static volatile uint8_t s_capture_state_ch2 = 0;
static volatile uint32_t s_callback_count_ch2 = 0;
static volatile uint32_t s_last_update_ms_ch2 = 0;

/* RC3 = PC7 / TIM3_CH2 */
static volatile uint32_t s_pulse_width_rc3_us = RC_MID_US;
static volatile uint32_t s_pwm_update_count_rc3 = 0;
static volatile uint32_t s_rising_edge_time_rc3 = 0;
static volatile uint8_t s_capture_state_rc3 = 0;
static volatile uint32_t s_callback_count_rc3 = 0;
static volatile uint32_t s_last_update_ms_rc3 = 0;

/* RC4 = PC6 / TIM3_CH1 */
static volatile uint32_t s_pulse_width_rc4_us = RC_MID_US;
static volatile uint32_t s_pwm_update_count_rc4 = 0;
static volatile uint32_t s_rising_edge_time_rc4 = 0;
static volatile uint8_t s_capture_state_rc4 = 0;
static volatile uint32_t s_callback_count_rc4 = 0;
static volatile uint32_t s_last_update_ms_rc4 = 0;


/*
 * 一路遥控通道现在算不算活的。
 *
 * 从前这个文件里没有一个时间戳：脉宽只在捕获到一个 500..2500us 的脉冲时被
 * 覆盖，此外永远保持上一次的值。于是遥控链路一断(接收机掉电、天线脱落、
 * 信号线断)，RC_GetPulseWidth_CH2_us() 会一直返回断线那一刻的油门位置 ——
 * 在 MODE_REMOTE 里那是唯一的油门来源。这不是"最后写入者获胜"，
 * 是"最后写入者永远获胜"。
 *
 * 判活的口径和整车其余部分一致：**不活的源什么都不主张**。
 * 所以这里分三态，不是两态：
 *
 *   RC_LINK_NEVER —— 从来没收到过脉冲。这一路上没有接收机，它不是一个源,
 *                    什么都不主张。(本板今天就是这个状态：RC? 显示 upd=0。)
 *   RC_LINK_UP    —— 最近 RC_STALE_MS 内收到过。
 *   RC_LINK_LOST  —— 曾经活过，现在不新鲜了。**这一路失效了**，
 *                    要主张它的安全值,而不是装作它还在。
 *
 * "从来没接过"和"接过然后断了"必须是两个不同的答案 —— 否则要么没插接收机
 * 的车永远刹着不动，要么断了链路的车带着最后一次油门继续跑。
 */
static RcLink_t link_state(uint32_t update_count, uint32_t last_ms)
{
  if (update_count == 0U) {
    return RC_LINK_NEVER;
  }
  /* ⚠ 运行时值：rc_stale 是可写参数。见 cart_param.h 那条规矩 ——
   * 表里说可写而这里读宏，就会造出一个"设了不生效"的参数。 */
  return ((int32_t)(HAL_GetTick() - last_ms) <= g_p_rc_stale) ? RC_LINK_UP : RC_LINK_LOST;
}

RcLink_t RC_LinkState_CH1(void) { return link_state(s_pwm_update_count,     s_last_update_ms);     }
RcLink_t RC_LinkState_CH2(void) { return link_state(s_pwm_update_count_ch2, s_last_update_ms_ch2); }
RcLink_t RC_LinkState_RC3(void) { return link_state(s_pwm_update_count_rc3, s_last_update_ms_rc3); }
RcLink_t RC_LinkState_RC4(void) { return link_state(s_pwm_update_count_rc4, s_last_update_ms_rc4); }

void RC_Input_Init(void)
{
  HAL_TIM_IC_Stop_IT(&RC_TIM_HANDLE, RC_STEER_CHANNEL);

  TIM_IC_InitTypeDef sConfigIC = {0};
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  (void)HAL_TIM_IC_ConfigChannel(&RC_TIM_HANDLE, &sConfigIC, RC_STEER_CHANNEL);

  (void)HAL_TIM_Base_Start(&RC_TIM_HANDLE);
  (void)HAL_TIM_IC_Start_IT(&RC_TIM_HANDLE, RC_STEER_CHANNEL);

  s_pulse_width_us = RC_MID_US;
  s_capture_state = 0;
  s_rising_edge_time = 0;

  HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void RC_Input_OnCapture(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    s_callback_count++;
    uint32_t current_time = HAL_TIM_ReadCapturedValue(htim, RC_STEER_CHANNEL);

    if (s_capture_state == 0)
    {
      s_rising_edge_time = current_time;
      s_capture_state = 1;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, RC_STEER_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
      uint32_t pulse_width = (current_time - s_rising_edge_time) & 0xFFFFU;
      if (pulse_width >= 500U && pulse_width <= 2500U)
      {
        s_pulse_width_us = pulse_width;
        s_pwm_update_count++;
        s_last_update_ms = HAL_GetTick();
      }
      s_capture_state = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, RC_STEER_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
  else if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    s_callback_count_ch2++;
    uint32_t current_time = HAL_TIM_ReadCapturedValue(htim, RC_DRIVE_CHANNEL);

    if (s_capture_state_ch2 == 0)
    {
      s_rising_edge_time_ch2 = current_time;
      s_capture_state_ch2 = 1;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, RC_DRIVE_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
      uint32_t pulse_width = (current_time - s_rising_edge_time_ch2) & 0xFFFFU;
      if (pulse_width >= 500U && pulse_width <= 2500U)
      {
        s_pulse_width_ch2_us = pulse_width;
        s_pwm_update_count_ch2++;
        s_last_update_ms_ch2 = HAL_GetTick();
      }
      s_capture_state_ch2 = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, RC_DRIVE_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
  else if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    /* RC3 = PC7 / TIM3_CH2 */
    s_callback_count_rc3++;
    uint32_t current_time = HAL_TIM_ReadCapturedValue(htim, RC3_CHANNEL);

    if (s_capture_state_rc3 == 0)
    {
      s_rising_edge_time_rc3 = current_time;
      s_capture_state_rc3 = 1;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, RC3_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
      uint32_t pulse_width = (current_time - s_rising_edge_time_rc3) & 0xFFFFU;
      if (pulse_width >= 500U && pulse_width <= 2500U)
      {
        s_pulse_width_rc3_us = pulse_width;
        s_pwm_update_count_rc3++;
        s_last_update_ms_rc3 = HAL_GetTick();
      }
      s_capture_state_rc3 = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, RC3_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
  else if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    /* RC4 = PC6 / TIM3_CH1 */
    s_callback_count_rc4++;
    uint32_t current_time = HAL_TIM_ReadCapturedValue(htim, RC4_CHANNEL);

    if (s_capture_state_rc4 == 0)
    {
      s_rising_edge_time_rc4 = current_time;
      s_capture_state_rc4 = 1;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, RC4_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
      uint32_t pulse_width = (current_time - s_rising_edge_time_rc4) & 0xFFFFU;
      if (pulse_width >= 500U && pulse_width <= 2500U)
      {
        s_pulse_width_rc4_us = pulse_width;
        s_pwm_update_count_rc4++;
        s_last_update_ms_rc4 = HAL_GetTick();
      }
      s_capture_state_rc4 = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, RC4_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
}

int16_t RC_GetPulseWidth_us(void)
{
  /* 链路曾经活过而现在断了：交出中位(转向回正)，而不是断线那一刻的摇杆位置。 */
  if (RC_LinkState_CH1() == RC_LINK_LOST) {
    return RC_MID_US;
  }
  uint32_t pulse = s_pulse_width_us;
  if (pulse < RC_MIN_US)
  {
    return RC_MIN_US;
  }
  if (pulse > RC_MAX_US)
  {
    return RC_MAX_US;
  }
  return (int16_t)pulse;
}

int16_t RC_GetNormalized(int16_t min, int16_t max)
{
  int16_t pulse = RC_GetPulseWidth_us();
  int32_t range = (int32_t)max - (int32_t)min;
  int32_t offset = (int32_t)pulse - RC_MIN_US;
  int32_t scaled = (offset * range) / (RC_MAX_US - RC_MIN_US);
  return (int16_t)(min + scaled);
}

uint32_t RC_GetUpdateCount(void)
{
  return s_pwm_update_count;
}

uint32_t RC_GetCallbackCount(void)
{
  return s_callback_count;
}

void RC_Input_Init_CH2(void)
{
  HAL_TIM_IC_Stop_IT(&RC_TIM_HANDLE, RC_DRIVE_CHANNEL);

  TIM_IC_InitTypeDef sConfigIC = {0};
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  (void)HAL_TIM_IC_ConfigChannel(&RC_TIM_HANDLE, &sConfigIC, RC_DRIVE_CHANNEL);

  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&RC_TIM_HANDLE) == 0)
  {
    (void)HAL_TIM_Base_Start(&RC_TIM_HANDLE);
  }

  (void)HAL_TIM_IC_Start_IT(&RC_TIM_HANDLE, RC_DRIVE_CHANNEL);

  s_pulse_width_ch2_us = RC_MID_US;
  s_capture_state_ch2 = 0;
  s_rising_edge_time_ch2 = 0;
}

int16_t RC_GetPulseWidth_CH2_us(void)
{
  /* 链路曾经活过而现在断了：交出中位(零油门)。MODE_REMOTE 下这是唯一的
   * 油门来源 —— 从前会一直返回断线那一刻的油门，车就那样跑下去。 */
  if (RC_LinkState_CH2() == RC_LINK_LOST) {
    return RC_MID_US;
  }
  uint32_t pulse = s_pulse_width_ch2_us;
  if (pulse < RC_MIN_US)
  {
    return RC_MIN_US;
  }
  if (pulse > RC_MAX_US)
  {
    return RC_MAX_US;
  }
  return (int16_t)pulse;
}

uint32_t RC_GetUpdateCount_CH2(void)
{
  return s_pwm_update_count_ch2;
}

uint32_t RC_GetCallbackCount_CH2(void)
{
  return s_callback_count_ch2;
}

/* ---- RC3 (PC7 / TIM3_CH2) / RC4 (PC6 / TIM3_CH1) ----
 * 引脚由 MSP 配为 TIM3 复用输入捕获(同 RC1/RC2)。即使按稳定电平使用，
 * 复用模式下 IDR 仍可读，RC_GetSwitch_* 在 RC_SW_USE_PWM=0 时直接读 GPIO 电平。*/
void RC_Input_Init_RC3(void)
{
  HAL_TIM_IC_Stop_IT(&RC_TIM_HANDLE, RC3_CHANNEL);

  TIM_IC_InitTypeDef sConfigIC = {0};
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  (void)HAL_TIM_IC_ConfigChannel(&RC_TIM_HANDLE, &sConfigIC, RC3_CHANNEL);

  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&RC_TIM_HANDLE) == 0)
  {
    (void)HAL_TIM_Base_Start(&RC_TIM_HANDLE);
  }

  (void)HAL_TIM_IC_Start_IT(&RC_TIM_HANDLE, RC3_CHANNEL);

  s_pulse_width_rc3_us = RC_MID_US;
  s_capture_state_rc3 = 0;
  s_rising_edge_time_rc3 = 0;
}

void RC_Input_Init_RC4(void)
{
  HAL_TIM_IC_Stop_IT(&RC_TIM_HANDLE, RC4_CHANNEL);

  TIM_IC_InitTypeDef sConfigIC = {0};
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  (void)HAL_TIM_IC_ConfigChannel(&RC_TIM_HANDLE, &sConfigIC, RC4_CHANNEL);

  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&RC_TIM_HANDLE) == 0)
  {
    (void)HAL_TIM_Base_Start(&RC_TIM_HANDLE);
  }

  (void)HAL_TIM_IC_Start_IT(&RC_TIM_HANDLE, RC4_CHANNEL);

  s_pulse_width_rc4_us = RC_MID_US;
  s_capture_state_rc4 = 0;
  s_rising_edge_time_rc4 = 0;
}

int16_t RC_GetPulseWidth_RC3_us(void)
{
  uint32_t pulse = s_pulse_width_rc3_us;
  if (pulse < RC_MIN_US) return RC_MIN_US;
  if (pulse > RC_MAX_US) return RC_MAX_US;
  return (int16_t)pulse;
}

int16_t RC_GetPulseWidth_RC4_us(void)
{
  uint32_t pulse = s_pulse_width_rc4_us;
  if (pulse < RC_MIN_US) return RC_MIN_US;
  if (pulse > RC_MAX_US) return RC_MAX_US;
  return (int16_t)pulse;
}

uint32_t RC_GetUpdateCount_RC3(void)   { return s_pwm_update_count_rc3; }
uint32_t RC_GetUpdateCount_RC4(void)   { return s_pwm_update_count_rc4; }
uint32_t RC_GetCallbackCount_RC3(void) { return s_callback_count_rc3; }
uint32_t RC_GetCallbackCount_RC4(void) { return s_callback_count_rc4; }

uint8_t RC_GetSwitch_RC3(void)
{
  /*
   * 急停通道断了要报"刹车"，不是报"未刹车"。
   *
   * 从前 s_pulse_width_rc3_us 初值是 RC_MID_US(1500)，而判据是 > 1500 ——
   * 所以在收到第一个脉冲之前(包括整个上电窗口)，远程急停读出来是"已释放"。
   * 断链之后同样保持断线那一刻的值。一个无信号态是"别停"的急停通道，
   * 把这个通道存在的理由整个反过来了。
   *
   * 但"从来没接过接收机"不能等于"一直刹着" —— 那样没插遥控的车永远不能动。
   * 所以只有 RC_LINK_LOST(曾经活过、现在断了)才主张刹车；RC_LINK_NEVER
   * 什么都不主张。不活的源什么都不主张，失效的源主张它的安全值。
   */
  if (RC_LinkState_RC3() == RC_LINK_LOST) {
    return 1U;
  }
#if RC_SW_USE_PWM
  return (RC_GetPulseWidth_RC3_us() > RC_SW_PWM_ACTIVE_HIGH_US) ? 1U : 0U;
#else
  return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET) ? 1U : 0U;
#endif
}

uint8_t RC_GetSwitch_RC4(void)
{
#if RC_SW_USE_PWM
  return (RC_GetPulseWidth_RC4_us() > RC_SW_PWM_ACTIVE_HIGH_US) ? 1U : 0U;
#else
  return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_SET) ? 1U : 0U;
#endif
}
