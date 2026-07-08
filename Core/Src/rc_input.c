#include "rc_input.h"
#include "stm32f4xx_hal.h"

static volatile uint32_t s_pulse_width_us = RC_MID_US;
static volatile uint32_t s_pwm_update_count = 0;
static volatile uint32_t s_rising_edge_time = 0;
static volatile uint8_t s_capture_state = 0;
static volatile uint32_t s_callback_count = 0;

static volatile uint32_t s_pulse_width_ch2_us = RC_MID_US;
static volatile uint32_t s_pwm_update_count_ch2 = 0;
static volatile uint32_t s_rising_edge_time_ch2 = 0;
static volatile uint8_t s_capture_state_ch2 = 0;
static volatile uint32_t s_callback_count_ch2 = 0;

/* RC3 = PC7 / TIM3_CH2 */
static volatile uint32_t s_pulse_width_rc3_us = RC_MID_US;
static volatile uint32_t s_pwm_update_count_rc3 = 0;
static volatile uint32_t s_rising_edge_time_rc3 = 0;
static volatile uint8_t s_capture_state_rc3 = 0;
static volatile uint32_t s_callback_count_rc3 = 0;

/* RC4 = PC6 / TIM3_CH1 */
static volatile uint32_t s_pulse_width_rc4_us = RC_MID_US;
static volatile uint32_t s_pwm_update_count_rc4 = 0;
static volatile uint32_t s_rising_edge_time_rc4 = 0;
static volatile uint8_t s_capture_state_rc4 = 0;
static volatile uint32_t s_callback_count_rc4 = 0;

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
      }
      s_capture_state_rc4 = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, RC4_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
}

int16_t RC_GetPulseWidth_us(void)
{
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
