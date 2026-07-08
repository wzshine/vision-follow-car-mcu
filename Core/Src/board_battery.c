#include "board_battery.h"
#include "main.h"
#include "stm32f4xx_hal.h"

/* 电池：PC1 = ADC1_IN11，经分压后送入 ADC。按实际电阻改 BAT_VDIV_RATIO = (R上+R下)/R下 */
#define BAT_ADC_REF_MV     3300U
#define BAT_VDIV_RATIO     (1049.0f / 51.0f)

/* 4S LiPo 粗阈值 (mV)，与 DlaCart 一致；可按实测标定 */
#define BAT_100_MV         15800U
#define BAT_75_MV          14200U
#define BAT_50_MV          12600U
#define BAT_25_MV          11200U

#define PEDAL_ADC_CHANNEL   ADC_CHANNEL_8
#define PEDAL_SAMPLE_TIME   ADC_SAMPLETIME_56CYCLES
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_11
#define BATTERY_SAMPLE_TIME ADC_SAMPLETIME_144CYCLES

uint8_t Battery_GetLevel(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint32_t adc_raw = 0U;
  uint32_t vbus_mv = 0U;

  (void)HAL_ADC_Stop(&hadc1);

  sConfig.Channel = BATTERY_ADC_CHANNEL;
  sConfig.Rank = 1U;
  sConfig.SamplingTime = BATTERY_SAMPLE_TIME;
  sConfig.Offset = 0U;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    goto restore_pedal;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    goto restore_pedal;
  }
  if (HAL_ADC_PollForConversion(&hadc1, 50U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    goto restore_pedal;
  }
  adc_raw = HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);

  vbus_mv = (uint32_t)((float)adc_raw * (float)BAT_ADC_REF_MV / 4096.0f * BAT_VDIV_RATIO);

restore_pedal:
  sConfig.Channel = PEDAL_ADC_CHANNEL;
  sConfig.Rank = 1U;
  sConfig.SamplingTime = PEDAL_SAMPLE_TIME;
  sConfig.Offset = 0U;
  (void)HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  (void)HAL_ADC_Start(&hadc1);

  if (vbus_mv >= BAT_100_MV)
  {
    return 0U;
  }
  if (vbus_mv >= BAT_75_MV)
  {
    return 1U;
  }
  if (vbus_mv >= BAT_50_MV)
  {
    return 2U;
  }
  if (vbus_mv >= BAT_25_MV)
  {
    return 3U;
  }
  return 4U;
}
