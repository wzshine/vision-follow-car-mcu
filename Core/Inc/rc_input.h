#ifndef __RC_INPUT_H
#define __RC_INPUT_H

#include "main.h"

#define RC_TIM_HANDLE htim3
#define RC_STEER_CHANNEL    TIM_CHANNEL_3   /* RC1 = PC8 (转向) */
#define RC_DRIVE_CHANNEL    TIM_CHANNEL_4   /* RC2 = PC9 (油门) */
#define RC3_CHANNEL         TIM_CHANNEL_2   /* RC3 = PC7 (远程刹车-置位) */
#define RC4_CHANNEL         TIM_CHANNEL_1   /* RC4 = PC6 (远程刹车-复位/释放) */

#define RC_MIN_US     1000
#define RC_MID_US     1500
#define RC_MAX_US     2000

/* RC3 当作"开关量"使用(单通道两位开关即够：~1000/~2000us 两个位置直接表示刹车/释放，
 * 开关位置本身保持状态，无需软件自锁，故 RC4 不参与刹车逻辑，仅保留采集供诊断)。
 * 外部电路：100Ω 串联 + 10k 下拉到地，空闲 0V，高电平有效。
 *  - RC_SW_USE_PWM=1：按 RC PWM 处理(实测为此)。脉宽 > RC_SW_PWM_ACTIVE_HIGH_US 视为"激活=刹车"。
 *    实测拨动在 ~1000(低)/~2000(高) 间切换，阈值取中点 1500us：≈2000(高)→刹车，≈1000(低)→释放。
 *    若刹车/释放方向反了，把 RC_GetSwitch_RC3 里的 '>' 改成 '<' 即可。
 *  - RC_SW_USE_PWM=0：按稳定 DC 电平处理，直接读 GPIO(激活=高电平)。
 * 用 CLI "RC?" 看 sw 字段验证。 */
#define RC_SW_USE_PWM             1
#define RC_SW_PWM_ACTIVE_HIGH_US  1500   /* PWM 模式：脉宽 > 此值视为激活(刹车位/高位) */

void RC_Input_Init(void);
void RC_Input_OnCapture(TIM_HandleTypeDef *htim);
int16_t RC_GetPulseWidth_us(void);
int16_t RC_GetNormalized(int16_t min, int16_t max);
uint32_t RC_GetUpdateCount(void);
uint32_t RC_GetCallbackCount(void);
void RC_Input_Init_CH2(void);
int16_t RC_GetPulseWidth_CH2_us(void);
uint32_t RC_GetUpdateCount_CH2(void);
uint32_t RC_GetCallbackCount_CH2(void);

/* RC3 (PC7 / TIM3_CH2) 与 RC4 (PC6 / TIM3_CH1) */
void RC_Input_Init_RC3(void);
void RC_Input_Init_RC4(void);
int16_t RC_GetPulseWidth_RC3_us(void);
int16_t RC_GetPulseWidth_RC4_us(void);
uint32_t RC_GetUpdateCount_RC3(void);
uint32_t RC_GetUpdateCount_RC4(void);
uint32_t RC_GetCallbackCount_RC3(void);
uint32_t RC_GetCallbackCount_RC4(void);
uint8_t RC_GetSwitch_RC3(void);   /* 1=激活 → 刹车置位 */
uint8_t RC_GetSwitch_RC4(void);   /* 1=激活 → 刹车复位/释放 */

#endif
