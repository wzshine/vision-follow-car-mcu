#ifndef SCREEN_H
#define SCREEN_H

#include <stdint.h>

/* 档位：屏 0x05E1；PRND 另有四路 GPIO（board_inputs.h） */
#define SCREEN_ADDR_GEAR        0x05E1
#define SCREEN_ADDR_SPEED       0x05C1
/* 工作模式：仅由屏 0x05A1 键值切换，无独立模式 GPIO */
#define SCREEN_ADDR_MODE        0x05A1
#define SCREEN_ADDR_CONNECTION  0x05E6
#define SCREEN_ADDR_SPEED_DISPLAY 0x02E5
#define SCREEN_ADDR_BATTERY       0x05EB
/* VBUS 母线电压 / 母线电流显示。数据来自 FOC 0x212 遥测：
 * 电压 0.1V/字(mV/100)，电流 0.1A/字(cA/10，含符号)。 */
#define SCREEN_ADDR_VBUS_VOLT     0x05EC
#define SCREEN_ADDR_VBUS_CURR     0x05F0
/* 整车使能文本指示：使能时显示 "R"，未使能显示空格。屏文本控件，UTF-8。 */
#define SCREEN_ADDR_ENABLE        0x0608
/* 采集(SLAM)确认指示：Jetson 确认 slam 已起来建图时置 1，否则 0。
 * 写法同连接状态(0x05E6)：Screen_SendData(addr, 1/0)。仅采集模式刷新。 */
#define SCREEN_ADDR_COLLECT_ACK   0x0618
/* 保存状态指示：Jetson 最近一次存图成功且新鲜(自动存图在跑)时置 1，否则 0。
 * 新鲜度判断在 Jetson 侧(存图失败/停滞即 0)，控制板仅 1.5s 超时兜底。仅采集模式刷新。 */
#define SCREEN_ADDR_SAVE_STATUS   0x0619

#define SCREEN_KEY_GEAR_P     0x0008
#define SCREEN_KEY_GEAR_R     0x0004
#define SCREEN_KEY_GEAR_N     0x0002
#define SCREEN_KEY_GEAR_D     0x0001

#define SCREEN_KEY_SPEED_EXTRA_LOW  0x0008
#define SCREEN_KEY_SPEED_LOW        0x0004
#define SCREEN_KEY_SPEED_MEDIUM     0x0002
#define SCREEN_KEY_SPEED_HIGH       0x0001

#define SCREEN_KEY_MODE_MANUAL      0x0010
#define SCREEN_KEY_MODE_REMOTE      0x0008
#define SCREEN_KEY_MODE_TRAINING    0x0004
#define SCREEN_KEY_MODE_FOLLOW      0x0002
#define SCREEN_KEY_MODE_AUTO        0x0001
#define SCREEN_KEY_MODE_COLLECT     0x0020  /* 采集模式(SLAM 数据上报)，与训练分开 */

#define SPEED_SCALE_EXTRA_LOW   (0.35f)
#define SPEED_SCALE_LOW         (0.60f)
#define SPEED_SCALE_MEDIUM      (0.80f)
#define SPEED_SCALE_HIGH        (1.00f)

static inline float Screen_GetSpeedScale(uint16_t speed_key)
{
  if (speed_key == SCREEN_KEY_SPEED_EXTRA_LOW)
  {
    return SPEED_SCALE_EXTRA_LOW;
  }
  if (speed_key == SCREEN_KEY_SPEED_LOW)
  {
    return SPEED_SCALE_LOW;
  }
  if (speed_key == SCREEN_KEY_SPEED_MEDIUM)
  {
    return SPEED_SCALE_MEDIUM;
  }
  if (speed_key == SCREEN_KEY_SPEED_HIGH)
  {
    return SPEED_SCALE_HIGH;
  }
  /* 认不出的键值取**最慢**的档，不是 100%。
   * 从前这里返回 1.0f —— 一个认不出来的按键值会静静地主张满速上限。
   * 主循环确实在上游过滤过取值，所以今天走不到这里；但一个默认值的方向
   * 不该指望调用方替它兜底，而这个默认值的方向原来是反的。 */
  return SPEED_SCALE_EXTRA_LOW;
}

void Screen_Init(void);
void Screen_SetInitialValues(void);
void Screen_SendData(uint16_t addr, uint16_t data);
void Screen_ReadVar(uint16_t addr, uint8_t n_words);
uint16_t Screen_GetKey(void);
void Screen_GetKeyAndAddr(uint16_t *key_out, uint16_t *addr_out);
void Screen_ReceiveISR(void);
uint32_t Screen_GetLastRxMs(void);
void Screen_SetGear(uint16_t gear_key);
void Screen_SetSpeed(uint16_t speed_key);
void Screen_SetMode(uint16_t mode_key);
void Screen_SetConnectionStatus(uint8_t connected);
void Screen_SetCollectAck(uint8_t on);                /* 采集确认灯 0x0618(1/0) */
void Screen_SetSaveStatus(uint8_t on);                /* 保存状态灯 0x0619(1/0) */
void Screen_SendText(uint16_t addr, const char *s);   /* 写文本(UTF-8字节)+0xFFFF结束符 */
void Screen_SetEnableIndicator(uint8_t enabled);      /* 使能→"R"，未使能→空格 @0x0608 */

#endif
