#ifndef FOC_CAN_H
#define FOC_CAN_H

#include <stdint.h>
#include "main.h"

/*
 * DlaBoard_A -> FOC2 正式驾驶请求协议：1Mbps 经典 CAN，标准帧 DLC=8。
 * 旧 0x200 PWM 脉宽协议已废弃，控制板只向 FOC2 下发车辆语义。
 *
 * 控制板 -> FOC2  标准帧 ID=FOC_CAN_DRIVE_REQ_ID_STD
 *   [0:1] throttle_permille uint16 LE  0..1000
 *   [2:3] steer_permille    int16 LE   -1000..1000
 *   [4]   flags             bit0=allow_drive bit1=brake
 *   [5]   gear              0=N 1=F 2=R
 *   [6]   seq               alive counter
 *   [7]   timeout_10ms      0=FOC2 默认超时；非 0 建议 30(300ms)
 *
 * 驱动板 -> 控制板  状态帧 DLC=8（周期上行, ~20ms）
 *   [0]   health flags bit0=foc1_run bit1=foc2_run bit2=foc1_edge_ok
 *         bit3=foc2_edge_ok bit4=foc1_nfault_ok bit5=foc2_nfault_ok
 *         bit6=cmd_recent
 *   [1]   seq
 *   [2:3] foc1 vehicle mechanical rpm int16 LE
 *   [4:5] foc2 vehicle mechanical rpm int16 LE
 *   [6]   echoed last command flags
 *   [7]   bit7=last command valid, bit0-3=last command gear
 *
 * 驱动板 -> 控制板  母线遥测 DLC=8（供串口屏显示）
 *   [0:1] vbus_mV  uint16 LE  母线电压(毫伏)
 *   [2:3] ibus_cA  int16 LE   母线电流(厘安, 10mA/LSB)
 *   [4]   seq      心跳计数
 *   [5:7] 预留
 *
 * 两种驱动板拓扑，状态/遥测帧格式一致：
 *   A) 单块 FOC_double(双电机)：状态发 0x211(同时填 foc1/foc2 两个轮速),
 *      遥测发 0x212。
 *   B) 两块 FOC_single(各一路)：左轮板 状态 0x211(只填 foc1 槽)+遥测 0x212;
 *      右轮板 状态 0x213(只填 foc2 槽)+遥测 0x214。本板把两块板的上行合并成
 *      与双电机板等价的 foc1/foc2 视图(轮速、健康位、母线电流求和)。
 */

#define FOC_CAN_DRIVE_REQ_ID_STD  0x210U   /* 广播行走请求：两块驱动板都接收 */
#define FOC_CAN_STATUS_ID_FOC2    0x211U   /* 双电机板 / 单路左轮板 状态      */
#define FOC_CAN_TELEM_ID_FOC2     0x212U   /* 双电机板 / 单路左轮板 遥测      */
#define FOC_CAN_STATUS_ID_R       0x213U   /* 单路右轮板 状态                 */
#define FOC_CAN_TELEM_ID_R        0x214U   /* 单路右轮板 遥测                 */

#define FOC_CAN_TX_PERIOD_MS      20U
#define FOC_CAN_DEFAULT_TIMEOUT_10MS 30U
#define FOC_CAN_FLAG_ALLOW_DRIVE  0x01U
#define FOC_CAN_FLAG_BRAKE        0x02U

typedef struct {
  uint16_t throttle_permille;
  int16_t steer_permille;
  uint8_t flags;
  uint8_t gear;
  uint8_t seq;
  uint8_t timeout_10ms;
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t last_status_ms;
  uint8_t status_seen;
  uint8_t status_health;
  uint8_t status_seq;
  int16_t foc1_rpm;
  int16_t foc2_rpm;
  uint8_t status_cmd_flags;
  uint8_t status_cmd_valid;
  uint8_t status_cmd_gear;
  uint8_t last_status[8];
  /* 母线遥测：左轮板/双电机板(0x212) */
  uint16_t vbus_mV;
  int16_t  ibus_cA;
  uint8_t  telem_seq;
  uint8_t  telem_seen;
  uint32_t last_telem_ms;
  /* 母线遥测：单路右轮板(0x214)。双电机拓扑下这些字段保持 0，不影响显示。 */
  uint16_t vbus_mV_r;
  int16_t  ibus_cA_r;
  uint8_t  telem_seq_r;
  uint8_t  telem_seen_r;
  uint32_t last_telem_ms_r;
} FocCAN_t;

void FocCAN_Init(FocCAN_t *fc);
void FocCAN_OnRxFrame(FocCAN_t *fc, uint32_t std_id, const uint8_t data[8]);
HAL_StatusTypeDef FocCAN_SendDriveRequest(FocCAN_t *fc,
                                          uint16_t throttle_permille,
                                          int16_t steer_permille,
                                          uint8_t allow_drive,
                                          uint8_t brake,
                                          uint8_t gear,
                                          uint8_t timeout_10ms);
void FocCAN_TestPollUart(FocCAN_t *fc);
uint8_t FocCAN_TestActive(void);
uint8_t FocCAN_BrakeTestActive(void);   /* UART BRAKE 测试保持期间为 1，正式刹车应让位 */
void FocCAN_TestGet(uint16_t *throttle_permille, int16_t *steer_permille,
                    uint8_t *allow_drive, uint8_t *brake, uint8_t *gear);
uint8_t FocCAN_TestTimeout10ms(void);
uint8_t FocCAN_StatusFresh(const FocCAN_t *fc, uint32_t now_ms);
uint16_t FocCAN_GetSpeed_01kmh(const FocCAN_t *fc, uint32_t now_ms, uint16_t fallback_01kmh);
uint8_t FocCAN_TelemFresh(const FocCAN_t *fc, uint32_t now_ms);
uint16_t FocCAN_GetVbus_mV(const FocCAN_t *fc, uint32_t now_ms);  /* 不新鲜返回 0 */
int16_t  FocCAN_GetIbus_cA(const FocCAN_t *fc, uint32_t now_ms);  /* 厘安(10mA/LSB)，不新鲜返回 0 */

#endif /* FOC_CAN_H */
