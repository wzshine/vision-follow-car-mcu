#ifndef BRAKE_CAN_H
#define BRAKE_CAN_H

#include <stdint.h>
#include "main.h"

/*
 * DlaBoard_A -> 刹车板 DlaCart_v2.1 刹车命令：经典 CAN，标准帧 DLC=8，hcan1。
 *
 * 控制板 -> 刹车板  标准帧 ID=BRAKE_CAN_CMD_ID_STD
 *   [0]   flags          bit0=apply(拉闸刹车)
 *   [1]   seq            alive counter
 *   [2]   timeout_10ms   0=刹车板默认(300ms)；非 0 建议 30(=300ms)
 *   [3]   magic          固定 0x42（刹车板据此校验帧有效）
 *   [4:7] 预留
 *
 * 刹车板为脉冲式电动刹车执行器：收到 apply 跳变即驱动 ~500ms 脉冲到位，
 * 之后保持；同状态的周期帧仅作保活(刷新超时)，不重复脉冲。超时(默认 300ms)
 * 无帧→刹车板自动松刹车。故控制板需以 < timeout 的周期持续发送当前刹车意图。
 */

#define BRAKE_CAN_CMD_ID_STD      0x220U
#define BRAKE_CAN_FLAG_APPLY      0x01U
#define BRAKE_CAN_MAGIC           0x42U
#define BRAKE_CAN_TX_PERIOD_MS    100U   /* 同状态保活周期 */
#define BRAKE_CAN_TIMEOUT_10MS    30U    /* 下发给刹车板的失效超时(=300ms) */

/* 向刹车板下发刹车意图(CAN 0x220)。want_brake!=0=拉闸，=0=松刹车。
 * 内部限频：状态变化立即发，同状态每 BRAKE_CAN_TX_PERIOD_MS 保活一次。
 * 应在主循环中以足够高的频率周期调用(传入当前刹车仲裁结果)。 */
void BrakeCAN_Send(uint8_t want_brake);

#endif /* BRAKE_CAN_H */
