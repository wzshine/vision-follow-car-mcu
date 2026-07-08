#include "brake_can.h"

/*
 * 刹车板 0x220 发送实现。原先错误地放在 board_relays.c 的
 * Board_Relay_BrakeCylinder()（命名暗示本地继电器，实为 CAN 发送），
 * 现拆为独立模块，与 foc_can / servo_can 同级。
 */

void BrakeCAN_Send(uint8_t want_brake)
{
  static uint8_t s_last_brake = 0xFFU;
  static uint32_t s_last_tx_ms = 0U;
  static uint8_t s_seq = 0U;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8] = {0};
  uint32_t mailbox;
  uint32_t now_ms = HAL_GetTick();

  want_brake = (want_brake != 0U) ? 1U : 0U;

  /* 同状态限频：仅每 BRAKE_CAN_TX_PERIOD_MS 保活一次；状态变化立即发 */
  if ((want_brake == s_last_brake) &&
      ((now_ms - s_last_tx_ms) < BRAKE_CAN_TX_PERIOD_MS)) {
    return;
  }

  tx_header.StdId = BRAKE_CAN_CMD_ID_STD;
  tx_header.ExtId = 0U;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.IDE = CAN_ID_STD;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;

  tx_data[0] = (want_brake != 0U) ? BRAKE_CAN_FLAG_APPLY : 0U;
  tx_data[1] = s_seq++;
  tx_data[2] = BRAKE_CAN_TIMEOUT_10MS;
  tx_data[3] = BRAKE_CAN_MAGIC;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U) {
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox) == HAL_OK) {
      s_last_brake = want_brake;
      s_last_tx_ms = now_ms;
    }
  }
}
