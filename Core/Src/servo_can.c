#include "servo_can.h"
#include <string.h>
#include <stdbool.h>

static void pack_int32_to_bytes(int32_t value, uint8_t *bytes)
{
  bytes[0] = (uint8_t)(value & 0xFF);
  bytes[1] = (uint8_t)((value >> 8) & 0xFF);
  bytes[2] = (uint8_t)((value >> 16) & 0xFF);
  bytes[3] = (uint8_t)((value >> 24) & 0xFF);
}

static int32_t unpack_int32_from_bytes(const uint8_t *bytes)
{
  return ((int32_t)bytes[0]) | ((int32_t)bytes[1] << 8) | ((int32_t)bytes[2] << 16) | ((int32_t)bytes[3] << 24);
}

void ServoCAN_Init(ServoCAN_t *s, uint8_t node_id)
{
  if (s == NULL || node_id > 99)
  {
    return;
  }
  s->node_id = node_id;
  s->can_id = 0x64U + node_id;
  s->heartbeat_seen = false;
  s->status = 0;
  s->abs_pos = 0;
  s->alarm_code_low = 0;
  s->alarm_code_high = 0;
}

void ServoCAN_OnRxFrame(ServoCAN_t *s, uint32_t std_id, const uint8_t data[8])
{
  if (s == NULL || std_id != s->can_id || data == NULL)
  {
    return;
  }
  if (data[0] != s->node_id)
  {
    return;
  }

  switch (data[1])
  {
    case 0x10U:
      s->status = data[2];
      s->abs_pos = unpack_int32_from_bytes(&data[3]);
      s->heartbeat_seen = true;
      break;
    case 0x05U:
      s->alarm_code_low = data[2];
      s->alarm_code_high = data[3];
      break;
    default:
      break;
  }
}

static HAL_StatusTypeDef send_frame(ServoCAN_t *s, uint8_t cmd, const uint8_t payload[6])
{
  if (s == NULL)
  {
    return HAL_ERROR;
  }

  CAN_TxHeaderTypeDef txHeader;
  uint8_t txData[8];
  uint32_t TxMailbox;

  txHeader.StdId = s->can_id;
  txHeader.ExtId = 0;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.IDE = CAN_ID_STD;
  txHeader.DLC = 8;
  txHeader.TransmitGlobalTime = DISABLE;

  txData[0] = s->node_id;
  txData[1] = cmd;
  memcpy(&txData[2], payload, 6);
  txData[7] = 0xFF;

  return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &TxMailbox);
}

HAL_StatusTypeDef ServoCAN_SendLink(ServoCAN_t *s)
{
  uint8_t payload[6] = {0};
  return send_frame(s, 0x08U, payload);
}

HAL_StatusTypeDef ServoCAN_Enable(ServoCAN_t *s, uint8_t enable)
{
  uint8_t payload[6] = {0};
  payload[0] = enable ? 1U : 0U;
  return send_frame(s, 0x25U, payload);
}

HAL_StatusTypeDef ServoCAN_SetMaxSpeedForPositioning(ServoCAN_t *s, int32_t speed)
{
  uint8_t payload[6] = {0};
  pack_int32_to_bytes(speed, &payload[1]);
  return send_frame(s, 0x23U, payload);
}

HAL_StatusTypeDef ServoCAN_GoToAbsolutePosition(ServoCAN_t *s, int32_t position)
{
  uint8_t payload[6] = {0};
  pack_int32_to_bytes(position, &payload[1]);
  return send_frame(s, 0x22U, payload);
}

HAL_StatusTypeDef ServoCAN_MoveRelative(ServoCAN_t *s, int32_t offset)
{
  uint8_t payload[6] = {0};
  pack_int32_to_bytes(offset, &payload[1]);
  return send_frame(s, 0x21U, payload);
}

HAL_StatusTypeDef ServoCAN_SetSpeedMode(ServoCAN_t *s, int32_t speed)
{
  uint8_t payload[6] = {0};
  pack_int32_to_bytes(speed, &payload[1]);
  return send_frame(s, 0x20U, payload);
}

HAL_StatusTypeDef ServoCAN_StopHoldEnable(ServoCAN_t *s)
{
  return send_frame(s, 0x26U, (uint8_t[6]){0});
}

HAL_StatusTypeDef ServoCAN_StopAndDisable(ServoCAN_t *s)
{
  return send_frame(s, 0x28U, (uint8_t[6]){0});
}

HAL_StatusTypeDef ServoCAN_ClearAlarm(ServoCAN_t *s)
{
  return send_frame(s, 0x81U, (uint8_t[6]){0});
}

HAL_StatusTypeDef ServoCAN_WriteCurrentPositionAsZero(ServoCAN_t *s, int32_t new_pos)
{
  uint8_t payload[6] = {0};
  pack_int32_to_bytes(new_pos, &payload[1]);
  return send_frame(s, 0x50U, payload);
}
