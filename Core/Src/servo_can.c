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

/*
 * 会让转向机构**动**的命令，统一在这里过一道闸。
 *
 * 放在被调用的这一侧，不是放在调用点：原来的毛病正是"规则写在调用点里,
 * 下一个调用点不会知道" —— 整车使能 PD10 在驱动那几处都判了，而
 * ServoCAN_Enable 六处、GoToAbsolutePosition 三处一处都没判。
 * 在九个地方补 if 是把同一个病抄九遍；放进这里，新加的调用点想漏也漏不掉。
 *
 * ! 只挡"让它动"的：使能=1、设定位置、相对移动、速度模式、写零点。
 *   放行的是使能=0、停止、心跳、清报警 —— **停下来永远是允许的**,
 *   一个连"停"都要许可的闸，在最需要它的时候就是障碍。
 */
static uint8_t may_actuate(ServoCAN_t *s)
{
  if (s == NULL) {
    return 0U;
  }
  if (s->actuation_allowed != 0U) {
    return 1U;
  }
  s->refused_count++;   /* 挡下要看得见 —— 静默的拒绝和坏掉的链路从外面一样 */
  return 0U;
}

void ServoCAN_SetActuationAllowed(ServoCAN_t *s, uint8_t allowed)
{
  if (s != NULL) {
    s->actuation_allowed = (allowed != 0U) ? 1U : 0U;
  }
}

HAL_StatusTypeDef ServoCAN_SendLink(ServoCAN_t *s)
{
  uint8_t payload[6] = {0};
  return send_frame(s, 0x08U, payload);
}

HAL_StatusTypeDef ServoCAN_Enable(ServoCAN_t *s, uint8_t enable)
{
  uint8_t payload[6] = {0};
  /* 关使能不过闸：停下来永远是允许的。 */
  if ((enable != 0U) && (may_actuate(s) == 0U)) {
    return HAL_ERROR;
  }

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
  if (may_actuate(s) == 0U) {
    return HAL_ERROR;
  }

  pack_int32_to_bytes(position, &payload[1]);
  return send_frame(s, 0x22U, payload);
}

HAL_StatusTypeDef ServoCAN_MoveRelative(ServoCAN_t *s, int32_t offset)
{
  uint8_t payload[6] = {0};
  if (may_actuate(s) == 0U) {
    return HAL_ERROR;
  }

  pack_int32_to_bytes(offset, &payload[1]);
  return send_frame(s, 0x21U, payload);
}

HAL_StatusTypeDef ServoCAN_SetSpeedMode(ServoCAN_t *s, int32_t speed)
{
  uint8_t payload[6] = {0};
  if (may_actuate(s) == 0U) {
    return HAL_ERROR;
  }

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
  if (may_actuate(s) == 0U) {
    return HAL_ERROR;
  }

  pack_int32_to_bytes(new_pos, &payload[1]);
  return send_frame(s, 0x50U, payload);
}

/* 主循环里那个实例的拒绝计数，给诊断用(ST? / 参数表)。
 * 挡下要看得见 —— 一个静默的拒绝，和一条坏掉的链路，从外面看是一样的。 */
extern ServoCAN_t g_servo;
uint32_t g_servo_refused_count(void)
{
  return g_servo.refused_count;
}
