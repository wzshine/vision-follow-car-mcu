#ifndef __SERVO_CAN_H
#define __SERVO_CAN_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

typedef struct {
  uint8_t node_id;
  uint32_t can_id;
  bool heartbeat_seen;
  uint8_t status;
  int32_t abs_pos;
  uint8_t alarm_code_low;
  uint8_t alarm_code_high;
} ServoCAN_t;

void ServoCAN_Init(ServoCAN_t *s, uint8_t node_id);
void ServoCAN_OnRxFrame(ServoCAN_t *s, uint32_t std_id, const uint8_t data[8]);
HAL_StatusTypeDef ServoCAN_SendLink(ServoCAN_t *s);
HAL_StatusTypeDef ServoCAN_Enable(ServoCAN_t *s, uint8_t enable);
HAL_StatusTypeDef ServoCAN_SetMaxSpeedForPositioning(ServoCAN_t *s, int32_t speed);
HAL_StatusTypeDef ServoCAN_GoToAbsolutePosition(ServoCAN_t *s, int32_t position);
HAL_StatusTypeDef ServoCAN_MoveRelative(ServoCAN_t *s, int32_t offset);
HAL_StatusTypeDef ServoCAN_SetSpeedMode(ServoCAN_t *s, int32_t speed);
HAL_StatusTypeDef ServoCAN_StopHoldEnable(ServoCAN_t *s);
HAL_StatusTypeDef ServoCAN_StopAndDisable(ServoCAN_t *s);
HAL_StatusTypeDef ServoCAN_ClearAlarm(ServoCAN_t *s);
HAL_StatusTypeDef ServoCAN_WriteCurrentPositionAsZero(ServoCAN_t *s, int32_t new_pos);
#define ServoCAN_SetPosition(s, pos) ServoCAN_GoToAbsolutePosition((s), (pos))

#endif
