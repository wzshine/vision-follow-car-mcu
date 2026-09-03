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
  /*
   * 允不允许**动**。由主循环每趟从整车使能(PD10)喂进来。
   *
   * ! 这一位放在被调用的这一侧，而不是九个调用点各写一个 if ——
   *   因为原来的毛病正是"规则写在调用点里，下一个调用点不会知道"：
   *   PD10 在驱动那几处都判了(allow_motor / foc_allow_drive / AUTO arm /
   *   台架命令)，而 ServoCAN_Enable 六处、GoToAbsolutePosition 三处
   *   **一处都没判** —— 使能开关断开时驱动被禁止，转向照样使能、照样
   *   下发目标，路轮在一台"未被授权移动"的车上转。
   *   在九个地方补 if 是把同一个病再抄九遍；放进这里，新加的调用点
   *   **想漏也漏不掉**。
   */
  uint8_t actuation_allowed;
  uint32_t refused_count;   /* 因不允许而挡下的动作次数；挡下要看得见，不能静默 */
} ServoCAN_t;

void ServoCAN_Init(ServoCAN_t *s, uint8_t node_id);
/* 每趟主循环喂一次。1 = 整车使能合上、允许转向动作。
 * 只挡"让它动"的命令；关使能、心跳、读位置任何时候都放行 ——
 * **停下来永远是允许的**。 */
void ServoCAN_SetActuationAllowed(ServoCAN_t *s, uint8_t allowed);
uint32_t g_servo_refused_count(void);   /* 被闸挡下的动作次数(诊断) */
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
