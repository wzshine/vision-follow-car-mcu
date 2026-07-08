#ifndef INC_STEERING_H_
#define INC_STEERING_H_

#include <stdint.h>

typedef enum {
  MODE_MANUAL = 0,
  MODE_REMOTE,
  MODE_TRAINING,
  MODE_FOLLOW,
  MODE_COLLECT,     /* 采集(SLAM)：行为同训练，但上报 TYPE_COLLECT 帧，与训练分开 */
  MODE_AUTO         /* 自动驾驶：闭环速度 + 0x11 指令 / 0x03 状态(见 AUTONOMOUS_MODE_SPEC) */
} SteeringMode_t;

extern volatile SteeringMode_t g_steering_mode;
extern uint8_t servo_enabled_sent;

#endif
