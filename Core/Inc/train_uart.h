#ifndef INC_TRAIN_UART_H_
#define INC_TRAIN_UART_H_

#include <stdint.h>

#define TRAIN_FRAME_LEN     22
#define TRAIN_SOF0           0xAA
#define TRAIN_SOF1           0x55
#define TRAIN_VER            0x01

#define TRAIN_TYPE_CTRL      0x01
#define TRAIN_TYPE_FOLLOW    0x02
#define TRAIN_TYPE_AUTO_RDY  0x03
#define TRAIN_TYPE_HEARTBEAT 0x04
#define TRAIN_TYPE_MANUAL    0x05
#define TRAIN_TYPE_REMOTE    0x06
#define TRAIN_TYPE_COLLECT   0x07   /* 采集(SLAM)上报：基础行驶数据(speed/steer/gear/brake)。
                                     * IMU 字段为后续帧扩展(当前 22 字节帧 [18:19] 仅 2 字节预留,
                                     * 容不下 IMU)，待 IMU 驱动 + Jetson 帧格式约定后再加长。 */

#define TRAIN_TYPE_CMD_FOLLOW   0x10
#define TRAIN_TYPE_CMD_AUTO     0x11
#define TRAIN_TYPE_CMD_CONFIG   0x12

/* 自动驾驶 0x11 指令帧 FLAGS 位(见 AUTONOMOUS_MODE_SPEC §3) */
#define TRAIN_AUTO_FLAG_ENABLE   0x01U   /* bit0: Jetson 请求使能(MCU 仍以自身安全条件为准) */

/* 自动驾驶 0x03 状态帧 STATUS 位(见 AUTONOMOUS_MODE_SPEC §4) */
#define TRAIN_AUTO_ST_ARMED      0x01U   /* 已 arm,可接受 0x11 */
#define TRAIN_AUTO_ST_FOC_VALID  0x02U   /* FOC 状态新鲜 */
#define TRAIN_AUTO_ST_VEH_EN     0x04U   /* 整车使能 */
#define TRAIN_AUTO_ST_OVERRIDE   0x08U   /* 急停/接管(遥控/手动)生效 */
#define TRAIN_AUTO_ST_IN_AUTO    0x10U   /* 当前处于 AUTO 模式 */

typedef struct {
  uint16_t seq;
  uint32_t ts_ms;
  uint16_t speed_01;
  int16_t  steer_01;
  uint16_t thr_ref;
  uint8_t  brake;
  uint8_t  gear;
} TrainFrameData_t;

typedef struct {
  int16_t  steer_01;
  uint16_t thr_ref;
  uint8_t  brake;
} FollowCmd_t;

/* Host→MCU 自动驾驶指令 0x11 解析结果(见 AUTONOMOUS_MODE_SPEC §3) */
typedef struct {
  int16_t  steer_01;            /* 目标前轮转角 0.1°，左负右正 */
  int16_t  target_speed_mm_s;   /* 目标车速 mm/s，+前进 −后退，0=停 */
  uint8_t  brake;               /* 1=主动刹车(优先于 target_speed) */
  uint8_t  gear;                /* 0=P 1=R 2=N 3=D，回显/校验 */
  uint8_t  flags;               /* bit0=AUTO_ENABLE */
} AutoCmd_t;

/* MCU→Host 自动驾驶就绪/状态 0x03(见 AUTONOMOUS_MODE_SPEC §4) */
typedef struct {
  uint16_t seq;
  uint32_t ts_ms;
  int16_t  spd_meas_mm_s;       /* 实测车速 mm/s，+前进 −后退 */
  int16_t  steer_fb_01;         /* 实测转向 0.1°，左负右正 */
  uint8_t  status;              /* TRAIN_AUTO_ST_* 位组合 */
  uint8_t  gear;                /* 0=P 1=R 2=N 3=D */
} AutoStatus_t;

void TrainUART_Init(void);
uint16_t TrainUART_GetNextSeq(void);
int TrainUART_SendFrame(const TrainFrameData_t *data);
int TrainUART_SendFollowReady(const TrainFrameData_t *data);
int TrainUART_SendAutoStatus(const AutoStatus_t *st);          /* 自动驾驶就绪/状态帧 TYPE=0x03 */
int TrainUART_SendManualNotify(const TrainFrameData_t *data);
int TrainUART_SendRemoteNotify(const TrainFrameData_t *data);
int TrainUART_SendCollectFrame(const TrainFrameData_t *data);  /* 采集(SLAM)上报帧 */
void TrainUART_ReceiveISR(void);
int TrainUART_PollFollowCmd(FollowCmd_t *out);
int TrainUART_PollAutoCmd(AutoCmd_t *out);                     /* 解析自动驾驶指令帧 TYPE=0x11 */
int TrainUART_PollLine(char *out, uint16_t out_size);
void TrainUART_SendText(const char *text);

#endif
