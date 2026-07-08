#include "foc_can.h"
#include "brake_can.h"
#include "rc_input.h"
#include "train_uart.h"
#include "imu_lsm6dsv.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FOC_CAN_TEST_TIMEOUT_MS  300U
#define BRAKE_CAN_TEST_HOLD_MS   3000U
#define FOC_CAN_STATUS_FRESH_MS  500U
#define FOC_WHEEL_CIRCUMFERENCE_MM  (314L * 260L / 100L)

static uint16_t clamp_throttle_permille(long value)
{
  if (value < 0L) {
    return 0U;
  }
  if (value > 1000L) {
    return 1000U;
  }
  return (uint16_t)value;
}

static int16_t clamp_steer_permille(long value)
{
  if (value < -1000L) {
    return -1000;
  }
  if (value > 1000L) {
    return 1000;
  }
  return (int16_t)value;
}

static uint8_t clamp_u8(long value)
{
  if (value < 0L) {
    return 0U;
  }
  if (value > 255L) {
    return 255U;
  }
  return (uint8_t)value;
}

static uint8_t s_test_active;
static uint16_t s_test_throttle_permille;
static int16_t s_test_steer_permille;
static uint8_t s_test_allow;
static uint8_t s_test_brake;
static uint8_t s_test_gear;
static uint8_t s_test_timeout_10ms = FOC_CAN_DEFAULT_TIMEOUT_10MS;
static uint32_t s_test_last_cmd_ms;
static uint8_t s_brake_test_active;
static uint8_t s_brake_test_level;
static uint32_t s_brake_test_until_ms;

static uint16_t pct_to_permille(long pct)
{
  if (pct < 0L) {
    pct = 0L;
  }
  if (pct > 100L) {
    pct = 100L;
  }
  return (uint16_t)(pct * 10L);
}

static int16_t steer_pct_to_permille(long steer_pct)
{
  if (steer_pct < -100L) {
    steer_pct = -100L;
  }
  if (steer_pct > 100L) {
    steer_pct = 100L;
  }
  return (int16_t)(steer_pct * 10L);
}

static void test_set_stop(void)
{
  s_test_throttle_permille = 0U;
  s_test_steer_permille = 0;
  s_test_allow = 1U;
  s_test_brake = 0U;
  s_test_gear = 0U;
  s_test_timeout_10ms = FOC_CAN_DEFAULT_TIMEOUT_10MS;
}

static void test_ack(const char *text)
{
  TrainUART_SendText(text);
  TrainUART_SendText("\r\n");
}

static int16_t unpack_s16_le(const uint8_t *data)
{
  uint16_t u = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));

  return (int16_t)u;
}

void FocCAN_Init(FocCAN_t *fc)
{
  if (fc == NULL) {
    return;
  }
  memset(fc, 0, sizeof(*fc));
  s_test_active = 0U;
  s_brake_test_active = 0U;
  s_brake_test_level = 0U;
  s_brake_test_until_ms = 0U;
  test_set_stop();
  s_test_last_cmd_ms = 0U;
}

void FocCAN_TestPollUart(FocCAN_t *fc)
{
  char line[48];
  (void)fc;

  while (TrainUART_PollLine(line, sizeof(line)) != 0)
  {
    /* Jetson 建图/存图状态行(采集模式)：路由到全局，供屏 0x0618/0x0619。
     * 必须在此唯一的行消费者里处理——主循环不要再单独 PollLine，否则与 CLI 抢行。 */
    if (line[0] == 'M' && line[1] == 'A' && line[2] == 'P' && line[3] == ',')
    {
      extern volatile uint8_t g_host_collect_ack; extern volatile uint32_t g_host_map_rx_ms;
      if (line[4] == '1') g_host_collect_ack = 1U;
      else if (line[4] == '0') g_host_collect_ack = 0U;
      g_host_map_rx_ms = HAL_GetTick();
      continue;
    }
    if (line[0] == 'S' && line[1] == 'A' && line[2] == 'V' && line[3] == 'E' && line[4] == ',')
    {
      extern volatile uint8_t g_host_save_ok; extern volatile uint32_t g_host_save_rx_ms;
      if (line[5] == '1') g_host_save_ok = 1U;
      else if (line[5] == '0') g_host_save_ok = 0U;
      g_host_save_rx_ms = HAL_GetTick();
      continue;
    }
    if ((strcmp(line, "FOC?") == 0) || (strcmp(line, "FOC HELP") == 0))
    {
      test_ack("FOC D|R <0-100> [steer=-100..100] | STOP | OFF | RAW <thr0-1000> <steer> <flags> <gear> [timeout10ms]");
      test_ack("BRAKE 1|0 | BRAKE OFF | RC? | IMU? | ST?(FOC状态/VBUS)");
      continue;
    }
    if (strcmp(line, "RC?") == 0)
    {
      char buf[100];
      (void)snprintf(buf, sizeof(buf), "RC1(PC8)=%d upd=%lu cb=%lu | RC2(PC9)=%d upd=%lu cb=%lu",
                     (int)RC_GetPulseWidth_us(),
                     (unsigned long)RC_GetUpdateCount(),
                     (unsigned long)RC_GetCallbackCount(),
                     (int)RC_GetPulseWidth_CH2_us(),
                     (unsigned long)RC_GetUpdateCount_CH2(),
                     (unsigned long)RC_GetCallbackCount_CH2());
      test_ack(buf);
      {
        extern volatile uint8_t g_rc_brake;
        char buf2[100];
        (void)snprintf(buf2, sizeof(buf2),
                       "RC3(PC7)=%d sw=%u upd=%lu | RC4(PC6)=%d sw=%u upd=%lu | brake=%u",
                       (int)RC_GetPulseWidth_RC3_us(), (unsigned)RC_GetSwitch_RC3(),
                       (unsigned long)RC_GetUpdateCount_RC3(),
                       (int)RC_GetPulseWidth_RC4_us(), (unsigned)RC_GetSwitch_RC4(),
                       (unsigned long)RC_GetUpdateCount_RC4(),
                       (unsigned)g_rc_brake);
        test_ack(buf2);
      }
      continue;
    }
    if (strcmp(line, "BRK?") == 0)
    {
      extern volatile uint16_t g_dbg_gear; extern volatile uint8_t g_dbg_bsw, g_dbg_ven, g_dbg_fbrake;
      extern volatile uint32_t g_dbg_fbrake_chg;
      char buf[96];
      (void)snprintf(buf, sizeof(buf),
                     "BRK gear=0x%04X bsw=%u ven=%u fbrake=%u chg=%lu brktest=%u",
                     (unsigned)g_dbg_gear, (unsigned)g_dbg_bsw, (unsigned)g_dbg_ven,
                     (unsigned)g_dbg_fbrake, (unsigned long)g_dbg_fbrake_chg,
                     (unsigned)FocCAN_BrakeTestActive());
      test_ack(buf);
      continue;
    }
    if ((strcmp(line, "ST?") == 0) || (strcmp(line, "FOC ST?") == 0))
    {
      char buf[128];
      uint32_t now = HAL_GetTick();
      (void)snprintf(buf, sizeof(buf),
                     "FOC st fresh=%u health=0x%02X rpm=%d,%d rx=%lu echo=0x%02X valid=%u gear=%u | VBUS=%umV I=%dcA telem=%u tx=%lu",
                     (unsigned)FocCAN_StatusFresh(fc, now), (unsigned)fc->status_health,
                     (int)fc->foc1_rpm, (int)fc->foc2_rpm, (unsigned long)fc->rx_count,
                     (unsigned)fc->status_cmd_flags, (unsigned)fc->status_cmd_valid,
                     (unsigned)fc->status_cmd_gear,
                     (unsigned)FocCAN_GetVbus_mV(fc, now), (int)FocCAN_GetIbus_cA(fc, now),
                     (unsigned)FocCAN_TelemFresh(fc, now), (unsigned long)fc->tx_count);
      test_ack(buf);
      continue;
    }
    if (strcmp(line, "IMU?") == 0)
    {
      char buf[96];
      (void)snprintf(buf, sizeof(buf),
                     "IMU ok=%u who=0x%02X c=%lu A=%d,%d,%d G=%d,%d,%d T=%d",
                     (unsigned)g_imu.ok, (unsigned)g_imu.whoami,
                     (unsigned long)g_imu.read_count,
                     (int)g_imu.ax, (int)g_imu.ay, (int)g_imu.az,
                     (int)g_imu.gx, (int)g_imu.gy, (int)g_imu.gz,
                     (int)g_imu.temp);
      test_ack(buf);
      continue;
    }
    if ((strcmp(line, "BRAKE 1") == 0) || (strcmp(line, "BRAKE 0") == 0))
    {
      s_brake_test_active = 1U;
      s_brake_test_level = (line[6] == '1') ? 1U : 0U;
      s_brake_test_until_ms = HAL_GetTick() + BRAKE_CAN_TEST_HOLD_MS;
      BrakeCAN_Send(s_brake_test_level);
      test_ack((s_brake_test_level != 0U) ? "BRAKE TEST ON" : "BRAKE TEST OFF");
      continue;
    }
    if (strcmp(line, "BRAKE OFF") == 0)
    {
      s_brake_test_active = 0U;
      s_brake_test_level = 0U;
      s_brake_test_until_ms = 0U;
      BrakeCAN_Send(0U);
      test_ack("BRAKE TEST STOP");
      continue;
    }
    if (strcmp(line, "FOC OFF") == 0)
    {
      s_test_active = 0U;
      test_set_stop();
      test_ack("FOC TEST OFF");
      continue;
    }
    if (strcmp(line, "FOC STOP") == 0)
    {
      s_test_active = 1U;
      test_set_stop();
      s_test_last_cmd_ms = HAL_GetTick();
      test_ack("FOC TEST STOP");
      continue;
    }
    if ((strncmp(line, "FOC D ", 6) == 0) || (strncmp(line, "FOC R ", 6) == 0))
    {
      char *p = line + 6;
      char *end = NULL;
      long pct = strtol(p, &end, 10);
      long steer = 0L;

      if ((end != NULL) && (*end == ' '))
      {
        steer = strtol(end + 1, NULL, 10);
      }
      s_test_active = 1U;
      s_test_throttle_permille = pct_to_permille(pct);
      s_test_steer_permille = steer_pct_to_permille(steer);
      s_test_allow = (pct > 0L) ? 1U : 0U;
      s_test_brake = 0U;
      s_test_gear = (line[4] == 'R') ? 2U : 1U;
      s_test_timeout_10ms = FOC_CAN_DEFAULT_TIMEOUT_10MS;
      s_test_last_cmd_ms = HAL_GetTick();
      test_ack((line[4] == 'R') ? "FOC TEST R" : "FOC TEST D");
      continue;
    }
    if (strncmp(line, "FOC RAW ", 8) == 0)
    {
      char *p = line + 8;
      char *end = NULL;
      long throttle = strtol(p, &end, 10);
      long steer = (end != NULL) ? strtol(end, &end, 10) : 0L;
      long flags = (end != NULL) ? strtol(end, &end, 0) : 0L;
      long gear = (end != NULL) ? strtol(end, &end, 0) : 0L;
      long timeout_10ms = (end != NULL) ? strtol(end, NULL, 0) : FOC_CAN_DEFAULT_TIMEOUT_10MS;

      s_test_active = 1U;
      s_test_throttle_permille = clamp_throttle_permille(throttle);
      s_test_steer_permille = clamp_steer_permille(steer);
      s_test_allow = ((uint8_t)flags & 0x01U) ? 1U : 0U;
      s_test_brake = ((uint8_t)flags & 0x02U) ? 1U : 0U;
      s_test_gear = (gear >= 0L && gear <= 2L) ? (uint8_t)gear : 0U;
      s_test_timeout_10ms = clamp_u8(timeout_10ms);
      s_test_last_cmd_ms = HAL_GetTick();
      test_ack("FOC TEST RAW");
      continue;
    }
    test_ack("FOC TEST ERR");
  }

  if (s_brake_test_active != 0U) {
    if ((int32_t)(HAL_GetTick() - s_brake_test_until_ms) < 0) {
      BrakeCAN_Send(s_brake_test_level);
    } else {
      s_brake_test_active = 0U;
      s_brake_test_level = 0U;
      BrakeCAN_Send(0U);
    }
  }
}

uint8_t FocCAN_TestActive(void)
{
  return s_test_active;
}

uint8_t FocCAN_BrakeTestActive(void)
{
  return s_brake_test_active;
}

void FocCAN_TestGet(uint16_t *throttle_permille, int16_t *steer_permille,
                    uint8_t *allow_drive, uint8_t *brake, uint8_t *gear)
{
  uint8_t expired = ((s_test_last_cmd_ms == 0U) ||
                     ((HAL_GetTick() - s_test_last_cmd_ms) > FOC_CAN_TEST_TIMEOUT_MS)) ? 1U : 0U;

  if (throttle_permille != NULL) {
    *throttle_permille = expired ? 0U : s_test_throttle_permille;
  }
  if (steer_permille != NULL) {
    *steer_permille = expired ? 0 : s_test_steer_permille;
  }
  if (allow_drive != NULL) {
    *allow_drive = expired ? 1U : s_test_allow;
  }
  if (brake != NULL) {
    *brake = expired ? 0U : s_test_brake;
  }
  if (gear != NULL) {
    *gear = expired ? 0U : s_test_gear;
  }
}

uint8_t FocCAN_TestTimeout10ms(void)
{
  return s_test_timeout_10ms;
}

uint8_t FocCAN_StatusFresh(const FocCAN_t *fc, uint32_t now_ms)
{
  if ((fc == NULL) || (fc->status_seen == 0U) || (fc->last_status_ms == 0U)) {
    return 0U;
  }
  return ((now_ms - fc->last_status_ms) <= FOC_CAN_STATUS_FRESH_MS) ? 1U : 0U;
}

uint16_t FocCAN_GetSpeed_01kmh(const FocCAN_t *fc, uint32_t now_ms, uint16_t fallback_01kmh)
{
  int32_t rpm1;
  int32_t rpm2;
  int32_t rpm_abs;
  int32_t speed_01kmh;

  if (FocCAN_StatusFresh(fc, now_ms) == 0U) {
    return fallback_01kmh;
  }

  rpm1 = fc->foc1_rpm;
  rpm2 = fc->foc2_rpm;
  if (rpm1 < 0) rpm1 = -rpm1;
  if (rpm2 < 0) rpm2 = -rpm2;
  rpm_abs = ((rpm1 > 0) && (rpm2 > 0)) ? ((rpm1 + rpm2) / 2) : (rpm1 + rpm2);

  speed_01kmh = (rpm_abs * FOC_WHEEL_CIRCUMFERENCE_MM * 600L + 500000L) / 1000000L;
  if (speed_01kmh < 0) {
    speed_01kmh = 0;
  }
  if (speed_01kmh > 9999L) {
    speed_01kmh = 9999L;
  }
  return (uint16_t)speed_01kmh;
}

void FocCAN_OnRxFrame(FocCAN_t *fc, uint32_t std_id, const uint8_t data[8])
{
  if (fc == NULL || data == NULL) {
    return;
  }

  /* 0x211：双电机板(填 foc1+foc2)，或单路左轮板(只填 foc1 槽 [2:3])。
   * 健康位按位合并：本帧提供 foc1 位(bit0/2/4)+cmd_recent(bit6)，保留右轮
   * (0x213)贡献的 foc2 位(bit1/3/5)。双电机板会在同帧填满两侧，行为不变。 */
  if (std_id == FOC_CAN_STATUS_ID_FOC2) {
    fc->rx_count++;
    fc->last_status_ms = HAL_GetTick();
    fc->status_seen = 1U;
    fc->status_health = (uint8_t)((fc->status_health & 0x2AU) | (data[0] & 0xD5U));
    fc->status_seq = data[1];
    fc->foc1_rpm = unpack_s16_le(&data[2]);
    /* 双电机板 [4:5] 为 foc2 轮速；单路左轮板该槽为 0，不覆盖右轮上报值。 */
    if (unpack_s16_le(&data[4]) != 0 || fc->foc2_rpm == 0) {
      fc->foc2_rpm = unpack_s16_le(&data[4]);
    }
    fc->status_cmd_flags = data[6];
    fc->status_cmd_valid = ((data[7] & 0x80U) != 0U) ? 1U : 0U;
    fc->status_cmd_gear = data[7] & 0x0FU;
    memcpy(fc->last_status, data, 8U);
    return;
  }

  /* 0x213：单路右轮板状态，只取 foc2 槽 [4:5] 与 foc2 健康位。 */
  if (std_id == FOC_CAN_STATUS_ID_R) {
    fc->rx_count++;
    fc->last_status_ms = HAL_GetTick();
    fc->status_seen = 1U;
    fc->status_health = (uint8_t)((fc->status_health & 0x15U) | (data[0] & 0xEAU));
    fc->foc2_rpm = unpack_s16_le(&data[4]);
    if (fc->status_cmd_valid == 0U) {          /* 双板同档，右轮补充档位回显 */
      fc->status_cmd_flags = data[6];
      fc->status_cmd_valid = ((data[7] & 0x80U) != 0U) ? 1U : 0U;
      fc->status_cmd_gear = data[7] & 0x0FU;
    }
    return;
  }

  if (std_id == FOC_CAN_TELEM_ID_FOC2) {
    fc->vbus_mV = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    fc->ibus_cA = unpack_s16_le(&data[2]);
    fc->telem_seq = data[4];
    fc->telem_seen = 1U;
    fc->last_telem_ms = HAL_GetTick();
    return;
  }

  if (std_id == FOC_CAN_TELEM_ID_R) {
    fc->vbus_mV_r = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    fc->ibus_cA_r = unpack_s16_le(&data[2]);
    fc->telem_seq_r = data[4];
    fc->telem_seen_r = 1U;
    fc->last_telem_ms_r = HAL_GetTick();
    return;
  }
}

static uint8_t telem_fresh_l(const FocCAN_t *fc, uint32_t now_ms)
{
  if ((fc == NULL) || (fc->telem_seen == 0U) || (fc->last_telem_ms == 0U)) {
    return 0U;
  }
  return ((now_ms - fc->last_telem_ms) <= FOC_CAN_STATUS_FRESH_MS) ? 1U : 0U;
}

static uint8_t telem_fresh_r(const FocCAN_t *fc, uint32_t now_ms)
{
  if ((fc == NULL) || (fc->telem_seen_r == 0U) || (fc->last_telem_ms_r == 0U)) {
    return 0U;
  }
  return ((now_ms - fc->last_telem_ms_r) <= FOC_CAN_STATUS_FRESH_MS) ? 1U : 0U;
}

uint8_t FocCAN_TelemFresh(const FocCAN_t *fc, uint32_t now_ms)
{
  /* 任一路(左轮/双电机板 或 右轮板)遥测新鲜即视为有效。 */
  return (telem_fresh_l(fc, now_ms) != 0U || telem_fresh_r(fc, now_ms) != 0U)
         ? 1U : 0U;
}

uint16_t FocCAN_GetVbus_mV(const FocCAN_t *fc, uint32_t now_ms)
{
  /* 母线电压共享，取新鲜的一路(优先左/双电机板)。 */
  if (telem_fresh_l(fc, now_ms) != 0U) return fc->vbus_mV;
  if (telem_fresh_r(fc, now_ms) != 0U) return fc->vbus_mV_r;
  return 0U;
}

int16_t FocCAN_GetIbus_cA(const FocCAN_t *fc, uint32_t now_ms)
{
  /* 母线电流：新鲜的各路求和(双电机板只有左路，行为不变；单路两块则相加)。 */
  int32_t sum = 0;
  if (telem_fresh_l(fc, now_ms) != 0U) sum += fc->ibus_cA;
  if (telem_fresh_r(fc, now_ms) != 0U) sum += fc->ibus_cA_r;
  if (sum > 32767) sum = 32767; else if (sum < -32768) sum = -32768;
  return (int16_t)sum;
}

HAL_StatusTypeDef FocCAN_SendDriveRequest(FocCAN_t *fc,
                                          uint16_t throttle_permille,
                                          int16_t steer_permille,
                                          uint8_t allow_drive,
                                          uint8_t brake,
                                          uint8_t gear,
                                          uint8_t timeout_10ms)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];
  uint32_t mailbox;
  uint8_t flags = 0U;

  if (fc == NULL) {
    return HAL_ERROR;
  }

  if (throttle_permille > 1000U) {
    throttle_permille = 1000U;
  }
  if (steer_permille < -1000) {
    steer_permille = -1000;
  } else if (steer_permille > 1000) {
    steer_permille = 1000;
  }
  if (gear > 2U) {
    gear = 0U;
  }

  if (allow_drive != 0U) {
    flags |= FOC_CAN_FLAG_ALLOW_DRIVE;
  }
  if (brake != 0U) {
    flags |= FOC_CAN_FLAG_BRAKE;
  }

  tx_header.StdId = FOC_CAN_DRIVE_REQ_ID_STD;
  tx_header.ExtId = 0U;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.IDE = CAN_ID_STD;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;

  tx_data[0] = (uint8_t)(throttle_permille & 0xFFU);
  tx_data[1] = (uint8_t)((throttle_permille >> 8) & 0xFFU);
  tx_data[2] = (uint8_t)((uint16_t)steer_permille & 0xFFU);
  tx_data[3] = (uint8_t)(((uint16_t)steer_permille >> 8) & 0xFFU);
  tx_data[4] = flags;
  tx_data[5] = gear;
  tx_data[6] = fc->seq++;
  tx_data[7] = timeout_10ms;

  fc->throttle_permille = throttle_permille;
  fc->steer_permille = steer_permille;
  fc->flags = flags;
  fc->gear = gear;
  fc->timeout_10ms = timeout_10ms;
  fc->tx_count++;

  return HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox);
}
