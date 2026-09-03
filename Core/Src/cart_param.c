#include "cart_param.h"
#include "cart_persist.h"
#include "imu_lsm6dsv.h"
#include "cart_params.h"
#include "fw_version.h"
#include "fw_image.h"
#include "drive_block.h"
#include "foc_can.h"
#include "rc_input.h"
#include "screen.h"
#include "steering.h"
#include "train_uart.h"
#include "servo_can.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* ---- 可写参数的运行时值 ----
 *
 * ⚠ 初值就是 cart_params.h 里那个常量 —— 表里的 `def` 和这里的初值是**同一个宏**，
 *   所以写不出"默认值不是板子真的会开机成为的值"。
 * ⚠⚠ 每加一个，先把消费方改成读它，再把 &它 填进表里的 rt。顺序反了就会造出
 *   一个"设了但不生效"的参数，而那种缺陷从外面看和成功一模一样。
 */
int32_t g_p_auto_max_speed     = (int32_t)AUTO_MAX_SPEED_MM_S;
int32_t g_p_auto_max_throttle  = (int32_t)AUTO_MAX_THROTTLE_PERMILLE;
int32_t g_p_auto_cmd_timeout   = (int32_t)AUTO_CMD_TIMEOUT_MS;
int32_t g_p_follow_cmd_timeout = (int32_t)FOLLOW_CMD_TIMEOUT_MS;
/* 踏板端点。⭐ 这三个就是"标定"要动的东西 —— 固件出功能，动作是用户的。
 * ⚠ 我 2026-08-30 说过这几个"消费方是 0 处"，**那是错的**：我 grep 的是
 *   参数名 thr_adc_dead，而宏叫 THROTTLE_ADC_DEAD，实际有 3 / 2 / 1 处消费方。 */
int32_t g_p_thr_adc_dead       = (int32_t)THROTTLE_ADC_DEAD;
int32_t g_p_thr_adc_full       = (int32_t)THROTTLE_ADC_FULL;
int32_t g_p_thr_dead           = (int32_t)THROTTLE_DEAD_PERMILLE;
int32_t g_p_steer_max          = (int32_t)MAX_STEER_01;
int32_t g_p_rc_stale           = (int32_t)RC_STALE_MS;
int32_t g_p_gear_debounce      = (int32_t)GEAR_DEBOUNCE_MS;

/* ---- 活值的来源 ---- */
extern FocCAN_t g_foc_can;
extern volatile uint16_t g_dbg_gear;
extern volatile uint8_t  g_gear_src;
extern volatile uint32_t g_gear_none_count;
extern volatile uint32_t g_gear_ambig_count;
extern volatile uint8_t  g_dbg_ven;
extern volatile uint8_t  g_dbg_fbrake;
extern volatile uint16_t g_pedal_permille;
extern volatile uint8_t  g_dbg_band;
extern volatile uint16_t g_drive_block;
extern volatile uint32_t g_reset_cause;
extern volatile uint32_t g_dbg_can_short;
extern volatile uint32_t g_dbg_can_fifo_ovf;
extern volatile uint32_t g_dbg_can_busoff;
extern volatile uint8_t  g_dbg_can_busoff_now;

static int32_t rd_mode(void)       { return (int32_t)g_steering_mode; }
static int32_t rd_gear(void)       { return (int32_t)g_dbg_gear; }
/* 档位来源三态：0=从没解出过有效档位（U7 大概没接） 1=实体开关说了算。
 * ⚠ 它和 gear= 是两个问题：一台没接 U7 的车也会稳定报 gear=P。 */
static int32_t rd_gear_src(void)   { return (int32_t)g_gear_src; }
/* 两个分开数，因为要查的线不一样：零路=断线/没插，多路=短路/先通后断。 */
static int32_t rd_gear_none(void)  { return (int32_t)g_gear_none_count; }
static int32_t rd_gear_ambig(void) { return (int32_t)g_gear_ambig_count; }
static int32_t rd_ven(void)        { return (int32_t)g_dbg_ven; }
static int32_t rd_brake(void)      { return (int32_t)g_dbg_fbrake; }
static int32_t rd_pedal(void)      { return (int32_t)g_pedal_permille; }
static int32_t rd_band(void)       { return (int32_t)g_dbg_band; }
static int32_t rd_block(void)      { return (int32_t)g_drive_block; }
static int32_t rd_rst(void)        { return (int32_t)(g_reset_cause & 0xFFU); }
static int32_t rd_shortrx(void)    { return (int32_t)g_dbg_can_short; }
static int32_t rd_spd(void)        { return (int32_t)FocCAN_GetSpeed_01kmh(&g_foc_can, HAL_GetTick()); }
static int32_t rd_spd_ok(void)     { return (int32_t)FocCAN_SpeedFeedbackValid(&g_foc_can, HAL_GetTick()); }
static int32_t rd_vbus(void)       { return (int32_t)FocCAN_GetVbus_mV(&g_foc_can, HAL_GetTick()); }
static int32_t rd_ibus(void)       { return (int32_t)FocCAN_GetIbus_cA(&g_foc_can, HAL_GetTick()); }
static int32_t rd_peer_alive(void) { return (int32_t)FocCAN_PeerAlive(&g_foc_can, HAL_GetTick()); }
static int32_t rd_peer_frozen(void){ return (int32_t)FocCAN_PeerFrozen(&g_foc_can, HAL_GetTick()); }
static int32_t rd_peer_belief(void){ return (int32_t)FocCAN_PeerBelievesUs(&g_foc_can); }
/* ⚠ 0 不等于"没锁" —— 0.4.8~0.4.10 锁着也发 0。见 foc_can.h b6。 */
static int32_t rd_peer_locked(void){ return (int32_t)FocCAN_PeerLocalLock(&g_foc_can); }
static int32_t rd_txfail(void)     { return (int32_t)g_foc_can.tx_fail_count; }
static int32_t rd_echomis(void)    { return (int32_t)g_foc_can.echo_mismatch_count; }
static int32_t rd_rc_ch1(void)     { return (int32_t)RC_LinkState_CH1(); }
static int32_t rd_rc_ch2(void)     { return (int32_t)RC_LinkState_CH2(); }
static int32_t rd_rc_rc3(void)     { return (int32_t)RC_LinkState_RC3(); }
static int32_t rd_fifo_ovf(void)   { return (int32_t)g_dbg_can_fifo_ovf; }
static int32_t rd_busoff(void)     { return (int32_t)g_dbg_can_busoff; }
static int32_t rd_busoff_now(void) { return (int32_t)g_dbg_can_busoff_now; }
static int32_t rd_uart_full(void)  { return (int32_t)TrainUART_GetRxDropFull(); }
static int32_t rd_uart_ore(void)   { return (int32_t)TrainUART_GetRxDropOre(); }
/* ⚠ 和上面两个分开：那两个是**上位机/调试**那根线(USART1)，这两个是
 * **Jetson**那根线(UART5)。合成一个总数就再也分不出是哪一根在丢。 */
static int32_t rd_juart_full(void) { return (int32_t)TrainUART_GetJetsonDropFull(); }
static int32_t rd_juart_ore(void)  { return (int32_t)TrainUART_GetJetsonDropOre(); }
static int32_t rd_spd_dir(void)    { return (int32_t)FocCAN_GetSpeedDirection(&g_foc_can, HAL_GetTick()); }
static int32_t rd_brake_drv(void)  { return (int32_t)FocCAN_BrakeDriveEnabled(&g_foc_can); }
static int32_t rd_topo_b(void)     { return (int32_t)g_foc_can.status_r_seen; }
static int32_t rd_steer_blk(void)  { return (int32_t)g_servo_refused_count(); }

/*
 * 表本身。
 *
 * 每一行的 min/max/default 都是从 cart_params.h / foc_can.h / rc_input.h 里
 * **真正执行那件事的那个常量**推来的，没有一个是手打的第二份。
 * 驱动板侧二十一行里曾有七行的"出厂默认"不是板子真正的开机值，其中三行
 * 是从手边一块已经标定过的板上读来的 —— 那正是手写第二份的代价。
 *
 * moves=1 的行要克制着标。如果每一行都标 moves，上位机的安全确认就变成壁纸；
 * 驱动板那边记过这个教训："一次标定问六遍，把真正要紧的那一步稀释成了六分之一。"
 * 所以 moves=0 的那些才是要写仔细的。
 *
 * moves 和 coasts 分成两位，词是从驱动板 calib list 借来的，理由也是他们写的：
 * "会让轮子转"和"发了停止命令之后还在转"是两句不同的话，第二句更危险 ——
 * 看着的人以为它已经停了。所以凡是改了之后车不会立刻停下的(超时、增益、
 * 限速上限)，coasts 都是 1。
 */
static const CartParamDef_t k_table[] = {
  /* id  name                  unit         min   max        default/常量                        value get   class                      silent moves coasts */
  {   1, "auto_max_speed",     "mm/s",        0,  AUTO_MAX_SPEED_MM_S, AUTO_MAX_SPEED_MM_S,        0, NULL, &g_p_auto_max_speed, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {   2, "auto_max_throttle",  "permille",    0,  1000,      AUTO_MAX_THROTTLE_PERMILLE,           0, NULL, &g_p_auto_max_throttle, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {   3, "auto_cmd_timeout",   "ms",         50,  1000,      AUTO_CMD_TIMEOUT_MS,                  0, NULL, &g_p_auto_cmd_timeout, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {   4, "auto_spd_kp_num",    "-",           0,  1000,      AUTO_SPD_KP_NUM,                      0, NULL, NULL, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {   5, "auto_spd_kp_den",    "-",           1,  1000,      AUTO_SPD_KP_DEN,                      0, NULL, NULL, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {   6, "auto_spd_ki_num",    "-",           0,  1000,      AUTO_SPD_KI_NUM,                      0, NULL, NULL, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {   7, "auto_spd_ki_den",    "-",           1,  1000,      AUTO_SPD_KI_DEN,                      0, NULL, NULL, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {   8, "auto_spd_i_max",     "-",           0,  100000,    AUTO_SPD_I_MAX,                       0, NULL, NULL, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {   9, "auto_status_period", "ms",         20,  1000,      AUTO_STATUS_PERIOD_MS,                0, NULL, NULL, CART_PARAM_CLASS_SETTING, 1U, 0U, 0U },
  {  10, "follow_cmd_timeout", "ms",         50,  1000,      FOLLOW_CMD_TIMEOUT_MS,                0, NULL, &g_p_follow_cmd_timeout, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {  11, "steer_max",          "0.1deg",      0,  900,       MAX_STEER_01,                         0, NULL, &g_p_steer_max, CART_PARAM_CLASS_SETTING, 0U, 0U, 0U },
  {  12, "steer_gear_ratio",   "-",           1,  100,       STEERING_GEAR_RATIO,                  0, NULL, NULL, CART_PARAM_CLASS_SETTING, 1U, 0U, 0U },
  {  13, "steer_range_deg",    "deg",         1,  720,       STEERING_RANGE_DEG,                   0, NULL, NULL, CART_PARAM_CLASS_SETTING, 1U, 0U, 0U },
  {  14, "steer_physical_deg", "deg",         1,  90,        STEERING_PHYSICAL_DEG,                0, NULL, NULL, CART_PARAM_CLASS_SETTING, 1U, 0U, 0U },
  {  15, "steer_pos_range",    "counts",      1,  10000000,  STEERING_POSITION_RANGE,              0, NULL, NULL, CART_PARAM_CLASS_SETTING, 1U, 0U, 0U },
  {  16, "thr_adc_dead",       "-",           0,  4095,      THROTTLE_ADC_DEAD,                    0, NULL, &g_p_thr_adc_dead, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {  17, "thr_adc_full",       "-",           0,  4095,      THROTTLE_ADC_FULL,                    0, NULL, &g_p_thr_adc_full, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {  18, "thr_dead",           "permille",    0,  1000,      THROTTLE_DEAD_PERMILLE,               0, NULL, &g_p_thr_dead, CART_PARAM_CLASS_SETTING, 0U, 1U, 0U },
  {  19, "stick_active_thr",   "us",          0,  500,       STICK_ACTIVE_THRESH,                  0, NULL, NULL, CART_PARAM_CLASS_SETTING, 0U, 1U, 0U },
  {  20, "foc_tx_period",      "ms",          5,  100,       FOC_CAN_TX_PERIOD_MS,                 0, NULL, NULL, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {  21, "foc_cmd_timeout",    "ms",         60,  500,       FOC_CAN_DEFAULT_TIMEOUT_10MS * 10,    0, NULL, NULL, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {  22, "foc_status_fresh",   "ms",         50,  2000,      FOC_CAN_STATUS_FRESH_MS,              0, NULL, NULL, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {  23, "foc_peer_frozen",    "ms",         50,  2000,      FOC_CAN_PEER_FROZEN_MS,               0, NULL, NULL, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {  24, "speed_max_plaus",    "0.1kph",      1,  9999,      FOC_CAN_SPEED_PLAUSIBLE_MAX_01KMH,    0, NULL, NULL, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {  25, "rc_stale",           "ms",         50,  2000,      RC_STALE_MS,                          0, NULL, &g_p_rc_stale, CART_PARAM_CLASS_SETTING, 0U, 1U, 1U },
  {  26, "gear_debounce",      "ms",          0,  1000,      GEAR_DEBOUNCE_MS,                     0, NULL, &g_p_gear_debounce, CART_PARAM_CLASS_SETTING, 0U, 1U, 0U },
  {  27, "gear_boot_ignore",   "ms",          0,  5000,      GEAR_BOOT_IGNORE_MS,                  0, NULL, NULL, CART_PARAM_CLASS_SETTING, 1U, 0U, 0U },
  {  28, "ven_debounce",       "ms",          0,  1000,      VEHICLE_ENABLE_DEBOUNCE_MS,           0, NULL, NULL, CART_PARAM_CLASS_SETTING, 0U, 1U, 0U },
  {  29, "wdg_timeout_nom",    "ms",        100,  10000,     2000,                                 0, NULL, NULL, CART_PARAM_CLASS_SETTING, 0U, 0U, 0U },

  /* ---- 活值。这里的 min/max 是取值域，不是可调范围。 ---- */
  {  60, "mode",               "-",           0,  5,          0, 0, rd_mode, NULL,        CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  61, "gear_key",           "-",           0,  0xFFFF,     0, 0, rd_gear, NULL,        CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  62, "veh_enable",         "-",           0,  1,          0, 0, rd_ven, NULL,         CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  63, "brake_intent",       "-",           0,  1,          1, 0, rd_brake, NULL,       CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  64, "pedal",              "permille",    0,  1000,       0, 0, rd_pedal, NULL,       CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  65, "band",               "percent",     0,  100,        0, 0, rd_band, NULL,        CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  66, "drive_block",        "bitmap",      0,  0xFFFF,     0, 0, rd_block, NULL,       CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  67, "speed_meas",         "0.1kph",      0,  9999,       0, 0, rd_spd, NULL,         CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  68, "speed_valid",        "-",           0,  1,          0, 0, rd_spd_ok, NULL,      CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  69, "vbus",               "mV",          0,  65535,      0, 0, rd_vbus, NULL,        CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  70, "ibus",               "cA",     -32768,  32767,      0, 0, rd_ibus, NULL,        CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  71, "foc_alive",          "-",           0,  1,          0, 0, rd_peer_alive, NULL,  CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  72, "foc_frozen",         "-",           0,  1,          0, 0, rd_peer_frozen, NULL, CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  73, "foc_believes_us",    "-",           0,  1,          0, 0, rd_peer_belief, NULL, CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  90, "foc_local_lock",     "-",           0,  1,          0, 0, rd_peer_locked, NULL, CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  91, "gear_src",           "-",           0,  1,          0, 0, rd_gear_src, NULL, CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  92, "gear_none_cnt",      "-",           0,  1000000,    0, 0, rd_gear_none, NULL, CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  93, "gear_ambig_cnt",     "-",           0,  1000000,    0, 0, rd_gear_ambig, NULL, CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  94, "juart_drop_full",    "-",           0,  1000000,    0, 0, rd_juart_full, NULL, CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  95, "juart_drop_ore",     "-",           0,  1000000,    0, 0, rd_juart_ore, NULL, CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  74, "foc_tx_fail",        "-",           0,  2000000000, 0, 0, rd_txfail, NULL,      CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  75, "foc_echo_mismatch",  "-",           0,  2000000000, 0, 0, rd_echomis, NULL,     CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  76, "can_short_rx",       "-",           0,  2000000000, 0, 0, rd_shortrx, NULL,     CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  77, "rc_link_ch1",        "-",           0,  2,          0, 0, rd_rc_ch1, NULL,      CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  78, "rc_link_ch2",        "-",           0,  2,          0, 0, rd_rc_ch2, NULL,      CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  79, "rc_link_rc3",        "-",           0,  2,          0, 0, rd_rc_rc3, NULL,      CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  80, "reset_cause",        "bitmap",      0,  255,        0, 0, rd_rst, NULL,         CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  81, "can_fifo_ovf",       "-",           0,  2000000000, 0, 0, rd_fifo_ovf, NULL,    CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  82, "can_busoff",         "-",           0,  2000000000, 0, 0, rd_busoff, NULL,      CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  83, "can_busoff_now",     "-",           0,  1,          0, 0, rd_busoff_now, NULL,  CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  84, "uart_rx_drop_full",  "-",           0,  2000000000, 0, 0, rd_uart_full, NULL,   CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  85, "uart_rx_drop_ore",   "-",           0,  2000000000, 0, 0, rd_uart_ore, NULL,    CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  86, "speed_dir",          "-",          -1,  1,          0, 0, rd_spd_dir, NULL,     CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  87, "foc_brake_drv",      "-",           0,  1,          0, 0, rd_brake_drv, NULL,   CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  88, "topology_b",         "-",           0,  1,          0, 0, rd_topo_b, NULL,      CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
  {  89, "steer_refused",      "-",           0,  2000000000, 0, 0, rd_steer_blk, NULL,   CART_PARAM_CLASS_DIAG, 0U, 0U, 0U },
};

uint16_t CartParam_Count(void)
{
  return (uint16_t)(sizeof(k_table) / sizeof(k_table[0]));
}

const CartParamDef_t *CartParam_At(uint16_t index)
{
  if (index >= CartParam_Count()) {
    return NULL;
  }
  return &k_table[index];
}

int32_t CartParam_Read(const CartParamDef_t *d)
{
  if (d == NULL) {
    return 0;
  }
  /* 元数据和读取是同一行：取值走这一行自己的 getter，没有 getter 的就是
   * 那个常量本身。表和执行方读的是同一个符号，所以写不出"表里一个数、
   * 机器里另一个数"。 */
  if (d->rt != NULL) {
    return *(d->rt);          /* 可写参数：运行时值就是执行方读的那个 */
  }
  if (d->get != NULL) {
    return d->get();
  }
  return d->def;
}

/* ---- 分页发送 ---- */
static uint16_t s_list_index;
static uint8_t  s_list_busy;

void CartParam_ListStart(void)
{
  s_list_index = 0U;
  s_list_busy = 1U;
}

uint8_t CartParam_ListBusy(void)
{
  return s_list_busy;
}

static void emit_line(const char *text)
{
  TrainUART_SendText(text);
  TrainUART_SendText("\r\n");
}

/*
 * 一行 `=param`。**只有这一处**拼它。
 *
 * `param list` 和 `param get` 发的必须是**同一行** —— 上位机 param.ts 里
 * 那条 ROW 正则两边都要匹配，而两份 snprintf 迟早会有一份先长出新字段。
 * （和 CartTlm_BuildLine 同一条理由。）
 */
static void format_row(const CartParamDef_t *d, char *buf, uint16_t size)
{
  if (d == NULL) {
    if (size > 0U) { buf[0] = 0; }
    return;
  }
  (void)snprintf(buf, size,
                 "=param %u %u name=%s type=i32 unit=%s scale=1 min=%ld max=%ld default=%ld perm=%s class=%s persist=%s channel=board silent=%u moves=%u coasts=%u value=%ld",
                 (unsigned)CART_PARAM_TABLE_VERSION, (unsigned)d->id, d->name, d->unit,
                 (long)d->min, (long)d->max, (long)d->def,
                 (d->rt != NULL) ? "rw" : "ro",
                 (d->cls == (uint8_t)CART_PARAM_CLASS_DIAG) ? "diag" : "setting",
                 /* ⚠ persist 跟着 rt 走，不是写死的：一个报 persist=yes 却
                  * 存不下来的参数，正是 DBIS §5.2 点名的那种静默失败。 */
                 (d->rt != NULL) ? "yes" : "no",
                 (unsigned)d->silent, (unsigned)d->moves, (unsigned)d->coasts,
                 (long)CartParam_Read(d));
}

static const CartParamDef_t *find_by_name(const char *name)
{
  uint16_t i;
  if (name == NULL) { return NULL; }
  for (i = 0U; i < CartParam_Count(); i++) {
    const CartParamDef_t *d = CartParam_At(i);
    if ((d != NULL) && (strcmp(d->name, name) == 0)) { return d; }
  }
  return NULL;
}

/*
 * `param get <名字>` —— 一行，就是表里那一行。
 *
 * ⚠ 它**不需要任何存储**，所以和"这块板还不能写参数"是两件事。
 *   在这条命令之前，上位机的 param_get 收到的是 CLI 的兜底回复，
 *   而它据此报的是"回话了，但里面没有 <名字> 的 =param 行" ——
 *   一句正确但没用的话：它分不出"这块板不认识这条命令"和"这个参数不存在"。
 */
uint8_t CartParam_IsWritable(const CartParamDef_t *d)
{
  return ((d != NULL) && (d->rt != NULL)) ? 1U : 0U;
}

static const CartParamDef_t *find_by_id(uint16_t id)
{
  uint16_t i;
  for (i = 0U; i < CartParam_Count(); i++) {
    const CartParamDef_t *d = CartParam_At(i);
    if ((d != NULL) && (d->id == id)) { return d; }
  }
  return NULL;
}

int CartParam_ApplyStored(uint16_t id, int32_t v)
{
  const CartParamDef_t *d = find_by_id(id);
  /* ⚠ 三道都要过：这个 id 还在、它现在仍然可写、值在范围内。
   *   ① 存了一个后来被删掉的 id ⇒ 忽略（"只允许追加"的另一半）；
   *   ② 存了一个后来变回只读的 ⇒ 忽略，不能让旧记录把只读参数写活；
   *   ③ 越界 ⇒ **丢掉保留默认**，不夹取。 */
  if ((d == NULL) || (d->rt == NULL)) { return 0; }
  if ((v < d->min) || (v > d->max)) { return 0; }
  *(d->rt) = v;
  return 1;
}

/*
 * ⚠⚠ 写入前置条件（DBIS §8.1「写入前置条件必须出声」）。
 *
 * 判据是**整车使能开关断开** —— 那是现场那个人能看见、能确认的物理状态，
 * 而不是一个软件标志。⇒ 车在通电待命时改不动限速和超时。
 *
 * ⚠ 为什么不用 "drive_block != 0"：那张位图里有十几条理由，其中
 *   `cli_test`、`host_dis` 这些在台架上天天为真 —— 拿它当闸等于没有闸。
 */
static int write_allowed(void)
{
  extern volatile uint8_t g_dbg_ven;
  return (g_dbg_ven == 0U) ? 1 : 0;
}

void CartParam_Set(const char *name, int32_t value)
{
  const CartParamDef_t *d = find_by_name(name);
  char buf[208];

  if (d == NULL) {
    (void)snprintf(buf, sizeof(buf), "=param-err %u 0 code=unknown-param name=%s",
                   (unsigned)CART_PARAM_TABLE_VERSION, (name != NULL) ? name : "");
    emit_line(buf); return;
  }
  if (d->rt == NULL) {
    (void)snprintf(buf, sizeof(buf), "=param-err %u %u code=read-only name=%s",
                   (unsigned)CART_PARAM_TABLE_VERSION, (unsigned)d->id, d->name);
    emit_line(buf); return;
  }
  if ((value < d->min) || (value > d->max)) {
    /* ⭐ 说明是**哪个**边界 —— DBIS §5.2 点名要求的。 */
    (void)snprintf(buf, sizeof(buf),
                   "=param-err %u %u code=out-of-range name=%s min=%ld max=%ld",
                   (unsigned)CART_PARAM_TABLE_VERSION, (unsigned)d->id, d->name,
                   (long)d->min, (long)d->max);
    emit_line(buf); return;
  }
  if (write_allowed() == 0) {
    (void)snprintf(buf, sizeof(buf),
                   "=param-err %u %u code=wrong-state name=%s",
                   (unsigned)CART_PARAM_TABLE_VERSION, (unsigned)d->id, d->name);
    emit_line(buf);
    emit_line("  ! the vehicle enable switch is closed; open it before changing settings");
    return;
  }

  *(d->rt) = value;
  /* 回读那一行发出去 —— 上位机对每个写操作**必须回读验证**（DBIS §4.4），
   * 这一行就是给它回读的。 */
  format_row(d, buf, (uint16_t)sizeof(buf));
  emit_line(buf);
}

void CartParam_Save(void)
{
  char buf[160];
  CartPersistStatus_t st;

  if (CartPersist_DevicePresent() == 0U) {
    (void)snprintf(buf, sizeof(buf), "=param-err %u 0 code=read-only name=*",
                   (unsigned)CART_PARAM_TABLE_VERSION);
    emit_line(buf);
    emit_line("  ! no non-volatile storage answered on I2C1; nothing can be saved");
    return;
  }
  if (write_allowed() == 0) {
    (void)snprintf(buf, sizeof(buf), "=param-err %u 0 code=wrong-state name=*",
                   (unsigned)CART_PARAM_TABLE_VERSION);
    emit_line(buf);
    emit_line("  ! the vehicle enable switch is closed; open it before saving");
    return;
  }

  st = CartPersist_Save();
  /* ⚠ ok=1 只在**回读自验也过了**的时候发。写进去了但读不回来，
   *   看起来和成功一模一样 —— 那正是自验存在的理由。 */
  (void)snprintf(buf, sizeof(buf), "=param-save %u record=%u ok=%u",
                 (unsigned)CART_PARAM_TABLE_VERSION,
                 (unsigned)CartPersist_Seq(),
                 (st == CART_PERSIST_OK) ? 1U : 0U);
  emit_line(buf);
  if (st == CART_PERSIST_VERIFY_FAILED) {
    emit_line("  ! written but the read-back did not parse -- do NOT treat this as saved");
  } else if (st == CART_PERSIST_IO_ERROR) {
    emit_line("  ! I2C error while writing");
  } else {
    /* 成功，无话可说 */
  }
}

void CartParam_LoadCmd(void)
{
  char buf[160];
  uint16_t n = CartPersist_Load();
  (void)snprintf(buf, sizeof(buf), "=param-save %u record=%u ok=%u",
                 (unsigned)CART_PARAM_TABLE_VERSION,
                 (unsigned)CartPersist_Seq(),
                 (CartPersist_LastStatus() == CART_PERSIST_OK) ? 1U : 0U);
  emit_line(buf);
  (void)snprintf(buf, sizeof(buf), "  ! %u stored values accepted", (unsigned)n);
  emit_line(buf);
}

void CartParam_Get(const char *name)
{
  const CartParamDef_t *d = find_by_name(name);
  char buf[208];

  if (d == NULL) {
    (void)snprintf(buf, sizeof(buf),
                   "=param-err %u 0 code=unknown-param name=%s",
                   (unsigned)CART_PARAM_TABLE_VERSION, (name != NULL) ? name : "");
    emit_line(buf);
    return;
  }
  format_row(d, buf, (uint16_t)sizeof(buf));
  emit_line(buf);
}

/*
 * `param set` / `param save` —— **说清楚地拒绝**，而不是不吭声。
 *
 * ⚠⚠ 这两条在这块板上做不到，而"做不到"和"没听见"是两个不同的答案：
 *   在这之前板子落到 CLI 的兜底回复，上位机据此报
 *   "板子既没有确认写入，也没有给出拒绝码 ⚠ 不要假设它写进去了" ——
 *   **正确，但它不知道为什么，也不知道明天会不会好**。
 *
 * ⇒ 现在回 `=param-err ... code=read-only`，那是上位机 PARAM_ERROR_CODES 里
 *   已经有的词，param_set / param_save 两个工具都会把它当成一个有信息的"不"。
 *
 * ⚠⚠ 2026-09-01：**这个函数已经没有调用方了**，而它原来的注释说的是一件
 *   不再成立的事（「整张表 read-only，这块板没有任何非易失存储可用」）。
 *
 *   `b99afbd`（2026-08-30）之后写入是真的：命令分派走 `CartParam_Set()`，
 *   它对 rt 为空的行答 `code=read-only`、越界答 `code=out-of-range`、
 *   使能开关合着答 `code=wrong-state`，写成了就回读一行。
 *
 * ⇒ 留着它只为一件事：下一个人看到这个名字时知道**不要把它接回去**。
 *   接回去就是把一块存得下参数的板子改回“什么都拒”，而拒绝理由读起来仍然很正当。
 */
void CartParam_RefuseSet(const char *name)
{
  const CartParamDef_t *d = find_by_name(name);
  char buf[160];

  (void)snprintf(buf, sizeof(buf),
                 "=param-err %u %u code=%s name=%s",
                 (unsigned)CART_PARAM_TABLE_VERSION,
                 (d != NULL) ? (unsigned)d->id : 0U,
                 (d != NULL) ? "read-only" : "unknown-param",
                 (name != NULL) ? name : "");
  emit_line(buf);
}

void CartParam_RefuseSave(void)
{
  char buf[160];
  /* ⚠ id=0、name=* ：这条拒绝是关于**整张表**的，不是某一行。 */
  (void)snprintf(buf, sizeof(buf),
                 "=param-err %u 0 code=read-only name=*",
                 (unsigned)CART_PARAM_TABLE_VERSION);
  emit_line(buf);
}

void CartParam_ListStep(void)
{
  char buf[208];

  if (s_list_busy == 0U) {
    return;
  }

  if (s_list_index == 0U) {
    /* count= 不是装饰：一张少了几行的表本身是**格式良好**的，
     * 收的一方不数就永远看不出少了什么。 */
    (void)snprintf(buf, sizeof(buf), "=param begin %u count=%u",
                   (unsigned)CART_PARAM_TABLE_VERSION, (unsigned)CartParam_Count());
    emit_line(buf);
  }

  if (s_list_index < CartParam_Count()) {
    format_row(CartParam_At(s_list_index), buf, (uint16_t)sizeof(buf));
    emit_line(buf);
    s_list_index++;
    return;
  }

  emit_line("=param end");
  s_list_busy = 0U;
}

/* ==================== `tlm` 一次遥测快照 ====================
 *
 * 为什么在这个文件里：上面那三十个 rd_*() 就是这块板全部的活值来源。
 * 换一个文件写 tlm，要么把它们全部公开出去，要么照抄一份 —— 而"表和执行方
 * 读的是同一个数"正是本文件开头第 1 条规矩。两条记录也是同一种东西：
 * 机器可读的控制台输出，都由主循环一趟发一行。
 *
 * 形状是照上位机 packages/board-tools/src/telemetry.ts 写的，不是照文档：
 *   `=tlm <版本> <组> <k=v> ...`，正则是 /^=tlm\s+(\d+)\s+(\w+)\s+(.*)$/，
 *   最后一行 `=tlm end` —— 有它，他们那边报"帧完整"；没有它，他们按
 *   **安静收尾**读，而安静分不出"发完了"和"发了一半停下喘口气"。
 *   这块板恰恰是会喘的那种(一趟主循环一行)，所以 end 不是装饰。
 *
 * ! 他们支持的遥测版本是 [1, 2]，本板报 1 在范围内(比最新的还新才拒)。
 *   ⇒ FW_TELEM_VERSION 不用动。
 *
 * ! 数字一律**十进制**，不写 0x。他们对每个值做 Number(text)，
 *   而 JS 的 Number("0x14") 是 20 —— 能读，但那个值在原文里长得像 20 又像
 *   一个位图，两种读法都说得通。**一个值同时是几种意思**，这个仓已经
 *   为它付过一次代价了(0 既是"停住了"也是"没数据")。
 *
 * ! 组名和字段名是本板自己的，没有硬凑成驱动板的 m1/m2/canreq。
 *   他们的 telem_snapshot 是通用的：收到什么组就报什么组。凑成驱动板的样子
 *   只会让一块没有桥、没有电机的板子在界面上长出两个电机。
 */
/*
 * build 记录只在这一个地方拼出来。
 *
 * `ver` 和 `tlm` 都要发它，而上位机的 buildOf() 从两条命令里的任意一条读到
 * 都该得出同一个答案。写两份 snprintf 就是抄两份 —— 这个仓已经为
 * "一条规则写在调用点里，下一个调用点不会知道"付过账了。
 *
 * ⚠ cmdhash 缺失时**整个字段不出现**，不是出现一个 0：
 *   上位机对两者的处理相同（整张表不用），但 0 是一个合法的 CRC32，
 *   而"没说"和"说了个 0"在人读日志时不是一回事。
 */
void CartTlm_BuildLine(char *buf, uint16_t size)
{
#ifdef FW_CMD_TABLE_HASH
  /* ⭐ stamped / crc 现在来自镜像自己的头，不是编译期常量。
   * ⚠ 两者仍然是**两个不同的答案**：stamped=0 是"没有这个信息"，
   *   crc=0 是"这个信息不好" —— 合并成一位就再也分不出来了。 */
  fw_image_state_t st = FwImage_Check();
  unsigned stamped = (st != FW_IMAGE_UNSTAMPED_) && (st != FW_IMAGE_BAD_MAGIC);
  unsigned crcok   = (st == FW_IMAGE_OK) ? 1U : 0U;
  (void)snprintf(buf, size,
                 "=tlm %u build debug=%u stamped=%u crc=%u major=%u minor=%u patch=%u board=%s scm=%s cmdhash=%lu",
                 (unsigned)FW_TELEM_VERSION, (unsigned)FW_IS_DEBUG,
                 stamped, crcok,
                 (unsigned)FW_VER_MAJOR, (unsigned)FW_VER_MINOR, (unsigned)FW_VER_PATCH,
                 FW_BOARD_ID, FW_SCM, (unsigned long)FW_CMD_TABLE_HASH);
#else
  (void)snprintf(buf, size,
                 "=tlm %u build debug=%u stamped=%u crc=%u major=%u minor=%u patch=%u board=%s scm=%s",
                 (unsigned)FW_TELEM_VERSION, (unsigned)FW_IS_DEBUG,
                 stamped, crcok,
                 (unsigned)FW_VER_MAJOR, (unsigned)FW_VER_MINOR, (unsigned)FW_VER_PATCH,
                 FW_BOARD_ID, FW_SCM);
#endif
}

/*
 * ==================== `fault` / `fault codes` ====================
 *
 * 形状照上位机 `packages/board-tools/src/fault.ts` 的三条正则写的，
 * **不是** `=` 前缀的机器记录 —— 这一族在他们那套里是**人读表格 + 正则**：
 *
 *   safety   : <一句话>
 *     [ N] <name>  board  <detail>            <- ENTRY
 *      -> <advice>                            <- ADVICE
 *
 *   code name            action   latched  meaning     <- 表头，随便写
 *      1 ven             cut      no       ...         <- CODE_ROW
 *                                       -> ...         <- ADVICE
 *
 * ⚠ ENTRY 的第三列只认 `board` 或 `motor <n>`。本板没有电机通道，一律 board。
 * ⚠ CODE_ROW 的 action 只认 cut / limit / note；latched 只认 yes / no。
 *   写别的词那一行会被**整行丢掉**，而表看起来仍然是完整的。
 *
 * ⚠⚠ 本板的"故障"和驱动板的不是一回事：驱动板的是闩住的跳闸（关栅），
 *   本板的是**此刻为什么不让车动**（drive_block，活值）。
 *   ⇒ 所以 latched=no，而 `safety : OK` 的意思是
 *     **"本板这一侧没有拦着"，不是"这台车是安全的"** —— 这句话写进了 safety 行本身。
 *
 * ⚠ 长输出分片：DBIS §4.5「长输出必须分片，不得因一次打印阻塞控制环」。
 *   一趟主循环发一行，同 param list / tlm。
 */
/*
 * ==================== 自检（DBIS §5.4，L2 强制） ====================
 *
 * 标准的原话：**自检必须编进正式固件**（或可切换 bank），不得只存在于独立构建 ——
 * 否则上位机无法在客户板子上按需触发。
 *
 * 格式也是标准给的（那一节说它"沿用 DlaBoard_B 已有的格式"）：
 *   =st 1 result item=<名> verdict=<pass|fail|info|skip> value=<数> unit=<单位>
 *   =st 1 summary total=<n> pass=<n> fail=<n>
 *
 * ---- ⚠⚠ 为什么不是把 selftest.c 搬过来 ----
 *
 * 那份是独立构建，直接搬进正式固件会出两件事，而且都不会报错：
 *
 *   1. 它的 EEPROM 测试写在 **0x0010** —— 那是参数存储**区 A 的记录头里面**。
 *      跑一次自检就把存好的参数写坏，而下一次开机只会说「CRC 不过」，
 *      没有人会想到是自检干的。⇒ 这里用 0x1F00，在两个区之外。
 *   2. 它的 CAN 测试把 CAN1 切成 **LOOPBACK** —— 那等于把这块板从总线上摘掉，
 *      `0x210` 停发，驱动板 300ms 后超时。⇒ 这一项在正式固件里 **skip**，
 *      并把理由说出来。`skip` 这个词就是为这种情形存在的。
 *
 * ---- 选项的判据 ----
 *
 * 只做**读已有状态**和**对隔离资源的读写**。不重配任何正在服役的外设 ——
 * 一块正在发行驶请求的板子，自检不该把它自己的通信摘掉。
 */
#define ST_ITEMS  8U

static uint8_t s_st_busy;
static uint8_t s_st_step;
static uint8_t s_st_pass;
static uint8_t s_st_fail;

void CartSelftest_Start(void)
{
  s_st_step = 0U; s_st_busy = 1U; s_st_pass = 0U; s_st_fail = 0U;
}

static void st_emit(const char *item, const char *verdict, long value, const char *unit)
{
  char buf[200];
  if (unit != NULL) {
    (void)snprintf(buf, sizeof(buf), "=st 1 result item=%s verdict=%s value=%ld unit=%s",
                   item, verdict, value, unit);
  } else {
    (void)snprintf(buf, sizeof(buf), "=st 1 result item=%s verdict=%s value=%ld",
                   item, verdict, value);
  }
  emit_line(buf);
  if (strcmp(verdict, "pass") == 0) { s_st_pass++; }
  else if (strcmp(verdict, "fail") == 0) { s_st_fail++; }
}

void CartSelftest_Step(void)
{
  extern volatile uint32_t g_reset_cause;
  extern volatile uint16_t g_dbg_adc_raw;
  char buf[200];

  if (s_st_busy == 0U) { return; }

  switch (s_st_step) {
  case 0U:
    st_emit("image", (FwImage_Check() == FW_IMAGE_OK) ? "pass" : "fail",
            (long)FwImage_Header()->image_bytes, "byte");
    break;
  case 1U:
    /* ⚠ 器件在不在，和存储里有没有东西，是两个答案。这一项只答前者。 */
    st_emit("eeprom_present", (CartPersist_DevicePresent() != 0U) ? "pass" : "fail", 0, NULL);
    break;
  case 2U:
    if (CartPersist_DevicePresent() == 0U) {
      st_emit("eeprom_rw", "skip", 0, NULL);
      emit_line("  ! no device answered on I2C1");
    } else {
      st_emit("eeprom_rw", (CartPersist_SelfTest() != 0) ? "pass" : "fail",
              (long)CART_PERSIST_SCRATCH, NULL);
    }
    break;
  case 3U:
    /* 存储里有没有一条有效记录 —— info，不是 fail：出厂的板子本来就是空的。 */
    st_emit("param_record", "info", (long)CartPersist_Seq(), NULL);
    break;
  case 4U:
    st_emit("imu", (g_imu.ok != 0U) ? "pass" : "fail", 0, NULL);
    break;
  case 5U:
    /* 踏板 ADC 的当前读数。info：这里没有"应该是多少"，
     * 有没有接、标没标定是 `calib` 那边的事。 */
    st_emit("pedal_adc", "info", (long)g_dbg_adc_raw, NULL);
    break;
  case 6U:
    /* ⭐ 看门狗咬过没有。⚠ info 不是 fail —— 它说的是**过去发生过**，
     * 而板子此刻是好的；把它记成 fail 会让一块已经恢复的板子永远红着。 */
    st_emit("watchdog_bit", "info",
            (long)(((g_reset_cause & (1U << 5)) != 0U) ? 1 : 0), NULL);
    break;
  case 7U:
    st_emit("can_loopback", "skip", 0, NULL);
    emit_line("  ! loopback would take CAN1 off the bus and stop the drive request");
    break;
  default:
    (void)snprintf(buf, sizeof(buf), "=st 1 summary total=%u pass=%u fail=%u",
                   (unsigned)ST_ITEMS, (unsigned)s_st_pass, (unsigned)s_st_fail);
    emit_line(buf);
    s_st_busy = 0U;
    return;
  }
  s_st_step++;
}

/*
 * ==================== 标定编排（DBIS §5.3，L2 强制） ====================
 *
 * 上位机一步步带着用户走 —— 和驱动板 `calib` 那一套是同一个协议，
 * **形状不是我定的**，是问 COM5 上真正被接进去的那块板问出来的：
 *
 *   calib list    -> =calib begin 1 count=N
 *                    =calib <n> stage=<名> required= moves= coasts= hands_on= needs_bus=
 *                    =calib end
 *   calib status  -> =calib begin 1 count=1
 *                    =calib 1 stage=<名> index= total= state= moves= coasts= hands_on= needs_bus= id=
 *                    =calib end
 *
 * ⚠ `state` 只认 ready / running / blocked / done / failed（他们的 CALIB_STATES）。
 *   写别的词他们读成 `unknown`，而**未知的不是 ready** —— 那是他们刻意的方向。
 *
 * ---- 本板要标什么 ----
 *
 * 只有一件：**油门踏板的两个端点**。它们决定"松开的踏板算不算踩下"，
 * 而这块板没有别的需要测量才知道的量（转向几何是"人告知"，不是测出来的）。
 *
 * ---- ⭐⭐ 复核编进状态机，不靠上位机记得 ----
 *
 * DBIS §5.3 的原话。这里的复核是第三步：**再松一次踏板，读数必须回到 0‰**。
 * 它挡的是一种真实的错法 —— 两个端点都在踩着的状态下采到（比如用户先踩住
 * 再点"松开那一步"），此时 full-rest 仍然是一个正数、比例算得出来、
 * 界面上一切正常，而**松开的踏板会给出一个非零油门**。
 * 单次采样自己证明不了自己，必须让它回到已知状态再看一眼。
 */
#define CALIB_STAGE_IDLE     0U
#define CALIB_STAGE_REST     1U
#define CALIB_STAGE_FULL     2U
#define CALIB_STAGE_VERIFY   3U
#define CALIB_STAGE_DONE     4U
#define CALIB_STAGE_FAILED   5U
#define CALIB_TOTAL          3U   /* rest / full / verify */

static uint8_t  s_cal_stage = CALIB_STAGE_IDLE;
static uint8_t  s_cal_busy;      /* list / status 的泵 */
static uint8_t  s_cal_step;
static uint8_t  s_cal_what;      /* 0 = list, 1 = status */
static int32_t  s_cal_rest;      /* 本次流程采到的两个端点，确认前不落到参数 */
static int32_t  s_cal_full;
static const char *s_cal_fail;

/* 当前 ADC 读数。⚠ 和踏板换算读的是同一个源，不另开一路 —— 两个源迟早会分岔。 */
static uint32_t cal_adc_now(void)
{
  extern volatile uint16_t g_dbg_adc_raw;
  return (uint32_t)g_dbg_adc_raw;
}

static const char *cal_stage_name(uint8_t st)
{
  switch (st) {
  case CALIB_STAGE_REST:   return "pedal_rest";
  case CALIB_STAGE_FULL:   return "pedal_full";
  case CALIB_STAGE_VERIFY: return "pedal_verify";
  case CALIB_STAGE_DONE:   return "done";
  case CALIB_STAGE_FAILED: return "failed";
  default:                 return "idle";
  }
}

static const char *cal_state_word(void)
{
  switch (s_cal_stage) {
  case CALIB_STAGE_IDLE:   return "ready";
  case CALIB_STAGE_DONE:   return "done";
  case CALIB_STAGE_FAILED: return "failed";
  default:                 return "running";
  }
}

void CartCalib_ListStart(void)   { s_cal_what = 0U; s_cal_step = 0U; s_cal_busy = 1U; }
void CartCalib_StatusStart(void) { s_cal_what = 1U; s_cal_step = 0U; s_cal_busy = 1U; }

void CartCalib_Step(void)
{
  char buf[208];
  if (s_cal_busy == 0U) { return; }

  if (s_cal_step == 0U) {
    (void)snprintf(buf, sizeof(buf), "=calib begin 1 count=%u",
                   (s_cal_what == 0U) ? (unsigned)CALIB_TOTAL : 1U);
    emit_line(buf);
    s_cal_step = 1U;
    return;
  }

  if (s_cal_what == 0U) {          /* ---- calib list ---- */
    static const struct { const char *stage; uint8_t moves; uint8_t hands_on; } k[] = {
      /* ⚠ moves 只在真会动的那一步标 1。松开踏板不会让车动；
       *   **踩下踏板会** —— 如果车恰好是使能着、挂着档的。
       *   全标 1 会把真正要紧的那一步稀释成三分之一（驱动板记过这个教训）。 */
      { "pedal_rest",   0U, 1U },
      { "pedal_full",   1U, 1U },
      { "pedal_verify", 0U, 1U },
    };
    unsigned i = (unsigned)(s_cal_step - 1U);
    if (i < CALIB_TOTAL) {
      (void)snprintf(buf, sizeof(buf),
                     "=calib %u stage=%s required=1 moves=%u coasts=0 hands_on=%u needs_bus=0",
                     i + 1U, k[i].stage, (unsigned)k[i].moves, (unsigned)k[i].hands_on);
      emit_line(buf);
      s_cal_step++;
      return;
    }
  } else {                         /* ---- calib status ---- */
    if (s_cal_step == 1U) {
      unsigned idx = (s_cal_stage <= CALIB_STAGE_VERIFY) ? (unsigned)s_cal_stage : CALIB_TOTAL;
      (void)snprintf(buf, sizeof(buf),
                     "=calib 1 stage=%s index=%u total=%u state=%s "
                     "moves=%u coasts=0 hands_on=%u needs_bus=0 id=1",
                     cal_stage_name(s_cal_stage), idx, (unsigned)CALIB_TOTAL,
                     cal_state_word(),
                     (s_cal_stage == CALIB_STAGE_FULL) ? 1U : 0U,
                     (s_cal_stage == CALIB_STAGE_IDLE) ? 0U : 1U);
      emit_line(buf);
      s_cal_step = 2U;
      return;
    }
    if ((s_cal_step == 2U) && (s_cal_stage == CALIB_STAGE_FAILED) && (s_cal_fail != NULL)) {
      (void)snprintf(buf, sizeof(buf), "  ! %s", s_cal_fail);
      emit_line(buf);
      s_cal_step = 3U;
      return;
    }
  }

  emit_line("=calib end");
  s_cal_busy = 0U;
}

/* `calib start pedal` —— 只有一个流程，名字对不上就出声。 */
void CartCalib_Start(const char *flow)
{
  char buf[160];
  if ((flow == NULL) || (strcmp(flow, "pedal") != 0)) {
    (void)snprintf(buf, sizeof(buf),
                   "=calib 1 stage=idle index=0 total=%u state=failed "
                   "moves=0 coasts=0 hands_on=0 needs_bus=0 id=1", (unsigned)CALIB_TOTAL);
    emit_line(buf);
    emit_line("  ! this board has one flow: 'pedal'. Try 'calib list'.");
    return;
  }
  s_cal_stage = CALIB_STAGE_REST;
  s_cal_rest = 0; s_cal_full = 0; s_cal_fail = NULL;
  emit_line("=ok calib start");
  emit_line("  ! step 1/3 pedal_rest: take your foot OFF the pedal, then 'calib confirm'.");
}

/*
 * `calib confirm` —— 人工步的确认。每一步采一次样、当场检查，**不合格就 failed**。
 *
 * ⚠ 检查写在这里而不是留给上位机：一个只有上位机才检查的流程，
 *   换一个上位机就没有检查了。
 */
void CartCalib_Confirm(void)
{
  char buf[200];
  int32_t adc = (int32_t)cal_adc_now();

  switch (s_cal_stage) {
  case CALIB_STAGE_REST:
    s_cal_rest = adc;
    s_cal_stage = CALIB_STAGE_FULL;
    (void)snprintf(buf, sizeof(buf), "=ok calib confirm");
    emit_line(buf);
    (void)snprintf(buf, sizeof(buf), "  ! rest = %ld ADC. step 2/3 pedal_full: "
                   "press the pedal ALL THE WAY DOWN and hold, then 'calib confirm'.",
                   (long)s_cal_rest);
    emit_line(buf);
    break;

  case CALIB_STAGE_FULL:
    s_cal_full = adc;
    /* ⚠ 两个端点之间的关系，参数层的单字段范围校验拦不住 —— 只能在这里挡。
     * 200 counts 不是随手写的：4095 满量程下它约等于 5%，
     * 比这更小的行程说明踏板没踩到底、或者根本没接。 */
    if ((s_cal_full - s_cal_rest) < 200) {
      s_cal_stage = CALIB_STAGE_FAILED;
      s_cal_fail = "full and rest are too close (<200 counts): the pedal was not "
                   "pressed, or it is not connected";
      emit_line("=ok calib confirm");
      emit_line("  ! FAILED: full and rest are too close. 'calib abort', check the pedal, start again.");
      break;
    }
    s_cal_stage = CALIB_STAGE_VERIFY;
    emit_line("=ok calib confirm");
    (void)snprintf(buf, sizeof(buf), "  ! full = %ld ADC. step 3/3 pedal_verify: "
                   "take your foot OFF again, then 'calib confirm'.", (long)s_cal_full);
    emit_line(buf);
    break;

  case CALIB_STAGE_VERIFY: {
    /* ⭐⭐ 复核：再松一次，读数必须落回靠近 rest 的地方。
     * 它挡的是"两个端点都在踩着时采到"——那种情形下比例算得出来、
     * 界面一切正常，而**松开的踏板会给出非零油门**。 */
    int32_t span = s_cal_full - s_cal_rest;
    int32_t drift = adc - s_cal_rest;
    if (drift < 0) { drift = -drift; }
    if (drift > (span / 10)) {
      s_cal_stage = CALIB_STAGE_FAILED;
      s_cal_fail = "the released pedal did not return to the rest point";
      emit_line("=ok calib confirm");
      emit_line("  ! FAILED: released pedal reads far from the rest point. "
                "Was the pedal held down during step 1? 'calib abort' and start again.");
      break;
    }
    g_p_thr_adc_dead = s_cal_rest;
    g_p_thr_adc_full = s_cal_full;
    s_cal_stage = CALIB_STAGE_DONE;
    emit_line("=ok calib confirm");
    (void)snprintf(buf, sizeof(buf),
                   "  ! done. thr_adc_dead=%ld thr_adc_full=%ld -- **in RAM only**. "
                   "Run 'param save' to keep them.", (long)s_cal_rest, (long)s_cal_full);
    emit_line(buf);
    break;
  }

  default:
    emit_line("=ok calib confirm");
    emit_line("  ! nothing to confirm. 'calib start pedal' first.");
    break;
  }
}

void CartCalib_Abort(void)
{
  /* ⚠ 放弃**不写回任何东西** —— 半套端点比旧的那套更糟。 */
  s_cal_stage = CALIB_STAGE_IDLE;
  s_cal_fail = NULL;
  emit_line("=ok calib abort");
  emit_line("  ! aborted. Nothing was written; the previous endpoints still apply.");
}

static uint8_t  s_fault_step;
static uint8_t  s_fault_busy;
static uint8_t  s_codes_step;
static uint8_t  s_codes_busy;

void CartFault_Start(void)   { s_fault_step = 0U; s_fault_busy = 1U; }
void CartFault_CodesStart(void) { s_codes_step = 0U; s_codes_busy = 1U; }

void CartFault_Step(void)
{
  extern volatile uint16_t g_drive_block;
  extern volatile uint32_t g_reset_cause;
  char buf[208];
  uint16_t blk = g_drive_block;
  unsigned i, hit = 0U, idx;

  if (s_fault_busy == 0U) { return; }

  if (s_fault_step == 0U) {
    /* ⚠ "TRIPPED" 是**借来的词**：上位机 fault.ts 只认 TRIPPED / ok 两个词，
     *   写 BLOCKED 那一位就变成"读不出"，而"读不出"和"没事"在他们那边
     *   是刻意分开的两个答案 —— 借错词会让这块板一直停在"读不出"。
     * ⚠ 但本板的它不是驱动板那种闩住的跳闸，所以行尾当场限定
     *   (live, not latched)。
     * 数一下有几条在挡着 —— 这个数要和下面真发出去的行数对得上，
     * 上位机拿它当"行有没有少"的判据（同 param list 的 count=）。 */
    for (i = 0U; i < DRIVE_BLOCK_DESC_COUNT; i++) {
      if ((blk & g_drive_block_desc[i].bit) != 0U) { hit++; }
    }
    if (hit == 0U) {
      (void)snprintf(buf, sizeof(buf),
                     "safety   : OK -- nothing on THIS board is blocking a drive request. "
                     "That is not the same as the vehicle being safe.");
    } else {
      (void)snprintf(buf, sizeof(buf),
                     "safety   : TRIPPED [%u] board   block=0x%04X (live, not latched)",
                     hit, (unsigned)blk);
    }
    emit_line(buf);
    s_fault_step = 1U;
    return;
  }

  idx = (unsigned)(s_fault_step - 1U);
  while (idx < DRIVE_BLOCK_DESC_COUNT) {
    if ((blk & g_drive_block_desc[idx].bit) != 0U) {
      (void)snprintf(buf, sizeof(buf),
                     "  [%2u] %-12s board           live     x1  block bit %u",
                     (unsigned)(idx + 1U), g_drive_block_desc[idx].id, idx);
      emit_line(buf);
      (void)snprintf(buf, sizeof(buf), "       -> %s", g_drive_block_desc[idx].advice);
      emit_line(buf);
      s_fault_step = (uint8_t)(idx + 2U);
      return;
    }
    idx++;
  }

  /* 复位原因：这一条是**闩住的**，而且它是唯一一条"看起来一切正常"的坏消息。 */
  if (((g_reset_cause & (1U << 5)) != 0U) && (s_fault_step != 0xFFU)) {
    (void)snprintf(buf, sizeof(buf),
                   "  [%2u] iwdg-reset   board           latched  x1  reset cause 0x%02X",
                   (unsigned)(DRIVE_BLOCK_DESC_COUNT + 1U),
                   (unsigned)(g_reset_cause & 0xFFU));
    emit_line(buf);
    emit_line("       -> the watchdog reset this board. A board that restarts quietly "
              "looks fine from outside; 'tlm' sys iwdg=1 keeps saying so until power-off.");
    s_fault_step = 0xFFU;
    return;
  }

  /* HardFault 的痕迹。⚠ 它回答的是 RCC->CSR 回答不了的那个问题：
   * 软件复位**为什么**发生。见 stm32f4xx_it.c 的 HardFault_Handler。 */
  if ((g_hardfault_mark == HARDFAULT_MARK_MAGIC) && (s_fault_step != 0xFEU)) {
    (void)snprintf(buf, sizeof(buf),
                   "  [%2u] hardfault    board           latched  x1  mark 0x%08lX",
                   (unsigned)(DRIVE_BLOCK_DESC_COUNT + 2U),
                   (unsigned long)g_hardfault_mark);
    emit_line(buf);
    emit_line("       -> the CPU hit a HardFault and reset itself. The trace survives reset "
              "but not power-off, so it is about THIS power cycle.");
    s_fault_step = 0xFEU;
    return;
  }

  s_fault_busy = 0U;
}

void CartFault_CodesStep(void)
{
  char buf[240];
  unsigned idx;

  if (s_codes_busy == 0U) { return; }

  if (s_codes_step == 0U) {
    emit_line("code name            action   latched  meaning");
    s_codes_step = 1U;
    return;
  }

  idx = (unsigned)(s_codes_step - 1U);
  if (idx < DRIVE_BLOCK_DESC_COUNT) {
    (void)snprintf(buf, sizeof(buf), "%4u %-15s %-8s %-7s %s",
                   idx + 1U, g_drive_block_desc[idx].id,
                   g_drive_block_desc[idx].action,
                   (g_drive_block_desc[idx].latched != 0U) ? "yes" : "no",
                   g_drive_block_desc[idx].meaning);
    emit_line(buf);
    (void)snprintf(buf, sizeof(buf), "                                     -> %s",
                   g_drive_block_desc[idx].advice);
    emit_line(buf);
    s_codes_step = (uint8_t)(idx + 2U);
    return;
  }

  if (idx == DRIVE_BLOCK_DESC_COUNT) {
    emit_line("  15 iwdg-reset      note     yes     the watchdog reset this board at some point since power-on");
    emit_line("                                     -> a board that restarts quietly looks fine from outside; 'tlm' sys iwdg=1");
    s_codes_step = (uint8_t)(idx + 2U);
    return;
  }
  if (idx == (DRIVE_BLOCK_DESC_COUNT + 1U)) {
    emit_line("  16 hardfault       note     yes     the CPU hit a HardFault and reset itself during this power cycle");
    emit_line("                                     -> look for what ran just before; the trace does not survive power-off");
    s_codes_step = (uint8_t)(idx + 2U);
    return;
  }

  s_codes_busy = 0U;
}

static uint8_t  s_tlm_step;
static uint8_t  s_tlm_busy;

void CartTlm_Start(void)
{
  s_tlm_step = 0U;
  s_tlm_busy = 1U;
}

uint8_t CartTlm_Busy(void)
{
  return s_tlm_busy;
}

void CartTlm_Step(void)
{
  char buf[208];

  if (s_tlm_busy == 0U) {
    return;
  }

  switch (s_tlm_step) {
  case 0U:
    /* `ver` 发的是同一行，同一个合成器 —— 见 CartTlm_BuildLine。 */
    CartTlm_BuildLine(buf, (uint16_t)sizeof(buf));
    break;

  case 1U:
    /* rst 是上电时的 RCC->CSR>>24，**十进制**。iwdg 单独拎出来是因为
     * "看门狗咬过"这件事要紧的不是它会复位，是**有人知道它复位过** ——
     * 一块悄悄重启的车控板，从外面看一切正常。 */
    (void)snprintf(buf, sizeof(buf),
                   "=tlm %u sys ms=%lu rst=%d iwdg=%u hardfault=%u estop=%u",
                   (unsigned)FW_TELEM_VERSION,
                   (unsigned long)HAL_GetTick(),
                   (int)rd_rst(),
                   (unsigned)(((g_reset_cause & (1U << 5)) != 0U) ? 1U : 0U),
                   (unsigned)((g_hardfault_mark == HARDFAULT_MARK_MAGIC) ? 1U : 0U),
                   (unsigned)FocCAN_EstopActive());
    break;

  case 2U:
    /* 这一行是"这块板现在到底让不让车动"的全部输入。
     * block 是位图(十进制)，位的含义在 drive_block.h，**位只增不重用** ——
     * 上位机把人话挂在位号上，重编号会静默换掉一整屏解释。
     * ⚠ block=0 只说"本板这一侧没有拦"，不说"车可以走"。 */
    (void)snprintf(buf, sizeof(buf),
                   "=tlm %u veh mode=%d gear=%d gearsrc=%d gearnone=%d gearambig=%d "
                   "ven=%d brake=%d pedal=%d band=%d block=%d",
                   (unsigned)FW_TELEM_VERSION,
                   (int)rd_mode(), (int)rd_gear(),
                   (int)rd_gear_src(), (int)rd_gear_none(), (int)rd_gear_ambig(),
                   (int)rd_ven(), (int)rd_brake(),
                   (int)rd_pedal(), (int)rd_band(), (int)rd_block());
    break;

  case 3U:
    /* spd 和 spdok 必须一起读。spd=0 同时符合"停住了"和"没数据",
     * 而 AUTO 上电使能问的是后者(见 FocCAN_SpeedFeedbackValid)。
     * dir 是**实测**的方向，不是从油门推的。 */
    (void)snprintf(buf, sizeof(buf),
                   "=tlm %u spd kph10=%d spdok=%d dir=%d vbus_mv=%d ibus_ca=%d",
                   (unsigned)FW_TELEM_VERSION,
                   (int)rd_spd(), (int)rd_spd_ok(), (int)rd_spd_dir(),
                   (int)rd_vbus(), (int)rd_ibus());
    break;

  case 4U:
    /* alive/frozen/believes 是三个不同的问题，不是一个的三种说法：
     * 对面在发帧 / 对面在发但数不动了 / 对面说它还信我们。
     * health 是 0x211 的原始健康字节(十进制)。
     * ⚠ brakedrv=0 同时符合"对端还没发 / 发了但没使能 / 缸没接"。 */
    (void)snprintf(buf, sizeof(buf),
                   "=tlm %u can alive=%d frozen=%d believes=%d locked=%d health=%u rx=%lu tx=%lu "
                   "txfail=%d echomis=%d shortrx=%d fifoovf=%d busoff=%d busoff_now=%d brakedrv=%d topo_b=%d",
                   (unsigned)FW_TELEM_VERSION,
                   (int)rd_peer_alive(), (int)rd_peer_frozen(), (int)rd_peer_belief(),
                   (int)rd_peer_locked(),
                   (unsigned)g_foc_can.status_health,
                   (unsigned long)g_foc_can.rx_count, (unsigned long)g_foc_can.tx_count,
                   (int)rd_txfail(), (int)rd_echomis(), (int)rd_shortrx(),
                   (int)rd_fifo_ovf(), (int)rd_busoff(), (int)rd_busoff_now(),
                   (int)rd_brake_drv(), (int)rd_topo_b());
    break;

  case 5U:
    /* 三态：0=从来没接过 1=在线 2=接过然后断了。
     * "从没接过"和"断了"是两个不同的答案，而**只有后者该去刹车**。
     * uart 那两个计数是这根串口自己的健康 —— 它同时跑 Jetson 的二进制帧、
     * IMU CSV 和这个 CLI，丢字节不是理论上的。 */
    (void)snprintf(buf, sizeof(buf),
                   "=tlm %u rc ch1=%d ch2=%d rc3=%d uart_full=%d uart_ore=%d "
                   "juart_full=%d juart_ore=%d steerblk=%d",
                   (unsigned)FW_TELEM_VERSION,
                   (int)rd_rc_ch1(), (int)rd_rc_ch2(), (int)rd_rc_rc3(),
                   (int)rd_uart_full(), (int)rd_uart_ore(),
                   (int)rd_juart_full(), (int)rd_juart_ore(), (int)rd_steer_blk());
    break;

  default:
    emit_line("=tlm end");
    s_tlm_busy = 0U;
    return;
  }

  emit_line(buf);
  s_tlm_step++;
}
