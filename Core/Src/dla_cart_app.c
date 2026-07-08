/*
 * DlaCart 主循环与状态机（从 DlaCart 工程迁移，外设绑定见 main.c / board 说明）。
 */
#include "dla_cart_app.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdio.h>
#include "steering.h"
#include "servo_can.h"
#include "foc_can.h"
#include "rc_input.h"
#include "motor_control.h"
#include "screen.h"
#include "wheel_speed.h"
#include "train_uart.h"
#include "board_relays.h"
#include "brake_can.h"
#include "board_battery.h"
#include "board_inputs.h"
#include "imu_lsm6dsv.h"

#define POSITION_PER_REVOLUTION  10000
#define AUTO_ROTATION_SPEED      280
#define POSITION_TOLERANCE       100
#define STEERING_GEAR_RATIO      15
#define STEERING_RANGE_DEG       180
#define STEERING_PHYSICAL_DEG    37
#define STEERING_POSITION_RANGE  ((STEERING_RANGE_DEG * STEERING_GEAR_RATIO * POSITION_PER_REVOLUTION / 360) * 40 / 10)

#define SCREEN_HEARTBEAT_INTERVAL_MS  2000
const uint32_t SCREEN_TIMEOUT_MS = 8000U;
#define FOLLOW_CMD_TIMEOUT_MS  200U
#define FOLLOW_READY_RESEND_MS 100U
#define MAX_STEER_01           370
#define VEHICLE_ENABLE_DEBOUNCE_MS 50U

/* 自动驾驶模式参数(见 AUTONOMOUS_MODE_SPEC §6)。速度环增益/限速为保守起点，
 * 需台架标定(§9)后回填本表 + 规范。 */
#define AUTO_CMD_TIMEOUT_MS        300U   /* §6.1 指令看门狗:超时→刹车+停+回中 */
#define AUTO_MAX_SPEED_MM_S        1500   /* §6.2 硬限速(1.5 m/s)，双保险独立 clamp */
#define AUTO_MAX_THROTTLE_PERMILLE 600U   /* 自动模式油门上限(保守)，0..1000 */
#define AUTO_STATUS_PERIOD_MS      100U   /* §4 状态帧 ~10Hz */
/* 速度环 PI(整数定点)：throttle = Kp*err + Ki*∫err，err 单位 mm/s，dt=20ms 定步长 */
#define AUTO_SPD_KP_NUM            3L
#define AUTO_SPD_KP_DEN            10L
#define AUTO_SPD_KI_NUM            1L
#define AUTO_SPD_KI_DEN            50L
#define AUTO_SPD_I_MAX             30000L /* 积分限幅(抗饱和) */

ServoCAN_t g_servo;
FocCAN_t g_foc_can;
static uint32_t last_link_ms = 0;
uint8_t servo_enabled_sent = 0;
volatile SteeringMode_t g_steering_mode = MODE_MANUAL;
static uint16_t current_gear = SCREEN_KEY_GEAR_P;

typedef enum {
  AUTO_STATE_IDLE = 0,
  AUTO_STATE_GOING_HOME,
  AUTO_STATE_LEFT,
  AUTO_STATE_RIGHT
} AutoState_t;

static AutoState_t auto_state = AUTO_STATE_IDLE;
static int32_t home_position = 0;
static uint8_t home_position_valid = 0;
static uint8_t need_return_home = 0;
static int32_t last_target_position = 0x7FFFFFFF;
static uint32_t last_test_send_ms = 0;
static uint16_t test_counter = 0;
static uint16_t current_speed_level = SCREEN_KEY_SPEED_LOW;

/* 联调诊断：暴露刹车仲裁输入到 CLI(ST?)，定位刹车继电器抖动源 */
volatile uint16_t g_dbg_gear = 0;
volatile uint8_t  g_dbg_bsw = 0;      /* 刹车开关 PE2/PE3 */
volatile uint8_t  g_dbg_ven = 0;      /* 整车使能 PD10 */
volatile uint8_t  g_dbg_fbrake = 0;   /* 当前刹车意图 */
volatile uint32_t g_dbg_fbrake_chg = 0; /* 刹车意图跳变累计次数 */

/* RC3 远程遥控刹车状态：1=刹车，0=释放。直接镜像 RC3 两位开关位置(无软件自锁)。供 RC? 诊断查看。 */
volatile uint8_t  g_rc_brake = 0;

/* Jetson 经 USART1 回传的建图/存图状态(由 FocCAN_TestPollUart 这唯一的行消费者解析)。
 * 采集模式下驱动串口屏 0x0618(采集确认)/0x0619(保存状态)。*_rx_ms 用于新鲜度判断。 */
volatile uint8_t  g_host_collect_ack = 0;
volatile uint8_t  g_host_save_ok = 0;
volatile uint32_t g_host_map_rx_ms = 0;
volatile uint32_t g_host_save_rx_ms = 0;

static FollowCmd_t s_follow_cmd = { .steer_01 = 0, .thr_ref = 0, .brake = 1 };
static uint32_t last_follow_cmd_ms = 0;
static uint8_t follow_ready_sent = 0;
static uint32_t last_ready_send_ms = 0;

/* 自动驾驶模式状态。上电默认安全:刹车、零速、未就绪(§6.4)。 */
static AutoCmd_t s_auto_cmd = { .steer_01 = 0, .target_speed_mm_s = 0, .brake = 1, .gear = 0, .flags = 0 };
static uint32_t last_auto_cmd_ms = 0;
static uint8_t auto_ready_sent = 0;
static uint32_t last_auto_status_ms = 0;
static int32_t s_auto_speed_i = 0;           /* 速度环积分项 */
static uint16_t s_auto_throttle_permille = 0; /* 速度环输出(由 PI 20ms 更新) */
static uint8_t s_auto_dir = 0;                /* FOC 方向: 0=N 1=F 2=R */

static uint8_t Board_VehicleEnableRead(void)
{
  /* 保持型整车使能开关(PD10, 外部上拉, 低有效=接通)。去抖电平读取：
   * 开关接通(PD10 低)期间持续返回使能。原"锁存翻转(点动按钮)"语义与保持开关不匹配
   * (上电即闭合时无按下边沿→永不使能)，已改为电平语义。 */
  static uint8_t init = 0U;
  static uint8_t raw_last = 0U;
  static uint8_t stable = 0U;
  static uint32_t raw_since_ms = 0U;
  uint32_t now_ms = HAL_GetTick();
  uint8_t raw_active =
      (HAL_GPIO_ReadPin(VEHICLE_ENABLE_GPIO_Port, VEHICLE_ENABLE_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

  if (init == 0U) {
    init = 1U;
    raw_last = raw_active;
    stable = raw_active;
    raw_since_ms = now_ms;
    return stable;
  }

  if (raw_active != raw_last) {
    raw_last = raw_active;
    raw_since_ms = now_ms;
  }

  if ((raw_active != stable) &&
      ((now_ms - raw_since_ms) >= VEHICLE_ENABLE_DEBOUNCE_MS)) {
    stable = raw_active;
  }

  return stable;
}

static void Board_TuneTimersForCart(void)
{
  TIM_IC_InitTypeDef ic_rc = {0};
  ic_rc.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  ic_rc.ICSelection = TIM_ICSELECTION_DIRECTTI;
  ic_rc.ICPrescaler = TIM_ICPSC_DIV1;
  ic_rc.ICFilter = 0U;

  TIM_IC_InitTypeDef ic_wheel = {0};
  ic_wheel.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  ic_wheel.ICSelection = TIM_ICSELECTION_DIRECTTI;
  ic_wheel.ICPrescaler = TIM_ICPSC_DIV1;
  /* ICFilter=8：要求电平稳定 N×采样周期后才认为是有效边沿，
   * 与 DlaCart 原工程 (TIM4_CH3/CH4) 一致，抑制 FG 线/电机 PWM 的毛刺。 */
  ic_wheel.ICFilter = 8U;

  /* TIM1: APB2 定时器时钟 168 MHz -> PSC 167, ARR 999 => 1 kHz PWM */
  __HAL_TIM_DISABLE(&htim1);
  TIM1->PSC = 167U;
  TIM1->ARR = 999U;
  TIM1->EGR = TIM_EGR_UG;
  __HAL_TIM_ENABLE(&htim1);

  /* TIM2: APB1 定时器 84 MHz -> PSC 83 => 1 MHz 计数。
   * 轮速实际接 CH3 (PA2) / CH4 (PA3)，外部 10k 下拉 + 100Ω 串接，空闲为 0V，
   * 上升沿为有效脉冲。CH1/CH2 未走线、保持浮空，不开 IC 中断。 */
  __HAL_TIM_DISABLE(&htim2);
  TIM2->PSC = 83U;
  TIM2->ARR = 0xFFFFFFFFU;
  TIM2->EGR = TIM_EGR_UG;
  (void)HAL_TIM_IC_ConfigChannel(&htim2, &ic_wheel, TIM_CHANNEL_3);
  (void)HAL_TIM_IC_ConfigChannel(&htim2, &ic_wheel, TIM_CHANNEL_4);
  __HAL_TIM_ENABLE(&htim2);

  /* TIM3: 遥控 PWM 输入捕获 1 MHz */
  __HAL_TIM_DISABLE(&htim3);
  TIM3->PSC = 83U;
  TIM3->ARR = 65535U;
  TIM3->EGR = TIM_EGR_UG;
  (void)ic_rc;  /* RC 通道配置仍由 RC_Input_Init/Init_CH2 完成（ICFilter=0） */
  __HAL_TIM_ENABLE(&htim3);
}

void DlaCart_AppInit(void)
{
  Board_TuneTimersForCart();

  RC_Input_Init();
  RC_Input_Init_CH2();
  RC_Input_Init_RC3();   /* RC3 = PC7 远程刹车(置位) */
  RC_Input_Init_RC4();   /* RC4 = PC6 远程刹车(复位/释放) */

  ServoCAN_Init(&g_servo, 0);
  FocCAN_Init(&g_foc_can);

  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  Motor_Control_Init();

  Screen_Init();
  HAL_Delay(500);
  Screen_SetInitialValues();

  (void)HAL_ADC_Start(&hadc1);

  WheelSpeed_Init();
  TrainUART_Init();

  (void)IMU_Init();   /* LSM6DSVETR；失败(g_imu.ok=0)不阻塞主流程，采集帧 IMU 字段填 0 */

  last_link_ms = HAL_GetTick();
  last_test_send_ms = HAL_GetTick();
  test_counter = 0;
  (void)auto_state;
  (void)need_return_home;
  (void)last_test_send_ms;
  (void)test_counter;
}

void DlaCart_AppLoopIteration(void)
{
#include "dla_cart_loop.inc"
}
