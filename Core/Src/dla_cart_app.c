/*
 * DlaCart 主循环与状态机（从 DlaCart 工程迁移，外设绑定见 main.c / board 说明）。
 */
#include "dla_cart_app.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
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
#include "drive_block.h"
#include "cart_persist.h"
#include "cart_params.h"
#include "cart_param.h"
#include "fw_version.h"

/* 可调常量统一住在 cart_params.h —— 见那里开头关于"每个事实一个家"的说明。
 * SCREEN_TIMEOUT_MS 仍是这里的一个 const 变量，因为 screen.h 按 extern 引用它。 */
const uint32_t SCREEN_TIMEOUT_MS = 8000U;

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
/* 四路档位输入的原始"是否为低"位图：bit0=P bit1=R bit2=N bit3=D（U7 脚 2..5）。
 * ⚠ 和解出来的档位分开报，因为接线之后要拨一遍杆确认开关是不是持续闭合型的，
 *   而"解出来的档位"看不出这件事。 */
volatile uint8_t  g_dbg_gear_raw = 0U;
/* 档位来源三态，见 board_inputs.h：NONE=从没解出过（屏可以设档）
 * SWITCH=解出过（实体开关说了算）。 */
volatile uint8_t  g_gear_src = GEAR_SRC_NONE;
/* 两个故障计数分开，因为要查的线不一样：零路=断线/没插，多路=短路/先通后断。 */
volatile uint32_t g_gear_none_count = 0U;
volatile uint32_t g_gear_ambig_count = 0U;
volatile uint8_t  g_dbg_bsw = 0;      /* 刹车开关 PE2/PE3 */
volatile uint8_t  g_dbg_ven = 0;      /* 整车使能 PD10 */
volatile uint8_t  g_dbg_fbrake = 0;   /* 当前刹车意图 */
volatile uint32_t g_dbg_fbrake_chg = 0; /* 刹车意图跳变累计次数 */
volatile uint32_t g_dbg_can_short = 0; /* 收到的短帧(DLC!=8)累计丢弃数 */
volatile uint32_t g_dbg_can_fifo_ovf = 0;  /* 接收 FIFO0 溢出次数(溢出即覆盖，不锁) */
volatile uint32_t g_dbg_can_busoff = 0;    /* 进入 bus-off 的次数(不是当前状态) */
volatile uint8_t  g_dbg_can_busoff_now = 0;/* 此刻是不是在 bus-off 里 */
/* 司机踩了多少油门，0..1000‰，与档位无关。整块板上只在主循环解算一次。 */
volatile uint16_t g_pedal_permille = 0;
/* 踏板 ADC 原始值。标定和换算读同一份。 */
volatile uint16_t g_dbg_adc_raw = 0;
volatile uint8_t  g_dbg_band = 0;      /* 当前主张的档位带 %，0 = 不声明 */

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


/* ---------------------------------------------------------------------------
 * 独立看门狗 (IWDG)
 *
 * 从前这块板上一个看门狗都没有：stm32f4xx_hal_conf.h 里 HAL_IWDG_MODULE_ENABLED
 * 和 HAL_WWDG_MODULE_ENABLED 都是注释掉的，整棵树没有初始化也没有喂狗。
 * 而 Error_Handler() 是 __disable_irq() 之后 while(1)，HardFault/MemManage/
 * BusFault/UsageFault/NMI 也全都是裸 while(1) —— 任何一次进去，板子就彻底
 * 不动了：0x210 停发，中断关掉，GPIO 冻在原地。下游驱动板 300ms 后会停力矩,
 * 但**刹车意图是写在那一帧的一个 bit 里的**，帧不发了，下游只知道"停力矩"，
 * 不知道"该刹车"。
 *
 * 直接写寄存器而不是走 HAL：Drivers/ 里根本没有 stm32f4xx_hal_iwdg.c，
 * 打开 HAL_IWDG_MODULE_ENABLED 也拿不到 HAL_IWDG_Init。IWDG 一共四个寄存器,
 * 与其往 Cube 管理的驱动集里塞文件、再去改它生成的 subdir.mk，不如照着
 * 参考手册写这十行。
 *
 * 超时取标称 2s：LSI 在 F407 上标称 32kHz，数据手册范围 17~47kHz，
 * 所以真实超时在 1.36s(47kHz) ~ 3.76s(17kHz) 之间。主循环最坏一趟约 100ms
 * (1 秒边界上电池 ADC + 四次 9600 波特的屏幕阻塞写)，1.36s 有十倍余量。
 * 初始化里有一次 HAL_Delay(500)，那一段中间补喂一次。
 * ------------------------------------------------------------------------- */
#define IWDG_KEY_RELOAD    0x0000AAAAU
#define IWDG_KEY_WRITE_EN  0x00005555U
#define IWDG_KEY_START     0x0000CCCCU
#define IWDG_PRESCALER_64  0x04U      /* PR=4 → 分频 64 → 32kHz/64 = 500Hz */
#define IWDG_RELOAD_2S     999U       /* 1000 拍 / 500Hz = 2.0s 标称 */

volatile uint32_t g_reset_cause = 0U;  /* 上电时的 RCC->CSR 高位，见 Board_WatchdogInit */

void Board_WatchdogInit(void)
{
  /* 先把复位原因取走再清 —— 不取的话，看门狗到底有没有咬过永远不会有人知道。
   * 位定义见参考手册 RCC_CSR：IWDGRSTF(29) WWDGRSTF(30) LPWRRSTF(31)
   * SFTRSTF(28) PORRSTF(27) PINRSTF(26) BORRSTF(25)。 */
  g_reset_cause = RCC->CSR >> 24;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /*
   * 顺序要紧：先写启动键，再解锁 PR/RLR。
   *
   * IWDG->SR 里的 PVU/RVU 要等 LSI 真的在跑之后才会清零，而 LSI 正是被
   * 启动键 0xCCCC 拉起来的。先写 PR/RLR 再等 SR，就会永远等不到 ——
   * 板子卡死在这个函数里，而且因为启动键还没写下去，看门狗本身也没跑起来,
   * 连复位都不会发生。ST 自己的 HAL_IWDG_Init 就是先启动再解锁的。
   * (这个顺序错误已经在台架上真的发生过一次：刷完之后串口彻底没有回应。)
   *
   * 等待也要有上限。这是一段在整车控制板启动路径最前面、看门狗还没生效
   * 时跑的代码 —— 一个没有上限的 while 放在这里，正是它要防的那种东西。
   */
  IWDG->KR = IWDG_KEY_START;      /* 启动，同时拉起 LSI */
  IWDG->KR = IWDG_KEY_WRITE_EN;   /* 解锁 PR/RLR */
  IWDG->PR = IWDG_PRESCALER_64;
  IWDG->RLR = IWDG_RELOAD_2S;
  {
    /* LSI 最慢 17kHz，一次 PR/RLR 写入生效约几个 LSI 周期。这里给
     * 200000 次空转，168MHz 下远超那个量级；超时就带着 Cube 复位默认值
     * (分频 4 / 重载 4095 ≈ 512ms @32kHz)继续跑，不卡在这里。 */
    uint32_t guard = 200000U;
    while ((IWDG->SR != 0U) && (guard != 0U)) {
      guard--;
    }
  }
  IWDG->KR = IWDG_KEY_RELOAD;
}

void Board_WatchdogKick(void)
{
  IWDG->KR = IWDG_KEY_RELOAD;
}


/*
 * 0.1km/h -> mm/s，饱和而不是回绕。
 *
 * 从前两处站点都是 `(int32_t)meas_01kmh * 1000L / 36L` 之后直接强转 int16_t。
 * 驱动板在 0x212[5:6] 里报一个 0xFFFF，算出来是 1 820 416 mm/s，截断成
 * **-14592 mm/s** —— 一个很大的**倒车**速度，原样交给 Jetson 的控制器。
 * 收下来的每个字节都来自另一块板，而没有任何地方检查过它的量级。
 *
 * 合理性上限在 FocCAN_SpeedFeedbackValid 里挡(超了整条反馈判为不算数)；
 * 这里再做一次饱和，是因为"判为不算数"和"这个数字被用了"是两件事：
 * 状态帧里那个字段总要填一个值，填一个错方向的大数比填一个饱和值糟得多。
 */
static int16_t speed_01kmh_to_mm_s(uint32_t meas_01kmh, uint8_t reverse)
{
  int32_t mm_s = (int32_t)((meas_01kmh * 1000UL) / 36UL);
  if (mm_s > 32767) {
    mm_s = 32767;
  }
  if (reverse != 0U) {
    mm_s = -mm_s;
  }
  return (int16_t)mm_s;
}


volatile uint16_t g_drive_block = 0U;

/*
 * 只给短标识符，不给句子。
 *
 * 人话归上位机(BoardPack) —— 这是驱动板侧付了代价学到的一条：
 * 句子写在固件里，改一句话就要发一次固件。
 */

/*
 * ==================== 位 → 人话 ====================
 *
 * ⚠⚠ 这几行是我 2026-08-27 说过"不写"的东西 —— 当时的话是
 * "固件只给短标识符和位图，**不给句子**，人话归上位机的 BoardPack，
 * 否则改一句话就要发一次固件"。那句话本身没错，但它错在**范围**：
 *
 *   上位机的 `fault codes` 解析器（fault.ts 的 CODE_ROW / ADVICE 正则）
 *   **要求固件给出 meaning 和 -> advice 两列** —— 它是照驱动板的实现写的，
 *   而 BoardPack 里没有放故障人话的地方。
 *   ⇒ 不给句子 = 这块板的 `fault_read` 永远是空的。
 *
 * ⇒ 接入的标准是**别人已经做成的那一套**，不是我一个人的品味。
 *   句子写在这里，短、稳定、只说"是什么"和"下一步查哪儿"。
 *   ⚠ 界面上的长篇解释仍然归 BoardPack —— 那一层没被这条推翻。
 *
 * ⚠ action 三选一，含义照上位机 fault.ts：
 *     cut   —— 它在阻止驱动
 *     limit —— 它在限制，但没有完全阻止
 *     note  —— 只是记录，不阻止任何事
 * ⚠ latched：本板的 drive_block 位全部是**活值**（条件消失就清），所以是 no。
 *   而复位原因那几条是闩住的（要重新上电才变），是 yes。
 */
const DriveBlockDesc_t g_drive_block_desc[] = {
  { DRIVE_BLOCK_VEH_ENABLE, "ven", "cut", 0,
    "the vehicle enable switch is open, so no drive request may be sent",
    "check the switch on PD10 / CN22 and its wiring; 'tlm' shows veh ven=" },
  { DRIVE_BLOCK_GEAR, "gear", "cut", 0,
    "the gear selector is not in a driving position (D, or R where allowed)",
    "'GEAR?' shows the four raw inputs and the decoded gear; U7 pins 2..5 are P R N D" },
  { DRIVE_BLOCK_BRAKE_SW, "brake_sw", "cut", 0,
    "the brake pedal switch is pressed",
    "release the pedal; a stuck switch reads pressed forever -- 'tlm' veh brake=" },
  { DRIVE_BLOCK_RC_BRAKE, "rc_brake", "cut", 0,
    "the remote emergency-stop channel is in the braking position",
    "move the RC3 switch back; 'RC?' shows the pulse width and the link state" },
  { DRIVE_BLOCK_RC_LOST, "rc_lost", "cut", 0,
    "the remote link was alive and is now gone, while remote is the command source",
    "check the receiver power and the bind; 'RC?' tells never/up/lost apart" },
  { DRIVE_BLOCK_FOC_STALE, "foc_stale", "cut", 0,
    "the drive board has stopped sending its status frame",
    "check the CAN wiring and the 120R termination; 'tlm' can rx= should be climbing" },
  { DRIVE_BLOCK_SPEED_INVALID, "spd_bad", "cut", 0,
    "the road-speed feedback does not count, so the speed loop has nothing to close on",
    "'tlm' spd spdok=0 says so; a stale telemetry frame looks exactly like a stopped vehicle" },
  { DRIVE_BLOCK_HOST_STALE, "host_stale", "cut", 0,
    "no fresh command from the host computer within the timeout",
    "check the host link on USB2; 'param get auto_cmd_timeout' is the window" },
  { DRIVE_BLOCK_HOST_DISABLE, "host_dis", "cut", 0,
    "the host computer is connected but has not asked for drive to be enabled",
    "the host must set the enable flag in its command frame" },
  { DRIVE_BLOCK_CLI_TEST, "cli_test", "cut", 0,
    "a bench CLI command is holding the drive request, so normal arbitration is bypassed",
    "'FOC OFF' hands it back; the latch also expires on its own" },
  { DRIVE_BLOCK_FOC_FROZEN, "foc_frozen", "cut", 0,
    "frames still arrive from the drive board but its sequence number stopped moving",
    "its application is stuck -- power-cycle the drive board; this is not a cable fault" },
  { DRIVE_BLOCK_FOC_DISBELIEF, "foc_disbel", "cut", 0,
    "the drive board says it no longer believes our command stream",
    "it heard us and stopped trusting us -- a different wire from 'it never heard us'" },
  { DRIVE_BLOCK_FOC_LOCKED, "foc_locked", "cut", 0,
    "the drive board reports it is in LOCAL mode and will not act on the bus",
    "unlock it at the drive board ('lock off'); a 0 here does NOT prove it is unlocked on firmware older than 0.4.11" },
  /* ⚠ latched=1：这一条和上面十三条不同 —— 条件消失它也不会自己清。 */
  { DRIVE_BLOCK_ESTOP, "estop", "cut", 1,
    "the emergency stop is latched; every drive request is forced to stop and brake",
    "'clear' releases it. This is a SOFTWARE e-stop -- there is no gate cut-off that bypasses the CPU" },
};

const unsigned g_drive_block_desc_count =
  sizeof(g_drive_block_desc) / sizeof(g_drive_block_desc[0]);

int DriveBlock_Describe(uint16_t bits, char *out, int out_size)
{
  const DriveBlockDesc_t *k_tab = g_drive_block_desc;
  int n = 0;
  unsigned i;

  if ((out == NULL) || (out_size <= 0)) {
    return 0;
  }
  out[0] = '\0';
  if (bits == 0U) {
    /* "没有任何一条在挡着"要有一个明确的词，不能是空字符串 ——
     * 空字符串既像"没有阻止"，也像"这一行坏了"。 */
    n = snprintf(out, (size_t)out_size, "none");
    return (n < 0) ? 0 : n;
  }
  for (i = 0U; i < (DRIVE_BLOCK_DESC_COUNT); i++) {
    if ((bits & k_tab[i].bit) != 0U) {
      int w = snprintf(out + n, (size_t)(out_size - n), "%s%s", (n > 0) ? "," : "", k_tab[i].id);
      if ((w < 0) || (w >= (out_size - n))) {
        break;
      }
      n += w;
    }
  }
  return n;
}

void DlaCart_AppInit(void)
{
  /* 参数持久化：先把存储层准备好，再按范围校验装载。
   * ⚠ 装载失败（没器件 / 空 / CRC 坏）**不是错误路径** —— 出厂的板子就是空的，
   *   此时全部参数保持编译期默认，而 `tlm` 的 persist 字段会说出是哪一种。 */
  CartPersist_Init();
  (void)CartPersist_Load();

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
  Board_WatchdogKick();   /* 这一段 500ms 阻塞比看门狗最坏超时的四分之一还长 */
  HAL_Delay(500);
  Board_WatchdogKick();
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
