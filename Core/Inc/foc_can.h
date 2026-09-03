#ifndef FOC_CAN_H
#define FOC_CAN_H

#include <stdint.h>
#include "main.h"

/*
 * DlaBoard_A -> FOC2 正式驾驶请求协议：1Mbps 经典 CAN，标准帧 DLC=8。
 * 旧 0x200 PWM 脉宽协议已废弃，控制板只向 FOC2 下发车辆语义。
 *
 * 控制板 -> FOC2  标准帧 ID=FOC_CAN_DRIVE_REQ_ID_STD
 *   [0:1] throttle_permille uint16 LE  0..1000
 *   [2:3] steer_permille    int16 LE   -1000..1000
 *   [4]   flags             bit0=allow_drive bit1=brake
 *                           bit2-7 = 档位(speed band)，2% 一档，0=不声明
 *                                    <-- 2026-08-25 新增
 *   [5]   gear              0=N 1=F 2=R
 *   [6]   seq               alive counter
 *   [7]   timeout_10ms      0=FOC2 默认超时；非 0 建议 30(300ms)
 *
 * 驱动板 -> 控制板  状态帧 DLC=8（周期上行, ~20ms）
 *   [0]   health flags bit0=foc1_run bit1=foc2_run bit2=foc1_edge_ok
 *         bit3=foc2_edge_ok bit4=foc1_nfault_ok bit5=foc2_nfault_ok
 *         bit6=cmd_recent bit7=brake_drv_enabled (2026-08-28 新增)
 *
 *         ! 该字节各位一律"1 = 好" —— 连低有效的 nFAULT 也已归一成 _ok。
 *           约定不是"照抄引脚电平"，是统一归一到 1 = 好。b7 跟随它。
 *
 *         ! b7 的精确含义是"**驱动板的刹车驱动使能开着**"(那侧的
 *           s_brake_enable)，也就是**一个被存下来的决定**，
 *           **不是**"确实接了一个刹车缸"。没有任何地方验证过缸真的接着。
 *           ⇒ 这两句话差一个字，但推论完全不同：前者能推出"我发的 brake
 *             位会走到 H 桥"，后者能推出"车真的会停"，而后者没人验证过。
 *
 *         ! 旧固件 b7 = 0，与"没使能"同形。见 FocCAN_BrakeDriveEnabled。
 *
 *         ! ✅ **b7 已上板双向验证(2026-08-28)**：驱动板置位时发 health=0xf0、
 *           本板读 brakedrv=1；关回后发 0x70、本板读 0。两个方向都由**接收端**
 *           观测过，不只是发送端看自己 —— 只验"能变成 1"证明不了它不是被写死的。
 *           ⚠ 严格说驱动板那侧走的是 ST-Link 复位，不是拔电：
 *             **"复位后已验、冷启动未验"**，不要合并成一条。
 *
 *         ! ⚠ 本文件是 0x211/0x212 字节布局的权威副本(驱动板
 *           docs/03_通信协议.md §3.1 也这么指)。CAN 上**不能单方面加位** ——
 *           判据不是"加还是改"，是两块板的头文件有没有一起改。
 *           b7 于 2026-08-28 与 FOC_Double_C session 约定后同时落地。
 *   [1]   seq
 *   [2:3] foc1 vehicle mechanical rpm int16 LE
 *   [4:5] foc2 vehicle mechanical rpm int16 LE
 *   [6]   echoed last command flags
 *   [7]   bit7=last command valid, bit6=local_lock, bit0-3=last command gear
 *
 *         ! b6 = **驱动板处在本地模式**(1 = 锁着)。定义在驱动板
 *           App/comm/can_proto.h 的 CAN_STATUS_LOCAL_LOCK，0.4.11 落地并烧板。
 *           本板 2026-08-30 才把接收侧补上 —— 这不是"单方面加位"：
 *           位已经在线上了，这边只是终于开始读它。
 *
 *         ! 极性 1 = 锁着，是一句陈述而不是双重否定。⚠ 字节 7 不是健康字节
 *           (bit7 是 valid、低四位是档位)，所以字节 0 那条"1 = 好"的约定
 *           **在这里不适用**。
 *
 *         ! 为什么必须单独一位、不能从别处推：**被锁的板和完全服从的板
 *           在这之前是同一帧**。线掉了的板会停发 0x211，本板看得见；
 *           而被锁的板照发、照报 cmd_valid、照回显 flags、照置 cmd_recent ——
 *           因为那四样回答的都是"帧在不在、新不新鲜"，而那仍然是真的。
 *           锁住在上一层(它的合并层 ctrl_input.c)，没有任何东西把它带下来。
 *           ⇒ 本板会从三个诚实地回答着错误问题的位，arm 一台不会听话的机器。
 *
 *         ! ⚠⚠ **0 在版本之间是有歧义的，而且不是良性的。**
 *           本地锁从 0.4.8 就有，而 0.4.8/0.4.9/0.4.10 无论锁没锁都发 0。
 *           这条总线上没有任何东西携带固件版本(0x212[7] 是空字节、0x211 没有)
 *           ⇒ 本板分不出"一块被锁的老板子"和"一块没锁的 0.4.11"。
 *           ⇒ **0 的意思是"没锁，或者固件比 0.4.11 老"。在成员不全是
 *             0.4.11+ 的总线上，不许把 0 读成"没锁"。** 台架第二块今天是
 *             0.4.10，这是活的歧义，不是假想。
 *           ⇒ 所以本板**只在读到 1 的时候置阻止位**。1 永远不是谎话
 *             (不实现它的固件发 0)；0 什么也不主张。
 *
 * 驱动板 -> 控制板  母线遥测 DLC=8（供串口屏显示）
 *   [0:1] vbus_mV  uint16 LE  母线电压(毫伏)
 *   [2:3] ibus_cA  int16 LE   母线电流(厘安, 10mA/LSB)
 *   [4]   seq      心跳计数
 *   [5:6] speed_01kmh uint16 LE  车速(0.1 km/h)  <-- 2026-08-25 新增
 *   [7]   预留
 *
 * ! [5:6] 为什么必须由驱动板换算好了再发，而不是本板从 0x211 的 rpm 自己算：
 *
 *   0x211 发 rpm 是对的 —— rpm 属于电机，和轮胎无关。但每一个想要车速的
 *   消费者都得乘一个轮周长，于是每一个都自己长了一份。2026-08-25 首次真
 *   总线联调实测，同一个物理量有三个家：驱动板 flash 里 798 mm(滚一圈量
 *   地面的)，本板 wheel_speed.h 里 816 mm，本板 foc_can.c 里又一个 816 mm
 *   —— 后两个都是从标称直径 260 mm 算的，而轮胎在载荷下不等于它的铭牌
 *   (798 mm 反推滚动直径 254 mm)。
 *
 *   后果不只是显示：屏上车速高 2.26%，而 MODE_AUTO 的速度闭环 PI 用的是
 *   同一个偏高的反馈量，于是自动驾驶会稳定在比目标低 2.26% 的速度上。
 *
 *   ⇒ 由那块被告知过轮周长、并把它存在 flash 里的板子换算(换个轮子改一条
 *   命令，不用重编译)。本板 foc_can.c 里那份常量已删除。
 *
 *   这是加法不是变更：[0:4] 一个字节没动。两块板要一起改才能用上新字段。
 *
 * 两种驱动板拓扑，状态/遥测帧格式一致：
 *   A) 单块 FOC_double(双电机)：状态发 0x211(同时填 foc1/foc2 两个轮速),
 *      遥测发 0x212。
 *   B) 两块 FOC_single(各一路)：左轮板 状态 0x211(只填 foc1 槽)+遥测 0x212;
 *      右轮板 状态 0x213(只填 foc2 槽)+遥测 0x214。本板把两块板的上行合并成
 *      与双电机板等价的 foc1/foc2 视图(轮速、健康位、母线电流求和)。
 */

#define FOC_CAN_DRIVE_REQ_ID_STD  0x210U   /* 广播行走请求：两块驱动板都接收 */
#define FOC_CAN_STATUS_ID_FOC2    0x211U   /* 双电机板 / 单路左轮板 状态      */
#define FOC_CAN_TELEM_ID_FOC2     0x212U   /* 双电机板 / 单路左轮板 遥测      */
#define FOC_CAN_STATUS_ID_R       0x213U   /* 单路右轮板 状态                 */
#define FOC_CAN_TELEM_ID_R        0x214U   /* 单路右轮板 遥测                 */

#define FOC_CAN_TX_PERIOD_MS      20U
#define FOC_CAN_DEFAULT_TIMEOUT_10MS 30U
#define FOC_CAN_FLAG_ALLOW_DRIVE  0x01U
#define FOC_CAN_FLAG_BRAKE        0x02U

/* 0x211[0] / 0x213[0] 健康位里的刹车驱动使能位。 */
#define FOC_CAN_HEALTH_BRAKE_DRV  0x80U

/*
 * 档位(屏上的速度档)现在发给驱动板，而不是在本板乘进油门。
 *
 * ! 原来本板做的是 foc_offset = offset * Screen_GetSpeedScale(档位)，也就是
 *   **把档位当成扭矩档**。驱动板上这个缺陷 2026-08-25 修过一次(那边叫缺陷 J)：
 *   低速档削掉 65% 扭矩 ⇒ 带人起不动车，而控制做的事和它的标签正好相反。
 *   本板原封不动地保留着同一个做法。
 *   ⇒ 档位只应该缩放**转速上限**，而转速上限在驱动板上，所以档位要过 CAN。
 *
 * ! 为什么发百分比而不是档位序号：序号要两块板对同一张表，一个产品决定两份
 *   拷贝，屏加第五个按钮的那天第二份就错了。百分比自己就是自己。
 *
 * ! 为什么 2% 一档：byte4 只剩 6 位，整帧其余部分是满的。0..63 覆盖 0..126%，
 *   驱动板夹到 100%。四个标准档 35/60/80/100 里只有 35% 落不准，到那边是 36%
 *   —— 250 rpm 上限的 2.5 rpm，本机 0.12 km/h。写下来而不是抹掉。
 *
 * ! 0 = 不声明。驱动板收到 0 时**不**主张这个字段，于是它的合并逻辑继续沿用
 *   上一次的档位 —— 那正是本字段出现之前的行为，所以旧控制板配新驱动板不会
 *   把车的上限压成零。
 */
#define FOC_CAN_BAND_SHIFT        2U
#define FOC_CAN_BAND_STEP_PCT     2U
#define FOC_CAN_BAND_MAX_STEPS    50U   /* 50 x 2% = 100% */

typedef struct {
  uint16_t throttle_permille;
  int16_t steer_permille;
  uint8_t flags;
  uint8_t gear;
  uint8_t seq;
  uint8_t timeout_10ms;
  uint32_t tx_count;        /* 真正进了发送邮箱的帧数(不是尝试数) */
  uint32_t tx_fail_count;   /* AddTxMessage 失败而丢掉的帧数 */
  uint32_t tx_abort_count;  /* 邮箱满、作废旧帧腾位置的次数 */
  uint32_t rx_count;
  uint32_t last_status_ms;
  uint8_t status_seen;
  /* 见过 0x213 = 现场是拓扑 B(两块单电机板)。
   * ! 拓扑必须靠"帧从哪来"判，不能靠"值恰好是不是 0" —— 见 foc_can.c
   *   里 0x211 的 foc2 槽。 */
  uint8_t status_r_seen;      /* 一路闩锁：见够 0x213 ⇒ 拓扑 B。只有断电才复位 */
  uint8_t status_r_evidence;  /* 收到过几帧 0x213；攒够才闩，一帧走失不算数 */
  /* 最后一帧 0x210 的**实际发出内容**。
   * ⚠ 加它是因为"急停会覆盖发出去的那一帧"这条规则，在总线上没有驱动板的时候
   *   **无法被观测** —— 而一条观测不到的安全规则和没有那条规则，从外面看一样。
   *   驱动板的回显(echo=)只在它在线时有；这一份是本板自己记的。 */
  uint8_t tx_last[8];
  uint8_t status_health;
  /* 0x211[7] b6。⚠ 0 是"没锁 **或** 固件比 0.4.11 老"，只许对 1 反应。 */
  uint8_t status_local_lock;
  uint8_t status_seq;
  int16_t foc1_rpm;
  int16_t foc2_rpm;
  uint8_t status_cmd_flags;   /* 0x211[6]：驱动板回显的、它最后一次解出的整个 flags 字节 */
  uint8_t status_cmd_valid;   /* 0x211[7] bit7：驱动板认为"我们这一路还活着" */
  uint8_t status_cmd_gear;
  /*
   * 回环用的簿记。以前 status_seq / status_cmd_flags / status_cmd_valid
   * 三个字段解出来只被 ST? 打印，没有任何判断读它们 —— 驱动板特意造的那个
   * 诊断("你根本没听见我" vs "我听见了但已经不信了")一直被原样丢掉。
   */
  uint8_t  status_seq_prev;          /* 上一帧的 0x211[1]，用来判序列号动没动 */
  uint32_t status_seq_move_ms;       /* 序列号最后一次变化的时刻 */
  uint32_t peer_frozen_count;        /* 帧还在来但序列号不动的累计次数 */
  uint32_t peer_disbelief_count;     /* 驱动板把我们判为不活的累计次数 */
  uint32_t echo_mismatch_count;      /* 回显的 flags 和我们发的对不上的累计次数 */
  uint8_t last_status[8];
  /* 两帧各自的原始健康字节。b6/b7 是关于**整车**的判断，两块板都会写,
   * 合成一个字节就成了"最后到的说了算"。合并规则见访问器。 */
  uint8_t status_raw_l;
  uint8_t status_raw_r;
  /* 母线遥测：左轮板/双电机板(0x212) */
  uint16_t vbus_mV;
  int16_t  ibus_cA;
  uint8_t  telem_seq;
  uint8_t  telem_seen;
  uint32_t last_telem_ms;
  uint16_t speed_01kmh;       /* [5:6]，驱动板换算好的车速 */
  /* 驱动板还在发旧固件(=[5:6] 仍是预留的 0)的次数。
   * ! 不是装饰：新固件的本板配旧固件的驱动板，屏上会安静地显示 0。
   *   这个计数让"安静地显示 0"变成"看得见地退回了轮速传感器"。ST? 会印它。 */
  uint32_t speed_wire_missing;
  /* 母线遥测：单路右轮板(0x214)。双电机拓扑下这些字段保持 0，不影响显示。 */
  uint16_t vbus_mV_r;
  int16_t  ibus_cA_r;
  uint8_t  telem_seq_r;
  uint8_t  telem_seen_r;
  uint32_t last_telem_ms_r;
  uint16_t speed_01kmh_r;
} FocCAN_t;

void FocCAN_Init(FocCAN_t *fc);
void FocCAN_OnRxFrame(FocCAN_t *fc, uint32_t std_id, const uint8_t data[8]);
HAL_StatusTypeDef FocCAN_SendDriveRequest(FocCAN_t *fc,
                                          uint16_t throttle_permille,
                                          int16_t steer_permille,
                                          uint8_t allow_drive,
                                          uint8_t brake,
                                          uint8_t gear,
                                          uint8_t timeout_10ms,
                                          uint8_t band_percent);
void FocCAN_TestPollUart(FocCAN_t *fc);      /* USART1：CLI，任何模式都可调 */
void FocCAN_PollJetsonLines(void);          /* UART5 ：Jetson 回的 MAP,/SAVE, */
/*
 * ==================== 紧急停止（DBIS §5.7，L2 强制） ====================
 *
 * 标准的原话：**任何状态下都能立即切断执行器输出，优先级高于一切**
 * （包括正在进行的标定）；应答后进入**锁存的安全态**，需显式 `clear` 才能恢复。
 *
 * ⚠⚠ 这条规则写在 **FocCAN_SendDriveRequest() 里面**，不是写在它的两个调用点上。
 *   理由是这个仓自己记过的那条：**一条规则写在调用点里，下一个调用点不会知道** ——
 *   而这块板的行驶请求今天有两个出口（CLI 测试路径和正常仲裁路径），
 *   明天可能有第三个。放进被调用的一侧，新出口天生就受它管。
 *
 * ⚠ 清除是**边沿触发**（DBIS §9.1 S6）：一条 `clear` 命令 = 一次事件。
 *   电平持续清除会让一个卡住的清除信号把闩锁永久压住。
 *
 * ⚠⚠ **它是软件急停。** 本板没有不经过 CPU 的硬件关断通路
 *   （DBIS §9.2 S7 的"应当"项，本板不满足，已在安全申报里写明）。
 *   ⇒ 载人车辆必须另配独立的物理断路器，不经过本板。
 */
/* HardFault 留下的痕迹。见 stm32f4xx_it.c 的 HardFault_Handler。 */
#define HARDFAULT_MARK_MAGIC  0xDEADFA17UL
extern uint32_t g_hardfault_mark;

void    FocCAN_Estop(void);        /* 立即闩死。任何状态可调 */
void    FocCAN_EstopClear(void);   /* 显式解除，边沿 */
uint8_t FocCAN_EstopActive(void);

uint8_t FocCAN_TestActive(void);
uint8_t FocCAN_BrakeTestActive(void);   /* UART BRAKE 测试保持期间为 1，正式刹车应让位 */
void FocCAN_TestGet(uint16_t *throttle_permille, int16_t *steer_permille,
                    uint8_t *allow_drive, uint8_t *brake, uint8_t *gear);
uint8_t FocCAN_TestTimeout10ms(void);
uint8_t FocCAN_StatusFresh(const FocCAN_t *fc, uint32_t now_ms);
uint16_t FocCAN_GetSpeed_01kmh(const FocCAN_t *fc, uint32_t now_ms);  /* 不新鲜返回 0 */
/*
 * 车速的合理性上限，单位 0.1km/h。
 * 这台车实际跑不到 6km/h(AUTO 上限 1500mm/s = 5.4km/h)，驱动板那侧夹在 9999。
 * 取 600(=60km/h)：远在任何真实车速之上，又远在坏帧之下 —— 它要挡的不是
 * "有点快"，是 0xFFFF 那种"这根本不是一个车速"。
 */
#define FOC_CAN_SPEED_PLAUSIBLE_MAX_01KMH  600U

/* 驱动板的状态/遥测帧多久没到就算不新鲜。0x211 周期 20ms、0x212 周期 100ms,
 * 500ms 对后者是 5 个周期。放在头文件里是因为参数表要引用它 ——
 * 表里的数必须由**真正执行那件事的常量**推导，不能手写第二份。 */
#define FOC_CAN_STATUS_FRESH_MS  500U

/* 序列号多久不动就算对端卡住。0x211 周期 20ms，300ms = 连续 15 帧序列号没变。
 * 比帧到达的 500ms 窗口更紧，因为它判的是更明确的一件事。 */
#define FOC_CAN_PEER_FROZEN_MS  300U

/* 车速反馈算不算数：区分"车停住了"和"线上没有车速"，两者 GetSpeed 都返回 0。
 * AUTO 上电使能(arm)必须问这个，不能只问 StatusFresh —— 状态帧和遥测帧是两帧。 */
uint8_t FocCAN_SpeedFeedbackValid(const FocCAN_t *fc, uint32_t now_ms);

/*
 * 驱动板到底是"没听见"、"卡住了"、还是"听见了但不信我们" —— 三个不同的答案。
 *
 * FocCAN_StatusFresh 只回答"最近有没有帧到"。这不够：一块应用挂死、
 * 而发送邮箱里还压着帧在自动重传的驱动板，从帧到达的角度看完全健康。
 * 驱动板判我们是不是活的，看的是**序列号动没动**，不是有没有帧 ——
 * 这一侧却一直只看后者。
 */
/* 量出来的行驶方向：+1 前进 / -1 后退 / 0 不知道(不新鲜或在停车噪声里)。
 * 0x212 的车速是绝对值，方向在 0x211 的带符号轮速里。 */
int8_t FocCAN_GetSpeedDirection(const FocCAN_t *fc, uint32_t now_ms);

/*
 * 驱动板报告的"它在本地模式"(0x211[7] b6)。1 = 锁着。
 *
 * ⚠ 返回 0 **不等于"没锁"** —— 见上面字节布局里 b6 那一段：
 *   0.4.8~0.4.10 锁着也发 0。⇒ 调用方只许对 1 作出反应。
 *
 * ⚠ 这一位**没有上板验证过**（2026-08-30）：台架上控制板与驱动板当时
 *   没挂在同一条总线上（rx=0）。已验的只有本板的**解码**那一半，
 *   用 `FOC RX211` 灌帧验的。**那两件事不要合并成一句。**
 */
uint8_t FocCAN_PeerLocalLock(const FocCAN_t *fc);

/* 驱动板报告的"刹车驱动已使能"(0x211[0]/0x213[0] b7)，跨两帧取或。
 * ⚠ 返回 0 分不出"没装刹车缸"和"装了但没使能"，**现在只报告不撤防**。
 * 接进判据的两个前置条件写在实现处。 */
uint8_t FocCAN_BrakeDriveEnabled(const FocCAN_t *fc);

uint8_t FocCAN_PeerFrozen(const FocCAN_t *fc, uint32_t now_ms);   /* 帧在来，序列号不动 */
uint8_t FocCAN_PeerBelievesUs(const FocCAN_t *fc);                /* 0x211[7] bit7 */
uint8_t FocCAN_PeerAlive(const FocCAN_t *fc, uint32_t now_ms);    /* 三条都成立才算 */
uint8_t FocCAN_TelemFresh(const FocCAN_t *fc, uint32_t now_ms);
uint16_t FocCAN_GetVbus_mV(const FocCAN_t *fc, uint32_t now_ms);  /* 不新鲜返回 0 */
int16_t  FocCAN_GetIbus_cA(const FocCAN_t *fc, uint32_t now_ms);  /* 厘安(10mA/LSB)，不新鲜返回 0 */

#endif /* FOC_CAN_H */
