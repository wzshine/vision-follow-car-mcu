#ifndef DRIVE_BLOCK_H
#define DRIVE_BLOCK_H

#include <stdint.h>

/*
 * "为什么这块板现在不允许驱动" —— 每一条理由一个位。
 *
 * 起因是驱动板那侧写来的一句话(BRIEF_协同工作启动_2026-08-26.md §2.2①)：
 *
 *   > 现在从驱动板那一侧看过去，"使能开关没合"和"控制板挂了"是同一个现象。
 *
 * 两者都表现为 allow_drive=0。可是这两种要查的线完全不一样：一个是去合
 * 那个开关，一个是去查控制板为什么死了。互锁本身不该绕 —— 要做的是
 * **让它能说出自己为什么不允许**。
 *
 * 同一条道理在这块板内部也成立：不允许驱动的理由今天散在七处各算各的
 * (PD10、档位、刹车开关、遥控、CLI 闩锁、若干超时)，从外面看全都长得一样。
 *
 * ⚠ 位只增不重用。上位机会把人话、图、警告挂在位号上，重编号会静默地
 *   换掉一整屏的解释 —— 这条规则抄自驱动板的参数表和故障码。
 */
#define DRIVE_BLOCK_VEH_ENABLE    (1U << 0)  /* PD10 整车使能开关未合 */
#define DRIVE_BLOCK_GEAR          (1U << 1)  /* 档位不在 D/R(AUTO 下要求 D) */
#define DRIVE_BLOCK_BRAKE_SW      (1U << 2)  /* 刹车踏板开关 PE2/PE3 按下 */
#define DRIVE_BLOCK_RC_BRAKE      (1U << 3)  /* RC3 远程急停在刹车位 */
#define DRIVE_BLOCK_RC_LOST       (1U << 4)  /* REMOTE/COLLECT 下遥控链路曾活过、现已断 */
#define DRIVE_BLOCK_FOC_STALE     (1U << 5)  /* 驱动板状态帧 0x211 不新鲜 */
#define DRIVE_BLOCK_SPEED_INVALID (1U << 6)  /* 车速反馈不算数(没数据/字段空/量级不合理) */
#define DRIVE_BLOCK_HOST_STALE    (1U << 7)  /* 上位机命令帧 0x10/0x11 不新鲜 */
#define DRIVE_BLOCK_HOST_DISABLE  (1U << 8)  /* 上位机没有置 AUTO_ENABLE */
#define DRIVE_BLOCK_CLI_TEST      (1U << 9)  /* 台架 CLI 命令正在接管 0x210 */
/* 下面两条是简报 §2.1 要的那个区分：**"你根本没听见我"和"我听见了但已经
 * 不信了"要查的线不一样**。从前两者都表现为帧还在来、判活照样为真。 */
#define DRIVE_BLOCK_FOC_FROZEN    (1U << 10) /* 帧在来但序列号不动 —— 驱动板应用卡住了 */
#define DRIVE_BLOCK_FOC_DISBELIEF (1U << 11) /* 驱动板回说它已不认我们这一路(0x211[7] bit7=0) */
/* 驱动板说它在本地模式(0x211[7] bit6=1) —— 它照收照回，但不按总线动作。
 * ⚠ **只在读到 1 时置位。** 0 同时符合"没锁"和"固件比 0.4.11 老"
 * (0.4.8~0.4.10 锁着也发 0)，所以 0 什么都不主张。见 foc_can.h b6 那一段。
 * ⚠ 和上面那些不一样，这一条在**任何模式**下都成立：被锁的驱动板
 * 不会因为司机在踩踏板就听话。 */
#define DRIVE_BLOCK_FOC_LOCKED    (1U << 12)

/* 急停闩锁（`estop` 命令）。⚠ 它和上面那些不一样：**闩住的**，
 * 条件消失也不会自己清，要一条 `clear`。 */
#define DRIVE_BLOCK_ESTOP         (1U << 13)

/* 当前的阻止理由位图。0 = 没有任何一条在挡着。 */
extern volatile uint16_t g_drive_block;

/* 把位图写成一行短标识符，人话归上位机(BoardPack)。返回写入的字符数。 */
int DriveBlock_Describe(uint16_t bits, char *out, int out_size);

/*
 * 一位一条：短标识符 + 给上位机 `fault codes` 的三列。
 *
 * ⚠ 表只有一张 —— `WHY?` 的短标识符和 `fault codes` 的人话读的是同一行。
 *   两张表迟早会有一张先加了位。
 */
typedef struct {
  uint16_t    bit;
  const char *id;       /* WHY? 里的短标识符 */
  const char *action;   /* cut / limit / note —— 上位机 fault.ts 只认这三个词 */
  uint8_t     latched;  /* 0 = 活值，条件消失就清 */
  const char *meaning;  /* 一句话：这是什么 */
  const char *advice;   /* 一句话：下一步查哪儿 */
} DriveBlockDesc_t;

extern const DriveBlockDesc_t g_drive_block_desc[];
extern const unsigned g_drive_block_desc_count;
#define DRIVE_BLOCK_DESC_COUNT  g_drive_block_desc_count

#endif
