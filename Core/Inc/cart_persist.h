#ifndef CART_PERSIST_H
#define CART_PERSIST_H

#include <stdint.h>

/*
 * ==================== 参数持久化（板载 EEPROM） ====================
 *
 * 形状照上位机的接入标准 **DBIS §8**（`DlaStudio/docs/spec/DBIS.md`）写的，
 * 那一节每一条都是"强制"，而且每一条都有来由。逐条对照：
 *
 *   记录格式  magic(4) | version(2) | size(2) | seq(2) | reserved(2) | payload | crc32(4)
 *
 *   ⭐ 双区轮换      —— 两个区交替写；两区都读，**校验通过且 seq 大的赢**。
 *                      杜绝掉电写坏：任何时刻至少有一个完整的旧记录还在。
 *   ⭐ CRC32 覆盖 version 和 size，不只覆盖 payload。
 *   ⭐ 只允许追加字段 —— crc 永远在末尾，**按记录自己的 `size-4` 校验**
 *                      ⇒ 旧记录按自己的条款通过，超出部分填默认。**不做迁移表。**
 *   ⭐ 字段级范围校验与 CRC 并列 —— CRC 只保证字节没错，**不保证值能用**。
 *   ⭐ save 后立刻按下次开机的方式回读自验（走同一个解析器 + CRC）。
 *   ⭐ 写入前置条件必须出声 —— 不满足时按 §4.4 拒绝并说明理由。
 *   ⭐ 参数活过重新烧录 —— 这一条本板天然满足：参数在**片外 EEPROM**，
 *                      而烧录只动 MCU 的 Flash，碰不到它。
 *
 * ---- payload 为什么是 (id, value) 对，而不是一个 struct ----
 *
 * 一个 struct 要求"字段顺序永不变"，而那正是需要迁移表的原因。
 * 用 (u16 id, i32 value) 对之后：
 *   - 加一个参数 = 多一对，`size` 变大，**旧记录仍按自己的 size-4 校验通过**；
 *   - 删/改一个参数 = 那个 id 从此不再出现，读到就忽略；
 *   - 顺序无关，缺失的取默认。
 * ⇒ "只允许追加"这条规矩由格式本身保证，不靠人记得。
 *
 * ---- ⚠⚠ 一条不许破的规矩 ----
 *
 * **一个参数报 `perm=rw`，当且仅当真正执行它的那段代码读的是运行时值。**
 *
 * 反过来（表里说可写、而执行方仍然读编译期常量）会造出这个仓一直在记的那种
 * 缺陷：`param set` 回显成功、`param get` 读回新值、**而机器的行为一点没变**。
 * 上位机那边对应 DBIS §4.4 的判据 ——「命令回显成功 ≠ 命令生效」。
 * ⇒ 宁可少几个可写参数，也不要一个说了不算的 `rw`。
 */

/* M24C64：64 Kbit = 8 KB，页 32 字节。A0/A1/A2 接地 ⇒ 器件地址 0xA0。 */
#define CART_PERSIST_MAGIC      0x424C4144U   /* "DALB" 小端 */
#define CART_PERSIST_VERSION    1U

/* 两个区，隔开 4 KB —— 远大于一条记录，也远大于一页。 */
#define CART_PERSIST_REGION_A   0x0000U
#define CART_PERSIST_REGION_B   0x1000U

/* 一条记录最多带多少对。⚠ 只增不减：减小它会让已经写进 EEPROM 的记录读不回来。 */
#define CART_PERSIST_MAX_PAIRS  64U

typedef enum {
  CART_PERSIST_OK = 0,
  CART_PERSIST_NO_DEVICE,     /* I2C 上没有应答 —— 板子没焊 EEPROM，或 I2C 坏了 */
  CART_PERSIST_EMPTY,         /* 器件在，但两个区都没有有效记录（出厂状态） */
  CART_PERSIST_BAD_CRC,       /* 器件在，有记录，但都校验不过 */
  CART_PERSIST_IO_ERROR,      /* 读写过程中 I2C 出错 */
  CART_PERSIST_VERIFY_FAILED  /* ⚠ 写完回读自验没过 —— **不要当成保存成功** */
} CartPersistStatus_t;

/*
 * 开机调一次。⚠ 它**不**改任何参数值 —— 只把存储层准备好并报告状态。
 * 真正的装载是 CartPersist_Load()，由参数层调用，因为范围校验要用参数表。
 */
/*
 * 自检用的 scratch：写一个图案再读回来，证明 I2C + 器件真的活着。
 *
 * ⚠⚠ 地址**必须在两个记录区之外**。独立构建那份 selftest.c 用的是 0x0010,
 *   那是**区 A 的记录头里面** —— 原样搬进正式固件会把存好的参数写坏，
 *   而下一次开机只会报「CRC 不过」，没有人会想到是自检干的。
 * ⇒ 0x1F00：远离区 A(0x0000) 和区 B(0x1000)，仍在 8KB 之内。
 */
#define CART_PERSIST_SCRATCH  0x1F00U
int CartPersist_SelfTest(void);   /* 1 = 写读一致 */

void CartPersist_Init(void);

/* EEPROM 在不在。⚠ 三态的第一态：**"没有存储"和"存储是空的"是两个答案**。 */
uint8_t CartPersist_DevicePresent(void);

/* 上一次 Init/Load/Save 的状态，给 `tlm` 和 `fault` 报。 */
CartPersistStatus_t CartPersist_LastStatus(void);

/* 当前记录的 seq（上位机 `=param-save record=` 报的就是它）。0 = 还没有记录。 */
uint16_t CartPersist_Seq(void);

/*
 * 从 EEPROM 装载：两区都读，CRC 通过且 seq 大的赢，逐个 (id,value) 交给
 * 参数层做**字段级范围校验**后写进运行时值。
 * 返回读到并接受了多少对；状态见 CartPersist_LastStatus()。
 */
uint16_t CartPersist_Load(void);

/*
 * 保存：写另一个区，seq+1，**然后按下次开机的方式回读自验**。
 * ⚠ 自验没过返回 CART_PERSIST_VERIFY_FAILED —— 调用方必须把它当成失败，
 *   因为那正是"写进去了但读不回来"的情形，而它看起来和成功一模一样。
 */
CartPersistStatus_t CartPersist_Save(void);

#endif
