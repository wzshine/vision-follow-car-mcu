#ifndef FW_VERSION_H
#define FW_VERSION_H

/*
 * 固件标识。
 *
 * 在这之前，整棵树里没有任何版本信息 —— 没有版本号、没有 commit、
 * 没有构建时间，CAN 上不报、0x03 状态帧的 [16:19] 是写死的 0、
 * 训练帧的 [18:19] 也是写死的 0，任何一条 CLI 命令都问不出来。
 * **面对一台正在跑的车，没有办法知道它刷的是哪一版固件。**
 *
 * 这件事在这块板上尤其要紧：0x210[4] 的档位带字段是 2026-08-25 才加的，
 * foc_can.h 顶上自己写着"两块板要一起改才能用上新字段"。
 * 驱动板那侧为此发明了 speed_wire_missing —— 一个靠"对面报的车速是不是
 * 一直为 0"来**推断对端固件年代**的启发式。那正是一个版本字段直接回答的问题。
 *
 * 三层，越往下越可能缺，但每一层都诚实：
 *   1. MAJOR/MINOR/PATCH —— 手工维护，改协议或行为时手动 bump。
 *   2. __DATE__/__TIME__ —— 编译器内建，零构建管道，永远有。
 *      ⚠ 它是**这个翻译单元被编译的时刻**，不是镜像链接的时刻。增量构建下
 *        可能偏旧。实际上它和 FW_SCM 是绑在一起动的：hash 一变，build_info.h
 *        重写，包含它的这个文件重编，时间戳跟着更新。只有工作区脏(scm 带 +)
 *        且改动不在这条包含链上时它才会停住 —— 而那时 `+` 已经在说
 *        "这不是一个可复现的标识"了。
 *   3. FW_SCM —— git commit + 脏标记，由 scripts/gen_build_info.sh 生成。
 *      取不到就是 "unknown"。**一个陈旧的 hash 比 "unknown" 糟得多，
 *      因为它看起来是个答案。**
 */

#define FW_VER_MAJOR   0U
#define FW_VER_MINOR   1U
#define FW_VER_PATCH   1U

/* build_info.h 是生成的、不入库的。缺了也要能编译 —— 构建不该依赖一个
 * 可选的诊断信息。 */
#if defined(__has_include)
#  if __has_include("build_info.h")
#    include "build_info.h"
#  endif
#endif
#ifndef FW_SCM
#  define FW_SCM "unknown"
#endif

/*
 * 命令表指纹。cmd_hash.h 也是生成的、不入库的，缺了要能编译 ——
 * 同 build_info.h 的理由：构建不该依赖一个可选的诊断信息。
 *
 * ⚠ 缺失时**不报这个字段**，而不是报 0。上位机对这两种情况的处理是
 * 一样的（整张表不用），但"没说"和"说了个 0"在人读日志时不是一回事，
 * 而 0 还恰好是一个合法的 CRC32。
 */
#if defined(__has_include)
#  if __has_include("cmd_hash.h")
#    include "cmd_hash.h"
#  endif
#endif

/* ! 这个板子今天没有镜像盖章，也没有镜像 CRC。驱动板有(tools/stamp_image.py
 *   往镜像头里写长度和 CRC-32)，本板还没有。报 stamped=0 crc=0 是**如实**，
 *   不是占位 —— "读不出来"和"是好的"必须是两个不同的答案。 */
/* ⚠ 2026-08-30 作废：盖章与 CRC 现在是**镜像自己头里的事实**
 * （见 fw_image.h / tools/stamp_image.py），不再是编译期写死的 0。
 * 留着这两个宏会让人以为它们还是答案。 */

#ifdef DEBUG
#  define FW_IS_DEBUG  1U
#else
#  define FW_IS_DEBUG  0U
#endif

/* 板型标识。上位机今天把 BoardPack 名写死成 'foc_double_c'，parseVer 里
 * 也没有板型字段 —— 它没法从一条链路上认出对面是哪种板。多给一个字段，
 * 是为了那一天到来时它有得可认。 */
#define FW_BOARD_ID    "dlaboard_b"

/* 本板遥测记录的版本。只有**含义**变了才动它；加字段不动。
 * 这条规则抄自驱动板 telem.c 的注释，理由相同：版本号一动，
 * 上位机就要分支处理，而加一个字段不需要任何人分支。 */
#define FW_TELEM_VERSION  1U

/* 产品型号。
 *
 * ⚠ 和 FW_BOARD_ID 不是同一个字符串，也不是它的好看写法：这一个是
 *   **服务器上绑板记录里写着的那个**。板卡包 board.yaml 的 model: 那一行
 *   旁边就写着「改这个字符串等于改一条已经发出去的记录的口径」。
 *   出处是 EDA 的产品规格书「产品型号 | DlaBoard_B」那一行。 */
#define FW_MODEL       "DlaBoard_B"

/* 硬件版本。
 *
 * 字母来自工程名（DlaBoard_B），那也是这块板的版本一直被记录的地方。
 * ⚠ 用户 2026-09-01 说完整的硬件版本规整他单独做；那件事落地之后，
 *   这个字符串跟着它走，不是由固件定的。
 * ⚠ 只能是编译期给的：板子测不出来自己焊在哪一版上。 */
#define FW_HW_REV      "B"

/* 两个不同的版本号，别合并。
 *
 * FW_DBIS_VERSION      本固件是照哪一版接口标准写的。
 * FW_ID_RECORD_VERSION `=id` 这一行的形状，按本板一贯的规矩走 ——
 *                      加字段不动，改一个已有字段的**含义**才动。 */
#define FW_DBIS_VERSION       1U
#define FW_ID_RECORD_VERSION  1U

#endif
