#include "foc_can.h"
#include "brake_can.h"
#include "rc_input.h"
#include "train_uart.h"
#include "imu_lsm6dsv.h"
#include "board_inputs.h"
#include "fw_version.h"
#include "fw_image.h"
#include "drive_block.h"
#include "cart_param.h"
#include "cart_persist.h"
#include "servo_can.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern volatile uint16_t g_dbg_gear;   /* 当前解析出的档位键值，见 dla_cart_app.c */
#define g_dbg_gear_pub g_dbg_gear

#define FOC_CAN_TEST_TIMEOUT_MS  300U
#define BRAKE_CAN_TEST_HOLD_MS   3000U
/* ! 这里曾经有 FOC_WHEEL_CIRCUMFERENCE_MM (314*260/100 = 816 mm)，用来把
 *   0x211 里驱动板报的 rpm 换成 km/h。2026-08-25 删除：那是拿别人的测量值
 *   配自己的常量，而那个常量是从标称直径算的、比实测周长大 2.26%。车速现在
 *   由驱动板换算好后经 0x212 [5:6] 送来。详见 foc_can.h 顶部。 */

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

/*
 * Jetson 那根线（UART5）上的 ASCII 行：只有它回的 "MAP,1/0" 和 "SAVE,1/0"。
 *
 * ⚠ 2026-08-30 分线之前这两种行和 CLI 命令挤在同一个消费者里，因为它们
 *   本来就在同一根线上。分开之后它们**不在同一根线上了** —— 硬塞回 CLI
 *   那个消费者就等于在 USART1 上等一句永远不会来的话。
 *
 * ⚠ 它和二进制帧的 poll 共用 UART5 的环，谁先跑谁先吃字节。那是 Jetson
 *   这一路本来就有的安排（两端按内容分流），不是分线引入的。
 */
void FocCAN_PollJetsonLines(void)
{
  char line[48];

  while (TrainUART_PollJetsonLine(line, sizeof(line)) != 0)
  {
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
    /* 其它行忽略：这根线上不跑 CLI。 */
  }
}

/*
 * 上位机 / 调试那根线（USART1）上的 CLI。
 *
 * ⭐ 2026-08-30 起**任何模式下都可以调**。从前 dla_cart_loop.inc 里有一个
 *   "AUTO/FOLLOW 就别调它"的模式判断，因为这个消费者会从同一个环里偷走
 *   Jetson 二进制帧的字节（2026-06-02 的 arm 闪断、刹车点动）。
 *   分线之后这一路上根本没有二进制 ⇒ 那个判断连同它的代价一起没了，
 *   **自动驾驶跑着的时候也能问板子话。**
 */
void FocCAN_TestPollUart(FocCAN_t *fc)
{
  char line[48];
  (void)fc;

  while (TrainUART_PollLine(line, sizeof(line)) != 0)
  {
    if ((strcmp(line, "param list") == 0) || (strcmp(line, "PARAM?") == 0))
    {
      /* 表本身由主循环一行一行发出去(CartParam_ListStep)，不在这里发完 ——
       * 理由见 cart_param.h：一口气灌完串口会让 0x210 停发到超过驱动板的超时。 */
      CartParam_ListStart();
      continue;
    }
    if ((strcmp(line, "selftest run") == 0) || (strcmp(line, "selftest run all") == 0))
    { CartSelftest_Start(); continue; }
    if (strcmp(line, "calib list") == 0)   { CartCalib_ListStart();   continue; }
    if (strcmp(line, "calib status") == 0) { CartCalib_StatusStart(); continue; }
    if (strncmp(line, "calib start ", 12) == 0) { CartCalib_Start(line + 12); continue; }
    if (strcmp(line, "calib confirm") == 0) { CartCalib_Confirm(); continue; }
    if (strcmp(line, "calib abort") == 0)   { CartCalib_Abort();   continue; }
    if (strcmp(line, "estop") == 0)
    {
      /* ⚠ 放在命令分派的**最前面**附近，理由见 foc_can.h：优先级高于一切。
       * ⚠ 它不依赖任何模式、任何状态 —— 包括 AUTO/FOLLOW，
       *   而分线之后 CLI 在那两个模式下也是活的（见 train_uart.c）。 */
      FocCAN_Estop();
      test_ack("=ok estop");
      test_ack("  ! latched. drive request forced to stop+brake. 'clear' to release.");
      test_ack("  ! this is a SOFTWARE e-stop: there is no gate cut-off that bypasses the CPU.");
      continue;
    }
    if (strcmp(line, "clear") == 0)
    {
      /* 边沿：一条命令 = 一次清除（DBIS §9.1 S6）。 */
      if (FocCAN_EstopActive() != 0U) {
        FocCAN_EstopClear();
        test_ack("=ok clear");
        test_ack("  ! e-stop released. Normal arbitration decides from here.");
      } else {
        test_ack("=ok clear");
        test_ack("  ! nothing was latched.");
      }
      continue;
    }
    if (strcmp(line, "fault codes") == 0)
    {
      CartFault_CodesStart();
      continue;
    }
    if (strcmp(line, "fault") == 0)
    {
      CartFault_Start();
      continue;
    }
    if (strncmp(line, "param get ", 10) == 0)
    {
      /*
       * 一行就答完，不用泵化 —— 一行约 190 字节，115200 下约 16ms，
       * 单行阻塞发送远在驱动板 300ms 超时之内。
       * ⚠ `param list` 要泵化是因为它有六十多行，不是因为一行贵。
       */
      CartParam_Get(line + 10);
      continue;
    }
    if (strncmp(line, "param set ", 10) == 0)
    {
      /*
       * ⚠⚠ 这段注释原来写着「整张表 perm=ro，因为没有非易失存储，
       *   无论多少都拒」—— 2026-08-30 的 `b99afbd` 之后就不成立了，而它在这里又多
       *   活了一周。上位机 2026-09-01 的固件调整请求 F-3 就是照着它写的。
       *
       *   现在：表里 10 行 rt 非空，它们报 `perm=rw persist=yes`，写入真的落板载
       *   EEPROM。拒绝不拒绝由 `CartParam_Set()` 逐行判，不是这里一刀切。
       *
       * ⚠ 只取名字仍然是对的，但理由变了：不是"反正都拒所以值不重要"，
       *   而是 `find_by_name()` 按**整串**比名字 —— 带着值的串找不到同名项，
       *   会答 unknown-param，而那是错的答案。所以在这里先把值切掉，
       *   再把值单独传给 `CartParam_Set()`。
       */
      char name[40];
      const char *p = line + 10;
      unsigned i = 0U;
      long v = 0L;
      while ((p[i] != '\0') && (p[i] != ' ') && (i < (sizeof(name) - 1U))) {
        name[i] = p[i]; i++;
      }
      name[i] = '\0';
      if (p[i] != ' ') {
        /* 少了值。⚠ 出声，别静默 —— DBIS §4.4「任何拒收都必须出声」。 */
        CartParam_Set(name, 0);   /* 名字不认识时它会答 unknown-param */
        test_ack("  ! usage: param set <name> <integer>");
        continue;
      }
      v = strtol(&p[i + 1U], NULL, 10);
      CartParam_Set(name, (int32_t)v);
      continue;
    }
    if (strcmp(line, "param save") == 0)
    {
      CartParam_Save();
      continue;
    }
    if (strcmp(line, "param load") == 0)
    {
      CartParam_LoadCmd();
      continue;
    }
    if ((strcmp(line, "tlm") == 0) || (strcmp(line, "TLM?") == 0))
    {
      /*
       * 上位机的 telem_snapshot 发的就是小写 `tlm`，不是 `TLM?` ——
       * 它那边是写死的一个字串。TLM? 只是顺手给人留的同义词，
       * 和 VER?/ver 一样。
       *
       * 表本身由主循环一行一行发(CartTlm_Step)，不在这里发完 ——
       * 理由和 param list 一模一样：一口气灌进串口会让 0x210 停发到
       * 超过驱动板的超时。
       */
      CartTlm_Start();
      continue;
    }
    if (strcmp(line, "WHY?") == 0)
    {
      /*
       * "这块板现在为什么不让车动" —— 一次问清楚，不用人去对七个地方。
       *
       * 起因见 drive_block.h 顶部：从驱动板那一侧看过去，使能开关没合和
       * 控制板挂了是同一个现象。这条命令就是那个区分。
       */
      extern volatile uint16_t g_drive_block;
      char ids[96];
      char buf[160];
      (void)DriveBlock_Describe(g_drive_block, ids, (int)sizeof(ids));
      (void)snprintf(buf, sizeof(buf), "WHY block=0x%04X %s",
                     (unsigned)g_drive_block, ids);
      test_ack(buf);
      continue;
    }
    if (strcmp(line, "id") == 0)
    {
      /*
       * 这块板是什么，一行说完。DBIS §5.1 —— 标准里唯一的 L0 强制项，
       * 而两块板到 2026-09-01 都没有它。
       *
       * ! 缺的从来不是数据，是**一次往返**：uid 在 `ver` 里，固件那几样在
       *   `=tlm build` 里，型号和硬件版本哪儿都没有。上位机为此对每块板
       *   写一套两三次往返的握手，写了两遍。第三块板要写第三遍 ——
       *   而那正是一份标准存在的理由。
       *
       * ! 没有 caps 能力位域，这不是漏了。2026-09-01 两边确认把它从 DBIS
       *   删掉：COMMANDS.json 说的是同一件事而且更细，还带 cmdhash 能证明
       *   它描述的就是这块镜像；位域证明不了自己。两份事实源留一份不能
       *   自证的，比只有一份更糟。
       *
       * ! channels=0 是**如实**，不是"没填"。这块板没有桥、没有电机通道，
       *   它是发命令的那一端。⚠ 上位机不能把 0 读成"不知道几路" ——
       *   那正是这个字段要区分的两件事。
       *
       * ! 版本和提交号用编译期宏，和本板 `=tlm build` 同一个来源。
       *   ⚠ 驱动板那边读的是镜像头（两者会因为 configure 过期而不一致），
       *   本板还没有把版本写进镜像头，所以这里跟着自己的那一条走 ——
       *   一块板里两个地方对同一件事给两个答案，比两块板不一致更难查。
       *
       * ! fw_crc_ok 来自 FwImage_Check()，它每次真的重算一遍 CRC。
       *   开机时算好的缓存答案，正是坏掉之后仍然会答"好"的那个。
       */
      char idbuf[256];
      const uint32_t *uid = (const uint32_t *)UID_BASE;
      const fw_image_state_t st = FwImage_Check();

      (void)snprintf(idbuf, sizeof(idbuf),
                     "=id %u board model=%s hw_rev=%s board=%s "
                     "uid=%08x%08x%08x fw_ver=%u.%u.%u fw_commit=%s "
                     "fw_crc_ok=%u fw_build=%s dbis_ver=%u telem_ver=%u "
                     "param_ver=%u persist_ver=%u channels=0",
                     (unsigned)FW_ID_RECORD_VERSION,
                     FW_MODEL, FW_HW_REV, FW_BOARD_ID,
                     (unsigned)uid[2], (unsigned)uid[1], (unsigned)uid[0],
                     (unsigned)FW_VER_MAJOR, (unsigned)FW_VER_MINOR,
                     (unsigned)FW_VER_PATCH, FW_SCM,
                     (st == FW_IMAGE_OK) ? 1U : 0U,
                     (FW_IS_DEBUG != 0U) ? "debug" : "release",
                     (unsigned)FW_DBIS_VERSION, (unsigned)FW_TELEM_VERSION,
                     (unsigned)CART_PARAM_TABLE_VERSION,
                     (unsigned)CART_PERSIST_VERSION);
      test_ack(idbuf);
      /* §4.4 要求每条命令有确定的结尾行。本板对 calib / clear / estop
       * 已经这么做了，这一条照办 —— 读的一方停在它上面，就不会把一行
       * 截断的回答当成完整的。 */
      test_ack("=ok id");
      continue;
    }
    if ((strcmp(line, "VER?") == 0) || (strcmp(line, "ver") == 0))
    {
      /*
       * 两行：一行给机器，一行给人。
       *
       * 机器那行照驱动板 `=tlm <ver> build` 的形状写，字段名一样 ——
       * 上位机 packages/board-tools 里已经有解析它的机器，不必让它再学一种。
       * 多了一个 board= 字段：上位机今天把 BoardPack 名写死成 'foc_double_c'，
       * 一条链路上认不出对面是哪种板；加字段不需要谁分支处理。
       *
       * stamped=0 crc=0 是**如实**的：本板还没有镜像盖章工具。
       * 报 0 而不是省略，是因为"没有这个信息"和"这个信息是好的"
       * 必须是两个不同的答案。
       */
      char buf[192];
      /* ⚠ 这一行和 `tlm` 发的**必须**是同一行，所以它只有一个合成器。
       * 上位机的 buildOf() 从两条命令里的任意一条读到都该得出同一个答案，
       * 而两份 snprintf 迟早会有一份先长出新字段。 */
      CartTlm_BuildLine(buf, (uint16_t)sizeof(buf));
      test_ack(buf);
      (void)snprintf(buf, sizeof(buf),
                     "DlaBoard_B %u.%u.%u %s %s built %s %s",
                     (unsigned)FW_VER_MAJOR, (unsigned)FW_VER_MINOR, (unsigned)FW_VER_PATCH,
                     FW_SCM, (FW_IS_DEBUG != 0U) ? "DEBUG" : "RELEASE",
                     __DATE__, __TIME__);
      test_ack(buf);

      /*
       * 下面两行是给上位机 DlaStudio 认板用的，形状不是我定的 ——
       * 是照 packages/board-tools/src/identify.ts 里那三条正则写的。
       *
       * !! 没有 uid 这一行，这块板在客户端里不是"一块板"。
       *   parseVer() 对 uid 是硬要求：取不到就抛 IdentifyError，
       *   而 board-state.ts 接住那个错之后走的是
       *   "一块什么都不说的板 = 被擦掉的板，正是要拿去刷的那种"分支 ——
       *   面板画的是问题不是板名，uid 取空串，用户选的型号连存都存不下来
       *   （型号答案按 uid 记在 board-models.json，空 uid 一律拒绝存）。
       *   ⇒ 少这一行的代价不是"少个字段"，是整块板认不出来。
       *
       * ! 字节序照驱动板 cli.c 印 uid 的写法（word[2] word[1] word[0]），
       *   不是我另挑的 —— 两块板的 uid 在服务器记录、客服记录和 fct.py
       *   的归档里必须读起来是同一种东西。
       *
       * ! crc 后面写 unstamped 而不是 bad：他们把这个词单独分了一支
       *   (stamped=false, imageOk=false)，而 bad 的意思是"盖过章、校验没过"。
       *   本板还没有盖章工具，说 bad 是在说一件没发生过的坏事。
       *
       * ! image 这一行**故意不印**。他们的正则要"多少字节 + 在哪个地址"
       *   在同一行里，而本板没有镜像自述头，字节数取不到。
       *   凑一个看起来合理的长度出来，正是这份计划一直在反对的那种答案；
       *   取不到就不印，他们那边留空，那是诚实的方向。等 P5.4 有了盖章
       *   再补，那时候 crc 也就能从 unstamped 变成 ok 了。
       */
      {
        const uint32_t *uid = (const uint32_t *)UID_BASE;

        /* ⭐ 现在这一行来自**镜像自己的头**，不再是编译期写死的 unstamped。
         * FwImage_Check() 每次真的重算一遍 CRC —— 它回答的是"flash 现在还是不是
         * 当初烧进去的那些字节"，而一个开机时算好的缓存答案正是会答错的那个。 */
        {
          char fw[64];
          FwImage_Describe(fw, sizeof(fw));
          (void)snprintf(buf, sizeof(buf), "firmware : %s", fw);
        }
        test_ack(buf);

        /* ⭐ `image :` 那一行现在印得出来了 —— 上位机的正则要"多少字节 +
         * 在哪个地址"在同一行。⚠ 没盖章时**不印**：字节数是 0xFFFFFFFF，
         * 印出来就是一个看起来合理而完全错误的长度。 */
        if (FwImage_Header()->image_bytes != FW_IMAGE_UNSTAMPED) {
          (void)snprintf(buf, sizeof(buf),
                         "image    : %lu bytes at 0x08000000, header at +0x%x",
                         (unsigned long)FwImage_Header()->image_bytes,
                         (unsigned)FW_IMAGE_HEADER_OFFSET);
          test_ack(buf);
        }

        (void)snprintf(buf, sizeof(buf), "uid      : %08x%08x%08x",
                       (unsigned)uid[2], (unsigned)uid[1], (unsigned)uid[0]);
        test_ack(buf);
      }
      continue;
    }
    if (strncmp(line, "FOC RX211 ", 10) == 0)
    {
      /*
       * 把八个字节喂进 0x211 的处理路径，就当它是从总线上来的。
       *
       * 为什么需要它：0x211[7] b6(本地锁)这一位**没有第二块板就验不了**,
       * 而台架第二块是 0.4.10 —— 它锁着也发 0，正是那个歧义本身。
       * 驱动板那侧为同一个理由做了同一件事(CanLink_Inject...)：
       * "这块板处在哪个模式，有没有走进它发出去的那一帧"要能在没有
       * 总线分析仪的情况下被断言。
       *
       * ⚠⚠ **它验的是解码，不是线。** 灌一帧证明本板读得对，
       * **不证明驱动板真的会发** —— 那两件事不要合并成一句。
       *
       * ⚠ 危险等级按 motion 报（命令表里）：它能让本板相信对端是活的、
       * 是信我们的，而那些正是 arm 判据要读的量。
       */
      unsigned b[8] = {0};
      uint8_t d[8];
      int i;
      if (sscanf(line + 10, "%x %x %x %x %x %x %x %x",
                 &b[0], &b[1], &b[2], &b[3], &b[4], &b[5], &b[6], &b[7]) != 8) {
        test_ack("RX211 用法: FOC RX211 <8 个十六进制字节>");
        continue;
      }
      for (i = 0; i < 8; i++) { d[i] = (uint8_t)(b[i] & 0xFFU); }
      FocCAN_OnRxFrame(fc, FOC_CAN_STATUS_ID_FOC2, d);
      {
        char buf[120];
        (void)snprintf(buf, sizeof(buf),
                       "RX211 injected: health=0x%02X valid=%u locked=%u gear=%u",
                       (unsigned)fc->status_health, (unsigned)fc->status_cmd_valid,
                       (unsigned)FocCAN_PeerLocalLock(fc), (unsigned)fc->status_cmd_gear);
        test_ack(buf);
      }
      continue;
    }
    if (strcmp(line, "JET?") == 0)
    {
      /*
       * Jetson 那根线（UART5 / USB2）自己的健康。
       *
       * ⚠ 分线之后**没有人在看那根线了** —— CLI 在 USART1 上，
       *   而 UART5 上跑的是二进制，问不出话。这条命令就是那双眼睛。
       *
       * ⚠ `jrx=0` 和 `jfull>0` 是两个不同的坏消息：
       *     jrx=0    一个字节都没来过 ⇒ 线没接 / 波特率不对 / 中断没开
       *     jfull>0  来了但环满了没接住 ⇒ 主循环某一趟太长
       *   合成一个"链路故障"就再也分不出该去查哪一头。
       */
      char buf[160];
      (void)snprintf(buf, sizeof(buf),
                     "JET uart5 rx=%lu full=%lu ore=%lu | cli usart1 rx=%lu full=%lu ore=%lu",
                     (unsigned long)TrainUART_GetJetsonRxBytes(),
                     (unsigned long)TrainUART_GetJetsonDropFull(),
                     (unsigned long)TrainUART_GetJetsonDropOre(),
                     (unsigned long)TrainUART_GetCliRxBytes(),
                     (unsigned long)TrainUART_GetRxDropFull(),
                     (unsigned long)TrainUART_GetRxDropOre());
      test_ack(buf);
      continue;
    }
    if (strcmp(line, "JET TX") == 0)
    {
      /*
       * 往 UART5 发一行标记。
       *
       * 台架上认口用的：USB2 那个 COM 在这条命令之前**永远是安静的**
       * （UART5 只在 COLLECT/AUTO 下发东西，而模式只能从串口屏进，
       * 台架这块板没接屏）⇒ 没有这条命令，"哪个 COM 是 USB2" 问不出来，
       * 而"没插线"和"插了但不通"看起来完全一样。
       *
       * ⚠ 它不改板子的任何状态，只是往一根诊断口上写一行字。
       */
      TrainUART_SendJetsonText("JETMARK,DlaBoard_B\r\n");
      test_ack("JET TX sent on UART5 (USB2)");
      continue;
    }
    if (strcmp(line, "GEAR?") == 0)
    {
      /*
       * 接线之后第一件事就是这条命令。它把三件事**分开**报：
       *   1. 四路原始电平（按 U7 的脚位排，不是按 MCU 引脚名）；
       *   2. 有几路为低 —— 这一个数字就能说出开关是不是"四位置选择"型：
       *      拨到任一档都应当**恰好一路**为低；始终 0 路 = 按钮型或没接；
       *   3. 解出来的档位、来源三态、两个故障计数。
       *
       * ⚠ 原始电平和解出来的档位必须分开看：只看档位，一台没接 U7 的车
       *   和一台接好了挂在 P 的车长得一模一样。
       *
       * 接线标准（U7 脚 2..5 = P R N D）在 board_inputs.h，那里是权威。
       *
       * ⚠⚠ 括号里的 MCU 引脚名不是装饰：**U7 脚 4/5 是交叉的**（4=PD3、5=PD4，
       *   见 2026-08-01 定版原理图），而**对外规格书 §8.2 那一行写反了**。
       *   ⇒ 接好线把某一脚拉到 GND，看这里哪个括号里的引脚变 1，当场定论。
       *   ⇒ 真要翻案，改的只有 board_inputs.h 里两行宏，
       *     接线标准（脚 2..5 = P R N D）不受影响。
       */
      extern volatile uint8_t  g_dbg_gear_raw;
      extern volatile uint8_t  g_gear_src;
      extern volatile uint32_t g_gear_none_count, g_gear_ambig_count;
      char buf[200];
      uint8_t raw = g_dbg_gear_raw;
      uint8_t n_low = (uint8_t)(((raw >> 0) & 1U) + ((raw >> 1) & 1U) +
                                ((raw >> 2) & 1U) + ((raw >> 3) & 1U));
      (void)snprintf(buf, sizeof(buf),
                     "GEAR U7 pin2/P(PD0)=%u pin3/R(PD1)=%u pin4/N(PD3)=%u pin5/D(PD4)=%u (1=selected) nlow=%u",
                     (unsigned)((raw >> 0) & 1U), (unsigned)((raw >> 1) & 1U),
                     (unsigned)((raw >> 2) & 1U), (unsigned)((raw >> 3) & 1U),
                     (unsigned)n_low);
      test_ack(buf);
      (void)snprintf(buf, sizeof(buf),
                     "GEAR gear=0x%04X (P=0x0008 R=0x0004 N=0x0002 D=0x0001) src=%s none=%lu ambig=%lu",
                     (unsigned)g_dbg_gear_pub,
                     (g_gear_src == GEAR_SRC_SWITCH) ? "switch" : "none",
                     (unsigned long)g_gear_none_count,
                     (unsigned long)g_gear_ambig_count);
      test_ack(buf);
      continue;
    }
    if ((strcmp(line, "FOC?") == 0) || (strcmp(line, "FOC HELP") == 0))
    {
      test_ack("FOC D|R <0-100> [steer=-100..100] | STOP | OFF | RAW <thr0-1000> <steer> <flags> <gear> [timeout10ms]");
      test_ack("BRAKE 1|0 | BRAKE OFF | RC? | IMU? | ST? | VER? | GEAR? | WHY? | JET? | tlm");
      test_ack("param list | param get <name> | param set <name> <v> (refused: read-only) | param save (refused)");
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
      {
        /* 三态链路状态：never/up/lost。"从来没接过"和"接过然后断了"是两个
         * 不同的答案 —— 上面的 upd= 计数能推出来，但要人心算，而这一行
         * 正是判断刹车与否的那个量。 */
        static const char *k_link[] = { "never", "up", "lost" };
        char buf3[100];
        (void)snprintf(buf3, sizeof(buf3),
                       "RClink ch1=%s ch2=%s rc3=%s rc4=%s (stale>%ums)",
                       k_link[(int)RC_LinkState_CH1()],
                       k_link[(int)RC_LinkState_CH2()],
                       k_link[(int)RC_LinkState_RC3()],
                       k_link[(int)RC_LinkState_RC4()],
                       (unsigned)g_p_rc_stale);
        test_ack(buf3);
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
      /* 192, 不是 128：加上 kph10/wiremiss 之后最坏情况是 174 字节，编译器
       * 当场算出来并报了 -Wformat-truncation。snprintf 不会越界，它会**截断**
       * —— 而截掉的恰好是排在最后的那两个新字段，也就是这次改动唯一想看的
       * 东西。一个安静截断的诊断行比没有诊断更糟。 */
      char buf[192];
      uint32_t now = HAL_GetTick();
      (void)snprintf(buf, sizeof(buf),
                     "FOC st fresh=%u health=0x%02X rpm=%d,%d rx=%lu echo=0x%02X valid=%u gear=%u | VBUS=%umV I=%dcA telem=%u tx=%lu | kph10=%u wiremiss=%lu",
                     (unsigned)FocCAN_StatusFresh(fc, now), (unsigned)fc->status_health,
                     (int)fc->foc1_rpm, (int)fc->foc2_rpm, (unsigned long)fc->rx_count,
                     (unsigned)fc->status_cmd_flags, (unsigned)fc->status_cmd_valid,
                     (unsigned)fc->status_cmd_gear,
                     (unsigned)FocCAN_GetVbus_mV(fc, now), (int)FocCAN_GetIbus_cA(fc, now),
                     (unsigned)FocCAN_TelemFresh(fc, now), (unsigned long)fc->tx_count,
                     (unsigned)fc->speed_01kmh, (unsigned long)fc->speed_wire_missing);
      test_ack(buf);
      {
        /* 链路健康单独一行：上面那行的 192 字节已经用掉约 174。
         * spdok 是 AUTO 上电使能真正要问的那个量(见 FocCAN_SpeedFeedbackValid)，
         * 它和 telem= 不是一回事：遥测帧新鲜但字段是空的时候，telem=1 而 spdok=0。
         * txfail/txabort/short 从前都不存在 —— 发不出去和收到短帧都是静默的。 */
        extern volatile uint32_t g_dbg_can_short;
        extern volatile uint32_t g_reset_cause;
        char buf2[224];
        /* rst 是上电时的 RCC->CSR>>24：bit5=IWDG 咬过。看门狗最要紧的不是它会
         * 复位，是**有人知道它复位过** —— 悄悄重启的板子看起来一切正常。 */
        (void)snprintf(buf2, sizeof(buf2),
                       "FOC link alive=%u frozen=%u believes=%u echomis=%lu spdok=%u txfail=%lu txabort=%lu shortrx=%lu rst=0x%02X%s",
                       (unsigned)FocCAN_PeerAlive(fc, now),
                       (unsigned)FocCAN_PeerFrozen(fc, now),
                       (unsigned)FocCAN_PeerBelievesUs(fc),
                       (unsigned long)fc->echo_mismatch_count,
                       (unsigned)FocCAN_SpeedFeedbackValid(fc, now),
                       (unsigned long)fc->tx_fail_count,
                       (unsigned long)fc->tx_abort_count,
                       (unsigned long)g_dbg_can_short,
                       (unsigned)(g_reset_cause & 0xFFU),
                       ((g_reset_cause & (1U << 5)) != 0U) ? "(IWDG)" : "");
        test_ack(buf2);
      }
      {
        /* 从前静默的那几种丢弃，现在都数着。一条反复进出 bus-off 的总线、
         * 一个满了在丢字节的环形缓冲，从外面看都只是"偶尔有一帧没到"。 */
        extern volatile uint32_t g_dbg_can_fifo_ovf;
        extern volatile uint32_t g_dbg_can_busoff;
        extern volatile uint8_t  g_dbg_can_busoff_now;
        /* 160，不是 144：加上 locked= 之后最坏情况编译器会当场算出来并
         * 报 -Wformat-truncation。snprintf 不越界，它**截断** —— 而截掉的
         * 恰好是排在最后的新字段，也就是这次改动唯一想看的东西。 */
        char buf3[160];
        (void)snprintf(buf3, sizeof(buf3),
                       "FOC err fifoovf=%lu busoff=%lu now=%u | uart dropfull=%lu dropore=%lu | dir=%d brakedrv=%u locked=%u steerblk=%lu",
                       (unsigned long)g_dbg_can_fifo_ovf,
                       (unsigned long)g_dbg_can_busoff,
                       (unsigned)g_dbg_can_busoff_now,
                       (unsigned long)TrainUART_GetRxDropFull(),
                       (unsigned long)TrainUART_GetRxDropOre(),
                       (int)FocCAN_GetSpeedDirection(fc, now),
                       (unsigned)FocCAN_BrakeDriveEnabled(fc),
                       (unsigned)FocCAN_PeerLocalLock(fc),
                       (unsigned long)g_servo_refused_count());
        test_ack(buf3);
        {
          /* 最后发出去的那一帧，原样。⚠ 这是"急停真的盖住了指令"唯一
           * 不依赖对端在线的证据。 */
          char buf4[96];
          (void)snprintf(buf4, sizeof(buf4),
                         "FOC tx210 %02X %02X %02X %02X %02X %02X %02X %02X estop=%u",
                         fc->tx_last[0], fc->tx_last[1], fc->tx_last[2], fc->tx_last[3],
                         fc->tx_last[4], fc->tx_last[5], fc->tx_last[6], fc->tx_last[7],
                         (unsigned)FocCAN_EstopActive());
          test_ack(buf4);
        }
      }
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

/* 台架命令是否已经过期(超过 FOC_CAN_TEST_TIMEOUT_MS 没有新的一行)。 */
static uint8_t test_expired(void)
{
  return ((s_test_last_cmd_ms == 0U) ||
          ((HAL_GetTick() - s_test_last_cmd_ms) > FOC_CAN_TEST_TIMEOUT_MS)) ? 1U : 0U;
}

/*
 * 台架命令一旦过期就自行退位，把 0x210 交还给正常仲裁(踏板/遥控/刹车开关/
 * 档位/跟随/自动)。
 *
 * 从前这个闩锁只有敲 "FOC OFF" 才会清：超时不清、切模式不清、PD10 断开也不清。
 * 于是台架上敲一句 "FOC D 20" 之后人走开，板子就一直按 §FocCAN_TestGet 的
 * 过期默认值发帧，而那组默认值恰好是 allow_drive=1、brake=0 —— 刹车被解除、
 * 踏板/P 档/RC3 急停全部被旁路，只剩 PD10 挡着。切到 AUTO/FOLLOW 更糟：
 * 那两个模式不轮询 CLI(见 dla_cart_loop.inc)，连 "FOC OFF" 都递不进来。
 *
 * 不活的源什么都不主张 —— 过期就是不活。
 */
uint8_t FocCAN_TestActive(void)
{
  if ((s_test_active != 0U) && (test_expired() != 0U)) {
    s_test_active = 0U;
    test_set_stop();
    s_test_last_cmd_ms = 0U;
  }
  return s_test_active;
}

uint8_t FocCAN_BrakeTestActive(void)
{
  return s_brake_test_active;
}

void FocCAN_TestGet(uint16_t *throttle_permille, int16_t *steer_permille,
                    uint8_t *allow_drive, uint8_t *brake, uint8_t *gear)
{
  /* 过期取安全方向：不允许驱动、刹车。调用方通常已经先问过
   * FocCAN_TestActive()(它会把过期的闩锁清掉)，这里是第二道保险 ——
   * 万一将来有别的调用方不经过那道门，也不能读出 allow=1/brake=0。 */
  uint8_t expired = test_expired();

  if (throttle_permille != NULL) {
    *throttle_permille = expired ? 0U : s_test_throttle_permille;
  }
  if (steer_permille != NULL) {
    *steer_permille = expired ? 0 : s_test_steer_permille;
  }
  if (allow_drive != NULL) {
    *allow_drive = expired ? 0U : s_test_allow;
  }
  if (brake != NULL) {
    *brake = expired ? 1U : s_test_brake;
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

/* 定义在本文件靠后，这里只要它们的形状。 */
static uint8_t telem_fresh_l(const FocCAN_t *fc, uint32_t now_ms);
static uint8_t telem_fresh_r(const FocCAN_t *fc, uint32_t now_ms);

uint16_t FocCAN_GetSpeed_01kmh(const FocCAN_t *fc, uint32_t now_ms)
{
  /*
   * 车速整个由驱动板给：它换算好了经 0x212 [5:6] 送来，本板不换算、也不再
   * 保存任何轮子参数。轮周长只有一个家，就是直接驱动轮子的那块板的 flash。
   *
   * ! 线上没有就返回 0，没有退路 —— 和 GetVbus_mV / GetIbus_cA 同一个约定，
   *   这块板上"0 = 没数据"是既有的。
   *
   * ! 原来有一条退路(本板磁钢轮速传感器)，删掉的理由不是它多余，是它**从来
   *   不被走到**：任何控制通路上 CAN 一断，AUTO 就 auto_armed=0 ⇒ foc_brake=1
   *   ⇒ 速度环直接输出 0 并清积分，读都不读它。它只到得了屏幕显示。
   *   ⚠ 而它用的那个轮周长是从标称直径算的、偏大 2.26%，错了很久没人发现 ——
   *   **一条从不被走到的退路，是一条没人知道它坏了的退路。**
   */
  uint16_t wire = 0U;
  uint8_t  n = 0U;

  /* ! 判据是"这一路新鲜吗"，不是"这一路的读数非零吗"。
   *   上面那段注释刚说完"把读到 0 当成没有数据"是本板改掉的坏习惯，
   *   从前这里就还留着 `&& fc->speed_01kmh != 0U` —— 一块真的停住的板
   *   会被排除在平均之外，于是双单板拓扑下报的是还在滚的那一边的车速，
   *   而不是两边的平均。 */
  if (telem_fresh_l(fc, now_ms) != 0U) {
    wire = fc->speed_01kmh; n++;
  }
  if (telem_fresh_r(fc, now_ms) != 0U) {
    wire = (uint16_t)((n != 0U) ? ((wire + fc->speed_01kmh_r) / 2U)
                                : fc->speed_01kmh_r);
    n++;
  }
  return wire;
}

/*
 * 车速反馈到底算不算数 —— 和 FocCAN_GetSpeed_01kmh 返回的那个数字分开问。
 *
 * 因为 "0" 有两个意思：车停住了，和线上根本没有车速。上面那个函数只能
 * 返回一个 uint16，两种情况都是 0。AUTO 是一条速度闭环：把"没有数据"
 * 当成"车速为零"，误差每 20 ms 都等于整个目标值，积分器绕到上限，
 * 油门钉在 AUTO_MAX_THROTTLE_PERMILLE 开环加速 —— 而 auto_armed 仍然是 1，
 * 报给 Jetson 的状态仍然是 FOC_VALID。
 *
 * ⚠ 这不是假想。arm 判据看的是 0x211(状态帧)的新鲜度，车速来自 0x212
 *   (遥测帧) —— 是两帧、两个周期(20ms / 100ms)、两个独立的失效原因。
 *   0x211 还在来而 0x212 停了，是一个完全正常的故障形态。
 *   GetSpeed 上面那段注释里"CAN 一断 AUTO 就 auto_armed=0 所以读不到这里"
 *   的推理，正是在这个缺口上不成立。
 *
 * 两种不算数：
 *   1. 没有任何一路遥测是新鲜的 —— 没有数据。
 *   2. 有新鲜遥测，但它说车速 0 而轮子明明在转 —— 对端的固件不填这个字段
 *      (2026-08-25 之前的驱动板就不填)。这条判据和 speed_wire_missing
 *      计数器用的是同一个证据，区别是那个计数器只被 ST? 打印，没有任何
 *      判断读它。
 */
uint8_t FocCAN_SpeedFeedbackValid(const FocCAN_t *fc, uint32_t now_ms)
{
  uint8_t fresh_l;
  uint8_t fresh_r;

  if (fc == NULL) {
    return 0U;
  }
  fresh_l = telem_fresh_l(fc, now_ms);
  fresh_r = telem_fresh_r(fc, now_ms);
  if ((fresh_l == 0U) && (fresh_r == 0U)) {
    return 0U;                       /* 1. 没有数据 */
  }
  {
    uint16_t spd = FocCAN_GetSpeed_01kmh(fc, now_ms);
    /* 2. 有帧但字段是空的。门限 10 rpm 的理由见 FocCAN_OnRxFrame 里
     *    speed_wire_missing 那段：4 rpm 以下的 kph10 本来就舍入到 0。 */
    int16_t r1 = fc->foc1_rpm; if (r1 < 0) r1 = (int16_t)-r1;
    int16_t r2 = fc->foc2_rpm; if (r2 < 0) r2 = (int16_t)-r2;
    if ((spd == 0U) && ((r1 >= 10) || (r2 >= 10))) {
      return 0U;
    }
    /* 3. 有帧，字段也不空，但那个数不可能是一个车速。收下来的每一个字节
     *    都来自另一块板，没有任何地方检查过它的量级。 */
    if (spd > FOC_CAN_SPEED_PLAUSIBLE_MAX_01KMH) {
      return 0U;
    }
  }
  return 1U;
}

/*
 * 帧还在来，但序列号不动 —— 对端的应用停了，而它的发送邮箱还在自动重传。
 * 从"最近有没有帧到"看过去，这样一块板完全健康。
 */
/*
 * 车往哪个方向在动 —— **量出来的**，不是我们命令的那个方向。
 *
 * 0x212[5:6] 的车速是**绝对值**，线上没有符号。但 0x211[2:3]/[4:5] 报的
 * 是带符号的车架机械 rpm，方向就在那里，一直都在。
 *
 * 从前 0x03 状态帧里 SPD_MEAS 的符号取自 foc_gear_proto，也就是**本板此刻
 * 正在下发的方向**。而 AUTONOMOUS_MODE_SPEC §4 写的是"实测车速，+前进 −后退",
 * §8 里 Jetson 把它直接折进里程计。命令倒车而车还在往前滑的那一段，
 * 报出去的是负速度 —— 幅值是量的，符号是编的，而收的一方分不出来。
 *
 * 返回 0 = 不知道(帧不新鲜，或者转速在停车噪声里)。调用方对 0 的处置
 * 要自己决定 —— 停着的时候符号本来也没有意义。
 * 门限沿用本文件里已有的 10 rpm，理由见 speed_wire_missing 那段。
 */
/*
 * 驱动板报告的"刹车驱动已使能" —— 0x211[0] / 0x213[0] 的 b7，2026-08-28 与
 * FOC_Double_C session 约定加入。极性同该字节其余各位：**1 = 好**。
 *
 * 起因：刹车电缸迁到驱动板板载 H 桥之后，那侧的 s_brake_enable 默认关闭
 * 且不存盘，要每次上电敲一次 brake on。**冷启动之后整车没有刹车** ——
 * 而控制板这一侧看不出来：帧的每个字段都正常，回显的 brake 位也正常,
 * 车却只是滑行。在这一位被报上来之前，"刹车已执行"这句话在这条总线上
 * 没有证据。
 *
 * 跨两帧取**或**，语义是"至少有一块驱动板报告刹车驱动已使能"。
 * 理由：问题是"这台车有没有刹车"，拓扑 B 下刹车缸只接在其中一块板上,
 * 另一块会如实报 0，一块有就是有。
 * ⇒ 这条只有在两侧都如实上报时才成立，已与对方确认。
 *
 * ⚠⚠ 返回 0 有**两个**意思，一位分不开：
 *       "这台车根本没装刹车缸"(简单拓扑，0 是永久且正确的)
 *       "装了，但没使能"(这是故障)
 *   所以本函数的结果**现在只报告，不参与任何撤防判断**。
 *   要把它接进判据，两件事都得先成立：
 *     1. 驱动板固件确实在发这一位 —— 旧固件那一位就是 0，和"没使能"同形;
 *     2. 本车确实装了刹车执行器 —— 这是整车级事实，不在驱动板那侧。
 *   在那之前拿它挡车，会让任何还没升级的车、和任何简单拓扑的车都开不了。
 *   三态("从来没人决定过"/"决定了不装"/"决定了装")在对方的控制台和
 *   param list 里，CAN 上这一位只说使能。
 */
uint8_t FocCAN_BrakeDriveEnabled(const FocCAN_t *fc)
{
  uint8_t any = 0U;

  if (fc == NULL) {
    return 0U;
  }
  if ((fc->status_seen != 0U) && ((fc->status_raw_l & FOC_CAN_HEALTH_BRAKE_DRV) != 0U)) {
    any = 1U;
  }
  if ((fc->status_r_seen != 0U) && ((fc->status_raw_r & FOC_CAN_HEALTH_BRAKE_DRV) != 0U)) {
    any = 1U;
  }
  return any;
}

int8_t FocCAN_GetSpeedDirection(const FocCAN_t *fc, uint32_t now_ms)
{
  int32_t sum;

  if ((fc == NULL) || (FocCAN_StatusFresh(fc, now_ms) == 0U)) {
    return 0;
  }
  sum = (int32_t)fc->foc1_rpm + (int32_t)fc->foc2_rpm;
  if ((sum >= -10) && (sum <= 10)) {
    return 0;
  }
  return (sum > 0) ? (int8_t)1 : (int8_t)-1;
}

uint8_t FocCAN_PeerFrozen(const FocCAN_t *fc, uint32_t now_ms)
{
  if ((fc == NULL) || (fc->status_seen == 0U)) {
    return 0U;   /* 从来没说过话的不叫"卡住"，那是 StatusFresh 的事 */
  }
  if (FocCAN_StatusFresh(fc, now_ms) == 0U) {
    return 0U;   /* 帧都不来了，那是"没听见"，不是"卡住" */
  }
  return ((now_ms - fc->status_seq_move_ms) > FOC_CAN_PEER_FROZEN_MS) ? 1U : 0U;
}

uint8_t FocCAN_PeerBelievesUs(const FocCAN_t *fc)
{
  if ((fc == NULL) || (fc->status_seen == 0U)) {
    return 0U;
  }
  return fc->status_cmd_valid;
}

uint8_t FocCAN_PeerLocalLock(const FocCAN_t *fc)
{
  /* ⚠ 故意没有"没收到过帧就当没锁"这种兜底：没收到帧这件事由
   * FocCAN_StatusFresh / PeerAlive 回答，而把两个问题塞进一个返回值里，
   * 正是这块板已经付过账的那个形状（0 既是"停住了"也是"没数据"）。 */
  return fc->status_local_lock;
}

/*
 * 三条都要：帧在来、它自己在跑、而且它还认我们这一路。
 * 少任何一条都不该继续把车交给它。
 */
uint8_t FocCAN_PeerAlive(const FocCAN_t *fc, uint32_t now_ms)
{
  return (FocCAN_StatusFresh(fc, now_ms) &&
          (FocCAN_PeerFrozen(fc, now_ms) == 0U) &&
          FocCAN_PeerBelievesUs(fc)) ? 1U : 0U;
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
    /*
     * 健康位合并，判据和下面轮速槽的**同一条**：见没见过右板(0x213)。
     *
     * ! 从前这里恒用掩码 0xD5(bit 0,2,4,6,7)，把 bit 1/3/5 留给右板的 0x213。
     *   注释还写着"双电机板会在同帧填满两侧，行为不变"——**最后那句是假的**：
     *   双电机板确实填满了两侧，而这个掩码把它填的那一半扔掉了。
     *   ⇒ 拓扑 A(一块双电机板)下，foc2_run / foc2_edge_ok / foc2_nfault_ok
     *     三位一直在被丢弃。
     *
     * ! 2026-08-28 台架对出来的：驱动板发 health=0xf0，本板解出 0xD0，
     *   差的正是 bit5(foc2_nfault_ok)。**不是读代码找到的，是两个独立
     *   观测点对不上找到的。**
     *
     * ! 而正确的判据在同一个函数里、往下 8 行就有 —— 轮速槽写的是
     *   `if (status_r_seen == 0) foc2_rpm = ...`。健康位没拿到同一个待遇。
     *   又是"规则写在一处，隔壁那处不知道"，这次隔壁只隔 8 行。
     */
    if (fc->status_r_seen == 0U) {
      fc->status_health = data[0];                 /* 拓扑 A：这一帧就是全部 */
    } else {
      fc->status_health = (uint8_t)((fc->status_health & 0x2AU) | (data[0] & 0xD5U));
    }
    fc->status_raw_l = data[0];   /* 原始字节留一份，见 status_raw_r 处的说明 */
    /*
     * 序列号动没动，是"这块板还在跑"和"有帧在到达"之间唯一的区别。
     * 驱动板判我们是不是活的用的就是这个口径；这一侧从前只把它存下来
     * 从来不读。
     */
    if ((fc->status_seen == 0U) || (data[1] != fc->status_seq_prev)) {
      fc->status_seq_prev = data[1];
      fc->status_seq_move_ms = HAL_GetTick();
    }
    fc->status_seq = data[1];
    fc->foc1_rpm = unpack_s16_le(&data[2]);
    /*
     * [4:5] 是 foc2 轮速。只有在没见过右轮板(0x213)时才采信 —— 拓扑 B 的
     * 单路左轮板不填这个槽。
     *
     * ! 这里原来写的是 "收到 0 就不更新"，用值本身去猜拓扑。在拓扑 A
     *   (一块双电机板)下 0 是一个真实读数：那个轮子停了。于是轮子一停，
     *   foc2_rpm 就永远卡在最后一个非零值上 —— 2026-08-25 实测：车停了
     *   十一分钟，控制板仍然认为右轮在以 -16 rpm 倒转。
     *
     * ! 判据换成"帧从哪来"：右轮板自己的 0x213 就是它存在的证据。
     *   这和 FocCAN_GetSpeed_01kmh 里那个"求和还是平均"的分支是同一个
     *   病根 —— 控制板分不出"没数据的 0"和"停住的 0"，于是两处都把 0
     *   当成了没数据。两处都已改成用证据判。
     */
    if (fc->status_r_seen == 0U) {
      fc->foc2_rpm = unpack_s16_le(&data[4]);
    }
    fc->status_cmd_flags = data[6];
    fc->status_cmd_valid = ((data[7] & 0x80U) != 0U) ? 1U : 0U;
    fc->status_cmd_gear = data[7] & 0x0FU;
    /* 本地锁。⚠ 只有 1 是可信的 —— 0 同时符合"没锁"和"固件比 0.4.11 老"。
     * 理由写在 foc_can.h 字节布局的 b6 那一段。 */
    fc->status_local_lock = ((data[7] & 0x40U) != 0U) ? 1U : 0U;
    /*
     * 回环的两条证据，现在真的被读。
     *
     * [7] bit7 = 驱动板认为我们这一路还活着。它变 0 的意思是
     * "我听见你了，但我已经不信你了" —— 和"我根本没听见你"是两回事,
     * 要查的线也不一样。
     *
     * [6] = 它最后一次解出来的整个 flags 字节。和我们刚发的那个比对,
     * 就知道它在按什么命令动作。只比 allow/brake 两位：档位带那六位在
     * 我们改档位带的那一帧和它回显之间必然有一拍差，比了会一直报错;
     * 而 allow/brake 是安全语义，那两位对不上才是要紧的事。
     */
    if (fc->status_cmd_valid == 0U) {
      fc->peer_disbelief_count++;
    }
    if (((fc->status_cmd_flags ^ fc->flags) &
         (FOC_CAN_FLAG_ALLOW_DRIVE | FOC_CAN_FLAG_BRAKE)) != 0U) {
      fc->echo_mismatch_count++;
    }
    memcpy(fc->last_status, data, 8U);
    fc->status_seen = 1U;
    return;
  }

  /* 0x213：单路右轮板状态，只取 foc2 槽 [4:5] 与 foc2 健康位。 */
  if (std_id == FOC_CAN_STATUS_ID_R) {
    fc->rx_count++;
    fc->last_status_ms = HAL_GetTick();
    fc->status_seen = 1U;
    /*
     * 拓扑判定：见过 0x213 ⇒ 现场是拓扑 B(两块单电机板)，
     * 此后 0x211 的 foc2 槽不再采信。
     *
     * ! 这是一条**一路闩锁**，全树没有任何地方清它 —— 只有断电才复位。
     *   从前它由**一帧**就置上：一帧走失的 0x213 就足以永久改变本板对
     *   拓扑的判断，而在拓扑 A(一块双电机板)下，那意味着 foc2_rpm 从此
     *   冻在那帧带来的值上，再也不更新。而 foc2_rpm 参与车速反馈的
     *   有效性判据。
     *
     * ! 闩锁本身是对的 —— 拓扑是一个物理事实，运行中不会变，
     *   而"右板死了"就自动改回去更糟：那会在故障时**静默地换一套解释**。
     *   要改的是**下结论所依据的证据量**。
     *
     * ! 真的右板每 20ms 发一帧，攒够 3 帧只要 60ms，代价为零；
     *   而一次性的走失帧攒不够。
     *   ⚠ 这挡不住一块**持续**误发 0x213 的板子 —— 那种情况要靠
     *     status_r_seen 被报出来让人看见(ST? / 参数表 id=88)，
     *     挡不住的东西至少要看得见。
     */
    if (fc->status_r_seen == 0U) {
      if (fc->status_r_evidence < 255U) {
        fc->status_r_evidence++;
      }
      if (fc->status_r_evidence >= 3U) {
        fc->status_r_seen = 1U;
      }
    }
    fc->status_health = (uint8_t)((fc->status_health & 0x15U) | (data[0] & 0xEAU));
    /*
     * ⚠ 两条合并掩码里 b6 和 b7 是**重叠**的：
     *     0x211 用 0xD5 = bit 0,2,4,6,7
     *     0x213 用 0xEA = bit 1,3,5,6,7
     *   其余位各管半边(左板 0/2/4，右板 1/3/5)，只有这两位两块板都写 ——
     *   于是在拓扑 B 下它们是**最后到的那一帧说了算**，以 20ms 的节奏来回翻。
     *
     *   b0..b5 那样分半是对的，因为它们本来就是每侧各一份。
     *   b6(cmd_recent)、b7(刹车驱动使能，2026-08-28 加) 不是每侧一份的量,
     *   它们是关于**整车**的判断，需要跨两帧合起来看，而不是覆盖。
     *
     *   所以两帧的原始字节各留一份，合并交给下面的访问器 ——
     *   合并规则写在它需要被读到的地方，而不是藏在一个掩码常量里。
     */
    fc->status_raw_r = data[0];
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
    fc->speed_01kmh = (uint16_t)((uint16_t)data[5] | ((uint16_t)data[6] << 8));
    /* 零车速配着非零转速 = 驱动板是旧固件([5:6] 还是预留的 0)。
     * ! 车真的停着时车速也是零，所以单看零判不出来；判据是"零，而状态帧
     *   同时说轮子在转" —— 这两件事只有旧固件才会同时成立。
     *   数在这里而不是在取值函数里：帧到达的地方本来就是可写的，而且
     *   每帧数一次比每次查询数一次更接近"发生了多少次"。 */
    /* 门限 10 rpm 而不是 "非零"：车速是两轮平均再乘周长，约 0.48 (0.1km/h)
     * 每 rpm，所以单轮 4 rpm 以下的 kph10 本来就舍入到 0 —— 那不是旧固件，
     * 那是快停住了。10 rpm 远在舍入边界之上，又远在任何真实车速之下。 */
    {
      int16_t r1 = fc->foc1_rpm; if (r1 < 0) r1 = (int16_t)-r1;
      int16_t r2 = fc->foc2_rpm; if (r2 < 0) r2 = (int16_t)-r2;
      if ((fc->speed_01kmh == 0U) && ((r1 >= 10) || (r2 >= 10))) {
        fc->speed_wire_missing++;
      }
    }
    fc->telem_seen = 1U;
    fc->last_telem_ms = HAL_GetTick();
    return;
  }

  if (std_id == FOC_CAN_TELEM_ID_R) {
    fc->vbus_mV_r = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    fc->ibus_cA_r = unpack_s16_le(&data[2]);
    fc->telem_seq_r = data[4];
    fc->speed_01kmh_r = (uint16_t)((uint16_t)data[5] | ((uint16_t)data[6] << 8));
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

static volatile uint8_t s_estop_latched;

void FocCAN_Estop(void)      { s_estop_latched = 1U; }
void FocCAN_EstopClear(void) { s_estop_latched = 0U; }
uint8_t FocCAN_EstopActive(void) { return s_estop_latched; }

HAL_StatusTypeDef FocCAN_SendDriveRequest(FocCAN_t *fc,
                                          uint16_t throttle_permille,
                                          int16_t steer_permille,
                                          uint8_t allow_drive,
                                          uint8_t brake,
                                          uint8_t gear,
                                          uint8_t timeout_10ms,
                                          uint8_t band_percent)
{
  /*
   * ⭐⭐ 急停优先于一切 —— 在这里，不在调用点。
   *
   * 参数照收，然后**当场覆盖**：油门 0、不允许驱动、刹车 1。
   * ⚠ 覆盖而不是提前 return：这一帧仍然要发出去，而且必须是**安全值**的那一帧。
   *   不发 = 驱动板要等它自己的 300ms 超时才停，期间它按最后一条指令动作。
   *   **发一帧"停"比不发快 300 毫秒。**
   */
  if (s_estop_latched != 0U) {
    throttle_permille = 0U;
    steer_permille = 0;
    allow_drive = 0U;
    brake = 1U;
  }

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
  /* 档位 -> byte4 bit2-7，2% 一档。0 = 不声明(驱动板沿用上一次的)。
   * ! 向上取整而不是四舍五入：这是一个**上限**，宁可让它比屏上写的高
   *   一个最小步长，也不要让车比司机以为的还慢 —— 35% -> 18 档 = 36%。 */
  if (band_percent > 0U) {
    uint32_t steps = ((uint32_t)band_percent + FOC_CAN_BAND_STEP_PCT - 1U)
                     / FOC_CAN_BAND_STEP_PCT;
    if (steps > FOC_CAN_BAND_MAX_STEPS) {
      steps = FOC_CAN_BAND_MAX_STEPS;
    }
    flags |= (uint8_t)(steps << FOC_CAN_BAND_SHIFT);
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

  /* 记下真正发出去的这八个字节 —— 见 foc_can.h tx_last 的注释。 */
  memcpy(fc->tx_last, tx_data, 8U);

  fc->throttle_permille = throttle_permille;
  fc->steer_permille = steer_permille;
  fc->flags = flags;
  fc->gear = gear;
  fc->timeout_10ms = timeout_10ms;

  /*
   * 邮箱满了就把里面的作废，让最新的这一帧进去。
   *
   * 排队等着发的那几帧携带的是更早的司机意图；这一帧携带的是此刻的。
   * 对一条 20ms 的周期性控制帧来说，队列里的旧帧没有任何价值 ——
   * 而如果不腾地方，AddTxMessage 会失败，失败的可能正好是刹车那一帧。
   */
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
    (void)HAL_CAN_AbortTxRequest(&hcan1,
                                 CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    fc->tx_abort_count++;
  }

  if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox) != HAL_OK) {
    /* ! tx_count 从前在这一行之前自增，数的是"尝试过几次"而不是"发出去几帧"。
     *   一块发不出任何东西的板子，它的 tx_count 照样在涨。 */
    fc->tx_fail_count++;
    return HAL_ERROR;
  }
  fc->tx_count++;
  return HAL_OK;
}
