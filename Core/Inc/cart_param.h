#ifndef CART_PARAM_H
#define CART_PARAM_H

#include <stdint.h>

/*
 * 控制板的自描述参数表。
 *
 * 形状照抄驱动板 FOC_Double_C 的 `param list`，字段名一样 —— 上位机
 * packages/board-tools 里已经有解析它的机器，不必让它再学一种格式。
 * 抄的不是格式，是驱动板付了代价才学到的那几条规则：
 *
 *   1. **元数据和读取必须是同一行。** 表里写一个范围、代码执行另一个范围,
 *      上位机就会按表画滑块、按代码夹值，而**框里的数不是机器里的数**。
 *      所以 max/default 一律由真正执行那件事的常量推导 —— 表和执行方
 *      include 同一个 cart_params.h，写不出第二份。
 *   2. **默认值必须是板子真的会开机成为的值**，不是"看起来合理的值"。
 *   3. **"读不出来"和"是安全的"必须是两个不同的答案。**
 *   4. **id 只增不重用。** 上位机把人话、图、警告挂在 id 上，
 *      重编号会静默地换掉一整屏的解释。
 *   5. **人话归上位机(BoardPack)。** 固件给短标识符和数字，不给句子 ——
 *      否则改一句话就要发一次固件。
 *
 * ⚠ 还有一条是控制板特有的，驱动板没有：
 *   **这块板是发命令的那一端。** 上位机对它的每一个写操作都应该能问出
 *   "这会让车动吗"。所以每一行多带 moves= 和 coasts= 两位 ——
 *   词是从驱动板 `calib list` 借的，不另发明一套。
 *   分成两位而不是一位的理由也是他们写下的：**"会让轮子转"和"发了停止
 *   命令之后还在转"是两句不同的话，而第二句更危险，因为看着的人以为它
 *   已经停了。**
 *
 * ⚠⚠ 上面这段话 2026-08-30 之前是对的，今天不是了。保留它的形状是为了说清楚变了什么：
 *
 *   旧说法（已作废）：「整张表 perm=ro，这块板没有任何非易失存储，
 *   板上那颗 M24C64 应用固件没初始化 I2C1，够不着。」
 *
 *   现在：`b99afbd`（2026-08-30）把持久化接上了 —— I2C1 在 main.c 里配好（PB6/PB7 AF4），
 *   `cart_persist.c` 自己开时钟并初始化外设，`DlaCart_AppInit()` 开机调
 *   `CartPersist_Init()` + `CartPersist_Load()`。表里 **10 行 rt 非空**，
 *   它们报 `perm=rw persist=yes`，`param set` 真写、`param save` 真落 EEPROM。
 *
 * ⚠ 一句过期的注释不是无害的：上位机 2026-09-01 的《固件调整请求》 F-3
 *   就是照着这三处注释写的，把“已经做完的事”当成了两块板之间唯一的实质功能差。
 *   ⇒ 要断言板子能不能存，去看 `param list` 的 perm=，不要看任何人写的拄本。
 *
 * ⚠ 仍然成立的那半句：**一个改完重启就悄悄变回去的可写参数，比不可写更糟**。
 *   这正是剩下 55 行不给 rt 的理由：执行方还在读编译期常量的，就不该报 rw。
 */

#define CART_PARAM_TABLE_VERSION  1U

typedef enum {
  CART_PARAM_CLASS_SETTING = 0,   /* 行为参数 */
  CART_PARAM_CLASS_DIAG           /* 只读的活值 */
} CartParamClass_t;

typedef struct {
  uint16_t    id;
  const char *name;
  const char *unit;          /* "-" 表示无量纲 */
  int32_t     min;
  int32_t     max;
  int32_t     def;           /* 板子开机真的会成为的值 */
  int32_t     value;         /* getter 为 NULL 时用它 */
  int32_t   (*get)(void);    /* 活值：和元数据同一行 */
  /*
   * ⚠⚠ 可写参数的**运行时值**在哪里。非 NULL ⇒ 这一行报 perm=rw。
   *
   * 判据只有一条：**真正执行它的那段代码读的就是这个变量。**
   * 一个 rt 非空、而执行方仍然读编译期常量的参数，会造出这个仓一直在记的
   * 那种缺陷 —— `param set` 回显成功、`param get` 读回新值、
   * **而机器的行为一点没变**（上位机 DBIS §4.4：命令回显成功 ≠ 命令生效）。
   * ⇒ 加一个 rw 参数时，先把消费方改成读这个变量，再把指针填进来。
   */
  int32_t    *rt;
  uint8_t     cls;           /* CartParamClass_t */
  uint8_t     silent;        /* 1 = 这里错了没有症状 */
  uint8_t     moves;         /* 1 = 改它会让车动 */
  uint8_t     coasts;        /* 1 = 停止命令发出之后还会继续动 */
} CartParamDef_t;

/* 表的行数与逐行取用。 */
uint16_t CartParam_Count(void);
const CartParamDef_t *CartParam_At(uint16_t index);
int32_t  CartParam_Read(const CartParamDef_t *d);

/* 这一行能不能写。⇒ 就是 `rt != NULL`，见结构体里那段。 */
uint8_t  CartParam_IsWritable(const CartParamDef_t *d);

/*
 * 从 EEPROM 装回来的一对 (id, value)。
 * ⚠ **范围校验在这里做**，和 CRC 并列（DBIS §8.1）：CRC 只保证字节没错，
 *   不保证值能用。越界的**丢掉并保留默认**，不夹取 —— 夹一个坏值等于把
 *   一个错误变成一个看起来合理的错误。
 * 返回 1 = 收下了。
 */
int      CartParam_ApplyStored(uint16_t id, int32_t v);

/*
 * `param set <名字> <值>`。自己把回应发出去：成功发那一行 `=param`，
 * 失败发 `=param-err ... code=`（unknown-param / read-only / out-of-range /
 * wrong-state），都是上位机 PARAM_ERROR_CODES 里已有的词。
 */
void     CartParam_Set(const char *name, int32_t value);

/* `param save` / `param load`，自己发应答。 */
void     CartParam_Save(void);
void     CartParam_LoadCmd(void);

/* ---- 可写参数的运行时值。⚠ 消费方**必须**读这些，不要读 cart_params.h 的宏。 ---- */
extern int32_t g_p_auto_max_speed;      /* mm/s   */
extern int32_t g_p_auto_max_throttle;   /* permille */
extern int32_t g_p_auto_cmd_timeout;    /* ms     */
extern int32_t g_p_follow_cmd_timeout;  /* ms     */
extern int32_t g_p_thr_adc_dead;        /* ADC    踏板零点 */
extern int32_t g_p_thr_adc_full;        /* ADC    踏板满点 */
extern int32_t g_p_thr_dead;            /* permille 回位间隙 */
extern int32_t g_p_steer_max;           /* 0.1deg */
extern int32_t g_p_rc_stale;            /* ms     */
extern int32_t g_p_gear_debounce;       /* ms     */

/*
 * `param list` 的分页发送。
 *
 * 不能一口气把整张表灌进串口：一行约 190 字节，115200 下阻塞发送约 16.5ms,
 * 五十行就是 800ms 不回主循环 —— 而 0x210 的周期是 20ms，驱动板的超时是
 * 300ms。**列一次参数会让车失去指令。**
 * 所以每次主循环发一行，帧照发。这和驱动板每行之后 BspUart_Drain() 是
 * 同一个理由，只是这边阻塞发送的代价更大。
 *
 * 台架实测(2026-08-27，COM9，驱动板在线)：
 *   基线 0x210 周期 19.5ms(51.3 帧/秒)，正在目标上；
 *   列表期间约 32ms/帧 —— 每行 16.5ms 阻塞发送，与预测吻合；
 *   全程 0x211[7] bit7 保持 1(**驱动板一次都没有不信过我们**)，tx_fail 为 0。
 */
void    CartParam_ListStart(void);
uint8_t CartParam_ListBusy(void);
void    CartParam_ListStep(void);   /* 主循环每趟调一次，发一行 */

/*
 * `tlm` 一次遥测快照。
 *
 * 与参数表同一个泵化理由，且同一个代价：阻塞发送期间 0x210 停发。
 * 六行，一行约 120～190 字节 ⇒ 单行约 10～17ms，一趟主循环发一行。
 *
 * ⚠ 上位机那边 open() 固定返回 false（见 mcp/tools.ts），
 *   意思是**安静依然会结束读取**，即使帧还没关。
 *   他们的安静窗口是 400ms ⇒ **行与行之间的间隔不得接近 400ms**。
 *   一趟主循环远不到那个量级；但那是余量，不是保证。
 */
/*
 * `param get <名字>` —— 发表里那一行；名字不认识就发
 * `=param-err ... code=unknown-param`。⚠ **不需要存储**。
 */
void    CartParam_Get(const char *name);

/*
 * `param set` / `param save` 的**结构化拒绝**。
 *
 * ⚠⚠ "做不到"和"没听见"必须是两个不同的答案。在这之前这两条命令落到
 * CLI 的兜底回复，上位机据此只能说"板子没给拒绝码，别假设它写进去了" ——
 * 正确，但它不知道为什么。
 * ⇒ 回 `code=read-only`，那是上位机 PARAM_ERROR_CODES 里已有的词。
 */
void    CartParam_RefuseSet(const char *name);
void    CartParam_RefuseSave(void);

/* `=tlm ... build ...` 那一行的**唯一**合成器。`ver` 和 `tlm` 都调它。 */
void    CartTlm_BuildLine(char *buf, uint16_t size);

/*
 * `fault` / `fault codes` —— 形状照上位机 fault.ts 的正则，见 cart_param.c。
 * ⚠ 两条都是长输出，按 DBIS §4.5 分片：一趟主循环一行。
 */
/*
 * 标定编排（DBIS §5.3）。上位机一步步带着用户走。
 * ⚠ 形状问的是 COM5 上真正被接进去的驱动板，不是照 DBIS 草案写的。
 */
/*
 * 自检（DBIS §5.4）。⚠ 标准要求它**编进正式固件** ——
 * 只存在于独立构建 = 上位机在客户板子上触发不了。
 */
void    CartSelftest_Start(void);
void    CartSelftest_Step(void);     /* 主循环每趟一项 */

void    CartCalib_ListStart(void);
void    CartCalib_StatusStart(void);
void    CartCalib_Step(void);        /* 主循环每趟一行 */
void    CartCalib_Start(const char *flow);
void    CartCalib_Confirm(void);
void    CartCalib_Abort(void);

void    CartFault_Start(void);
void    CartFault_Step(void);
void    CartFault_CodesStart(void);
void    CartFault_CodesStep(void);

void    CartTlm_Start(void);
uint8_t CartTlm_Busy(void);
void    CartTlm_Step(void);         /* 主循环每趟调一次，发一行 */

#endif
