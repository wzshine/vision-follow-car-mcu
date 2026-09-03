#include "cart_persist.h"
#include "cart_param.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* ---------- I2C1：板载 EEPROM ----------
 *
 * PB6=SCL / PB7=SDA，AF4，板上已有 4.7k 上拉。
 * ⚠ 这段初始化抄自 selftest.c 里那一份 —— 那一份是**在真板上验过**的
 *   （写图案 → 回读比对 → PASS）。不另起炉灶，是因为"能不能通"这件事
 *   已经有人付过一次代价了。
 * ⚠ 用自己的句柄而不是共用 selftest 的：selftest 是独立构建，正式固件里没有它。
 */
#define EE_ADDR8      0xA0U   /* M24C64, A0/A1/A2 = GND */
#define EE_PAGE       32U     /* 页大小；跨页写会绕回页首，不是继续往后 */
#define EE_WRITE_MS   6U      /* 写周期 5ms 典型，取 6 */
#define EE_TIMEOUT    100U

static I2C_HandleTypeDef s_i2c;
static uint8_t  s_ready;
static uint8_t  s_present;
static uint8_t  s_region;                  /* 当前有效记录在哪个区：0=A 1=B */
static uint16_t s_seq;
static CartPersistStatus_t s_status = CART_PERSIST_EMPTY;

/* ---------- 记录 ---------- */
#define HDR_BYTES     12U     /* magic4 + version2 + size2 + seq2 + reserved2 */
#define PAIR_BYTES    6U      /* id2 + value4 */
#define CRC_BYTES     4U
#define MAX_RECORD    (HDR_BYTES + (CART_PERSIST_MAX_PAIRS * PAIR_BYTES) + CRC_BYTES)

static uint8_t s_buf[MAX_RECORD];

/*
 * CRC-32（反射，多项式 0xEDB88320）—— 和上位机 isp.ts / zlib 同一个。
 * ⚠ 用软件实现而不是 MCU 的 CRC 外设：F4 的硬件 CRC 是 MPEG-2 那一族，
 *   算出来是**另一个数**，而这个数将来可能要被 PC 上的工具复核。
 */
static uint32_t crc32(const uint8_t *p, uint32_t n)
{
  uint32_t crc = 0xFFFFFFFFU;
  uint32_t i, b;
  for (i = 0U; i < n; i++) {
    crc ^= (uint32_t)p[i];
    for (b = 0U; b < 8U; b++) {
      crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)(-(int32_t)(crc & 1U)));
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void i2c_init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_AF_OD;
  g.Pull = GPIO_NOPULL;                 /* 板上已有 4.7k 外部上拉 */
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &g);

  s_i2c.Instance = I2C1;
  s_i2c.Init.ClockSpeed = 100000U;
  s_i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
  s_i2c.Init.OwnAddress1 = 0U;
  s_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  s_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  s_i2c.Init.OwnAddress2 = 0U;
  s_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  s_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  s_ready = (HAL_I2C_Init(&s_i2c) == HAL_OK) ? 1U : 0U;
}

static int ee_read(uint16_t addr, uint8_t *dst, uint16_t n)
{
  if (s_ready == 0U) { return 0; }
  return (HAL_I2C_Mem_Read(&s_i2c, EE_ADDR8, addr, I2C_MEMADD_SIZE_16BIT,
                           dst, n, EE_TIMEOUT) == HAL_OK) ? 1 : 0;
}

/*
 * 按页写。⚠ M24C64 跨页写会**绕回本页页首**覆盖已写的字节，
 * 而不是继续写到下一页 —— 那是一次静默的数据损坏。
 */
static int ee_write(uint16_t addr, const uint8_t *src, uint16_t n)
{
  uint16_t done = 0U;
  if (s_ready == 0U) { return 0; }
  while (done < n) {
    uint16_t a = (uint16_t)(addr + done);
    uint16_t room = (uint16_t)(EE_PAGE - (a % EE_PAGE));
    uint16_t chunk = ((uint16_t)(n - done) < room) ? (uint16_t)(n - done) : room;
    if (HAL_I2C_Mem_Write(&s_i2c, EE_ADDR8, a, I2C_MEMADD_SIZE_16BIT,
                          (uint8_t *)(uintptr_t)&src[done], chunk, EE_TIMEOUT) != HAL_OK) {
      return 0;
    }
    HAL_Delay(EE_WRITE_MS);
    done = (uint16_t)(done + chunk);
  }
  return 1;
}

/*
 * 读一个区并校验。返回 1 = 这条记录可用，并把 seq / size 交出去。
 *
 * ⚠⚠ **按记录自己的 `size-4` 校验**，不是按当前固件以为的长度。
 *   这一条就是"只允许追加字段"能成立的全部原因：一条旧的、短的记录，
 *   按它自己的条款算 CRC 仍然通过，多出来的字段这边填默认。
 */
static int read_region(uint16_t base, uint16_t *out_seq, uint16_t *out_size)
{
  uint16_t size;

  if (!ee_read(base, s_buf, HDR_BYTES)) { return 0; }
  if (get_u32(&s_buf[0]) != CART_PERSIST_MAGIC) { return 0; }
  size = get_u16(&s_buf[6]);
  if ((size < (HDR_BYTES + CRC_BYTES)) || (size > MAX_RECORD)) { return 0; }
  if (!ee_read(base, s_buf, size)) { return 0; }
  if (crc32(s_buf, (uint32_t)(size - CRC_BYTES)) != get_u32(&s_buf[size - CRC_BYTES])) {
    return 0;
  }
  *out_seq = get_u16(&s_buf[8]);
  *out_size = size;
  return 1;
}

int CartPersist_SelfTest(void)
{
  /* 图案里带一个随时间变的字节 —— 一个恒定图案在"读回来的其实是上次那份"
   * 的情况下也会通过。 */
  uint8_t wr[4];
  uint8_t rd[4] = {0, 0, 0, 0};

  if (s_present == 0U) { return 0; }
  wr[0] = 0xA5U; wr[1] = 0x5AU;
  wr[2] = (uint8_t)(HAL_GetTick() & 0xFFU);
  wr[3] = (uint8_t)(~wr[2]);
  if (!ee_write(CART_PERSIST_SCRATCH, wr, 4U)) { return 0; }
  if (!ee_read(CART_PERSIST_SCRATCH, rd, 4U))  { return 0; }
  return (memcmp(wr, rd, 4U) == 0) ? 1 : 0;
}

void CartPersist_Init(void)
{
  i2c_init();
  s_present = 0U;
  if (s_ready != 0U) {
    /* 只问一句"你在吗" —— 三次尝试，不写任何东西。 */
    s_present = (HAL_I2C_IsDeviceReady(&s_i2c, EE_ADDR8, 3U, EE_TIMEOUT) == HAL_OK) ? 1U : 0U;
  }
  s_status = (s_present != 0U) ? CART_PERSIST_EMPTY : CART_PERSIST_NO_DEVICE;
  s_seq = 0U;
  s_region = 0U;
}

uint8_t CartPersist_DevicePresent(void) { return s_present; }
CartPersistStatus_t CartPersist_LastStatus(void) { return s_status; }
uint16_t CartPersist_Seq(void) { return s_seq; }

uint16_t CartPersist_Load(void)
{
  uint16_t seq_a = 0U, seq_b = 0U, size_a = 0U, size_b = 0U;
  int ok_a, ok_b;
  uint16_t size, pairs, i, taken = 0U;

  if (s_present == 0U) { s_status = CART_PERSIST_NO_DEVICE; return 0U; }

  ok_a = read_region(CART_PERSIST_REGION_A, &seq_a, &size_a);
  ok_b = read_region(CART_PERSIST_REGION_B, &seq_b, &size_b);

  if ((ok_a == 0) && (ok_b == 0)) {
    /* ⚠ "两个区都读不出记录"在出厂的板子上是**正常**的，不是故障。
     *   分成 EMPTY 和 BAD_CRC 两个状态，是因为它们要查的东西不一样：
     *   前者什么都没写过，后者写过但坏了。这里没有办法区分——
     *   magic 对不上既可能是空白也可能是烂掉的 —— 所以报 EMPTY 并如实
     *   在文档里说它包含了这两种。 */
    s_status = CART_PERSIST_EMPTY;
    s_seq = 0U;
    return 0U;
  }

  /* ⭐ 校验通过且 seq 大的赢。⚠ 只在"两个都通过"时才比较 —— 一个通不过的区
   *   里的 seq 是垃圾，拿它去比较等于让坏数据参与决策。 */
  if ((ok_a != 0) && ((ok_b == 0) || (seq_a >= seq_b))) {
    s_region = 0U; size = size_a; s_seq = seq_a;
    (void)read_region(CART_PERSIST_REGION_A, &seq_a, &size_a);
  } else {
    s_region = 1U; size = size_b; s_seq = seq_b;
    (void)read_region(CART_PERSIST_REGION_B, &seq_b, &size_b);
  }

  pairs = (uint16_t)((size - HDR_BYTES - CRC_BYTES) / PAIR_BYTES);
  for (i = 0U; i < pairs; i++) {
    const uint8_t *p = &s_buf[HDR_BYTES + (i * PAIR_BYTES)];
    uint16_t id = get_u16(p);
    int32_t  v  = (int32_t)get_u32(&p[2]);
    /* ⭐ 字段级范围校验与 CRC **并列**：CRC 只保证字节没错，不保证值能用。
     *   越界的那一对**丢掉并保留默认**，不夹取 —— 夹一个坏值等于把一个
     *   错误变成一个看起来合理的错误。 */
    if (CartParam_ApplyStored(id, v) != 0) { taken++; }
  }
  s_status = CART_PERSIST_OK;
  return taken;
}

CartPersistStatus_t CartPersist_Save(void)
{
  uint16_t base, i, n = 0U, size;
  uint16_t vseq = 0U, vsize = 0U;

  if (s_present == 0U) { s_status = CART_PERSIST_NO_DEVICE; return s_status; }

  /* 写**另一个**区。⇒ 任何时刻至少有一个完整的旧记录还在 EEPROM 里。 */
  base = (s_region == 0U) ? CART_PERSIST_REGION_B : CART_PERSIST_REGION_A;

  memset(s_buf, 0, sizeof(s_buf));
  put_u32(&s_buf[0], CART_PERSIST_MAGIC);
  put_u16(&s_buf[4], (uint16_t)CART_PERSIST_VERSION);
  put_u16(&s_buf[8], (uint16_t)(s_seq + 1U));
  /* [10..11] reserved = 0 */

  for (i = 0U; i < CartParam_Count(); i++) {
    const CartParamDef_t *d = CartParam_At(i);
    if ((d == NULL) || (CartParam_IsWritable(d) == 0U)) { continue; }
    if (n >= CART_PERSIST_MAX_PAIRS) { break; }
    put_u16(&s_buf[HDR_BYTES + (n * PAIR_BYTES)], d->id);
    put_u32(&s_buf[HDR_BYTES + (n * PAIR_BYTES) + 2U], (uint32_t)CartParam_Read(d));
    n++;
  }

  size = (uint16_t)(HDR_BYTES + (n * PAIR_BYTES) + CRC_BYTES);
  put_u16(&s_buf[6], size);
  put_u32(&s_buf[size - CRC_BYTES], crc32(s_buf, (uint32_t)(size - CRC_BYTES)));

  if (!ee_write(base, s_buf, size)) { s_status = CART_PERSIST_IO_ERROR; return s_status; }

  /* ⭐⭐ 回读自验，**走的是下次开机那个解析器**（read_region），不是
   *   "把刚写的缓冲区和自己比一比"。后者永远通过，也就永远证明不了任何事。 */
  if (!read_region(base, &vseq, &vsize)) {
    s_status = CART_PERSIST_VERIFY_FAILED;
    return s_status;
  }
  if ((vseq != (uint16_t)(s_seq + 1U)) || (vsize != size)) {
    s_status = CART_PERSIST_VERIFY_FAILED;
    return s_status;
  }

  s_seq = vseq;
  s_region = (base == CART_PERSIST_REGION_A) ? 0U : 1U;
  s_status = CART_PERSIST_OK;
  return s_status;
}
