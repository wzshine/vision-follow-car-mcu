#include "train_uart.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define TRAIN_PAYLOAD_OFF  2
#define TRAIN_PAYLOAD_LEN  18
#define TRAIN_CRC_POLY     0xA001U
/*
 * 64 -> 256。
 *
 * 上行是 33Hz 的 22 字节帧，约 726 B/s；而主循环最坏一趟约 60~90ms
 * (1 秒边界上电池 ADC + 四次 9600 波特的屏幕阻塞写挤在一起)，
 * 那个窗口里能到 65 字节 —— 64 的环恰好**刚刚不够**。
 * 256 给出真正的余量，代价是 192 字节 RAM。
 */
#define RX_RB_SIZE        256

static uint16_t s_seq = 0;

/*
 * ==================== 两根线，两种协议 ====================
 *
 * 2026-08-30 之前这个模块只认 huart1，USART1 上同时跑三样东西：
 * Jetson 的 22 字节二进制帧（双向）、COLLECT/AUTO 下 50Hz 的 `IMU,` CSV、
 * 以及 CR/LF 的 CLI。靠内容分流，代价写在两处：
 *
 *   - AUTO/FOLLOW 下**整个 CLI 被关掉**（dla_cart_loop.inc 里那个模式判断），
 *     因为行消费者会从同一个环里偷走二进制帧的字节 —— 那是 2026-06-02
 *     修一个真实缺陷（arm 闪断、刹车点动）时付的账；
 *   - `PollLine()` 遇到任何非可打印字节就清行缓冲 ⇒ Jetson 一发帧，
 *     **一条正在收的 CLI 命令会被无声打掉**，任何模式下都可能。
 *
 * 现在分开（用户 2026-08-30 定）：
 *
 *   USART1 / USB1  = 上位机 + 调试 + 刷写。**只跑 ASCII。**
 *   UART5  / USB2  = Jetson 通信。二进制帧 + `IMU,` CSV + Jetson 回的 MAP,/SAVE,。
 *
 * ⇒ CLI 那一侧再也不会被二进制字节打掉，二进制帧也再不会被 CLI 偷走
 *   ⇒ **AUTO/FOLLOW 下的 CLI 不必再关**。
 *
 * ⭐ 硬件上这个分工顺带买到一条安全性质：USB2 那颗 CH340X（U4）的
 *   RTS#/CTS#/TNOW/DTR# **三个脚全部未接**（定版原理图第 4 页），所以
 *   **接在 USB2 上的 Jetson 不可能把这块板复位或踢进 BootLoader**；
 *   而 USB1 那颗（U12）的 DTR# 经 Q2/Q3 直通 NRST + BOOT0，**可以**。
 *
 * ⚠ 两个环是同一份结构的两份实例，不是两套代码：一份收发逻辑写两遍，
 *   迟早只修好一遍。
 */
typedef struct {
  volatile uint8_t  buf[RX_RB_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  /* 丢了多少字节，以及硬件告诉过我们多少次它自己丢了(ORE)。
   * 从前两者都是静默的：环满了就丢，ORE 标志清掉就算完 ——
   * **硬件明确说了"这里少了一个字节"，而我们把这句话扔了。** */
  volatile uint32_t drop_full;
  volatile uint32_t drop_ore;
  /* 收进环的字节总数。⚠ 它回答的是"这根线上有没有东西来过"，而丢字节计数
   * 回答的是"来了但没接住"。**0 和 0 是两个不同的坏消息**：前者是线没接、
   * 波特率不对或者中断没开，后者是接住了但处理不过来。 */
  volatile uint32_t rx_bytes;
  char              line_buf[48];
  uint8_t           line_len;
} rx_ring_t;

static rx_ring_t s_cli;      /* USART1：只有 ASCII */
static rx_ring_t s_jet;      /* UART5 ：二进制帧 + MAP,/SAVE, 行 */

/* 二进制帧的解析状态机 —— **只属于 Jetson 那一路**（UART5）。
 * CLI 那一路上不存在二进制，所以它没有、也不需要这套东西。 */
#define RX_IDLE      0
#define RX_HEADER    1
#define RX_PAYLOAD   2
static uint8_t s_rx_state = RX_IDLE;
static uint8_t s_rx_buf[22];
static uint8_t s_rx_idx = 0;
static uint8_t s_rx_expect = 0;

static uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i];
    for (uint16_t j = 0; j < 8U; j++)
    {
      if (crc & 1U)
      {
        crc = (uint16_t)((crc >> 1) ^ TRAIN_CRC_POLY);
      }
      else
      {
        crc = (uint16_t)(crc >> 1);
      }
    }
  }
  return crc;
}

static void pack_u16_le(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)(v >> 8);
}

static void pack_u32_le(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)((v >> 8) & 0xFFU);
  p[2] = (uint8_t)((v >> 16) & 0xFFU);
  p[3] = (uint8_t)(v >> 24);
}

static void ring_reset(rx_ring_t *r)
{
  r->head = 0;
  r->tail = 0;
  r->line_len = 0;
}

void TrainUART_Init(void)
{
  s_seq = 0;
  ring_reset(&s_cli);
  ring_reset(&s_jet);
  s_rx_state = RX_IDLE;

  /* USART1：CLI。 */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* UART5：Jetson。⚠ 优先级和 USART1 同档 —— 两路都只是把字节搬进环，
   * 谁先谁后不影响正确性；把任一路调低只会让那一路更容易 ORE。
   * ⚠⚠ MX_UART5_Init() 早就在 main.c 里跑了，但**中断从来没开过** ——
   * 在这一步之前，UART5 收到的每一个字节都直接掉在地上。 */
  __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
  HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
}

/* CLI 回复 —— USART1。⚠ 名字没改：foc_can.c 里几十处 test_ack 都走它，
 * 而它们的去处**就是**上位机那根线。 */
void TrainUART_SendText(const char *text)
{
  if (text == NULL)
  {
    return;
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 50U);
}

/* 发给 Jetson 的 ASCII —— UART5。今天只有 `IMU,` CSV 走这里。
 * ⚠ 和上面那个是两个不同的去处，别因为都叫 SendText 就合并。 */
void TrainUART_SendJetsonText(const char *text)
{
  if (text == NULL)
  {
    return;
  }
  (void)HAL_UART_Transmit(&huart5, (uint8_t *)text, (uint16_t)strlen(text), 50U);
}

static void ring_isr(rx_ring_t *r, UART_HandleTypeDef *h)
{
  if (__HAL_UART_GET_FLAG(h, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_OREFLAG(h);
    r->drop_ore++;
  }
  while (__HAL_UART_GET_FLAG(h, UART_FLAG_RXNE))
  {
    uint8_t b = (uint8_t)(READ_REG(h->Instance->DR) & 0xFFU);
    uint16_t next = (uint16_t)((r->head + 1U) % RX_RB_SIZE);
    if (next != r->tail)
    {
      r->buf[r->head] = b;
      r->head = next;
      r->rx_bytes++;
    }
    else
    {
      r->drop_full++;   /* 满了。丢一个字节就够毁掉一整帧的 CRC。 */
    }
  }
}

void TrainUART_CliISR(void)
{
  ring_isr(&s_cli, &huart1);
}

void TrainUART_JetsonISR(void)
{
  ring_isr(&s_jet, &huart5);
}

uint16_t TrainUART_GetNextSeq(void)
{
  return s_seq++;
}

static int send_frame_internal(const TrainFrameData_t *data, uint8_t type)
{
  uint8_t buf[TRAIN_FRAME_LEN];
  if (data == NULL)
  {
    return -1;
  }

  buf[0] = TRAIN_SOF0;
  buf[1] = TRAIN_SOF1;
  buf[2] = TRAIN_VER;
  buf[3] = type;

  pack_u16_le(&buf[4], data->seq);
  pack_u32_le(&buf[6], data->ts_ms);
  pack_u16_le(&buf[10], data->speed_01);
  pack_u16_le((uint8_t *)&buf[12], (uint16_t)data->steer_01);
  pack_u16_le(&buf[14], data->thr_ref);
  buf[16] = data->brake;
  buf[17] = data->gear;
  buf[18] = 0;
  buf[19] = 0;

  uint16_t crc = crc16_modbus(&buf[TRAIN_PAYLOAD_OFF], TRAIN_PAYLOAD_LEN);
  pack_u16_le(&buf[20], crc);

  /* 二进制帧走 Jetson 那根线（UART5），不是 CLI 那根。 */
  HAL_StatusTypeDef st = HAL_UART_Transmit(&huart5, buf, TRAIN_FRAME_LEN, 10U);
  return (st == HAL_OK) ? 0 : -1;
}

int TrainUART_SendFrame(const TrainFrameData_t *data)
{
  return send_frame_internal(data, TRAIN_TYPE_CTRL);
}

int TrainUART_SendFollowReady(const TrainFrameData_t *data)
{
  return send_frame_internal(data, TRAIN_TYPE_FOLLOW);
}

int TrainUART_SendAutoStatus(const AutoStatus_t *st)
{
  uint8_t buf[TRAIN_FRAME_LEN];
  if (st == NULL)
  {
    return -1;
  }

  buf[0] = TRAIN_SOF0;
  buf[1] = TRAIN_SOF1;
  buf[2] = TRAIN_VER;
  buf[3] = TRAIN_TYPE_AUTO_RDY;            /* 0x03 */

  pack_u16_le(&buf[4], st->seq);
  pack_u32_le(&buf[6], st->ts_ms);
  pack_u16_le(&buf[10], (uint16_t)st->spd_meas_mm_s);
  pack_u16_le(&buf[12], (uint16_t)st->steer_fb_01);
  buf[14] = st->status;
  buf[15] = st->gear;
  /* [16:19] 从前是写死的 0。理由见 AutoStatus_t 的注释：
   * AUTO 模式下 CLI 关闭，这四个字节是"为什么不让走"唯一到得了上位机的路。 */
  pack_u16_le(&buf[16], st->block);
  buf[18] = st->fw_ver;
  buf[19] = st->fw_patch;

  uint16_t crc = crc16_modbus(&buf[TRAIN_PAYLOAD_OFF], TRAIN_PAYLOAD_LEN);
  pack_u16_le(&buf[20], crc);

  /* 同上：二进制帧走 UART5。 */
  HAL_StatusTypeDef status = HAL_UART_Transmit(&huart5, buf, TRAIN_FRAME_LEN, 10U);
  return (status == HAL_OK) ? 0 : -1;
}

int TrainUART_SendManualNotify(const TrainFrameData_t *data)
{
  return send_frame_internal(data, TRAIN_TYPE_MANUAL);
}

int TrainUART_SendRemoteNotify(const TrainFrameData_t *data)
{
  return send_frame_internal(data, TRAIN_TYPE_REMOTE);
}

int TrainUART_SendCollectFrame(const TrainFrameData_t *data)
{
  return send_frame_internal(data, TRAIN_TYPE_COLLECT);
}

static uint16_t unpack_u16_le(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t unpack_s16_le(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int rb_get(rx_ring_t *r, uint8_t *out)
{
  if (r->tail == r->head)
  {
    return 0;
  }
  *out = r->buf[r->tail];
  r->tail = (uint16_t)((r->tail + 1U) % RX_RB_SIZE);
  return 1;
}

/*
 * 从某一个环里取出一整行 ASCII。
 *
 * ⚠ 非可打印字节把当前行清零 —— 在 CLI 那根线上这已经是纯粹的抗噪；
 *   在 Jetson 那根线上它同时是分流器（二进制帧的字节会清掉半行 MAP,/SAVE,）。
 *   ⇒ 这正是当初把两者挤在一根线上时，一条 CLI 命令会被无声打掉的原因。
 */
static int ring_poll_line(rx_ring_t *r, char *out, uint16_t out_size)
{
  uint8_t byte;

  if ((out == NULL) || (out_size == 0U))
  {
    return 0;
  }

  while (rb_get(r, &byte))
  {
    if ((byte == '\r') || (byte == '\n'))
    {
      if (r->line_len == 0U)
      {
        continue;
      }
      if (r->line_len >= out_size)
      {
        r->line_len = (uint8_t)(out_size - 1U);
      }
      memcpy(out, r->line_buf, r->line_len);
      out[r->line_len] = '\0';
      r->line_len = 0U;
      return 1;
    }
    if ((byte >= 0x20U) && (byte <= 0x7EU))
    {
      if (r->line_len < (sizeof(r->line_buf) - 1U))
      {
        r->line_buf[r->line_len++] = (char)byte;
      }
    }
    else
    {
      r->line_len = 0U;
    }
  }
  return 0;
}

/* 通用二进制帧解析：用共享状态机解析下位机收到的一帧(0x10/0x11/...)。
 * CRC 校验通过即返回 1，并填 *type 与 payload(去掉 SOF 的 20 字节: VER..CRC)。
 * 每次调用至多返回一帧；解析失败/缓冲排空返回 0。模式互斥(同一时刻只有一个
 * 模式调用其对应 wrapper)，故共享状态机安全。 */

/* 上位机 / 调试那根线（USART1）。⚠ 这一路上没有二进制，所以它**任何模式下
 * 都可以轮询** —— 从前那个"AUTO/FOLLOW 下不要调它"的限制随分线一起没了。 */
int TrainUART_PollLine(char *out, uint16_t out_size)
{
  return ring_poll_line(&s_cli, out, out_size);
}

/* Jetson 那根线（UART5）上的 ASCII：今天只有它回的 "MAP,1/0" / "SAVE,1/0"。
 * ⚠ 它和 poll_frame() 共用同一个环，谁先跑谁先吃字节 —— 那是 Jetson 这一路
 * 本来就有的安排（两端按内容分流），不是这次分线引入的。 */
int TrainUART_PollJetsonLine(char *out, uint16_t out_size)
{
  return ring_poll_line(&s_jet, out, out_size);
}

static int poll_frame(uint8_t *type, uint8_t payload[20])
{
  uint8_t byte;

  while (rb_get(&s_jet, &byte))
  {
    switch (s_rx_state)
    {
      case RX_IDLE:
        if (byte == TRAIN_SOF0)
        {
          s_rx_state = RX_HEADER;
        }
        break;
      case RX_HEADER:
        if (byte == TRAIN_SOF1)
        {
          s_rx_idx = 0;
          s_rx_expect = TRAIN_FRAME_LEN - 2U;
          s_rx_state = RX_PAYLOAD;
        }
        else
        {
          s_rx_state = (byte == TRAIN_SOF0) ? RX_HEADER : RX_IDLE;
        }
        break;
      case RX_PAYLOAD:
        s_rx_buf[s_rx_idx++] = byte;
        if (s_rx_idx >= s_rx_expect)
        {
          s_rx_state = RX_IDLE;
          uint8_t *b = s_rx_buf;
          if (b[0] != TRAIN_VER)
          {
            return 0;
          }
          uint16_t crc_recv = unpack_u16_le(&b[18]);
          uint16_t crc_calc = crc16_modbus(b, TRAIN_PAYLOAD_LEN);
          if (crc_recv != crc_calc)
          {
            return 0;
          }
          if (type != NULL)
          {
            *type = b[1];
          }
          if (payload != NULL)
          {
            memcpy(payload, b, 20U);
          }
          return 1;
        }
        break;
      default:
        s_rx_state = RX_IDLE;
        break;
    }
  }
  return 0;
}

int TrainUART_PollFollowCmd(FollowCmd_t *out)
{
  uint8_t type;
  uint8_t b[20];

  if (out == NULL)
  {
    return 0;
  }

  while (poll_frame(&type, b))
  {
    if (type != TRAIN_TYPE_CMD_FOLLOW)
    {
      continue;
    }
    out->steer_01 = unpack_s16_le(&b[10]);
    out->thr_ref = unpack_u16_le(&b[12]);
    out->brake = b[14];
    return 1;
  }
  return 0;
}

int TrainUART_PollAutoCmd(AutoCmd_t *out)
{
  uint8_t type;
  uint8_t b[20];

  if (out == NULL)
  {
    return 0;
  }

  /* 帧布局(s_rx_buf 索引 = 帧偏移 − 2，见 AUTONOMOUS_MODE_SPEC §3):
   *   b[10:11] STEER int16 0.1°   b[12:13] TARGET_SPEED int16 mm/s
   *   b[14] BRAKE  b[15] GEAR  b[16] FLAGS  b[17] RES  b[18:19] CRC */
  while (poll_frame(&type, b))
  {
    if (type != TRAIN_TYPE_CMD_AUTO)
    {
      continue;
    }
    out->steer_01 = unpack_s16_le(&b[10]);
    out->target_speed_mm_s = unpack_s16_le(&b[12]);
    out->brake = b[14];
    out->gear = b[15];
    out->flags = b[16];
    return 1;
  }
  return 0;
}

/* ⚠ 四个数分开报，不合并。"上位机那根线丢字节"和"Jetson 那根线丢字节"
 * 要查的东西完全不一样，合成一个总数就再也分不出是哪一根。 */
uint32_t TrainUART_GetRxDropFull(void)       { return s_cli.drop_full; }
uint32_t TrainUART_GetRxDropOre(void)        { return s_cli.drop_ore; }
uint32_t TrainUART_GetJetsonDropFull(void)   { return s_jet.drop_full; }
uint32_t TrainUART_GetJetsonDropOre(void)    { return s_jet.drop_ore; }
uint32_t TrainUART_GetJetsonRxBytes(void)    { return s_jet.rx_bytes; }
uint32_t TrainUART_GetCliRxBytes(void)       { return s_cli.rx_bytes; }
