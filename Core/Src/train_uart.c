#include "train_uart.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define TRAIN_PAYLOAD_OFF  2
#define TRAIN_PAYLOAD_LEN  18
#define TRAIN_CRC_POLY     0xA001U
#define RX_RB_SIZE        64

static uint16_t s_seq = 0;
static volatile uint8_t s_rb_buf[RX_RB_SIZE];
static volatile uint8_t s_rb_head = 0;
static volatile uint8_t s_rb_tail = 0;
static char s_line_buf[48];
static uint8_t s_line_len = 0;

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

void TrainUART_Init(void)
{
  s_seq = 0;
  s_rb_head = 0;
  s_rb_tail = 0;
  s_line_len = 0;
  s_rx_state = RX_IDLE;
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void TrainUART_SendText(const char *text)
{
  if (text == NULL)
  {
    return;
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 50U);
}

void TrainUART_ReceiveISR(void)
{
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_OREFLAG(&huart1);
  }
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
  {
    uint8_t b = (uint8_t)(READ_REG(huart1.Instance->DR) & 0xFFU);
    uint8_t next = (uint8_t)((s_rb_head + 1U) % RX_RB_SIZE);
    if (next != s_rb_tail)
    {
      s_rb_buf[s_rb_head] = b;
      s_rb_head = next;
    }
  }
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

  HAL_StatusTypeDef st = HAL_UART_Transmit(&huart1, buf, TRAIN_FRAME_LEN, 10U);
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
  buf[16] = 0;
  buf[17] = 0;
  buf[18] = 0;
  buf[19] = 0;

  uint16_t crc = crc16_modbus(&buf[TRAIN_PAYLOAD_OFF], TRAIN_PAYLOAD_LEN);
  pack_u16_le(&buf[20], crc);

  HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, buf, TRAIN_FRAME_LEN, 10U);
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

static int rb_get(uint8_t *out)
{
  if (s_rb_tail == s_rb_head)
  {
    return 0;
  }
  *out = s_rb_buf[s_rb_tail];
  s_rb_tail = (uint8_t)((s_rb_tail + 1U) % RX_RB_SIZE);
  return 1;
}

int TrainUART_PollLine(char *out, uint16_t out_size)
{
  uint8_t byte;

  if ((out == NULL) || (out_size == 0U))
  {
    return 0;
  }

  while (rb_get(&byte))
  {
    if ((byte == '\r') || (byte == '\n'))
    {
      if (s_line_len == 0U)
      {
        continue;
      }
      if (s_line_len >= out_size)
      {
        s_line_len = (uint8_t)(out_size - 1U);
      }
      memcpy(out, s_line_buf, s_line_len);
      out[s_line_len] = '\0';
      s_line_len = 0U;
      return 1;
    }
    if ((byte >= 0x20U) && (byte <= 0x7EU))
    {
      if (s_line_len < (sizeof(s_line_buf) - 1U))
      {
        s_line_buf[s_line_len++] = (char)byte;
      }
    }
    else
    {
      s_line_len = 0U;
    }
  }
  return 0;
}

/* 通用二进制帧解析：用共享状态机解析下位机收到的一帧(0x10/0x11/...)。
 * CRC 校验通过即返回 1，并填 *type 与 payload(去掉 SOF 的 20 字节: VER..CRC)。
 * 每次调用至多返回一帧；解析失败/缓冲排空返回 0。模式互斥(同一时刻只有一个
 * 模式调用其对应 wrapper)，故共享状态机安全。 */
static int poll_frame(uint8_t *type, uint8_t payload[20])
{
  uint8_t byte;

  while (rb_get(&byte))
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
