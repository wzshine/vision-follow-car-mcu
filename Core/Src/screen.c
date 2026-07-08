#include "screen.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define CMD_WRITE_VAR  0x82
#define CMD_READ_VAR   0x83

#define RX_PAYLOAD_MAX  64
static volatile uint8_t rx_buffer[RX_PAYLOAD_MAX];
static volatile uint8_t rx_index = 0;
static volatile uint8_t rx_expect = 0;
static volatile uint8_t rx_state = 0;
static volatile uint16_t current_addr = 0;
static volatile uint16_t current_data = 0;
static volatile uint32_t screen_last_rx_ms = 0;

#define STATE_IDLE        0
#define STATE_WAIT_HEADER 1
#define STATE_WAIT_LEN    2
#define STATE_WAIT_PAYLOAD 3

static void Screen_RxByte(uint8_t byte)
{
  switch (rx_state)
  {
    case STATE_IDLE:
      if (byte == 0xA5)
      {
        rx_state = STATE_WAIT_HEADER;
      }
      break;

    case STATE_WAIT_HEADER:
      if (byte == 0x5A)
      {
        rx_state = STATE_WAIT_LEN;
      }
      else
      {
        rx_state = STATE_IDLE;
      }
      break;

    case STATE_WAIT_LEN:
      rx_expect = byte;
      if (rx_expect == 0 || rx_expect > RX_PAYLOAD_MAX)
      {
        rx_state = STATE_IDLE;
        break;
      }
      rx_index = 0;
      rx_state = STATE_WAIT_PAYLOAD;
      break;

    case STATE_WAIT_PAYLOAD:
      rx_buffer[rx_index++] = byte;
      if (rx_index >= rx_expect)
      {
        uint8_t cmd = rx_buffer[0];
        if (cmd == 0x83)
        {
          if (rx_expect >= 6)
          {
            uint16_t addr = ((uint16_t)rx_buffer[1] << 8) | rx_buffer[2];
            uint8_t n = rx_buffer[3];
            if (n >= 1 && (4u + 2u * n) <= (unsigned)rx_expect)
            {
              uint16_t key = ((uint16_t)rx_buffer[4] << 8) | rx_buffer[5];
              current_addr = addr;
              current_data = key;
            }
          }
        }
        rx_index = 0;
        rx_state = STATE_IDLE;
      }
      break;

    default:
      rx_state = STATE_IDLE;
      rx_index = 0;
      rx_expect = 0;
      break;
  }
}

static void Screen_RxDrain(void)
{
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    rx_state = STATE_IDLE;
    rx_index = 0;
    rx_expect = 0;
  }
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
  {
    uint8_t byte = (uint8_t)(READ_REG(huart2.Instance->DR) & 0xFFU);
    screen_last_rx_ms = HAL_GetTick();
    Screen_RxByte(byte);
  }
}

void Screen_ReceiveISR(void)
{
  Screen_RxDrain();
}

void Screen_Init(void)
{
  rx_state = STATE_IDLE;
  rx_index = 0;
  rx_expect = 0;
  screen_last_rx_ms = HAL_GetTick();
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

uint32_t Screen_GetLastRxMs(void)
{
  return screen_last_rx_ms;
}

void Screen_SetInitialValues(void)
{
  Screen_SendData(SCREEN_ADDR_GEAR, SCREEN_KEY_GEAR_P);
  Screen_SendData(SCREEN_ADDR_SPEED, SCREEN_KEY_SPEED_LOW);
  Screen_SendData(SCREEN_ADDR_MODE, SCREEN_KEY_MODE_MANUAL);
  Screen_SendData(SCREEN_ADDR_CONNECTION, 1);
  Screen_SendData(SCREEN_ADDR_SPEED_DISPLAY, 0);
  Screen_SendData(SCREEN_ADDR_COLLECT_ACK, 0);   /* 采集确认灯上电置 0 */
  Screen_SendData(SCREEN_ADDR_SAVE_STATUS, 0);   /* 保存状态灯上电置 0 */
}

void Screen_SendCommand(uint8_t cmd, uint16_t addr, uint16_t data)
{
  uint8_t tx_buf[8] = {0};

  tx_buf[0] = 0xA5;
  tx_buf[1] = 0x5A;
  tx_buf[2] = 1 + 2 + 2;
  tx_buf[3] = cmd;
  tx_buf[4] = (uint8_t)(addr >> 8);
  tx_buf[5] = (uint8_t)(addr & 0xFF);
  tx_buf[6] = (uint8_t)(data >> 8);
  tx_buf[7] = (uint8_t)(data & 0xFF);

  (void)HAL_UART_Transmit(&huart2, tx_buf, 8U, 100U);
}

void Screen_SendData(uint16_t addr, uint16_t data)
{
  Screen_SendCommand(CMD_WRITE_VAR, addr, data);
}

void Screen_ReadVar(uint16_t addr, uint8_t n_words)
{
  if (n_words == 0 || n_words > 0x7F)
  {
    return;
  }
  uint8_t tx_buf[8];
  tx_buf[0] = 0xA5;
  tx_buf[1] = 0x5A;
  tx_buf[2] = 1 + 2 + 1;
  tx_buf[3] = CMD_READ_VAR;
  tx_buf[4] = (uint8_t)(addr >> 8);
  tx_buf[5] = (uint8_t)(addr & 0xFF);
  tx_buf[6] = n_words;
  (void)HAL_UART_Transmit(&huart2, tx_buf, 7U, 100U);
}

void Screen_SetGear(uint16_t gear_key)
{
  Screen_SendData(SCREEN_ADDR_GEAR, gear_key);
}

void Screen_SetSpeed(uint16_t speed_key)
{
  Screen_SendData(SCREEN_ADDR_SPEED, speed_key);
}

void Screen_SetMode(uint16_t mode_key)
{
  Screen_SendData(SCREEN_ADDR_MODE, mode_key);
}

void Screen_SetConnectionStatus(uint8_t connected)
{
  Screen_SendData(SCREEN_ADDR_CONNECTION, connected ? 1 : 0);
}

void Screen_SetCollectAck(uint8_t on)
{
  Screen_SendData(SCREEN_ADDR_COLLECT_ACK, on ? 1 : 0);
}

void Screen_SetSaveStatus(uint8_t on)
{
  Screen_SendData(SCREEN_ADDR_SAVE_STATUS, on ? 1 : 0);
}

/* 写文本变量：A5 5A <len> 0x82 <addr_hi> <addr_lo> <UTF-8 字节...> 0xFF 0xFF。
 * 末尾 0xFFFF 为 VGUS/DGUS 文本控件的字符串结束符(清除该控件后续残留字符)。 */
void Screen_SendText(uint16_t addr, const char *s)
{
  uint8_t tx[64];
  uint8_t n;
  uint8_t slen = 0;

  if (s == NULL)
  {
    return;
  }
  while (s[slen] != '\0' && slen < 56)   /* 限长，留出帧头/地址/结束符余量 */
  {
    slen++;
  }

  tx[0] = 0xA5;
  tx[1] = 0x5A;
  tx[2] = (uint8_t)(1 + 2 + slen + 2);   /* cmd + addr(2) + 文本 + 结束符(2) */
  tx[3] = CMD_WRITE_VAR;
  tx[4] = (uint8_t)(addr >> 8);
  tx[5] = (uint8_t)(addr & 0xFF);
  n = 6;
  for (uint8_t i = 0; i < slen; i++)
  {
    tx[n++] = (uint8_t)s[i];
  }
  tx[n++] = 0xFF;
  tx[n++] = 0xFF;
  (void)HAL_UART_Transmit(&huart2, tx, n, 100U);
}

void Screen_SetEnableIndicator(uint8_t enabled)
{
  Screen_SendText(SCREEN_ADDR_ENABLE, enabled ? "R" : " ");
}

uint16_t Screen_GetKey(void)
{
  uint16_t key = current_data;
  current_data = 0;
  current_addr = 0;
  return key;
}

void Screen_GetKeyAndAddr(uint16_t *key_out, uint16_t *addr_out)
{
  *key_out = current_data;
  *addr_out = current_addr;
  current_data = 0;
  current_addr = 0;
}
