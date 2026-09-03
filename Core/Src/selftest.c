/*
 * DlaBoard_B 接口自测固件 —— 阶段 1（板载 / 无需跳线）。
 *
 * 链路：USART1（USB1 的 CH340，115200 8N1）文本 CLI。每行一个命令，回车结束。
 * 命令：
 *   help            列出命令
 *   imu             SPI1 板载 IMU：校验 WHO_AM_I + 读一帧
 *   eeprom          I2C1 板载 EEPROM：写入图案后回读比对
 *   vbus            ADC 输入电压检测通道（IN11）读数，估算输入电压
 *   adc             读全部 6 路外部 ADC（IN8/9/12/13/14/15）原始值+毫伏
 *   led 0|1         PE0 用户 LED 关/开（人工目视）
 *   buzz [ms]       PE1 蜂鸣器鸣响 ms（默认 200，人工耳听）
 *   can             CAN1 内部 LOOPBACK 自测（无需总线）
 *   runall          顺序跑全部自动项，输出汇总
 *
 * 每个测试输出一行： RESULT <name> PASS|FAIL|INFO <detail>
 * runall 末尾输出： SUMMARY <pass>/<total>  与  DONE
 */
#include "main.h"
#include "selftest.h"
#include "imu_lsm6dsv.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

/* 主工程已初始化并 extern 的句柄 */
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;   /* 主工程已 init（9600） */
extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef htim1;     /* 主工程已 init */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* CLI 控制台串口：UART5（=USB2，无一键下载电路）。
 * CLI 能从 USB2 跑通 = UART5+CH340+USB2 全链路验证通过。
 * 如需换回 USB1，把下行改成 huart1。 */
#define CLI_UART huart5

/* 本文件自管的测试外设句柄（主工程未初始化） */
static I2C_HandleTypeDef hi2c1_t;

/* ---------- 串口输出助手（仅用整数格式，兼容 nano specs） ---------- */
static void u_tx(const char *s){ HAL_UART_Transmit(&CLI_UART,(uint8_t*)s,(uint16_t)strlen(s),200); }
static void u_pf(const char *fmt, ...){
  char b[160]; va_list a; va_start(a,fmt); vsnprintf(b,sizeof b,fmt,a); va_end(a); u_tx(b);
}
static void result(const char *name, const char *verdict, const char *detail){
  u_pf("RESULT %s %s %s\r\n", name, verdict, detail ? detail : "");
}

/* ---------- 读取一行命令 ---------- */
static int read_line(char *buf, int max){
  int n=0; uint8_t c;
  for(;;){
    if(HAL_UART_Receive(&CLI_UART,&c,1,HAL_MAX_DELAY)==HAL_OK){
      if(c=='\r'||c=='\n'){ if(n>0){ buf[n]=0; return n; } else continue; }
      if(n<max-1) buf[n++]=c;
    }
  }
}

/* ---------- ADC 单通道读取 ---------- */
static uint32_t adc_read(uint32_t channel, uint32_t sample){
  ADC_ChannelConfTypeDef c={0};
  c.Channel=channel; c.Rank=1; c.SamplingTime=sample; c.Offset=0;
  (void)HAL_ADC_Stop(&hadc1);
  if(HAL_ADC_ConfigChannel(&hadc1,&c)!=HAL_OK) return 0xFFFFFFFF;
  if(HAL_ADC_Start(&hadc1)!=HAL_OK) return 0xFFFFFFFF;
  uint32_t v=0xFFFFFFFF;
  if(HAL_ADC_PollForConversion(&hadc1,50)==HAL_OK) v=HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);
  return v;
}

/* ---------- I2C1 初始化（板载 EEPROM，PB6/PB7 AF4，板上已有 4.7k 上拉） ---------- */
static uint8_t i2c1_ready=0;
static void i2c1_init(void){
  GPIO_InitTypeDef g={0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  g.Pin=GPIO_PIN_6|GPIO_PIN_7; g.Mode=GPIO_MODE_AF_OD; g.Pull=GPIO_NOPULL;
  g.Speed=GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate=GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB,&g);
  hi2c1_t.Instance=I2C1;
  hi2c1_t.Init.ClockSpeed=100000;
  hi2c1_t.Init.DutyCycle=I2C_DUTYCYCLE_2;
  hi2c1_t.Init.OwnAddress1=0;
  hi2c1_t.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
  hi2c1_t.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
  hi2c1_t.Init.OwnAddress2=0;
  hi2c1_t.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
  hi2c1_t.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
  i2c1_ready=(HAL_I2C_Init(&hi2c1_t)==HAL_OK);
}

#define EE_ADDR8   0xA0   /* M24C64, A0/A1/A2=GND */
#define EE_MEMADDR 0x0010

/* ---------- 各测试，返回 1=PASS ---------- */
static int test_imu(void){
  uint8_t ok=IMU_Init();
  char d[64];
  snprintf(d,sizeof d,"whoami=0x%02X(exp 0x70)", g_imu.whoami);
  if(ok && g_imu.whoami==0x70){ IMU_Task(); result("IMU","PASS",d); return 1; }
  result("IMU","FAIL",d); return 0;
}

static int test_eeprom(void){
  if(!i2c1_ready){ result("EEPROM","FAIL","i2c1 init failed"); return 0; }
  uint8_t wr[4], rd[4]={0};
  uint32_t t=HAL_GetTick();
  wr[0]=(uint8_t)t; wr[1]=(uint8_t)(t>>8); wr[2]=0x5A; wr[3]=0xA5;
  if(HAL_I2C_Mem_Write(&hi2c1_t,EE_ADDR8,EE_MEMADDR,I2C_MEMADD_SIZE_16BIT,wr,4,100)!=HAL_OK){
    result("EEPROM","FAIL","write nack"); return 0; }
  HAL_Delay(8); /* 写周期 */
  if(HAL_I2C_Mem_Read(&hi2c1_t,EE_ADDR8,EE_MEMADDR,I2C_MEMADD_SIZE_16BIT,rd,4,100)!=HAL_OK){
    result("EEPROM","FAIL","read nack"); return 0; }
  if(memcmp(wr,rd,4)==0){ result("EEPROM","PASS","write/readback match"); return 1; }
  char d[48]; snprintf(d,sizeof d,"mismatch %02X%02X%02X%02X",rd[0],rd[1],rd[2],rd[3]);
  result("EEPROM","FAIL",d); return 0;
}

static int test_vbus(void){
  uint32_t raw=adc_read(ADC_CHANNEL_11, ADC_SAMPLETIME_144CYCLES);
  if(raw==0xFFFFFFFF){ result("VBUS","FAIL","adc error"); return 0; }
  /* 引脚毫伏（3.3V 满量程 12bit）；分压比 (330k+18k)/18k = 19.33（已与 32V 实测标定一致） */
  uint32_t pin_mv=raw*3300u/4095u;
  uint32_t vin_mv=pin_mv*348u/18u;
  char d[64]; snprintf(d,sizeof d,"raw=%lu pin=%lumV Vin~%lu.%luV",
    (unsigned long)raw,(unsigned long)pin_mv,(unsigned long)(vin_mv/1000),(unsigned long)((vin_mv%1000)/100));
  if(vin_mv>=6000 && vin_mv<=52000){ result("VBUS","PASS",d); return 1; }
  /* 量程外只报值不判失败(供电可能本来就不同)，但**也不能算通过** ——
   * 从前两条分支都 return 1，而 cmd_runall 里是 `total++; pass+=test_vbus();`，
   * 于是 0V 和 60V 在 SUMMARY 里都记一个 pass。这一项从写下来那天起就没有
   * 失败的可能，而它每次都显示 OK，所有人都信。
   * 返回 -1 = INFO：既不计通过也不计失败，和 test_adc_report 的既有约定一致。 */
  result("VBUS","INFO",d); return -1;
}

static void test_adc_report(void){
  static const struct { const char*n; uint32_t ch; } chs[6]={
    {"IN8",ADC_CHANNEL_8},{"IN9",ADC_CHANNEL_9},{"IN12",ADC_CHANNEL_12},
    {"IN13",ADC_CHANNEL_13},{"IN14",ADC_CHANNEL_14},{"IN15",ADC_CHANNEL_15}};
  for(int i=0;i<6;i++){
    uint32_t raw=adc_read(chs[i].ch, ADC_SAMPLETIME_56CYCLES);
    uint32_t mv=(raw==0xFFFFFFFF)?0:raw*3300u/4095u;
    char d[48]; snprintf(d,sizeof d,"raw=%lu %lumV",(unsigned long)raw,(unsigned long)mv);
    char nm[16]; snprintf(nm,sizeof nm,"ADC_%s",chs[i].n);
    result(nm,"INFO",d);
  }
}

static void test_led(int on){
  HAL_GPIO_WritePin(OUTPUT_PE0_LED_GPIO_Port, OUTPUT_PE0_LED_Pin, on?GPIO_PIN_SET:GPIO_PIN_RESET);
  result("LED","INFO", on?"on (look)":"off");
}
static void test_buzz(uint32_t ms){
  HAL_GPIO_WritePin(OUTPUT_PE1_BUZZER_GPIO_Port, OUTPUT_PE1_BUZZER_Pin, GPIO_PIN_SET);
  HAL_Delay(ms);
  HAL_GPIO_WritePin(OUTPUT_PE1_BUZZER_GPIO_Port, OUTPUT_PE1_BUZZER_Pin, GPIO_PIN_RESET);
  result("BUZZER","INFO","beeped (listen)");
}

static int test_can_loopback(void){
  CAN_FilterTypeDef f={0};
  CAN_TxHeaderTypeDef th={0};
  CAN_RxHeaderTypeDef rh={0};
  uint8_t tx[8]={1,2,3,4,5,6,7,8}, rx[8]={0};
  uint32_t mbx;
  (void)HAL_CAN_Stop(&hcan1);
  (void)HAL_CAN_DeInit(&hcan1);
  hcan1.Init.Mode=CAN_MODE_LOOPBACK;
  if(HAL_CAN_Init(&hcan1)!=HAL_OK){ result("CAN","FAIL","init"); return 0; }
  f.FilterBank=0; f.FilterMode=CAN_FILTERMODE_IDMASK; f.FilterScale=CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh=0; f.FilterIdLow=0; f.FilterMaskIdHigh=0; f.FilterMaskIdLow=0;
  f.FilterFIFOAssignment=CAN_RX_FIFO0; f.FilterActivation=ENABLE;
  if(HAL_CAN_ConfigFilter(&hcan1,&f)!=HAL_OK){ result("CAN","FAIL","filter"); return 0; }
  if(HAL_CAN_Start(&hcan1)!=HAL_OK){ result("CAN","FAIL","start"); return 0; }
  th.StdId=0x123; th.IDE=CAN_ID_STD; th.RTR=CAN_RTR_DATA; th.DLC=8;
  if(HAL_CAN_AddTxMessage(&hcan1,&th,tx,&mbx)!=HAL_OK){ result("CAN","FAIL","tx"); return 0; }
  uint32_t t0=HAL_GetTick();
  while(HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0)==0){
    if(HAL_GetTick()-t0>20){ result("CAN","FAIL","no rx (loopback)"); return 0; }
  }
  if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&rh,rx)!=HAL_OK){ result("CAN","FAIL","rx get"); return 0; }
  if(rh.StdId==0x123 && memcmp(tx,rx,8)==0){ result("CAN","PASS","loopback id+data ok"); return 1; }
  result("CAN","FAIL","data mismatch"); return 0;
}

/* ================= 阶段 2：跳线自环 ================= */
static UART_HandleTypeDef huart3_t;
static SPI_HandleTypeDef  hspi2_t;
static I2C_HandleTypeDef  hi2c2_t;
static TIM_HandleTypeDef  htim4_t;
static uint8_t s2_ready=0;
static uint8_t i2c2_ok=0;

static void s2_init(void){
  GPIO_InitTypeDef g={0};
  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* USART3: PC10 TX / PC11 RX, AF7, 115200 */
  __HAL_RCC_USART3_CLK_ENABLE();
  g.Pin=GPIO_PIN_10|GPIO_PIN_11; g.Mode=GPIO_MODE_AF_PP; g.Pull=GPIO_NOPULL;
  g.Speed=GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate=GPIO_AF7_USART3; HAL_GPIO_Init(GPIOC,&g);
  huart3_t.Instance=USART3; huart3_t.Init.BaudRate=115200; huart3_t.Init.WordLength=UART_WORDLENGTH_8B;
  huart3_t.Init.StopBits=UART_STOPBITS_1; huart3_t.Init.Parity=UART_PARITY_NONE;
  huart3_t.Init.Mode=UART_MODE_TX_RX; huart3_t.Init.HwFlowCtl=UART_HWCONTROL_NONE;
  huart3_t.Init.OverSampling=UART_OVERSAMPLING_16; HAL_UART_Init(&huart3_t);

  /* SPI2 master: PB13 SCK / PB14 MISO / PB15 MOSI, AF5；NSS 软件 */
  __HAL_RCC_SPI2_CLK_ENABLE();
  g.Pin=GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15; g.Mode=GPIO_MODE_AF_PP; g.Pull=GPIO_NOPULL;
  g.Speed=GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate=GPIO_AF5_SPI2; HAL_GPIO_Init(GPIOB,&g);
  hspi2_t.Instance=SPI2; hspi2_t.Init.Mode=SPI_MODE_MASTER; hspi2_t.Init.Direction=SPI_DIRECTION_2LINES;
  hspi2_t.Init.DataSize=SPI_DATASIZE_8BIT; hspi2_t.Init.CLKPolarity=SPI_POLARITY_LOW;
  hspi2_t.Init.CLKPhase=SPI_PHASE_1EDGE; hspi2_t.Init.NSS=SPI_NSS_SOFT;
  hspi2_t.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_32; hspi2_t.Init.FirstBit=SPI_FIRSTBIT_MSB;
  hspi2_t.Init.TIMode=SPI_TIMODE_DISABLE; hspi2_t.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
  hspi2_t.Init.CRCPolynomial=10; HAL_SPI_Init(&hspi2_t);

  /* I2C2: PB10 SCL / PB11 SDA, AF4 OD（上拉靠跳线借 EEPROM 的 4.7k） */
  __HAL_RCC_I2C2_CLK_ENABLE();
  g.Pin=GPIO_PIN_10|GPIO_PIN_11; g.Mode=GPIO_MODE_AF_OD; g.Pull=GPIO_PULLUP; /* 内部上拉兜底 */
  g.Speed=GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate=GPIO_AF4_I2C2; HAL_GPIO_Init(GPIOB,&g);
  hi2c2_t.Instance=I2C2; hi2c2_t.Init.ClockSpeed=20000; hi2c2_t.Init.DutyCycle=I2C_DUTYCYCLE_2;
  hi2c2_t.Init.OwnAddress1=0; hi2c2_t.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
  hi2c2_t.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE; hi2c2_t.Init.OwnAddress2=0;
  hi2c2_t.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE; hi2c2_t.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
  i2c2_ok=(HAL_I2C_Init(&hi2c2_t)==HAL_OK);

  /* PWM 源：TIM1 CH1/2/3/4 = PA8/PE11/PE13/PE14 (AF1), 1kHz 50% */
  g.Pin=GPIO_PIN_8; g.Mode=GPIO_MODE_AF_PP; g.Pull=GPIO_NOPULL;
  g.Speed=GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate=GPIO_AF1_TIM1; HAL_GPIO_Init(GPIOA,&g);
  g.Pin=GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14; HAL_GPIO_Init(GPIOE,&g);
  __HAL_TIM_SET_PRESCALER(&htim1,168-1); __HAL_TIM_SET_AUTORELOAD(&htim1,1000-1);
  { TIM_OC_InitTypeDef oc={0}; oc.OCMode=TIM_OCMODE_PWM1; oc.Pulse=500;
    oc.OCPolarity=TIM_OCPOLARITY_HIGH; oc.OCNPolarity=TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode=TIM_OCFAST_DISABLE; oc.OCIdleState=TIM_OCIDLESTATE_RESET; oc.OCNIdleState=TIM_OCNIDLESTATE_RESET;
    for(uint32_t ch=TIM_CHANNEL_1; ch<=TIM_CHANNEL_4; ch+=4){
      HAL_TIM_PWM_ConfigChannel(&htim1,&oc,ch); HAL_TIM_PWM_Start(&htim1,ch); } }

  /* 捕获：TIM2 CH1-4=PA0/1/2/3(AF1); TIM3 CH1-4=PC6/7/8/9(AF2); TIM4 CH3/4=PD14/15(AF2); 1µs */
  g.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3; g.Mode=GPIO_MODE_AF_PP; g.Pull=GPIO_PULLDOWN;
  g.Speed=GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate=GPIO_AF1_TIM2; HAL_GPIO_Init(GPIOA,&g);
  g.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9; g.Alternate=GPIO_AF2_TIM3; HAL_GPIO_Init(GPIOC,&g);
  g.Pin=GPIO_PIN_14|GPIO_PIN_15; g.Alternate=GPIO_AF2_TIM4; HAL_GPIO_Init(GPIOD,&g);
  __HAL_RCC_TIM4_CLK_ENABLE();
  htim4_t.Instance=TIM4; htim4_t.Init.Prescaler=84-1; htim4_t.Init.CounterMode=TIM_COUNTERMODE_UP;
  htim4_t.Init.Period=0xFFFF; htim4_t.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  htim4_t.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE; HAL_TIM_IC_Init(&htim4_t);
  { TIM_IC_InitTypeDef ic={0}; ic.ICPolarity=TIM_ICPOLARITY_RISING; ic.ICSelection=TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler=TIM_ICPSC_DIV1; ic.ICFilter=0;
    __HAL_TIM_SET_PRESCALER(&htim2,84-1); __HAL_TIM_SET_AUTORELOAD(&htim2,0xFFFF);
    __HAL_TIM_SET_PRESCALER(&htim3,84-1); __HAL_TIM_SET_AUTORELOAD(&htim3,0xFFFF);
    for(uint32_t ch=TIM_CHANNEL_1; ch<=TIM_CHANNEL_4; ch+=4){
      HAL_TIM_IC_ConfigChannel(&htim2,&ic,ch); HAL_TIM_IC_Start(&htim2,ch);
      HAL_TIM_IC_ConfigChannel(&htim3,&ic,ch); HAL_TIM_IC_Start(&htim3,ch); }
    HAL_TIM_IC_ConfigChannel(&htim4_t,&ic,TIM_CHANNEL_3); HAL_TIM_IC_Start(&htim4_t,TIM_CHANNEL_3);
    HAL_TIM_IC_ConfigChannel(&htim4_t,&ic,TIM_CHANNEL_4); HAL_TIM_IC_Start(&htim4_t,TIM_CHANNEL_4); }
  s2_ready=1;
}

static void uart_flush(UART_HandleTypeDef*h){ uint8_t j; while(HAL_UART_Receive(h,&j,1,2)==HAL_OK){} }
static int test_uart_loop(UART_HandleTypeDef*h, const char*name){
  uint8_t tx[4]={0xA5,0x3C,0x5A,0xC3}, rx[4]={0}; int ok=1;
  uart_flush(h);
  for(int i=0;i<4;i++){
    if(HAL_UART_Transmit(h,&tx[i],1,30)!=HAL_OK){ ok=0; break; }
    if(HAL_UART_Receive(h,&rx[i],1,30)!=HAL_OK){ ok=0; break; }
  }
  if(ok && memcmp(tx,rx,4)==0){ result(name,"PASS","TX->RX loopback ok"); return 1; }
  char d[48]; snprintf(d,sizeof d, ok?"rx=%02X%02X%02X%02X":"no echo(接 TX-RX?)",rx[0],rx[1],rx[2],rx[3]);
  result(name,"FAIL",d); return 0;
}

static int test_spi2(void){
  uint8_t tx[4]={0xA5,0x3C,0x5A,0xC3}, rx[4]={0};
  if(HAL_SPI_TransmitReceive(&hspi2_t,tx,rx,4,100)!=HAL_OK){ result("SPI2","FAIL","xfer"); return 0; }
  if(memcmp(tx,rx,4)==0){ result("SPI2","PASS","MOSI->MISO ok"); return 1; }
  char d[48]; snprintf(d,sizeof d,"rx=%02X%02X%02X%02X(接 PB15-PB14?)",rx[0],rx[1],rx[2],rx[3]);
  result("SPI2","FAIL",d); return 0;
}

/* 读 I2C2(PB10/PB11) 空闲电平诊断：带内部上拉。1,1=线正常空闲；出现 0=被拉死/短路/未供电 */
static void i2c2_lines(void){
  GPIO_InitTypeDef g={0}; g.Pin=GPIO_PIN_10|GPIO_PIN_11; g.Mode=GPIO_MODE_INPUT;
  /* NOPULL：只有外部 10k 上拉才会读到高。读到低=PB10/PB11 上没有外部上拉/器件 */
  g.Pull=GPIO_NOPULL; HAL_GPIO_Init(GPIOB,&g); HAL_Delay(1);
  int scl=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10), sda=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11);
  g.Mode=GPIO_MODE_AF_OD; g.Pull=GPIO_PULLUP; g.Speed=GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate=GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB,&g);
  /* 96 而不是 56：这串中文是 UTF-8，一个字三字节，56 装不下，编译器一直在
   * 报 -Wformat-truncation。截断掉的正好是"0 是什么意思"那半句 ——
   * 一条诊断信息被砍掉的，恰恰是它存在的理由。 */
  char d[96]; snprintf(d,sizeof d,"NOPULL SCL=%d SDA=%d (1=有外部上拉, 0=PB10/11无外部上拉)",scl,sda);
  result("I2C2_LINES","INFO",d);
}

/* I2C2 总线扫描：CN33 外接任意 I²C 从机（带上拉），扫到地址即 PASS。 */
static int test_i2c2(void){
  i2c2_lines();
  { char d[40]; snprintf(d,sizeof d,"i2c2 init=%s", i2c2_ok?"OK":"FAIL"); result("I2C2_INIT","INFO",d); }
  int found=0, n=0; char list[80]; list[0]=0;
  for(uint8_t a=0x08;a<=0x77;a++){
    if(HAL_I2C_IsDeviceReady(&hi2c2_t,(uint16_t)(a<<1),3,20)==HAL_OK){
      found++; if(n<70) n+=snprintf(list+n,(size_t)(sizeof(list)-n)," 0x%02X",a);
    }
  }
  if(found){ char d[96]; snprintf(d,sizeof d,"found %d:%s",found,list); result("I2C2","PASS",d); return 1; }
  result("I2C2","FAIL","no device (CN33 接 I2C 从机, SDA/SCL 需上拉)"); return 0;
}

static int test_gpio_loop(void){
  static const struct { GPIO_TypeDef*op; uint16_t opin; GPIO_TypeDef*ip; uint16_t ipin; const char*nm; } m[10]={
    {GPIOE,GPIO_PIN_2, GPIOD,GPIO_PIN_0, "PE2-PD0"},
    {GPIOE,GPIO_PIN_3, GPIOD,GPIO_PIN_1, "PE3-PD1"},
    {GPIOE,GPIO_PIN_4, GPIOD,GPIO_PIN_3, "PE4-PD3"},
    {GPIOE,GPIO_PIN_7, GPIOD,GPIO_PIN_4, "PE7-PD4"},
    {GPIOE,GPIO_PIN_8, GPIOD,GPIO_PIN_8, "PE8-PD8"},
    {GPIOE,GPIO_PIN_9, GPIOD,GPIO_PIN_9, "PE9-PD9"},
    {GPIOE,GPIO_PIN_10,GPIOD,GPIO_PIN_10,"PE10-PD10"},
    {GPIOE,GPIO_PIN_12,GPIOD,GPIO_PIN_11,"PE12-PD11"},
    {GPIOE,GPIO_PIN_15,GPIOD,GPIO_PIN_12,"PE15-PD12"},
    {GPIOE,GPIO_PIN_15,GPIOD,GPIO_PIN_13,"PE15-PD13"},  /* 第10输入,由PE15驱动(也接CN15) */
  };
  /* 主程序 MX_GPIO_Init 的 USER CODE 把 PE2/PE3 改成了上拉输入（刹车开关用），
   * 这里把 9 个输出脚统一重配为推挽输出，否则 PE2/PE3 不驱动 → PD0/PD1 恒高。 */
  { GPIO_InitTypeDef g={0};
    g.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_15;
    g.Mode=GPIO_MODE_OUTPUT_PP; g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE,&g); }
  int passn=0;
  for(int i=0;i<10;i++){
    HAL_GPIO_WritePin(m[i].op,m[i].opin,GPIO_PIN_SET);   HAL_Delay(3);
    int hi=(HAL_GPIO_ReadPin(m[i].ip,m[i].ipin)==GPIO_PIN_SET);
    HAL_GPIO_WritePin(m[i].op,m[i].opin,GPIO_PIN_RESET); HAL_Delay(3);
    int lo=(HAL_GPIO_ReadPin(m[i].ip,m[i].ipin)==GPIO_PIN_RESET);
    if(hi&&lo){ passn++; result(m[i].nm,"PASS",""); }
    else result(m[i].nm,"FAIL", hi?"stuck-hi": (lo?"stuck-lo":"no link(接跳线?)"));
  }
  return passn;
}

static uint32_t cc_flag(uint32_t ch){
  return ch==TIM_CHANNEL_1?TIM_FLAG_CC1: ch==TIM_CHANNEL_2?TIM_FLAG_CC2:
         ch==TIM_CHANNEL_3?TIM_FLAG_CC3: TIM_FLAG_CC4;
}
static int test_pwmcap(TIM_HandleTypeDef*ht, uint32_t ch, const char*nm){
  uint32_t fl=cc_flag(ch);
  __HAL_TIM_CLEAR_FLAG(ht,fl);
  uint32_t t0=HAL_GetTick(), c1=0,c2=0; int got=0;
  while(HAL_GetTick()-t0<60){
    if(__HAL_TIM_GET_FLAG(ht,fl)){
      uint32_t v=HAL_TIM_ReadCapturedValue(ht,ch); __HAL_TIM_CLEAR_FLAG(ht,fl);
      if(got==0){ c1=v; got=1; } else { c2=v; got=2; break; }
    }
  }
  if(got<2){ result(nm,"FAIL","no edges(接 PWM->捕获?)"); return 0; }
  uint32_t us=(c2-c1)&0xFFFF;
  char d[48]; snprintf(d,sizeof d,"period=%luus(exp~1000)",(unsigned long)us);
  if(us>=850&&us<=1150){ result(nm,"PASS",d); return 1; }
  result(nm,"FAIL",d); return 0;
}

static void print_jumper_map(void){
  u_tx("JUMPER MAP (阶段2):\r\n");
  u_tx(" SPI2 : PB15-PB14\r\n");
  u_tx(" UART2: PD5-PD6(CN35)   UART3: PC10-PC11(CN36)\r\n");
  u_tx(" UART5/USB2: 无排针，改插 USB2 跑 u5echo 校验\r\n");
  u_tx(" I2C2 : CN33 外接 I2C 从机(SCL/SDA/3V3/GND, 带上拉)\r\n");
  u_tx(" DO-DI: PE2-PD0 PE3-PD1 PE4-PD3 PE7-PD4 PE8-PD8 PE9-PD9 PE10-PD10 PE12-PD11 PE15-PD12\r\n");
  u_tx(" PWM->CAP: PA8-PA0  PE11-PC6  PE13-PD14\r\n");
}

/* UART5/USB2 回环：在 UART5 上回显一段时间；上位机从 USB2 口发数据校验。 */
static void cmd_u5echo(uint32_t ms){
  if(!s2_ready) s2_init();
  u_pf("U5ECHO START %lums\r\n",(unsigned long)ms);
  uint32_t t0=HAL_GetTick(); uint8_t c;
  while(HAL_GetTick()-t0<ms){
    if(HAL_UART_Receive(&huart5,&c,1,10)==HAL_OK) HAL_UART_Transmit(&huart5,&c,1,10);
  }
  u_tx("U5ECHO END\r\n");
}

/* ---- 软件 I²C 位操作：诊断 CN33 的 SCL/SDA 是否在板内对调 ---- */
static uint16_t BB_SCL, BB_SDA;
static void bb_dly(void){ for(volatile int i=0;i<2000;i++){ __NOP(); } } /* 放慢: 容忍弱上拉/长线慢上升沿 */
static void bb_od(uint16_t pin){ GPIO_InitTypeDef g={0}; g.Pin=pin; g.Mode=GPIO_MODE_OUTPUT_OD;
  g.Pull=GPIO_PULLUP; g.Speed=GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOB,&g); }
static void bb_init(uint16_t scl,uint16_t sda){ BB_SCL=scl; BB_SDA=sda; bb_od(scl); bb_od(sda);
  HAL_GPIO_WritePin(GPIOB,scl,GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOB,sda,GPIO_PIN_SET); bb_dly(); }
static void SCL(int v){ HAL_GPIO_WritePin(GPIOB,BB_SCL,v?GPIO_PIN_SET:GPIO_PIN_RESET); bb_dly(); }
static void SDA(int v){ HAL_GPIO_WritePin(GPIOB,BB_SDA,v?GPIO_PIN_SET:GPIO_PIN_RESET); bb_dly(); }
static int  SDAr(void){ return HAL_GPIO_ReadPin(GPIOB,BB_SDA)==GPIO_PIN_SET; }
static int bb_probe(uint8_t a7){
  SDA(1); SCL(1); SDA(0); SCL(0);                 /* START */
  uint8_t b=(uint8_t)((a7<<1)|0);
  for(int i=0;i<8;i++){ SDA(b&0x80); b<<=1; SCL(1); SCL(0); }
  SDA(1); SCL(1); int ack=(SDAr()==0); SCL(0);    /* read ACK */
  SDA(0); SCL(1); SDA(1);                          /* STOP */
  return ack;
}
static int bb_scan(const char*tag){
  int found=0,n=0; char list[60]; list[0]=0;
  for(uint8_t a=0x08;a<=0x77;a++){ if(bb_probe(a)){ found++; if(n<50)n+=snprintf(list+n,(size_t)(sizeof(list)-n)," 0x%02X",a); } }
  char d[96]; snprintf(d,sizeof d,"%s -> found %d:%s",tag,found,list);
  result("I2CBB", found?"PASS":"FAIL", d);
  return found;
}
static void cmd_i2cbb(void){
  /* 先测 PB10/PB11 能否下拉（OD 输出驱低应读 0，释放经上拉应读 1） */
  bb_od(GPIO_PIN_10); bb_od(GPIO_PIN_11);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET); HAL_Delay(1);
  int b10lo=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10), b11lo=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET); HAL_Delay(1);
  int b10hi=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10), b11hi=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11);
  { char d[64]; snprintf(d,sizeof d,"PB10 lo=%d hi=%d  PB11 lo=%d hi=%d (期望 0/1)",b10lo,b10hi,b11lo,b11hi);
    result("I2C2_DRIVE","INFO",d); }
  bb_init(GPIO_PIN_10,GPIO_PIN_11); bb_scan("PB10=SCL,PB11=SDA(正向)");
  bb_init(GPIO_PIN_11,GPIO_PIN_10); bb_scan("PB10=SDA,PB11=SCL(对调)");
  /* 还原 PB10/PB11 为 I2C2 AF */
  GPIO_InitTypeDef g={0}; g.Pin=GPIO_PIN_10|GPIO_PIN_11; g.Mode=GPIO_MODE_AF_OD; g.Pull=GPIO_PULLUP;
  g.Speed=GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate=GPIO_AF4_I2C2; HAL_GPIO_Init(GPIOB,&g);
}

/* 连通性保持：把 PB10(SCL)/PB11(SDA) 拉到指定电平并保持，便于在屏脚上量电压判断断点。
 * i2chold 1 = SCL(PB10)拉低; 2 = SDA(PB11)拉低; 0 = 两者释放(高)。 */
static void cmd_i2chold(int n){
  bb_od(GPIO_PIN_10); bb_od(GPIO_PIN_11);
  int b10=(n==1)?0:1, b11=(n==2)?0:1;
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,b10?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,b11?GPIO_PIN_SET:GPIO_PIN_RESET);
  char d[64]; snprintf(d,sizeof d,"PB10/SCL=%d PB11/SDA=%d 保持中, 去屏脚量电压", b10,b11);
  result("I2C2_HOLD","INFO",d);
}

/* 全部捕获通道：TIM2 CH1-4 / TIM3 CH1-4 / TIM4 CH3-4，返回通过数(共10)。 */
static int pwmcap_all(void){
  int p=0;
  p+=test_pwmcap(&htim2,TIM_CHANNEL_1,"CAP_TIM2_CH1(PA0)");
  p+=test_pwmcap(&htim2,TIM_CHANNEL_2,"CAP_TIM2_CH2(PA1)");
  p+=test_pwmcap(&htim2,TIM_CHANNEL_3,"CAP_TIM2_CH3(PA2)");
  p+=test_pwmcap(&htim2,TIM_CHANNEL_4,"CAP_TIM2_CH4(PA3)");
  p+=test_pwmcap(&htim3,TIM_CHANNEL_1,"CAP_TIM3_CH1(PC6)");
  p+=test_pwmcap(&htim3,TIM_CHANNEL_2,"CAP_TIM3_CH2(PC7)");
  p+=test_pwmcap(&htim3,TIM_CHANNEL_3,"CAP_TIM3_CH3(PC8)");
  p+=test_pwmcap(&htim3,TIM_CHANNEL_4,"CAP_TIM3_CH4(PC9)");
  p+=test_pwmcap(&htim4_t,TIM_CHANNEL_3,"CAP_TIM4_CH3(PD14)");
  p+=test_pwmcap(&htim4_t,TIM_CHANNEL_4,"CAP_TIM4_CH4(PD15)");
  return p;
}

/* PC13 用户按键：外部上拉，按下=低。5s 窗口检测一次按下。 */
static void cmd_btn(void){
  GPIO_InitTypeDef g={0}; g.Pin=GPIO_PIN_13; g.Mode=GPIO_MODE_INPUT; g.Pull=GPIO_PULLUP;
  __HAL_RCC_GPIOC_CLK_ENABLE(); HAL_GPIO_Init(GPIOC,&g);
  u_tx("请在 5s 内按下 PC13 用户键...\r\n");
  uint32_t t0=HAL_GetTick(); int pressed=0;
  while(HAL_GetTick()-t0<5000){ if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==GPIO_PIN_RESET){ pressed=1; break; } }
  result("BUTTON_PC13", pressed?"PASS":"FAIL", pressed?"检测到按下":"5s 内未检测到");
}

static void cmd_loopback(void){
  if(!s2_ready) s2_init();
  int pass=0,total=0;
  total++; pass+=test_spi2();
  total++; pass+=test_uart_loop(&huart2,"UART2");
  total++; pass+=test_uart_loop(&huart3_t,"UART3");
  total++; pass+=test_i2c2();
  { int p=test_gpio_loop(); pass+=p; total+=10; }
  { int p=pwmcap_all(); pass+=p; total+=10; }
  u_pf("SUMMARY-LOOPBACK %d/%d\r\n", pass, total);
}

static void cmd_runall(void){
  int pass=0, total=0;
  total++; pass+=test_imu();
  total++; pass+=test_eeprom();
  { int v=test_vbus();           /* -1 = INFO：报了值但不构成判据，两边都不计 */
    if(v>=0){ total++; pass+=v; } }
  test_adc_report();             /* 仅报值，不计入 pass/total */
  total++; pass+=test_can_loopback();
  test_led(1); HAL_Delay(300); test_led(0);
  test_buzz(200);
  u_pf("SUMMARY %d/%d auto-pass\r\n", pass, total);
  cmd_loopback();   /* 阶段 2：跳线自环（未接跳线的项会 FAIL，属正常） */
  u_tx("DONE\r\n");
}

static void help(void){
  u_tx("DlaBoard_B SELFTEST\r\n"
       "stage1(板载): imu eeprom vbus adc led<0|1> buzz[ms] can\r\n"
       "stage2(跳线): spi2 uart2 uart3 i2c2 gpio pwmcap btn loopback  map=看跳线表\r\n"
       "USB2/UART5: u5echo (插 USB2，上位机自动从 USB2 校验)\r\n"
       "runall = 全部(板载+跳线)\r\n");
}

void SelfTest_Run(void){
  i2c1_init();
  HAL_Delay(50);
  u_tx("\r\n=== DLABOARD_B SELFTEST READY ===\r\n");
  help();
  char line[64];
  for(;;){
    read_line(line,sizeof line);
    if(!strcmp(line,"help")) help();
    else if(!strcmp(line,"imu")) test_imu();
    else if(!strcmp(line,"eeprom")) test_eeprom();
    else if(!strcmp(line,"vbus")) test_vbus();
    else if(!strcmp(line,"adc")) test_adc_report();
    else if(!strncmp(line,"led",3)) test_led(atoi(line+3));
    else if(!strncmp(line,"buzz",4)){ int ms=atoi(line+4); test_buzz(ms>0?(uint32_t)ms:200); }
    else if(!strcmp(line,"can")) test_can_loopback();
    else if(!strcmp(line,"map")) print_jumper_map();
    else if(!strcmp(line,"spi2")){ if(!s2_ready)s2_init(); test_spi2(); }
    else if(!strcmp(line,"uart2")){ if(!s2_ready)s2_init(); test_uart_loop(&huart2,"UART2"); }
    else if(!strcmp(line,"uart3")){ if(!s2_ready)s2_init(); test_uart_loop(&huart3_t,"UART3"); }
    else if(!strncmp(line,"u5echo",6)){ int ms=atoi(line+6); cmd_u5echo(ms>0?(uint32_t)ms:6000); }
    else if(!strcmp(line,"i2c2")){ if(!s2_ready)s2_init(); test_i2c2(); }
    else if(!strcmp(line,"i2cbb")){ if(!s2_ready)s2_init(); cmd_i2cbb(); }
    else if(!strncmp(line,"i2chold",7)){ cmd_i2chold(atoi(line+7)); }
    else if(!strcmp(line,"i2cbb1")){ bb_init(GPIO_PIN_6,GPIO_PIN_7); bb_scan("PB6=SCL,PB7=SDA(I2C1/EEPROM,验证位操作)"); }
    else if(!strcmp(line,"gpio")){ if(!s2_ready)s2_init(); test_gpio_loop(); }
    else if(!strcmp(line,"pwmcap")){ if(!s2_ready)s2_init(); pwmcap_all(); }
    else if(!strcmp(line,"btn")) cmd_btn();
    else if(!strcmp(line,"loopback")) cmd_loopback();
    else if(!strcmp(line,"runall")) cmd_runall();
    else u_pf("ERR unknown: %s\r\n", line);
  }
}
