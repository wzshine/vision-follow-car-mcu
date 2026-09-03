/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "train_uart.h"
#include "screen.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "foc_can.h"

/*
 * 复位原因的痕迹。放 .noinit ⇒ 复位不清零，而上电（真的断电）会是随机值，
 * 所以用一个魔数判断"这一位是不是我们写的"。
 * ⚠ 它回答的问题是 RCC->CSR 回答不了的那个：软件复位**为什么**发生。
 */
uint32_t g_hardfault_mark __attribute__((section(".noinit")));

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /*
   * ⚠⚠ 这里原来是一个**裸 while(1)**，而接入标准 DBIS §9.1 S2 明文禁止它：
   *
   *   「HardFault_Handler 首行必须强制关断执行器，然后复位并记录复位原因。
   *     **不得是裸 while(1)**（MCU 挂掉时输出保持最后状态）」
   *
   * 裸循环在这块板上的实际后果：
   *   - CPU 停了，但**继电器和数字输出保持最后状态**；
   *   - 0x210 停发 ⇒ 驱动板要等它自己的 300ms 超时才停；
   *   - 看门狗要等 wdg_timeout_nom（默认 2000ms）才咬。
   *   ⇒ 最坏情况下，一台车按最后一条指令继续走**两秒**，
   *     而板子从外面看只是"没反应"。
   *
   * 现在做三件，按这个顺序：
   *   1. 闩死急停 —— 万一在复位真的发生之前还有任何一帧发出去，它是安全值；
   *   2. 在 noinit 段留下痕迹 —— 复位会清掉 RCC 的故障标志，而
   *      NVIC_SystemReset() 之后 RCC->CSR 只会说"软件复位"，
   *      说不出**为什么**软件要复位自己；
   *   3. 立刻复位。复位后板子回到上电安全态（ven=0 / P 档 ⇒ 断言刹车）。
   *
   * ⚠ 这里**不调用任何 HAL 的发送函数**：HardFault 之后处理器状态不可信，
   *   一个会阻塞、会等外设标志的调用可能永远回不来 —— 那就又变成 while(1) 了。
   */
  g_hardfault_mark = HARDFAULT_MARK_MAGIC;
  FocCAN_Estop();
  __DSB();
  NVIC_SystemReset();
  /* NVIC_SystemReset() 不返回。留一个循环只是为了让编译器闭嘴。 */
  while (1) { }
  /* USER CODE END HardFault_IRQn 0 */
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
/**
  * @brief  UART5 全局中断 —— Jetson 那根线（USB2）。
  *
  * ⚠ 不走 HAL_UART_IRQHandler：和 USART1 一样，这里只把字节搬进环形缓冲。
  *   HAL 的那套要先 Receive_IT 起一个事务，收满才回调，而这是一条
  *   长度不定、两种协议混跑的流。
  */
void UART5_IRQHandler(void)
{
  TrainUART_JetsonISR();
}


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

void USART1_IRQHandler(void)
{
  TrainUART_CliISR();
}

void USART2_IRQHandler(void)
{
  Screen_ReceiveISR();
  HAL_UART_IRQHandler(&huart2);
}

/* USER CODE END 1 */
