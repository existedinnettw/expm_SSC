/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <SSC-Device.h>    //SSC-DeviceObjects.h --> ApplicationObjDic
#include <applInterface.h> //pAPPL_MainLoop
#include <ecatappl.h>
// aware that cubeMX not support CPP generation (only cubeIDE does)
// https://community.st.com/t5/stm32cubemx-mcus/how-to-configure-stm32cubemx-to-support-c-development/td-p/109965

#include <stdint.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void
SystemClock_Config(void);
static void
MX_GPIO_Init(void);
static void
MX_RTC_Init(void);
static void
MX_SPI1_Init(void);
static void
MX_USART1_UART_Init(void);
static void
MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/**
 * @see
 * https://github.com/STMicroelectronics/STM32CubeF4/blob/master/Projects/STM32F410xx-Nucleo/Examples/UART/UART_Printf/Src/main.c
 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE* f)
#endif /* __GNUC__ */

#define UART_PRINTF FALSE

#if UART_PRINTF == TRUE
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 * @attention This function is severely slow(>1ms), and slow down ISR
 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  /* Add carriage return before newline for proper terminal display */
  if (ch == '\n') {
    uint8_t cr = '\r';
    HAL_UART_Transmit(&huart1, &cr, 1, 0xFFFF);
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);

  return ch;
}
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  UNUSED(GPIO_Pin);
  // if inited?
  printf("IO value: %d\n", HAL_GPIO_ReadPin(ESC_SPI_IRQ_GPIO_Port, ESC_SPI_IRQ_Pin));

#if INTERRUPTS_SUPPORTED == 1
  DISABLE_ESC_INT();
#endif

  printf("EXTI CB: 0x%x\n", GPIO_Pin);
#if AL_EVENT_ENABLED == 1
  // read gpio pin state
  if (HAL_GPIO_ReadPin(ESC_SPI_IRQ_GPIO_Port, ESC_SPI_IRQ_Pin) == GPIO_PIN_RESET) {
    printf("PDI trig\n");
    PDI_Isr();
  }
#endif

#if DC_SUPPORTED == 1
  if (HAL_GPIO_ReadPin(ESC_SYNC0_GPIO_Port, ESC_SYNC0_Pin) == GPIO_PIN_RESET) {
    printf("Sync0 trig\n");
    Sync0_Isr();
  }
#endif

#if DC_SUPPORTED == 1
  if (HAL_GPIO_ReadPin(ESC_SYNC1_GPIO_Port, ESC_SYNC1_Pin) == GPIO_PIN_RESET) {
    printf("Sync1 trig\n");
    Sync1_Isr();
  }
#endif
  // printf("Unknown GPIO_EXTI Callback: 0x%x\n", GPIO_Pin);

#if INTERRUPTS_SUPPORTED == 1
  ENABLE_ESC_INT();
#endif
}

void
ssc_appl_application_impl()
{
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int
main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  SSC_appl_singleton_instance.APPL_Application = ssc_appl_application_impl;
  pAPPL_MainLoop = NULL;

  {
    uint8_t rst = HW_Init();
    if (rst != 0) {
      printf("HW_Init failed with code: %d\n", rst);
      return rst; // exit if HW_Init failed
    }
  }

  MainInit(); // COE_ObjDictionaryInit
  // al mask had reset in MainInit

  printf("start\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bRunApplication = TRUE;
  uint32_t loop_timer = 0;
  while (1) {
    // this busy loop will break zephyr scheduling
    MainLoop();

    // k_sleep(K_USEC(100)); // this is a simple workaround to avoid busy loop
    loop_timer++;
    // if (loop_timer >= 10000) { // 1s
    //   loop_timer = 0;
    //   UINT16 al_req_ev = HW_GetALEventRegister();
    //   printf("ESC ALEvent register: 0x%x\n", al_req_ev);
    //   // check if al event is existed, check if al event mask existed
    //   // check if SSC consider it's error (by led)
    //   UINT32 intMask = 0;
    //   HW_EscReadDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
    //   printf("ESC ALEvent mask: 0x%lx\n", intMask);
    // }

    if (bRunApplication == FALSE) {
      break;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  HW_Release();
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void
SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void
MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
   */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
  TIM_MasterConfigTypeDef sMasterConfig = { 0 };

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void
MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led0_GPIO_Port, led0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BOOT1_Pin | SPI1_SS_ESC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : K1_Pin K0_Pin */
  GPIO_InitStruct.Pin = K1_Pin | K0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : K_UP_Pin */
  GPIO_InitStruct.Pin = K_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(K_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : led0_Pin */
  GPIO_InitStruct.Pin = led0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ESC_SPI_IRQ_Pin ESC_SYNC0_Pin ESC_SYNC1_Pin */
  GPIO_InitStruct.Pin = ESC_SPI_IRQ_Pin | ESC_SYNC0_Pin | ESC_SYNC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : ESC_EEP_LOAD_Pin */
  GPIO_InitStruct.Pin = ESC_EEP_LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ESC_EEP_LOAD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_SS_ESC_Pin */
  GPIO_InitStruct.Pin = SPI1_SS_ESC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_SS_ESC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  // enable later
  HAL_NVIC_DisableIRQ(ESC_SPI_IRQ_EXTI_IRQn);
  HAL_NVIC_DisableIRQ(ESC_SYNC0_EXTI_IRQn);
  HAL_NVIC_DisableIRQ(ESC_SYNC1_EXTI_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void
Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  printf("Error_Handler\n");
  while (1) {
    HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void
assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
