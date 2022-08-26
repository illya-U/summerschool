/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "baro.h"
#include "lcd.h"
#include "wifi_at.h"
#include "wifi_creds.h"
#include "streams.h"
#include <stdio.h>	// snprintf
#include <string.h>	// memset
#include <math.h>	// sin
#include <stdlib.h>	// rand

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define countof(_a)	(sizeof(_a) / sizeof(_a[0]))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for taskWiFi */
osThreadId_t taskWiFiHandle;
const osThreadAttr_t taskWiFi_attributes = {
  .name = "taskWiFi",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskBaro */
osThreadId_t taskBaroHandle;
const osThreadAttr_t taskBaro_attributes = {
  .name = "taskBaro",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for taskDefault */
osThreadId_t taskDefaultHandle;
const osThreadAttr_t taskDefault_attributes = {
  .name = "taskDefault",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for taskWiFiListen */
osThreadId_t taskWiFiListenHandle;
const osThreadAttr_t taskWiFiListen_attributes = {
  .name = "taskWiFiListen",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for muxUART_LOG */
osMutexId_t muxUART_LOGHandle;
const osMutexAttr_t muxUART_LOG_attributes = {
  .name = "muxUART_LOG"
};
/* Definitions for muxUART_WIFI */
osMutexId_t muxUART_WIFIHandle;
const osMutexAttr_t muxUART_WIFI_attributes = {
  .name = "muxUART_WIFI"
};
/* Definitions for muxLCD */
osMutexId_t muxLCDHandle;
const osMutexAttr_t muxLCD_attributes = {
  .name = "muxLCD"
};
/* Definitions for semUART_TX_WIFI */
osSemaphoreId_t semUART_TX_WIFIHandle;
const osSemaphoreAttr_t semUART_TX_WIFI_attributes = {
  .name = "semUART_TX_WIFI"
};
/* Definitions for semUART_RX_WIFI */
osSemaphoreId_t semUART_RX_WIFIHandle;
const osSemaphoreAttr_t semUART_RX_WIFI_attributes = {
  .name = "semUART_RX_WIFI"
};
/* USER CODE BEGIN PV */
volatile uint32_t pressure;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
void StartTaskWiFi(void *argument);
void StartTaskBaro(void *argument);
void StartTaskDefault(void *argument);
void StartTaskWiFiListen(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char text[100];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of muxUART_LOG */
  muxUART_LOGHandle = osMutexNew(&muxUART_LOG_attributes);

  /* creation of muxUART_WIFI */
  muxUART_WIFIHandle = osMutexNew(&muxUART_WIFI_attributes);

  /* creation of muxLCD */
  muxLCDHandle = osMutexNew(&muxLCD_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semUART_TX_WIFI */
  semUART_TX_WIFIHandle = osSemaphoreNew(1, 1, &semUART_TX_WIFI_attributes);

  /* creation of semUART_RX_WIFI */
  semUART_RX_WIFIHandle = osSemaphoreNew(1, 1, &semUART_RX_WIFI_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of taskWiFi */
  taskWiFiHandle = osThreadNew(StartTaskWiFi, NULL, &taskWiFi_attributes);

  /* creation of taskBaro */
  taskBaroHandle = osThreadNew(StartTaskBaro, NULL, &taskBaro_attributes);

  /* creation of taskDefault */
  taskDefaultHandle = osThreadNew(StartTaskDefault, NULL, &taskDefault_attributes);

  /* creation of taskWiFiListen */
  taskWiFiListenHandle = osThreadNew(StartTaskWiFiListen, NULL, &taskWiFiListen_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  streams_init();

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_A0_Pin|LCD_RESET_Pin|LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_A0_Pin LCD_RESET_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_A0_Pin|LCD_RESET_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskWiFi */
/**
  * @brief  Function implementing the taskLEDBlink thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskWiFi */
void StartTaskWiFi(void *argument)
{
  /* USER CODE BEGIN 5 */
  // Wait for WiFi init
  osThreadFlagsWait(FLAG_WIFI_RDY, osFlagsWaitAll, osWaitForever);

  // And try to connect to AP
  printf("Try to connect to '%s'... ", wifi_ap);
  wifi_status_t st = wifi_connect_to_ap(wifi_ap, wifi_pass, pdMS_TO_TICKS(30000));
  if (st) {
    if (st == WIFI_ERR_TMT)
      printf("timeout\n");
    else if (st == WIFI_ERR_NO_AP)
      printf("no AP\n");
    else if (st == WIFI_ERR_NO_IP)
      printf("didn't get IP\n");
    else
      printf("ERROR\n");
    while (1) {
      osDelay(10);
    }
  }
  printf("OK\n");
  lcd_set_text_size(2, 2);
  lcd_set_cursor(4, 4);
  lcd_print(wifi_ap);

  uint32_t ip[4], mac[6];
  if (wifi_get_own_ip(ip, mac) == WIFI_OK) {
    printf("IP: %lu.%lu.%lu.%lu, MAC: %02lX:%02lX:%02lX:%02lX:%02lX:%02lX\n",
        ip[0], ip[1], ip[2], ip[3],
        mac[0], mac[1], mac[2],
        mac[3], mac[4], mac[5]);
    sprintf(text, "%ld.%ld.%ld.%ld", ip[0], ip[1], ip[2], ip[3]);
    lcd_set_text_size(2, 2);
    lcd_set_cursor(3, 2);
    lcd_print(text);
  } else {
    sprintf(text, "none");
    lcd_set_text_size(2, 2);
    lcd_set_cursor(3, 2);
    lcd_print(text);
  }

  wifi_server(1, 80);
  lcd_set_text_size(2, 2);
  lcd_set_cursor(0, 5);
  lcd_print("SRV START");

  /* Infinite loop */
  for(;;) {
    osDelay(pdMS_TO_TICKS(1000));
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskBaro */
/**
* @brief Function implementing the taskBaro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskBaro */
void StartTaskBaro(void *argument)
{
  /* USER CODE BEGIN StartTaskBaro */
	  osThreadFlagsWait(FLAG_BARO_RDY, osFlagsNoClear | osFlagsWaitAny, osWaitForever);
	  lcd_set_text_size(1, 1);
	  lcd_set_cursor(0, 12);
	  lcd_print("Pressure:");

	  /* Infinite loop */
	  for(;;) {
	    pressure = baro_read_press();
	    sprintf(text, "%ld.%02ld hPa", pressure/100, pressure%100);
	    lcd_set_text_size(1, 1);
	    lcd_set_cursor(10, 12);
	    lcd_print(text);
	    osDelay(pdMS_TO_TICKS(1000));
	  }
  /* USER CODE END StartTaskBaro */
}

/* USER CODE BEGIN Header_StartTaskDefault */
/**
* @brief Function implementing the taskDefault thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskDefault */
void StartTaskDefault(void *argument)
{
  /* USER CODE BEGIN StartTaskDefault */
  printf("\nInit LCD... ");
  lcd_init();
  lcd_fill(ST7735_BLUE);
  lcd_set_text_bg_color(ST7735_BLUE);
  lcd_set_text_color(ST7735_YELLOW);
  lcd_set_text_size(2, 2);
  lcd_set_cursor(0, 0);
  lcd_print("Baro:");
  lcd_set_cursor(0, 1);
  lcd_print("WiFi:");
  lcd_set_cursor(0, 2);
  lcd_print("IP:");
  lcd_set_cursor(0, 4);
  lcd_print("AP:");
  printf("OK\n");

  printf("Init Barometer... ");
  baro_stat_t b_st = baro_init();
  lcd_set_text_size(2, 2);
  lcd_set_cursor(6, 0);
  if (b_st) {
    lcd_print("FAIL");
    printf("ERROR\n");
  } else {
    lcd_print("OK");
    printf("OK\n");
    osThreadFlagsSet(taskBaroHandle, FLAG_BARO_RDY);
  }

  printf("Init WIFI... ");
  wifi_status_t w_st = wifi_init();
  lcd_set_text_size(2, 2);
  lcd_set_cursor(6, 1);
  if (w_st) {
    lcd_print("FAIL");
    if (w_st == WIFI_ERR_TMT)
      printf("timeout\n");
    else
      printf("ERROR\n");

  } else {
    printf("OK\n");
    lcd_print("OK");
    osThreadFlagsSet(taskWiFiHandle, FLAG_WIFI_RDY);
  }

  /* Infinite loop */
  for(;;) {
    osDelay(pdMS_TO_TICKS(1000));
  }
  /* USER CODE END StartTaskDefault */
}

/* USER CODE BEGIN Header_StartTaskWiFiListen */
/**
* @brief Function implementing the taskWiFiListen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskWiFiListen */
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
