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
#include "oled.h"
#include "PID.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for displayOLED */
osThreadId_t displayOLEDHandle;
const osThreadAttr_t displayOLED_attributes = {
  .name = "displayOLED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM10_Init(void);
void StartDefaultTask(void *argument);
void displayMsg(void *argument);

/* USER CODE BEGIN PFP */
void motorMove();
void motorTurn(uint8_t amt);
void motorStop();
void acknowledgeTaskDone();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t BUFFER_SIZE = 5;
uint8_t aRxBuffer[10];

uint8_t rxTask = 99; // custom default task
uint8_t curTask[4];
uint16_t rxVal = 0;
uint8_t rxMsg[20];

char ch[20];

uint16_t speed_L = 0;
uint16_t speed_R = 0;
uint8_t dir = 0;
uint8_t turn = 74;
uint16_t pwm_speed_L = 0;
uint16_t pwm_speed_R = 0;
//int adj_dist_L = 0;
//int adj_dist_R = 0;
// PID
uint32_t tick;
PID_typedef pid_speed_L, pid_speed_R, pid_dist_L, pid_dist_R;
int cnt1_L, cnt1_R;
float targetSpeed = 50;
// sampling every 50ms
// 50ms achieved by tim10: 16Mhz, prescalar: 1000, AAR: 799
uint16_t sampling = 10L;
int targetTick = 33; // targetSpeed /WHEEL_LENGTH*PPR*4*sampling/1000

// straight line movement pwm duty
float initDuty_L = 2500;
float initDuty_R = 2000;

float targetDist = 0;
float positionNow = 0;

/// debug variables
//uint8_t turnStep[11] = {50, 55, 60, 64, 69, 74, 82, 90, 99, 107, 115};
//int8_t turnIndex = -1;

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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
//  PID_Init(&pid_speed_L, 0.4, 0.02, 0.05);

  HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer,BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of displayOLED */
  displayOLEDHandle = osThreadNew(displayMsg, NULL, &displayOLED_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 320;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 1000;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 159;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
uint8_t getTaskIndex(uint8_t ch1, uint8_t ch2) {
	if (ch1 == 'B' && ch2 == 'W') return 0; // backward
	if (ch1 == 'F' && ch2 == 'W') return 1; // forward
	if (ch1 == 'F' && ch2 == 'L') return 2; // forward left: turn left by 90 degree with 20cm displacement
	if (ch1 == 'F' && ch2 == 'R') return 3;	// forward right: turn right by 90 degree with 20cm displacement
	if (ch1 == 'B' && ch2 == 'L') return 4; // backward left
	if (ch1 == 'B' && ch2 == 'R') return 5; // backward right
	if (ch1 == 'S' && ch2 == 'T') return 6; // stop
	if (ch1 == '_' && ch2 == 'D') return 98; // debug test
	return 99;
}

// HAL_UART_RxCpltCallback evoked when buffer is full
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
	// prevent unused argument(s) compilation warning
	UNUSED(huart);

	sprintf(curTask, "%c%c%c%c", aRxBuffer[0],aRxBuffer[1],aRxBuffer[2],aRxBuffer[3]);
	sprintf(rxMsg, "doing:%-10s", curTask);
	rxTask = 99;
	//getTaskIndex(aRxBuffer[0], aRxBuffer[1]);
	if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'W') rxTask = 0; // backward
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'W') rxTask = 1; // forward
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'L') rxTask = 2; // forward left: turn left by 90 degree with 20cm displacement
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'R') rxTask = 3;	// forward right: turn right by 90 degree with 20cm displacement
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'L') rxTask = 4; // backward left
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'R') rxTask = 5; // backward right
	else if (aRxBuffer[0] == 'S' && aRxBuffer[1] == 'T') rxTask = 6; // stop
	else if (aRxBuffer[0] == 'D' && aRxBuffer[1] == 'L') rxTask = 96; // left duty test
	else if (aRxBuffer[0] == 'D' && aRxBuffer[1] == 'R') rxTask = 97; // right duty test
	else if (aRxBuffer[0] == 'T' && aRxBuffer[1] == 'T') rxTask = 98; // steering test

	rxVal = (aRxBuffer[2] - 48) * 10 + (aRxBuffer[3] - 48);
//		char ch[10];
//		sprintf(ch, "rxTask:%d|rxVal:%d\n",rxTask,rxVal);
//		HAL_UART_Transmit(&huart3, (uint8_t *) ch, 17, 0xFFFF);


	// clear aRx buffer
	__HAL_UART_FLUSH_DRREGISTER(&huart3);
	HAL_UART_Receive_IT(&huart3, aRxBuffer, BUFFER_SIZE);
	//HAL_UART_Transmit(&huart3, (uint8_t *) aTxBuffer, BUFFER_SIZE);
	//HAL_UART_Transmit(&huart3, (uint8_t *)  aTxBuffer, BUFFER_SIZE, 0xFFFF);
}

int clickOnce = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (clickOnce) return;
	if (GPIO_Pin == SW1_Pin) {
		HAL_UART_Transmit(&huart3,"button clicked\r\n", 16, 0xFFFF);
		clickOnce = 1;
	}

}

void motorTurn(uint8_t amt) {
	// 16Mhz, prescaler: 160, period: 1000
	// extreme left: 95, center: 145, extreme right: 245
	//htim1.Instance->CCR4 = (amt > 245) ? 245 : ((amt < 95) ? 95 : amt);

	// 16Mhz, prescaler: 320, period: 1000
	//extreme left: 50, center: 74, extreme right:115
	turn = (amt > 115) ? 115 : ((amt < 50) ? 50 : amt);
	htim1.Instance->CCR4 = turn;
}

void motorMove(uint8_t * dir) {
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, (*dir ? GPIO_PIN_RESET : GPIO_PIN_SET));
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, (*dir ? GPIO_PIN_SET: GPIO_PIN_RESET));
	// right wheel: clockwise (forward)
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, (*dir ? GPIO_PIN_RESET: GPIO_PIN_SET));
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, (*dir ? GPIO_PIN_SET: GPIO_PIN_RESET));

	pwm_speed_L = 7199 * initDuty_L / 10000;
	pwm_speed_R = 7199 * initDuty_R / 10000;
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm_speed_L);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm_speed_R);
}

void motorStop() {
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
	pwm_speed_L = 0;
	pwm_speed_R = 0;
}

void acknowledgeTaskDone() {
	//acknowledge rpi task done
//	uint8_t ch[7];
	sprintf(rxMsg, "done:%-11s", curTask);
	sprintf(ch,"ACK|%c%c\n", curTask[0], curTask[1]);
	HAL_UART_Transmit(&huart3, ch, 7, 0xFFFF);
	sprintf(curTask, "PEND");
	rxVal = 0;
}

// TIM10 timer interrupt evoked every 10ms
// periodically update DC motor's pwmDuty
// periodically check motor moved distance and speed
// this ISR block RTOS task when running, hence, must disable it whenever it is not used.
int count = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim != &htim10) return;

	//test travel for targetDist cm and stop
	if (positionNow >= targetDist) {
		HAL_TIM_Base_Stop_IT(&htim10); // compulsory to release control to  other tasks
		motorTurn(74);
		motorStop();
		targetDist = 0;
		positionNow = 0;
		acknowledgeTaskDone();
		clickOnce = 0; // button click flag to be cleared once reach travel distance (from HAL_GPIO_EXTI_Callback)
		return;
	}

	int cnt2_L, cnt2_R;
	// Get current speed_L and speed_R
	cnt2_L = __HAL_TIM_GET_COUNTER(&htim2);
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
		speed_L = (cnt2_L <= cnt1_L) ? cnt1_L - cnt2_L : (65535 - cnt2_L) + cnt1_L;
	} else {
		speed_L = (cnt2_L >= cnt1_L) ? cnt2_L - cnt1_L : (65535 - cnt1_L) + cnt2_L;
	}

	cnt2_R = __HAL_TIM_GET_COUNTER(&htim3);
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
		speed_R = (cnt2_R <= cnt1_R) ? cnt1_R - cnt2_R : (65535 - cnt2_R) + cnt1_R;
	} else {
		speed_R = (cnt2_R >= cnt1_R) ? cnt2_R - cnt1_R : (65535 - cnt1_R) + cnt2_R;
	}
//	  speed_L = speed_L == 65535 ? 0 : speed_L;
//	  speed_R = speed_R == 65535 ? 0 : speed_R;
	cnt1_L = __HAL_TIM_GET_COUNTER(&htim2);
	cnt1_R = __HAL_TIM_GET_COUNTER(&htim3);
//
	//PID
	positionNow += (speed_L + speed_R) / 2 * 0.015242;
	// 0.015242cm per encoder tick = WHEEL_LENGTH / (PPR*4)

	  // set target distance to be 100cm
//	  adj_dist_L = PID_Position(100, positionNow, &pid_dist_L);
//	  adj_dist_R = PID_Position(100, positionNow, &pid_dist_R);
//	  pwm_speed_L += adj_dist_L;
//	  pwm_speed_R += adj_dist_R;
//	  if (adj_dist_L > 0) motorLForward();
//	  else motorLBackward();
//	  if (adj_dist_R > 0) motorRForward();
//	  else motorRBackward();
//
//	  if (adj_dist_L == 0) motorLStop();
//	  if (adj_dist_R == 0) motorRStop();
//
//	  pwm_speed_L += PID_Duty(targetTick, speed_L, &pid_speed_L);
//	  pwm_speed_R += PID_Duty(targetTick, speed_R, &pid_speed_R);





//	  uint8_t ch[20];
//	 sprintf(ch,"time:%-6d\n", count);
//	HAL_UART_Transmit(&huart3, (uint8_t *) ch,12,0xFFFF);
//	count++;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	// servo motor turn
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	// motor backwheel move
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	// encoder monitor speed and distance
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	//adjust steering
	motorTurn(74);
  /* Infinite loop */
  for(;;)
  {
	  // FW, BW handle acknowledgeTaskDone() under HAL_TIM_PeriodElapsedCallback
	 switch(rxTask) {
	 case 0: //BW
	 case 1: //FW
		targetDist = rxVal*DIST_M - DIST_C;
//		sprintf(ch,"targetDist:%-8d\n", (int)targetDist);
//		HAL_UART_Transmit(&huart3,(uint8_t *) ch, 20, 0xFFFF);
		motorMove(&rxTask);
		rxTask = 99;
		cnt1_L = __HAL_TIM_GET_COUNTER(&htim2);
		cnt1_R = __HAL_TIM_GET_COUNTER(&htim3);
		HAL_TIM_Base_Start_IT(&htim10); // this MAY block other task

		 break;
	 case 2: //FL
//		 sprintf(ch, "rxTask: %d\n", rxTask);
//		 HAL_UART_Transmit(&huart3, ch, 20, 0xFFFF);
		 rxTask = 99;
		 acknowledgeTaskDone();
		 break;
	 case 3: //FR
//		 sprintf(ch, "rxTask: %d\n", rxTask);
//		 HAL_UART_Transmit(&huart3, ch, 20, 0xFFFF);
		 rxTask = 99;
		 acknowledgeTaskDone();
		 break;
	 case 4: //BL
//		 sprintf(ch, "rxTask: %d\n", rxTask);
//		 HAL_UART_Transmit(&huart3, ch, 20, 0xFFFF);
		 rxTask = 99;
		 acknowledgeTaskDone();
		 break;
	 case 5: //BR
//		 sprintf(ch, "rxTask: %d\n", rxTask);
//		 HAL_UART_Transmit(&huart3, ch, 20, 0xFFFF);
		 rxTask = 99;
		 acknowledgeTaskDone();
		 break;
	 case 6: // STOP
		motorStop();
		rxTask = 99;
		 acknowledgeTaskDone();
		 break;
	 case 96: // DL, left duty test
		 initDuty_L = rxVal * 100;
		 rxTask = 99;
		 acknowledgeTaskDone();
		 break;
	 case 97: // DR, right duty test
		 initDuty_R = rxVal * 100;
		 rxTask = 99;
		 acknowledgeTaskDone();
		 break;
	 case 98: // TT, steering test
		 // angle 50-74-115
		 motorTurn(50 + rxVal);
		 rxTask = 99;
		 acknowledgeTaskDone();
		 break;
	 case 99:
		 // wait for current command finish
		 break;
	 default:
		 rxTask = 99;
		 break;
	 }

    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_displayMsg */
/**
* @brief Function implementing the displayOLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_displayMsg */
void displayMsg(void *argument)
{
  /* USER CODE BEGIN displayMsg */
	char msg[20];
  /* Infinite loop */
  for(;;)
  {
	  OLED_ShowString(0, 0, rxMsg);
	  sprintf(msg, "rxTask:%-9d", rxTask);
//	  sprintf(msg,"cT:%-6d|%-6d", speed_L, speed_R);
	  OLED_ShowString(0, 12, msg);
//	  float real_dist = (positionNow + DIST_C) / DIST_M;
//	  sprintf(msg, "actpos:%-9d", (int) real_dist);
//	  OLED_ShowString(0, 24, msg);
//	  sprintf(msg, "turn:%-11d", (int) turn);
//	  OLED_ShowString(0, 36, msg);
//	  sprintf(msg, "targetDist:%-5d", (int)((targetDist + DIST_C) / DIST_M));
//	  OLED_ShowString(0, 36, msg);
//	  sprintf(msg, "test:%-11d", (int)distStep);
//	  OLED_ShowString(0, 48, msg);
	  sprintf(msg, "buf:%-12s", aRxBuffer);
	  OLED_ShowString(0, 48, msg);
	  OLED_Refresh_Gram();
	osDelay(10);
  }
  /* USER CODE END displayMsg */
}

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

