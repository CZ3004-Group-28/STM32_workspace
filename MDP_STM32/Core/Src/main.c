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

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for displayOLED */
osThreadId_t displayOLEDHandle;
const osThreadAttr_t displayOLED_attributes = {
  .name = "displayOLED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for encoderTask */
osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes = {
  .name = "encoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for movementTest */
osThreadId_t movementTestHandle;
const osThreadAttr_t movementTest_attributes = {
  .name = "movementTest",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for distanceTest */
osThreadId_t distanceTestHandle;
const osThreadAttr_t distanceTest_attributes = {
  .name = "distanceTest",
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
void StartDefaultTask(void *argument);
void displayMsg(void *argument);
void runEncoder(void *argument);
void movementTestTask(void *argument);
void moveDistTest(void *argument);

/* USER CODE BEGIN PFP */
void motorTurn(uint8_t amt);
void motorMoveForward();
void motorMoveBackward();
void motorStop();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t BUFFER_SIZE = 4;

// buffer format = command character + numeric value [cmd char][num0][num1][num2]
//			eg.  L120 L = turn left amount = 120
uint8_t aRxBuffer[10];
uint8_t aTxBuffer[10];


uint8_t rxTask = 'x';
uint16_t rxVal = 0;
uint8_t rxMsg[20];

uint16_t speed_L = 0;
uint16_t speed_R = 0;
uint8_t dir = 0;
uint8_t turn = 75;
uint16_t pwmDuty_L = 0;
uint16_t pwmDuty_R = 0;
uint16_t pwmVal = 0;

uint8_t manualMode = 0;

// PID
PID_typedef left_pid, right_pid;
uint16_t targetSpeed = 0; // eg. 10 = 10cm/s
uint16_t PID_sample_rate = 250;
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
  /* USER CODE BEGIN 2 */
  OLED_Init();

  PID_Init(&left_pid, 0.4, 0.02, 0.05);
  PID_Init(&right_pid, 0.4, 0.02, 0.05);

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
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of displayOLED */
//  displayOLEDHandle = osThreadNew(displayMsg, NULL, &displayOLED_attributes);

  /* creation of encoderTask */
//  encoderTaskHandle = osThreadNew(runEncoder, NULL, &encoderTask_attributes);

  /* creation of movementTest */
  movementTestHandle = osThreadNew(movementTestTask, NULL, &movementTest_attributes);

  /* creation of distanceTest */
//  distanceTestHandle = osThreadNew(moveDistTest, NULL, &distanceTest_attributes);

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
// HAL_UART_RxCpltCallback evoked when buffer is full

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
	// prevent unused argument(s) compilation warning
	UNUSED(huart);
	if (aRxBuffer[0] == '_') {
		// task unrelated command: log message
	    switch(aRxBuffer[3]) {
	    case 'r': // rpi connected
			sprintf(rxMsg, "%-16s", "rpi connected\0");
			uint8_t * ch = "STM32 connected to rpi\r\n";
			HAL_UART_Transmit(&huart3, (uint8_t *) ch,24,0xFFFF);
			break;
	    case 'p': // debug bluetooth gamepad connected
			sprintf(rxMsg, "%-16s","gpad connected\0");
	    	break;
	    case 'e':
	    	manualMode = 1;
	    	sprintf(rxMsg, "%-16s","manual mode\0");
	    	break;
	    case 'f':
	    	manualMode = 0;
	    	sprintf(rxMsg, "%-16s","exit manual\0");
	    }
	} else {
    	sprintf(rxMsg, "%-16s", "state");
		rxTask = aRxBuffer[0];
		rxVal = (aRxBuffer[1] - 48) * 100 + (aRxBuffer[2] - 48) * 10 + (aRxBuffer[3] - 48);
	}

	// clear aRx buffer
	__HAL_UART_FLUSH_DRREGISTER(&huart3);
	HAL_UART_Receive_IT(&huart3, aRxBuffer, BUFFER_SIZE);
	//HAL_UART_Transmit(&huart3, (uint8_t *) aTxBuffer, BUFFER_SIZE);
	//HAL_UART_Transmit(&huart3, (uint8_t *) aTxBuffer, BUFFER_SIZE, 0xFFFF);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == SW1_Pin) {
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		sprintf(aTxBuffer, "proC\n");
		HAL_UART_Transmit(&huart3, (uint8_t *) aTxBuffer, 5, 0xFFFF);
	}

}

void motorTurn(uint8_t amt) {
	// 16Mhz, prescaler: 160, period: 1000
	// extreme left: 95, center: 145, extreme right: 245
	//htim1.Instance->CCR4 = (amt > 245) ? 245 : ((amt < 95) ? 95 : amt);

	// 16Mhz, prescaler: 320, period: 1000
	//extreme left: 50, center: 75, extreme right:115
	turn = (amt > 115) ? 115 : ((amt < 50) ? 50 : amt);
	htim1.Instance->CCR4 = turn;
}

void motorMoveForwardByDuty(uint8_t duty) {
	uint16_t targetDuty = 7199 * duty / 100;
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	// clockwise (forward)
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, targetDuty);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, targetDuty);
}

void motorMoveForward() {
	//pwmVal = 7199 * duty / 100;
	// anticlockwise (forward)
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	// clockwise (forward)
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmDuty_L);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmDuty_R);
}

void motorMoveBackward() {
	//pwmVal = 7199 * duty / 100;
	 // clockwise (backward)
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);

	 // anticlockwise (backward)
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmDuty_L);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmDuty_R);
}

void motorStop() {
	left_pid.EkSum = 0;
	right_pid.EkSum = 0;
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
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
	// servo pwm
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

//	 DC motor pwm
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  /* Infinite loop */
  for(;;)
  {
	 switch(rxTask) {
	 case 'c':
		 PID_sample_rate = rxVal*10;
		 break;
	 case '_':
		 break;
	 case 'f':
		 targetSpeed = rxVal;
		 pwmDuty_L = 2000;
		 pwmDuty_R = 2000;
		 motorMoveForward();
		 break;
	 case 'F':
		 motorMoveForwardByDuty(rxVal);
		 break;
	 case 'b':
		 targetSpeed = rxVal;
		 pwmDuty_L = 2000;
		 pwmDuty_R = 2000;
		 motorMoveBackward();
		 break;
	 case 's':
		 targetSpeed = 0;
		 pwmDuty_L = 0;
		 pwmDuty_R = 0;
		 motorStop();
		 break;
	 case 't':
		 motorTurn(rxVal);
		 break;
	 default:
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
	uint8_t cur_line = 0;
  for(;;)
  {
	  cur_line = 0;
	  // clear message
	  OLED_ShowString(0,cur_line, rxMsg); cur_line += CHAR_H;
	  sprintf(msg, "");
//
	  sprintf(msg, "st:%c,val:%-7d", rxTask, rxVal);
	  OLED_ShowString(0, cur_line, msg); cur_line += CHAR_H;

	  switch(rxTask) {
	 	 case 'f':
	 	 case 'b':
	 	 case 's':
	 		 sprintf(msg, "%-6d|%-6d|%-2d", speed_L, speed_R, dir);
			 OLED_ShowString(0, cur_line, msg); cur_line += CHAR_H;
	 		 break;
	 	 case 't':
	 		sprintf(msg, "turn:%-11d", turn);
	 		OLED_ShowString(0, cur_line, msg); cur_line += CHAR_H;
	 		 break;
	 	 default:
	 		// HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	 		 break;
	 	 }

	sprintf(msg, "bf:%-16s", aRxBuffer);
	OLED_ShowString(0, 48, msg);
//	OLED_Refresh_Gram();
	osDelay(10);
  }
  /* USER CODE END displayMsg */
}

/* USER CODE BEGIN Header_runEncoder */
/**
* @brief Function implementing the encoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runEncoder */
void runEncoder(void *argument)
{
  /* USER CODE BEGIN runEncoder */
	int cnt1_L, cnt2_L, cnt1_R, cnt2_R;
	uint32_t tick;

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	cnt1_L = __HAL_TIM_GET_COUNTER(&htim2);
	cnt1_R = __HAL_TIM_GET_COUNTER(&htim3);
	tick = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
	  if (manualMode == 1) continue;
	  if (HAL_GetTick() - tick > 1000L) { // check wheels' speed every 1 second
		  cnt2_L = __HAL_TIM_GET_COUNTER(&htim2);
		  cnt2_R = __HAL_TIM_GET_COUNTER(&htim3);
		 // uint16_t lastSpeed_L = speed_L;
		  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
			  speed_L = (cnt2_L <= cnt1_L) ? cnt1_L - cnt2_L : (65535 - cnt2_L) + cnt1_L;
		  } else {
			  speed_L = (cnt2_L >= cnt1_L) ? cnt2_L - cnt1_L : (65535 - cnt1_L) + cnt2_L;
		  }

		  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
			  speed_R = (cnt2_R <= cnt1_R) ? cnt1_R - cnt2_R : (65535 - cnt2_R) + cnt1_R;
		  } else {
			  speed_R = (cnt2_R >= cnt1_R) ? cnt2_R - cnt1_R : (65535 - cnt1_R) + cnt2_R;
		  }

		  speed_L = speed_L == 65535 ? 0 : speed_L;
		  speed_R = speed_R == 65535 ? 0 : speed_R;
		  dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
		  cnt1_L = __HAL_TIM_GET_COUNTER(&htim2);
		  cnt1_R = __HAL_TIM_GET_COUNTER(&htim3);
		  tick = HAL_GetTick();

	  }

   // osDelay(10); //delay not suitable here, we try to get accurate encoder reading
  }
  /* USER CODE END runEncoder */
}

/* USER CODE BEGIN Header_movementTestTask */
/**
* @brief Function implementing the movementTest thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_movementTestTask */
void movementTestTask(void *argument)
{
  /* USER CODE BEGIN movementTestTask */
	char msg[20];

	int cnt1_L, cnt2_L, cnt1_R, cnt2_R;
	uint32_t tick;

	uint16_t initDuty = 1310;
	uint16_t newDuty_L = initDuty, newDuty_R = initDuty;

	pwmDuty_L = 7199 * newDuty_L / 10000;
	pwmDuty_R = 7199 * newDuty_R / 10000;
	// set target speed we want to reach to be 40cm/s
	//targetSpeed = 60;
	int targetTick = 1571;


	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	cnt1_L = __HAL_TIM_GET_COUNTER(&htim2);
	cnt1_R = __HAL_TIM_GET_COUNTER(&htim3);
	tick = HAL_GetTick();

	//int targetTick = (double)targetSpeed/WHEEL_LENGTH * ENCODER_TICK_PER_REV * PID_Interval / 1000;
	// 100cm/s = 100/21*330 = 1571

  /* Infinite loop */
  for(;;)
  {
	  motorMoveForward();
	  if (HAL_GetTick() - tick >= 1000L) {
		  cnt2_L = __HAL_TIM_GET_COUNTER(&htim2);
		  cnt2_R = __HAL_TIM_GET_COUNTER(&htim3);
		  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
			  speed_L = (cnt2_L <= cnt1_L) ? cnt1_L - cnt2_L : (65535 - cnt2_L) + cnt1_L;
		  } else {
			  speed_L = (cnt2_L >= cnt1_L) ? cnt2_L - cnt1_L : (65535 - cnt1_L) + cnt2_L;
		  }

		  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
			  speed_R = (cnt2_R <= cnt1_R) ? cnt1_R - cnt2_R : (65535 - cnt2_R) + cnt1_R;
		  } else {
			  speed_R = (cnt2_R >= cnt1_R) ? cnt2_R - cnt1_R : (65535 - cnt1_R) + cnt2_R;
		  }

		  speed_L = speed_L == 65535 ? 0 : speed_L;
		  speed_R = speed_R == 65535 ? 0 : speed_R;

		  cnt1_L = __HAL_TIM_GET_COUNTER(&htim2);
		  cnt1_R = __HAL_TIM_GET_COUNTER(&htim3);
		  tick = HAL_GetTick();


		  //PID
		  newDuty_L += PID_Speed(targetTick, speed_L, &left_pid);
		  newDuty_R += PID_Speed(targetTick, speed_R, &right_pid);
		  pwmDuty_L = 7199 * newDuty_L / 10000;
		  pwmDuty_R = 7199 * newDuty_R / 10000;

		  sprintf(msg,"targetTick:%-5d", targetTick);
		  OLED_ShowString(0, 0, msg);
		  sprintf(msg,"cT:%-6d|%-6d", speed_L, speed_R);
//		  OLED_ShowString(0, 12, msg);
//		  sprintf(msg, "bf:%-16s", aRxBuffer);
		  OLED_ShowString(0, 24, msg);
		  sprintf(msg, "cD:%-6d|%-6d", newDuty_L, newDuty_R);
		  OLED_ShowString(0, 36, msg);
//		  sprintf(msg, "cP:%-6d|%-6d", pwmDuty_L, pwmDuty_R);
//		  OLED_ShowString(0, 48, msg);
	  }
  }
  /* USER CODE END movementTestTask */
}

/* USER CODE BEGIN Header_moveDistTest */
/**
* @brief Function implementing the distanceTest thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_moveDistTest */
void moveDistTest(void *argument)
{
  /* USER CODE BEGIN moveDistTest */
  /* Infinite loop */
	int cnt1_L, cnt2_L, cnt1_R, cnt2_R;
	char msg[20];
	uint16_t initDuty = 2000;
	uint16_t newDuty_L = initDuty, newDuty_R = initDuty;

	pwmDuty_L = 7199 * newDuty_L / 10000;
	pwmDuty_R = 7199 * newDuty_R / 10000;

	// set target distance we want to reach to be 100cm
	double targetDist = 100;
	// PPR * 4 as the encoder is quadrature
	int targetTick = targetDist / WHEEL_LENGTH * PPR * 4; //we want the car to stop after moving for 100cm
	int diff_L = 0;
	long sampling = 10L; // sample every 20ms
	int travelledTick = 0;

	// one tick is about 1/(330*4)*20.12 = 0.015 second

	int tick = HAL_GetTick();
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	cnt1_L = __HAL_TIM_GET_COUNTER(&htim2);
	cnt1_R = __HAL_TIM_GET_COUNTER(&htim3);
	motorMoveForward();
  /* Infinite loop */
  for(;;)
  {
	  // we used left wheel to check move distance
	  if (HAL_GetTick() - tick >= sampling) {
		  cnt2_L = __HAL_TIM_GET_COUNTER(&htim2);
		  if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
			  diff_L = (cnt2_L <= cnt1_L) ? cnt1_L - cnt2_L : (65535 - cnt2_L) + cnt1_L;
		  } else {
			  diff_L = (cnt2_L >= cnt1_L) ? cnt2_L - cnt1_L : (65535 - cnt1_L) + cnt2_L;
		  }
		  tick = HAL_GetTick();
		  cnt1_L = __HAL_TIM_GET_COUNTER(&htim2);
		  //empirical test, encoder value won't go beyond 20000 based on 12v supply, in a 20-100ms sampling rate
		  // hence, anything beyond 20000 can be inaccuracy due to low speed
		  // only sampling value below 20000
		  if (diff_L < 20000) travelledTick += diff_L;
		  if (travelledTick >= targetTick) motorStop();




		  sprintf(msg,"targetTick:%-5d", targetTick);
		  OLED_ShowString(0, 0, msg);
		  sprintf(msg,"travelTick:%-5d", travelledTick);
		  OLED_ShowString(0, 12, msg);
		  sprintf(msg,"curDiff:%-8d", diff_L);
		  OLED_ShowString(0, 24, msg);
		  sprintf(msg,"wperi:%-10d", (int)WHEEL_LENGTH);
		  OLED_ShowString(0, 36, msg);



	  }



//	  }





  }
  /* USER CODE END moveDistTest */
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

