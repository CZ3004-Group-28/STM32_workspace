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
I2C_HandleTypeDef hi2c1;

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
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void displayMsg(void *argument);

/* USER CODE BEGIN PFP */
void motorTurn(uint8_t amt);
void motorMoveForward();
void motorMoveBackward();
void motorStop();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// buffer format = command character + numeric value [cmd char][num0][num1][num2]
//			eg.  L120 L = turn left amount = 120
uint8_t aRxBuffer[10];

uint8_t rxTask = 'x'; // custom default task
uint8_t curTask[4];
uint16_t rxVal = 0;
uint8_t rxMsg[20];

uint16_t speed_L = 0;
uint16_t speed_R = 0;
uint8_t dir = 0;
uint8_t turn = 74;
uint16_t pwm_speed_L = 0;
uint16_t pwm_speed_R = 0;
int adj_dist_L = 0;
int adj_dist_R = 0;
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

// for distance specific movement
float targetDist = 120;
float positionNow = 0;

// I2C
uint8_t buf[12];
uint8_t ICM_addr = 0x68;
uint8_t ICM_REG_addr = 0x0;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
//  PID_Init(&pid_speed_L, 0.4, 0.02, 0.05);

  HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer,4);

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
	    	sprintf(rxMsg, "%-16s","manual mode\0");
	    	break;
	    case 'f':
	    	sprintf(rxMsg, "%-16s","exit manual\0");
	    }
	} else {
		sprintf(rxMsg, "doing:%-10s", aRxBuffer);
		sprintf(curTask, "%4s", aRxBuffer);
		rxTask = aRxBuffer[0];
		rxVal = (aRxBuffer[1] - 48) * 100 + (aRxBuffer[2] - 48) * 10 + (aRxBuffer[3] - 48);
	}

	// clear aRx buffer
	__HAL_UART_FLUSH_DRREGISTER(&huart3);
	HAL_UART_Receive_IT(&huart3, aRxBuffer, 4);
	//HAL_UART_Transmit(&huart3, (uint8_t *) aTxBuffer, BUFFER_SIZE);
	//HAL_UART_Transmit(&huart3, (uint8_t *) aTxBuffer, BUFFER_SIZE, 0xFFFF);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == SW1_Pin) {
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		uint8_t ch[5];
		sprintf(ch, "proC\n");
		HAL_UART_Transmit(&huart3, (uint8_t *) ch, 5, 0xFFFF);
	}

}

void motorTurn(uint8_t amt) {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	// 16Mhz, prescaler: 160, period: 1000
	// extreme left: 95, center: 145, extreme right: 245
	//htim1.Instance->CCR4 = (amt > 245) ? 245 : ((amt < 95) ? 95 : amt);

	// 16Mhz, prescaler: 320, period: 1000
	//extreme left: 50, center: 74, extreme right:115
	turn = (amt > 115) ? 115 : ((amt < 50) ? 50 : amt);
	htim1.Instance->CCR4 = turn;
	acknowledgeTaskDone();

}

void motorMoveForward() {
	pwm_speed_L = 7199 * initDuty_L / 10000;
	pwm_speed_R = 7199 * initDuty_R / 10000;
	// anticlockwise (forward)
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	// clockwise (forward)
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm_speed_L);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm_speed_R);
}

void motorMoveBackward() {
	pwm_speed_L = 7199 * initDuty_L / 10000;
	pwm_speed_R = 7199 * initDuty_R / 10000;
	 // clockwise (backward)
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);

	 // anticlockwise (backward)
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm_speed_L);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm_speed_R);
}

void motorStop() {
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
	pwm_speed_L = 0;
	pwm_speed_R = 0;
	pid_speed_L.EkSum = 0;
	pid_speed_R.EkSum = 0;
}


void Start_Monitoring_Speed_Dist() {
//	PID_Init(&pid_speed_L, 2.5, 0.0, 0.0);
//	PID_Init(&pid_speed_R, 2.5, 0.0, 0.0);
//	PID_Init(&pid_dist_L, 1, 0.0, 0.0);
//	PID_Init(&pid_dist_R, 1, 0.0, 0.0);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
//	motorTurn(74);
	HAL_TIM_Base_Start_IT(&htim10);

	cnt1_L = __HAL_TIM_GET_COUNTER(&htim2);
	cnt1_R = __HAL_TIM_GET_COUNTER(&htim3);
}

void Stop_Monitoring_Speed_Dist() {
	motorStop();
	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Stop_IT(&htim10);
	targetDist = 0;
	positionNow = 0;

}

void motorMoveByDist(float dist, int dir) {
	targetDist = 1.133145885 * dist - 2.419816382;
	Start_Monitoring_Speed_Dist();
	if (dir) motorMoveForward();
	else motorMoveBackward();
}

void acknowledgeTaskDone() {
	//acknowledge rpi task done
	uint8_t ch[20];
	sprintf(rxMsg, "done:%-11s", curTask);
	sprintf(ch,"ack|%4s|done\n", curTask);
	HAL_UART_Transmit(&huart3, ch, 14, 0xFFFF);
	sprintf(curTask, "x000");
	rxVal = 0;
}

// TIM6 timer interrupt evoked every 10ms
// periodically update DC motor's pwmDuty
// periodically check motor moved distance and speed
int count = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim != &htim10) return;

	//test travel for 100cm and stop
	if (positionNow >= targetDist) {
		Stop_Monitoring_Speed_Dist();
		acknowledgeTaskDone();
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
//	HAL_UART_Transmit(&huart3, (uint8_t *) ch,20,0xFFFF);
//	sprintf(ch,"cT:%-6d|%-6d\n", speed_L, speed_R);
//	HAL_UART_Transmit(&huart3, (uint8_t *) ch,20,0xFFFF);
//  sprintf(ch, "pos:%-12d\n\n", (int) positionNow);
//  HAL_UART_Transmit(&huart3, (uint8_t *) ch,20,0xFFFF);
	count++;
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

  /* Infinite loop */
  for(;;)
  {
	 switch(rxTask) {
	 case '_':
		 break;
	 case 'f':
		 rxTask = 'x';
		 motorMoveByDist(rxVal, 1);
		 break;
	 case 'b':
		 rxTask = 'x';
		 motorMoveByDist(rxVal, 0);
		 break;
	 case 's':
		 rxTask = 'x';
		 motorStop();
		 acknowledgeTaskDone();
		 break;
	 case 't':
		 rxTask = 'x';
		 motorTurn(rxVal);
		 break;
	 case 'u':
		 initDuty_L = rxVal * 10;
		 break;
	 case 'v':
		 initDuty_R = rxVal * 10;
		 break;
	 case 'x':
		 // wait for next command
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
	  OLED_ShowString(0, 0, rxMsg);
	  sprintf(msg,"cT:%-6d|%-6d", speed_L, speed_R);
	  OLED_ShowString(0, 12, msg);
	  sprintf(msg, "pos:%-12d", (int) positionNow);
	  OLED_ShowString(0, 24, msg);
	  sprintf(msg, "buf:%-12s", aRxBuffer);
	  OLED_ShowString(0, 36, msg);

//	  cur_line = 0;
//	  // clear message
//	  OLED_ShowString(0,cur_line, rxMsg); cur_line += CHAR_H;
//	  sprintf(msg, "");
////
//	  sprintf(msg, "st:%c,val:%-7d", rxTask, rxVal);
//	  OLED_ShowString(0, cur_line, msg); cur_line += CHAR_H;
//
//	  switch(rxTask) {
//	 	 case 'f':
//	 	 case 'b':
//	 	 case 's':
//	 		 sprintf(msg, "%-6d|%-6d|%-2d", speed_L, speed_R, dir);
//			 OLED_ShowString(0, cur_line, msg); cur_line += CHAR_H;
//	 		 break;
//	 	 case 't':
//	 		sprintf(msg, "turn:%-11d", turn);
//	 		OLED_ShowString(0, cur_line, msg); cur_line += CHAR_H;
//	 		 break;
//	 	 default:
//	 		// HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
//	 		 break;
//	 	 }
//
//	sprintf(msg, "bf:%-16s", aRxBuffer);
//	OLED_ShowString(0, 48, msg);
//	OLED_Refresh_Gram();
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

