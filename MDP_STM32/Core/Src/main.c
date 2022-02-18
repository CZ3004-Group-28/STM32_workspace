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
#include "ICM20948.h"
#include <stdio.h>
#include <math.h>
#include "ICM20948.h"
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
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for runCommandTask */
osThreadId_t runCommandTaskHandle;
const osThreadAttr_t runCommandTask_attributes = {
  .name = "runCommandTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t BUFFER_SIZE = 5;
uint8_t aRxBuffer[10];

typedef struct _command {
	uint8_t index;
	uint16_t val;
} Command;

typedef struct _commandQueue {
	uint8_t head;
	uint8_t tail;
	uint8_t size;
	Command buffer[4];
} CommandQueue;

CommandQueue cQueue;


Command curCmd;
uint8_t rxMsg[16];

char ch[16];

uint16_t dist_dL = 0;
uint16_t dist_dR = 0;
int8_t dir = 1;
uint8_t manualMode = 0;

int lastTick_L, lastTick_R;
//float targetSpeed = 50;
// sampling every 50ms
// 50ms achieved by tim10: 16Mhz, prescalar: 1000, AAR: 799
//uint16_t sampling = 10L;
//int targetTick = 33; // targetSpeed /WHEEL_LENGTH*PPR*4*sampling/1000

float initDuty_L = 1200;
float initDuty_R = 1200;
float targetDist = 0;
float positionNow = 0;

//I2C
int16_t targetAngle = 0;
float angleNow = 0;
int correction = 0;
uint8_t readGyroData[6];
uint8_t readAccelData[6];
uint8_t readMagData[6];
int16_t gyroYaw;
int16_t accelPitch, accelRoll, accelYaw;
int16_t gyro[3];
int16_t accel[3];
int16_t mag[3];
float pitch, roll, yaw=0;

// PID
float ek = 0, ek1 = 0, ekSum = 0;
float Kp=3000, Kd=0, Ki=0;
uint8_t PIDOn = 1;

uint8_t moveMode = 0; // 0: forward/backward 1:turn

typedef struct _commandConfig {
	uint16_t leftDuty;
	uint16_t rightDuty;
	uint8_t servoTurnVal;
	int16_t targetAngle;
	uint8_t direction;
} CmdConfig;

CmdConfig cfgs[19] = {
	{0,0,74,0, DIR_FORWARD}, // STOP
	{1200, 1200, 74, 0, DIR_FORWARD}, // FW00
	{1200, 1200, 74, 0, DIR_BACKWARD}, // BW00

	{800, 1200, 50, 0, DIR_FORWARD}, // FL--
	{1200, 800, 115, 0, DIR_FORWARD}, // FR--
	{800, 1200, 50, 0, DIR_BACKWARD}, // BL--
	{1200, 800, 115, 0, DIR_BACKWARD}, // BR--

	{300, 1800, 50, 87, DIR_FORWARD}, // FL20
	{2000, 300, 115 ,-84, DIR_FORWARD}, // FR20
	{400, 1700, 50, -87, DIR_BACKWARD}, // BL20
	{2200, 300, 115, 88, DIR_BACKWARD}, // BR20

	{300, 1900, 57, 88, DIR_FORWARD}, // FL30
	{1800, 400, 99, -88, DIR_FORWARD}, // FR30
	{300, 1700, 55, -87, DIR_BACKWARD}, //BL30
	{1700, 400, 105, 88, DIR_BACKWARD}, // BR30

	{900, 1500, 59, 89, DIR_FORWARD}, // FL40
	{1300, 1300, 96, -86, DIR_FORWARD}, // FR40
	{900, 1300, 61, -89, DIR_BACKWARD}, // BL40
	{1300, 900, 98, 89, DIR_BACKWARD}, // BR40
//	{},
};


//debug variable
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
void defaultDisplayTask(void *argument);
void runCmdTask(void *argument);

/* USER CODE BEGIN PFP */
void motorStop();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  ICM20948_init(&hi2c1,0,GYRO_FULL_SCALE_1000DPS,ACCEL_FULL_SCALE_2G);

  // initialise command queue
  curCmd.index = 100;
  curCmd.val = 0;

  cQueue.head = 0;
  cQueue.tail = 0;
  cQueue.size = 4;
  for (int i = 0; i < 4;i++) {
	  Command cmd;
	  cmd.index = 100;
	  cmd.val = 0;
	  cQueue.buffer[i] = cmd;
  }

  	HAL_UART_Receive_IT(&huart3, aRxBuffer,BUFFER_SIZE);

	// servo motor turn
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	// motor backwheel move
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	// encoder monitor speed and distance
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	//adjust steering
	__RESET_SERVO_TURN(&htim1);


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
  defaultTaskHandle = osThreadNew(defaultDisplayTask, NULL, &defaultTask_attributes);

  /* creation of runCommandTask */
  runCommandTaskHandle = osThreadNew(runCmdTask, NULL, &runCommandTask_attributes);

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
	int val;

	val = (aRxBuffer[2] - 48) * 10 + (aRxBuffer[3] - 48);
	if (aRxBuffer[1] >= '0' && aRxBuffer[1] <= '9') val += (aRxBuffer[1] - 48) * 100;

	manualMode = 0;

	if (aRxBuffer[0] == 'S' && aRxBuffer[1] == 'T') __ADD_COMMAND(cQueue, 0, 0); // stop
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] != 'L' && aRxBuffer[1] != 'R') { //FW
		if (aRxBuffer[2] == '-' && aRxBuffer[3] == '-') manualMode = 1;
		__ADD_COMMAND(cQueue, 1, val);
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] != 'L' && aRxBuffer[1] != 'R') { //BW
		if (aRxBuffer[2] == '-' && aRxBuffer[3] == '-') manualMode = 1;
		__ADD_COMMAND(cQueue, 2, val);
	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'L') { // FL
		if (aRxBuffer[2] == '-' && aRxBuffer[3] == '-') {
			manualMode = 1;
			__ADD_COMMAND(cQueue, 3, 0); // FL manual
		} else {
			switch (val) {
			case 0:
			case 20:
				 __ADD_COMMAND(cQueue, 2, 3); // BW03
				 __ADD_COMMAND(cQueue, 7, val); // FL00
				 __ADD_COMMAND(cQueue, 2, 5); // BW05
				break;
			case 30:
				 __ADD_COMMAND(cQueue, 7, val); // FL00
				 __ADD_COMMAND(cQueue, 2, 3); // BW03
				break;
			case 40:
				 __ADD_COMMAND(cQueue, 7, val); // FL00
				 __ADD_COMMAND(cQueue, 1, 8); // FW08
				break;
			}
		}


	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'R') { // FR
		if (aRxBuffer[2] == '-' && aRxBuffer[3] == '-') {
			manualMode = 1;
			__ADD_COMMAND(cQueue, 4, 0); // FR manual
		} else {
			switch (val) {
			case 0:
			case 20:
				__ADD_COMMAND(cQueue, 2, 3); // BW03
				__ADD_COMMAND(cQueue, 8, val); // FR00
				__ADD_COMMAND(cQueue, 2, 6); // BW06
				break;
			case 30:
				__ADD_COMMAND(cQueue, 8, val); // FR00
				__ADD_COMMAND(cQueue, 2, 5); // BW05
				break;
			case 40:
				__ADD_COMMAND(cQueue, 8, val); // FR00
				__ADD_COMMAND(cQueue, 2, 5); // BW05
				break;
			}
		}

	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'L') { // BL
		if (aRxBuffer[2] == '-' && aRxBuffer[3] == '-') {
			manualMode = 1;
			__ADD_COMMAND(cQueue, 5, 0); // BL manual
		} else {
			switch (val) {
			case 0:
			case 20:
				__ADD_COMMAND(cQueue, 1, 6); // FW06
				__ADD_COMMAND(cQueue, 9, val); // BL00
				__ADD_COMMAND(cQueue, 1, 3); // FW03
				break;
			case 30:
				__ADD_COMMAND(cQueue, 9, val); // BL00
				__ADD_COMMAND(cQueue, 2, 3); // BW03
				break;
			case 40:
				__ADD_COMMAND(cQueue, 9, val); // BL00
				__ADD_COMMAND(cQueue, 1, 8); // FW08
				break;
			}
		}
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'R') { // BR
		if (aRxBuffer[2] == '-' && aRxBuffer[3] == '-') {
			manualMode = 1;
			__ADD_COMMAND(cQueue, 6, 0); // BR manual
		} else {
			switch (val) {
			case 0:
			case 20:
				__ADD_COMMAND(cQueue, 1, 6); // FW06
				__ADD_COMMAND(cQueue, 10, val); // BR00
				break;
			case 30:
				__ADD_COMMAND(cQueue, 10, val); // BR00
				__ADD_COMMAND(cQueue, 2, 6); // BW06
				break;
			case 40:
				__ADD_COMMAND(cQueue, 10, val); // BR00
				__ADD_COMMAND(cQueue, 2, 6); // BW06
				break;
			}
		}
	}
	else if (aRxBuffer[0] == 'K' && aRxBuffer[1] == 'P') __ADD_COMMAND(cQueue, 85, val); // pid
	else if (aRxBuffer[0] == 'K' && aRxBuffer[1] == 'I') __ADD_COMMAND(cQueue, 86, val); // pid
	else if (aRxBuffer[0] == 'K' && aRxBuffer[1] == 'D') __ADD_COMMAND(cQueue, 87, val); // pid
	else if (aRxBuffer[0] == 'A') __ADD_COMMAND(cQueue, 88, val); // anti-clockwise rotation with variable
	else if (aRxBuffer[0] == 'C') __ADD_COMMAND(cQueue, 89, val); // clockwise rotation with variable
	else if (aRxBuffer[0] == 'Q' && aRxBuffer[1] == 'T') __ADD_COMMAND(cQueue, 90, val); // forward turn test
	else if (aRxBuffer[0] == 'X' && aRxBuffer[1] == 'T') __ADD_COMMAND(cQueue, 91, val); // backward turn test
	else if (aRxBuffer[0] == 'R' && aRxBuffer[1] == 'N') __ADD_COMMAND(cQueue, 92, val); // change target angle (-ve)
	else if (aRxBuffer[0] == 'R' && aRxBuffer[1] == 'P') __ADD_COMMAND(cQueue, 93, val); // change target angle (+ve)
	else if (aRxBuffer[0] == 'D' && aRxBuffer[1] == 'D') __ADD_COMMAND(cQueue, 94, val); // set target distance
	else if (aRxBuffer[0] == 'P' && aRxBuffer[1] == 'P') __ADD_COMMAND(cQueue, 95, val); // toggle pid controller
	else if (aRxBuffer[0] == 'D' && aRxBuffer[1] == 'L') __ADD_COMMAND(cQueue, 96, val); // left duty test
	else if (aRxBuffer[0] == 'D' && aRxBuffer[1] == 'R') __ADD_COMMAND(cQueue, 97, val); // right duty test
	else if (aRxBuffer[0] == 'Q' && aRxBuffer[1] == 'Q') __ADD_COMMAND(cQueue, 98, val); // steering test

	if (!__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
		snprintf((char *)rxMsg, sizeof(rxMsg), "doing");
		__READ_COMMAND(cQueue, curCmd);
	}


	// clear aRx buffer
	__HAL_UART_FLUSH_DRREGISTER(&huart3);
	HAL_UART_Receive_IT(&huart3, aRxBuffer, BUFFER_SIZE);

}

int clickOnce = 0;
int targetRot = 360;
int targetD = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (clickOnce) return;
	if (GPIO_Pin == SW1_Pin) {
		clickOnce = 1;
		manualMode = 0;

//		targetD = targetD == 120 ? 0 : targetD + 5;
		__ADD_COMMAND(cQueue, 1, targetD);
		__READ_COMMAND(cQueue, curCmd);
		// test gyro+accel
//		__ADD_COMMAND(cQueue,1, 120);
//		__READ_COMMAND(cQueue, curCmd);
		targetRot = (targetRot + 15) % 375;
//		HAL_TIM_Base_Start_IT(&htim10);
	}

}

void motorStop() {
	HAL_TIM_Base_Stop_IT(&htim10);
	targetDist = 0; targetAngle = 0; yaw = 0;
	positionNow = 0; angleNow = 0;
	__SET_MOTOR_DUTY(&htim8, 0, 0);
	__SET_SERVO_TURN(&htim1, 74);
	ekSum = 0; ek1 = 0;
}

// TIM10 timer interrupt evoked every 10ms
// periodically update DC motor's pwmDuty
// periodically check motor moved distance and speed
// this ISR block RTOS task when running, hence, must disable it whenever it is not used.
int count = 0;
char temp[20];
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim != &htim10) return;

	__Gyro_Read(&hi2c1, readGyroData, gyro);
	angleNow += gyro[2] / GRYO_SENSITIVITY_SCALE_FACTOR_1000DPS * 0.01;

	if (moveMode) { // moveMode: turn
		// stop condition
		if (!manualMode && ((targetAngle < 0 && angleNow <= targetAngle) || (targetAngle > 0 && angleNow >= targetAngle))) {
			__RESET_SERVO_TURN(&htim1);
			motorStop();
			if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
			}
			else __READ_COMMAND(cQueue, curCmd);
			clickOnce = 0; // button click flag to be cleared once reach travel distance (from HAL_GPIO_EXTI_Callback)
			return;
		}
	} else { // moveMode: forward/backward
		//stop condition
		if (!manualMode && positionNow >= targetDist) { // forward/backward stop condition
			__RESET_SERVO_TURN(&htim1);
			motorStop();
			if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
			}
			else __READ_COMMAND(cQueue, curCmd);
			clickOnce = 0; // button click flag to be cleared once reach travel distance (from HAL_GPIO_EXTI_Callback)
			return;
		}

		// Get moved distance of left/right wheels since last tick
		__GET_ENCODER_TICK_DELTA(&htim2, lastTick_L, dist_dL);
		__GET_ENCODER_TICK_DELTA(&htim3, lastTick_R, dist_dR);
		__SET_ENCODER_LAST_TICKS(&htim2, lastTick_L, &htim3, lastTick_R);

		// 0.015242cm per encoder tick = WHEEL_LENGTH / (PPR*4)
		positionNow += (dist_dL + dist_dR) / 2 * 0.015242;

		// drive straight calibration
		if (PIDOn){
			if (angleNow != 0) {
				dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? 1 : -1; // use only one of the wheel to determine car direction
				ekSum += angleNow;
				correction = Kp * angleNow + Ki * ekSum + Kd * (ek1 - angleNow);
				ek1 = angleNow;
				correction = correction > 600 ? 600 : (correction < -600 ? -600 : correction);
				__SET_MOTOR_DUTY(&htim8, MAX_DUTY + correction*dir, MAX_DUTY - correction*dir);
			}
		}
	}

	if (manualMode) osDelay(10);

//	  uint8_t ch[20];
//	 sprintf(ch,"time:%-6d\n", count);
//	HAL_UART_Transmit(&huart3, (uint8_t *) ch,12,0xFFFF);
//	count++;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_defaultDisplayTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_defaultDisplayTask */
void defaultDisplayTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	OLED_ShowString(0, 0, (char *) rxMsg);
	OLED_ShowString(0, 12, (char *) aRxBuffer);
//	snprintf(ch, sizeof(ch), "angle:%4d", (int) angleNow);
//	OLED_ShowString(0, 24, (char *) ch);
	OLED_Refresh_Gram();
	osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_runCmdTask */
/**
* @brief Function implementing the runCommandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runCmdTask */
void runCmdTask(void *argument)
{
  /* USER CODE BEGIN runCmdTask */
  /* Infinite loop */
  for(;;)
  {
	  switch(curCmd.index) {
	  	 case 0: // STOP
	  		motorStop();
	  		if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
	  			__CLEAR_CURCMD(curCmd);
	  			__ACK_TASK_DONE(&huart3, rxMsg);
	  		}
	  		else {
	  			__READ_COMMAND(cQueue, curCmd);
	  		}
	  		 break;
	  	 case 1: //FW
	  	 case 2: //BW
	  		moveMode = 0;
	  		targetDist = manualMode ? 0 : (curCmd.val * DIST_M - DIST_C);

	  		__SET_CMD_CONFIG(cfgs[curCmd.index], &htim8, &htim1, targetAngle);
	  		if (manualMode) {
	  			if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
	  				__CLEAR_CURCMD(curCmd);
	  				__ACK_TASK_DONE(&huart3, rxMsg);
	  			}
	  			else {
	  				__READ_COMMAND(cQueue, curCmd);
	  			}
	  		}
	  		__SET_ENCODER_LAST_TICKS(&htim2, lastTick_L, &htim3, lastTick_R);
	  		__PEND_CURCMD(curCmd);
	  		HAL_TIM_Base_Start_IT(&htim10);

	  		 break;
	  	case 3: //FL manual
		case 4: //FR manual
		case 5: //BL manual
		case 6: //BR manual
			moveMode = 1;
			__SET_CMD_CONFIG(cfgs[curCmd.index], &htim8, &htim1, targetAngle);
			if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
			}
			else __READ_COMMAND(cQueue, curCmd);
			__PEND_CURCMD(curCmd);
			 HAL_TIM_Base_Start_IT(&htim10);

			 break;
	  	 case 7: // FL
	  	 case 8: // FR
	  	 case 9: // BL
	  	 case 10: //BR
	  		 moveMode = 1;
//	  		curCmd.index += curCmd.val == 30 ? 4 : (curCmd.val == 40 ? 8 : 0);
	  		 __SET_CMD_CONFIG(cfgs[curCmd.index + (curCmd.val == 30 ? 4 : (curCmd.val == 40 ? 8 : 0))], &htim8, &htim1, targetAngle);
	  		__PEND_CURCMD(curCmd);
	  		 HAL_TIM_Base_Start_IT(&htim10);
	  		 break;
	  	 case 85:
	  	 case 86:
	  	 case 87:
	  		 if (curCmd.index == 85) Kp = curCmd.val * 100;
	  		 else if (curCmd.index == 86) Ki = curCmd.val * 10;
	  		 else if (curCmd.index == 87) Kd = curCmd.val * 10;
	  		 break;
	  	 case 88: // Axxx, rotate left by xxx degree
	  	 case 89: // Cxxx, rotate right by xxx degree
	  		 moveMode = 1;
	  		 __SET_SERVO_TURN_MAX(&htim1, curCmd.index - 88);
	  		 __SET_MOTOR_DIRECTION(DIR_FORWARD);
	  		 if (curCmd.index == 88) {
	  			 targetAngle = curCmd.val;
	  			 __SET_MOTOR_DUTY(&htim8, 800, 1200);
	  		 } else {
	  			targetAngle = -curCmd.val;
	  			 __SET_MOTOR_DUTY(&htim8, 1200, 800);
	  		 }

	  		 HAL_TIM_Base_Start_IT(&htim10);
	  		 __PEND_CURCMD(curCmd);
	  		 break;
	  	 case 90: //QT forward turn test
	  	 case 91: //XT backward turn test
	  		 moveMode = 1;
	  		 __SET_MOTOR_DIRECTION(curCmd.index - 90? DIR_BACKWARD : DIR_FORWARD);
	  		__SET_MOTOR_DUTY(&htim8, initDuty_L, initDuty_R);
	  		HAL_TIM_Base_Start_IT(&htim10);
	  		__PEND_CURCMD(curCmd);
	  		 break;
	  	 case 92: // RN, change stop angle -ve
	  	 case 93: // RP, +ve
	  		 targetAngle = curCmd.val * (curCmd.index - 92 ? 1 : -1);
	  		__ACK_TASK_DONE(&huart3, rxMsg);
	  		 __CLEAR_CURCMD(curCmd);
	  		 break;
	  	 case 94: // DD set target distance
	  		 targetDist = curCmd.val;
	  		__ACK_TASK_DONE(&huart3, rxMsg);
	  		 __CLEAR_CURCMD(curCmd);
	  		 break;
	  	 case 95: // enable/disable pid
	  		 PIDOn = curCmd.val;
	  		__ACK_TASK_DONE(&huart3, rxMsg);
	  		 __CLEAR_CURCMD(curCmd);
	  		 break;
	  	 case 96: // DL, left duty test
	  		 initDuty_L = curCmd.val * 100;
	  		__ACK_TASK_DONE(&huart3, rxMsg);
	  		 __CLEAR_CURCMD(curCmd);
	  		 break;
	  	 case 97: // DR, right duty test
	  		 initDuty_R = curCmd.val * 100;
	  		__ACK_TASK_DONE(&huart3, rxMsg);
	  		 __CLEAR_CURCMD(curCmd);
	  		 break;
	  	 case 98: // QQ, steering test
	  		 // angle 50-74-115
	  		 // argument 0-65
	  		 __SET_SERVO_TURN(&htim1, SERVO_LEFT_MAX + curCmd.val);
	  		__ACK_TASK_DONE(&huart3, rxMsg);
	  		 __CLEAR_CURCMD(curCmd);
	  		 break;
	  	 case 99:
	  		 break;
	  	 case 100:
	  		 break;
	  	 default:
	  //		 curCmd.index = 99;
	  		 break;
	  	 }

	  osDelay(100);
  }
  /* USER CODE END runCmdTask */
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

