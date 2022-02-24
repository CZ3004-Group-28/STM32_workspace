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
#include <stdlib.h>
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for commandTask */
osThreadId_t commandTaskHandle;
const osThreadAttr_t commandTask_attributes = {
  .name = "commandTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ADCTask */
osThreadId_t ADCTaskHandle;
const osThreadAttr_t ADCTask_attributes = {
  .name = "ADCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for moveManualTask */
osThreadId_t moveManualTaskHandle;
const osThreadAttr_t moveManualTask_attributes = {
  .name = "moveManualTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for turnTask */
osThreadId_t turnTaskHandle;
const osThreadAttr_t turnTask_attributes = {
  .name = "turnTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for moveDistTask */
osThreadId_t moveDistTaskHandle;
const osThreadAttr_t moveDistTask_attributes = {
  .name = "moveDistTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for alignTask */
osThreadId_t alignTaskHandle;
const osThreadAttr_t alignTask_attributes = {
  .name = "alignTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t RX_BUFFER_SIZE = 5;
uint8_t aRxBuffer[10];

typedef struct _command {
	uint8_t index;
	uint16_t val;
} Command;

uint8_t CMD_BUFFER_SIZE = 12;
typedef struct _commandQueue {
	uint8_t head;
	uint8_t tail;
	uint8_t size;
	Command buffer[12];
} CommandQueue;

CommandQueue cQueue;

Command curCmd;
uint8_t rxMsg[16];
char ch[16];

uint8_t manualMode = 0;

float initDuty_L = 1200;
float initDuty_R = 1200;

// PID
int16_t targetAngle = 0;
float angleNow = 0;
uint8_t readGyroZData[2];
int16_t gyroZ;
uint16_t newDutyL = 1200, newDutyR = 1200;
uint16_t last_curTask_tick = 0;

typedef struct _pidConfig {
	float Kp;
	float Ki;
	float Kd;
	float ek1;
	float ekSum;
} PIDConfig;

PIDConfig pidConfigAngle;

typedef struct _commandConfig {
	uint16_t leftDuty;
	uint16_t rightDuty;
	float servoTurnVal;
	float targetAngle;
	uint8_t direction;
} CmdConfig;

CmdConfig cfgs[19] = {
	{0,0,74,0, DIR_FORWARD}, // STOP
//	{1400, 1200, 74, 0, DIR_FORWARD}, // FW00
//	{1300, 1200 , 74, 0, DIR_BACKWARD}, // BW00
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

enum TASK_TYPE{
	TASK_MOVE,
	TASK_MOVE_BACKWARD,
	TASK_TURN,
	TASK_ADC,
	TASK_ALIGN,
	TASK_NONE
};

HAL_StatusTypeDef UART_STATUS;

enum TASK_TYPE curTask = TASK_NONE, prevTask = TASK_NONE;

char temp[20];

float curObsDist = 0;
uint16_t curObsTick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void runOledTask(void *argument);
void runCmdTask(void *argument);
void runADCTask(void *argument);
void runMoveManualTask(void *argument);
void runTurnTask(void *argument);
void runMoveDistTask(void *argument);
void runAlignTask(void *argument);

/* USER CODE BEGIN PFP */
void PIDConfigInit(PIDConfig * cfg, const float Kp, const float Ki, const float Kd);
void PIDConfigReset(PIDConfig * cfg);

void StraightLineMove(uint16_t * last_curTask_tick);
void StraightLineMoveObstacle(uint16_t * last_curTask_tick, float * speedScale);
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  ICM20948_init(&hi2c1,0,GYRO_FULL_SCALE_2000DPS,ACCEL_FULL_SCALE_2G);

  // initialise command queue
  curCmd.index = 100;
  curCmd.val = 0;

  cQueue.head = 0;
  cQueue.tail = 0;
  cQueue.size = CMD_BUFFER_SIZE;
  for (int i = 0; i < CMD_BUFFER_SIZE;i++) {
	  Command cmd;
	  cmd.index = 100;
	  cmd.val = 0;
	  cQueue.buffer[i] = cmd;
  }

//	PIDConfigInit(&pidConfigAngle, 2000.0, 0.0, 0.0);
  	PIDConfigInit(&pidConfigAngle, 2.5, 0.0,0.8);
//	PIDConfigInit(&pidConfigSpeed, 1.2, 0.0, 0.0);

  	HAL_UART_Receive_IT(&huart3, aRxBuffer,RX_BUFFER_SIZE);

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
  /* creation of oledTask */
  oledTaskHandle = osThreadNew(runOledTask, NULL, &oledTask_attributes);

  /* creation of commandTask */
  commandTaskHandle = osThreadNew(runCmdTask, NULL, &commandTask_attributes);

  /* creation of ADCTask */
  ADCTaskHandle = osThreadNew(runADCTask, NULL, &ADCTask_attributes);

  /* creation of moveManualTask */
  moveManualTaskHandle = osThreadNew(runMoveManualTask, NULL, &moveManualTask_attributes);

  /* creation of turnTask */
  turnTaskHandle = osThreadNew(runTurnTask, NULL, &turnTask_attributes);

  /* creation of moveDistTask */
  moveDistTaskHandle = osThreadNew(runMoveDistTask, NULL, &moveDistTask_attributes);

  /* creation of alignTask */
  alignTaskHandle = osThreadNew(runAlignTask, NULL, &alignTask_attributes);

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  sConfig.Channel = ADC_CHANNEL_11;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
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

	if (aRxBuffer[0] == 'S' && aRxBuffer[1] == 'T') { // only STOP can preempt any greedy task
//		__ADD_COMMAND(cQueue, 0, 0); // stop
		__ON_TASK_END(&htim8, prevTask, curTask);
		if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, rxMsg);
		}
		else {
			__READ_COMMAND(cQueue, curCmd);
		}
	}
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
	else if (aRxBuffer[0] == 'T' && aRxBuffer[1] == 'L') __ADD_COMMAND(cQueue, 11, val); // TL turn left max
	else if (aRxBuffer[0] == 'T' && aRxBuffer[1] == 'R') __ADD_COMMAND(cQueue, 12, val); // TR turn right max
	else if (aRxBuffer[0] == 'D' && aRxBuffer[1] == 'T') __ADD_COMMAND(cQueue, 13, val); // DT move until distance to obstacle
	else if (aRxBuffer[0] == 'K' && aRxBuffer[1] == 'P') __ADD_COMMAND(cQueue, 85, val); // pid
	else if (aRxBuffer[0] == 'K' && aRxBuffer[1] == 'I') __ADD_COMMAND(cQueue, 86, val); // pid
	else if (aRxBuffer[0] == 'K' && aRxBuffer[1] == 'D') __ADD_COMMAND(cQueue, 87, val); // pid
	else if (aRxBuffer[0] == 'A') __ADD_COMMAND(cQueue, 88, val); // anti-clockwise rotation with variable
	else if (aRxBuffer[0] == 'C') __ADD_COMMAND(cQueue, 89, val); // clockwise rotation with variable

	// debug commands
	else if (aRxBuffer[0] == 'Q' && aRxBuffer[1] == 'T') __ADD_COMMAND(cQueue, 90, val); // forward turn test
	else if (aRxBuffer[0] == 'X' && aRxBuffer[1] == 'T') __ADD_COMMAND(cQueue, 91, val); // backward turn test
	else if (aRxBuffer[0] == 'R' && aRxBuffer[1] == 'N') __ADD_COMMAND(cQueue, 92, val); // change target angle (-ve)
	else if (aRxBuffer[0] == 'R' && aRxBuffer[1] == 'P') __ADD_COMMAND(cQueue, 93, val); // change target angle (+ve)
	else if (aRxBuffer[0] == 'D' && aRxBuffer[1] == 'L') __ADD_COMMAND(cQueue, 96, val); // left duty test
	else if (aRxBuffer[0] == 'D' && aRxBuffer[1] == 'R') __ADD_COMMAND(cQueue, 97, val); // right duty test
	else if (aRxBuffer[0] == 'Q' && aRxBuffer[1] == 'Q') __ADD_COMMAND(cQueue, 98, val); // steering test

	if (!__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
		snprintf((char *)rxMsg, sizeof(rxMsg), "doing");
		__READ_COMMAND(cQueue, curCmd);
	}

	// clear aRx buffer
	  __HAL_UART_FLUSH_DRREGISTER(&huart3);
	  UART_STATUS = HAL_UART_Receive_IT(&huart3, aRxBuffer, RX_BUFFER_SIZE);

//	  if (UART_STATUS != HAL_OK) {
//		  HAL_UART_Transmit(&huart3, "fail UART_Rx", 12,0xFFFF);
//	  }
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart) {
//	 UNUSED(huart);
//
//	 UART_Tx_Done = 1;
//}

int clickOnce = 0;
int targetRot = 360;
int targetD = 5;

uint8_t tempDir = 1 ;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (clickOnce) return;
	if (GPIO_Pin == SW1_Pin) {
		clickOnce = 1;
//
//		__ADD_COMMAND(cQueue, 1, targetD);
//		__READ_COMMAND(cQueue, curCmd);
//		targetRot = (targetRot + 15) % 375;


		// test PID straight line
//		manualMode = 0;
//		__ADD_COMMAND(cQueue, tempDir, 130);
//		__READ_COMMAND(cQueue, curCmd);
//		tempDir = tempDir == 2 ? 1 : 2;

		// test dist
		manualMode = 0;
		__ADD_COMMAND(cQueue, 13,15); // adc task
//		targetD = (targetD + 5) % 125;
		__READ_COMMAND(cQueue, curCmd);


		// test PID
//		manualMode = 1;
//		__ADD_COMMAND(cQueue, 1,0);
//		__READ_COMMAND(cQueue, curCmd);

		// test 10 step
//		for (int i = 0; i < 10; i++) __ADD_COMMAND(cQueue, 1,10);
//		__READ_COMMAND(cQueue, curCmd);
	}

}

void PIDConfigInit(PIDConfig * cfg, const float Kp, const float Ki, const float Kd) {
	cfg->Kp = Kp;
	cfg->Ki = Ki;
	cfg->Kd = Kd;
	cfg->ek1 = 0;
	cfg->ekSum = 0;
}

void PIDConfigReset(PIDConfig * cfg) {
	cfg->ek1 = 0;
	cfg->ekSum = 0;
}


int8_t dir = 1;
int correction = 0;
void StraightLineMove(uint16_t * last_curTask_tick) {
	__Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ); // polling
	if (HAL_GetTick() - *last_curTask_tick >= 10) {
		dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? 1 : -1; // use only one of the wheel to determine car direction

		angleNow += ((gyroZ >= -4 && gyroZ <= 11) ? 0 : gyroZ); //GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS;
		__PID_Angle(pidConfigAngle, angleNow, correction, dir, newDutyL, newDutyR);
		__SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);

		*last_curTask_tick = HAL_GetTick();
	}
}

void StraightLineMoveObstacle(uint16_t * last_curTask_tick, float * speedScale) {
	__Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ); // polling
	if (HAL_GetTick() - *last_curTask_tick >= 10) {
		dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? 1 : -1; // use only one of the wheel to determine car direction

		angleNow += ((gyroZ >= -4 && gyroZ <= 11) ? 0 : gyroZ); //GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS;
		__PID_Angle(pidConfigAngle, angleNow, correction, dir, newDutyL, newDutyR);

		newDutyL *= *speedScale;
		newDutyR *= *speedScale;
		__SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);

		*last_curTask_tick = HAL_GetTick();
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_runOledTask */
/**
  * @brief  Function implementing the oledTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_runOledTask */
void runOledTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	OLED_ShowString(0, 0, (char *) rxMsg);
//	OLED_ShowString(0, 12, (char *) aRxBuffer);

	// wheel speed test
	snprintf(ch, sizeof(ch), "obs:%-4d", (int)curObsDist);
	OLED_ShowString(0, 12, (char *) ch);
	snprintf(ch, sizeof(ch), "tick:%-6d", curObsTick);
	OLED_ShowString(0, 24, (char *) ch);
//	OLED_Refresh_Gram();
//	HAL_UART_Transmit(&huart3, "test\n", 5, 0xFFFF);
	osDelay(250);
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
//	  	 case 0: // STOP
//	  		__ON_TASK_END(&htim8, prevTask, curTask);
//	  		if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
//	  			__CLEAR_CURCMD(curCmd);
//	  			__ACK_TASK_DONE(&huart3, rxMsg);
//	  		}
//	  		else {
//	  			__READ_COMMAND(cQueue, curCmd);
//	  		}
//	  		 break;
	  	 case 1: //FW
	  	 case 2: //BW
	  		curTask = curCmd.index == 1 ? TASK_MOVE : TASK_MOVE_BACKWARD;
	  		 break;
	  	case 3: //FL manual
		case 4: //FR manual
		case 5: //BL manual
		case 6: //BR manual
			__SET_CMD_CONFIG(cfgs[curCmd.index], &htim8, &htim1, targetAngle);
			if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
			} else __READ_COMMAND(cQueue, curCmd);
			__PEND_CURCMD(curCmd);
			 break;
	  	 case 7: // FL
	  	 case 8: // FR
	  	 case 9: // BL
	  	 case 10: //BR
	  		 __SET_CMD_CONFIG(cfgs[curCmd.index + (curCmd.val == 30 ? 4 : (curCmd.val == 40 ? 8 : 0))], &htim8, &htim1, targetAngle);
	  		__PEND_CURCMD(curCmd);
	  		 curTask = TASK_TURN;
	  		 break;
	  	 case 11: // TL
	  	 case 12: // TR
	  		 __SET_SERVO_TURN_MAX(&htim1, curCmd.index - 11 ? 1 : 0);
	  		 break;
	  	 case 13: // DT, move forward until distance to obstacle
	  		 curTask = TASK_ADC;
	  		 break;
	  	 case 85:
	  	 case 86:
	  	 case 87:
//	  		 if (curCmd.index == 85) Kp = curCmd.val * 100;
//	  		 else if (curCmd.index == 86) Ki = curCmd.val * 10;
//	  		 else if (curCmd.index == 87) Kd = curCmd.val * 10;
	  		 break;
	  	 case 88: // Axxx, rotate left by xxx degree
	  	 case 89: // Cxxx, rotate right by xxx degree
	  		 __SET_SERVO_TURN_MAX(&htim1, curCmd.index - 88);
	  		 __SET_MOTOR_DIRECTION(DIR_FORWARD);
	  		 if (curCmd.index == 88) {
	  			 targetAngle = curCmd.val;
	  			 __SET_MOTOR_DUTY(&htim8, 800, 1200);
	  		 } else {
	  			targetAngle = -curCmd.val;
	  			 __SET_MOTOR_DUTY(&htim8, 1200, 800);
	  		 }
	  		 curTask = TASK_TURN;
	  		 __PEND_CURCMD(curCmd);
	  		 break;
	  	 case 90: //QT forward turn test
	  	 case 91: //XT backward turn test
	  		 __SET_MOTOR_DIRECTION(curCmd.index - 90? DIR_BACKWARD : DIR_FORWARD);
	  		__SET_MOTOR_DUTY(&htim8, initDuty_L, initDuty_R);
	  		curTask = TASK_TURN;
	  		__PEND_CURCMD(curCmd);
	  		 break;
	  	 case 92: // RN, change stop angle -ve
	  	 case 93: // RP, +ve
	  		 targetAngle = curCmd.val * (curCmd.index - 92 ? 1 : -1);
	  		__ACK_TASK_DONE(&huart3, rxMsg);
	  		 __CLEAR_CURCMD(curCmd);
	  		 break;
	  	 case 96: // DL, left duty test
	  		 initDuty_L = curCmd.val * 100;
//	  		initDuty = curCmd.val * 100;
	  		__SET_MOTOR_DUTY(&htim8, curCmd.val * 100, curCmd.val * 100);
	  		__ACK_TASK_DONE(&huart3, rxMsg);
	  		 __CLEAR_CURCMD(curCmd);
	  		 break;
	  	 case 97: // DR, right duty test
	  		 initDuty_R = curCmd.val * 100;
	  		__ACK_TASK_DONE(&huart3, rxMsg);
	  		 __CLEAR_CURCMD(curCmd);
	  		 break;
	  	 case 98: // QQ, steering test
	  		 // angle 50-75-115
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

/* USER CODE BEGIN Header_runADCTask */
/**
* @brief Function implementing the ADCTask thread.
* @param argument: Not used
* @retval None
* Greedy Task (can only preempted by UART IRQ or EXTI)
* When activate (curTask == TASK_ADC), function executes in 1MHz
*/
/* USER CODE END Header_runADCTask */

void runADCTask(void *argument)
{
  /* USER CODE BEGIN runADCTask */
	uint16_t dataPoint = 0;
	float targetDist = 0;
	uint16_t prevTargetDist = 0;
	uint32_t IR_data_raw_acc = 0;
	uint8_t firstRead = 0;
	float speedScale = 1;
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_ADC) osDelay(1000);
	  else {
		  if (prevTask != TASK_ADC || prevTargetDist == targetDist) {
			IR_data_raw_acc = 0;
			curObsDist = 255;
			targetDist = curCmd.val + 3.5;
			angleNow = 0;
			firstRead = 0;
			PIDConfigReset(&pidConfigAngle);
			__SET_CMD_CONFIG_WODUTY(cfgs[1], &htim1, targetAngle);
			last_curTask_tick = HAL_GetTick();
			__PEND_CURCMD(curCmd);

			do {
			  __ADC_Read_Dist(&hadc1, dataPoint, IR_data_raw_acc, curObsDist, targetDist, curObsTick);
			  if (abs(curObsDist - targetDist) < 0.01) break;
			  __SET_MOTOR_DIRECTION(curObsDist >= targetDist);

			  if (!firstRead && curObsDist != 255) firstRead = 1;

			  if (firstRead) {
				  speedScale = abs(curObsDist - targetDist) / 20; // start slow down from 20cm to target distance
				  speedScale = speedScale > 1 ? 1 : (speedScale < 0.6 ? 0.6 : speedScale);
				  StraightLineMoveObstacle(&last_curTask_tick, &speedScale);
			  }

//			  osDelay(1);
			} while (1);
		  }

		  __ON_TASK_END(&htim8, prevTask, curTask);
		  HAL_ADC_Stop(&hadc1);
		  prevTargetDist = targetDist;
		  clickOnce = 0;

		if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, rxMsg);

		} else __READ_COMMAND(cQueue, curCmd);
	  }

  }
  /* USER CODE END runADCTask */
}

/* USER CODE BEGIN Header_runMoveManualTask */
/**
* @brief Function implementing the moveManualTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runMoveManualTask */
void runMoveManualTask(void *argument)
{
  /* USER CODE BEGIN runMoveManualTask */
  /* Infinite loop */
  for(;;)
  {
	  if (!manualMode || (curTask != TASK_MOVE && curTask != TASK_MOVE_BACKWARD)) osDelay(1000);
	  else {
		  __SET_CMD_CONFIG_WODUTY(cfgs[curCmd.index], &htim1, targetAngle);

		  if ((prevTask == TASK_MOVE && curCmd.index != 1) || (prevTask == TASK_MOVE_BACKWARD && curCmd.index != 2)) {
			angleNow = 0;
			PIDConfigReset(&pidConfigAngle);
			__SET_MOTOR_DUTY(&htim8, cfgs[curCmd.index].leftDuty, cfgs[curCmd.index].rightDuty);
		 } else __SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);

		  __CLEAR_CURCMD(curCmd);
		  __ACK_TASK_DONE(&huart3, rxMsg);


		  do {
			  if (curTask != TASK_MOVE && curTask != TASK_MOVE_BACKWARD) break;
			  StraightLineMove(&last_curTask_tick);
			osDelay(1);
		  } while (1);


		  __ON_TASK_END(&htim8, prevTask, curTask);
		  clickOnce = 0;
	  }
  }
  /* USER CODE END runMoveManualTask */
}

/* USER CODE BEGIN Header_runTurnTask */
/**
* @brief Function implementing the turnTask thread.
* @param argument: Not used
* @retval None
* Greedy Task (can only preempted by UART IRQ or EXTI)
*/
/* USER CODE END Header_runTurnTask */
void runTurnTask(void *argument)
{
  /* USER CODE BEGIN runTurnTask */
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_TURN) osDelay(1000);
	  else {
		  angleNow = 0;
		  last_curTask_tick = HAL_GetTick();
		  do {
			  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);

			  if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
				  angleNow += (abs(gyroZ) < 20 ? 0 : gyroZ) / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
				  if (angleNow >= targetAngle) break;
				  last_curTask_tick = HAL_GetTick();
			  }

		  } while(1);

		  __ON_TASK_END(&htim8, prevTask, curTask);
		  clickOnce = 0;

		  if (!manualMode) {
			  __RESET_SERVO_TURN(&htim1);
			  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
			}
			else __READ_COMMAND(cQueue, curCmd);
		  }
	  }

  }
  /* USER CODE END runTurnTask */
}

/* USER CODE BEGIN Header_runMoveDistTask */
/**
* @brief Function implementing the moveDistTask thread.
* @param argument: Not used
* @retval None
* Greedy Task (can only preempted by UART IRQ or EXTI)
*/
/* USER CODE END Header_runMoveDistTask */
void runMoveDistTask(void *argument)
{
  /* USER CODE BEGIN runMoveDistTask */
	int lastDistTick_L = 0, lastDistTick_R = 0;
	uint16_t targetTick = 0, posTickNow = 0, dist_dL = 0, dist_dR = 0;

  /* Infinite loop */
  for(;;)
  {
	  if (manualMode || (curTask != TASK_MOVE && curTask != TASK_MOVE_BACKWARD)) osDelay(1000);
	  else {
		  targetTick = (curCmd.val * DIST_M - DIST_C) / WHEEL_LENGTH * 1320;

		  __SET_CMD_CONFIG_WODUTY(cfgs[curCmd.index], &htim1, targetAngle);

		  if ((prevTask == TASK_MOVE && curCmd.index != 1) || (prevTask == TASK_MOVE_BACKWARD && curCmd.index != 2)) {
			angleNow = 0;
			PIDConfigReset(&pidConfigAngle);
			__SET_MOTOR_DUTY(&htim8, cfgs[curCmd.index].leftDuty, cfgs[curCmd.index].rightDuty);
		 } else __SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);

		  __SET_ENCODER_LAST_TICKS(&htim2, lastDistTick_L, &htim3, lastDistTick_R);

		  __PEND_CURCMD(curCmd);

		  last_curTask_tick = HAL_GetTick();
		  do {
			  if (curTask != TASK_MOVE && curTask != TASK_MOVE_BACKWARD) break;
			  __GET_ENCODER_TICK_DELTA(&htim2, lastDistTick_L, dist_dL);
			  __GET_ENCODER_TICK_DELTA(&htim3, lastDistTick_R, dist_dR);

			  posTickNow += dist_dL; // use just left wheel to check traveled distance
			  if (posTickNow >= targetTick) break;

			  __SET_ENCODER_LAST_TICKS(&htim2, lastDistTick_L, &htim3, lastDistTick_R);

			  StraightLineMove(&last_curTask_tick);

		  } while (1);

		  __ON_TASK_END(&htim8, prevTask, curTask);
		  clickOnce = 0;
		  lastDistTick_L = 0; lastDistTick_R = 0;
		  targetTick = 0; posTickNow = 0;

		if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, rxMsg);
		} else __READ_COMMAND(cQueue, curCmd);
	  }
  }
  /* USER CODE END runMoveDistTask */
}

/* USER CODE BEGIN Header_runAlignTask */
/**
* @brief Function implementing the alignTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runAlignTask */
void runAlignTask(void *argument)
{
  /* USER CODE BEGIN runAlignTask */
	uint8_t angle_left = 0, angle_right = 0;
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_ALIGN) osDelay(1000);
	  else {
		  angleNow = 0;
		  angle_left = 0; angle_right = 0;
		  do {
			  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);

			  if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
				  angleNow += (abs(gyroZ) < 20 ? 0 : gyroZ) / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
				  if (angleNow >= targetAngle) break;
				  last_curTask_tick = HAL_GetTick();
			  }
			  osDelay(1);
		  } while (1);



	  }

  }
  /* USER CODE END runAlignTask */
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

