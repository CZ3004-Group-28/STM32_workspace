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
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
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
/* Definitions for fastestPathTask */
osThreadId_t fastestPathTaskHandle;
const osThreadAttr_t fastestPathTask_attributes = {
  .name = "fastestPathTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for buzzerTask */
osThreadId_t buzzerTaskHandle;
const osThreadAttr_t buzzerTask_attributes = {
  .name = "buzzerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FLTask */
osThreadId_t FLTaskHandle;
const osThreadAttr_t FLTask_attributes = {
  .name = "FLTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FRTask */
osThreadId_t FRTaskHandle;
const osThreadAttr_t FRTask_attributes = {
  .name = "FRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BLTask */
osThreadId_t BLTaskHandle;
const osThreadAttr_t BLTask_attributes = {
  .name = "BLTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BRTask */
osThreadId_t BRTaskHandle;
const osThreadAttr_t BRTask_attributes = {
  .name = "BRTask",
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
float targetAngle = 0;
float angleNow = 0;
uint8_t readGyroZData[2];
int16_t gyroZ;
uint16_t newDutyL = 1200, newDutyR = 1200;
uint32_t last_curTask_tick = 0;

float targetDist = 0;
uint16_t curDistTick = 0;
uint16_t targetDistTick = 0;
uint16_t dist_dL = 0;
uint16_t lastDistTick_L = 0;

typedef struct _pidConfig {
	float Kp;
	float Ki;
	float Kd;
	float ek1;
	float ekSum;
} PIDConfig;

PIDConfig pid1200, pid2500;

typedef struct _commandConfig {
	uint16_t leftDuty;
	uint16_t rightDuty;
	float servoTurnVal;
	float targetAngle;
	uint8_t direction;
} CmdConfig;

#define CONFIG_FL20 7
#define CONFIG_FR20 8
#define CONFIG_BL20 9
#define CONFIG_BR20 10

#define CONFIG_FL30 11
#define CONFIG_FR30 12
#define CONFIG_BL30 13
#define CONFIG_BR30 14

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

	{700, 1800, 50, 89, DIR_FORWARD}, // FLxx
	{1800, 400, 115 ,-87, DIR_FORWARD}, // FRxx
	{500, 1700, 50, -88, DIR_BACKWARD}, // BLxx
	{1800, 500, 115, 89, DIR_BACKWARD}, // BRxx,

	{1500, 1500, 53, 87.5, DIR_FORWARD}, // FL30
	{1500, 1500, 108, -86.5, DIR_FORWARD}, // FR30
	{1500, 1500, 51, -87.5, DIR_BACKWARD}, // BL30
	{1500, 1100, 115, 88, DIR_BACKWARD}, // BR30

//	{2000, 2000, 53.5, 87.5, DIR_FORWARD}, // FL30
//	{2000, 2000, 109, -86.5, DIR_FORWARD}, // FR30
};

enum TASK_TYPE{
	TASK_MOVE,
	TASK_MOVE_BACKWARD,
	TASK_FL,
	TASK_FR,
	TASK_BL,
	TASK_BR,
	TASK_TURN,
	TASK_ADC,
	TASK_MOVE_OBS,
	TASK_ALIGN,
	TASK_FASTESTPATH,
	TASK_BUZZER,
	TASK_NONE
};

HAL_StatusTypeDef UART_STATUS;

enum TASK_TYPE curTask = TASK_NONE, prevTask = TASK_NONE;

char temp[20];

float obsDistL = 0, obsDistR = 0;
uint16_t obsTickL = 0, obsTickR = 0;

float obsDist_IR = 0, obsDist_US = 0;
float angle_left = 0, angle_right = 0;
//uint16_t curObsTick = 0;

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
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
void runOledTask(void *argument);
void runCmdTask(void *argument);
void runADCTask(void *argument);
void runMoveManualTask(void *argument);
void runTurnTask(void *argument);
void runMoveDistTask(void *argument);
void runAlignTask(void *argument);
void runFastestPathTask(void *argument);
void runBuzzerTask(void *argument);
void runFLTask(void *argument);
void runFRTask(void *argument);
void runBLTask(void *argument);
void runBRTask(void *argument);

/* USER CODE BEGIN PFP */
void PIDConfigInit(PIDConfig * cfg, const float Kp, const float Ki, const float Kd);
void PIDConfigReset(PIDConfig * cfg);

void StraightLineMove(const uint8_t speedMode);
void StraightLineMoveSpeedScale(const uint8_t speedMode, float * speedScale);
void RobotMoveDist(float * targetDist, const uint8_t dir, const uint8_t speedMode);
void RobotMoveDistObstacle(uint32_t * last_curTask_tick);

void RobotTurn(float * targetAngle);

//void MoveDist(uint32_t * last_curTask_tick, )
uint32_t IC_Val1 = 0, IC_Val2 = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel2 (TRI: TIM4_CH2)
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			obsDist_US = (IC_Val2 > IC_Val1 ? (IC_Val2 - IC_Val1) : (65535 - IC_Val1 + IC_Val2)) * 0.034 / 2;
//			if (IC_Val2 > IC_Val1)
//			{
//				Difference = IC_Val2-IC_Val1;
//			}
//
//			else if (IC_Val1 > IC_Val2)
//			{
//				Difference = (0xffff - IC_Val1) + IC_Val2;
//			}
//
//			obsDist_IR = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC2);
		}
	}
}
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
  MX_ADC2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  ICM20948_init(&hi2c1,0,GYRO_FULL_SCALE_2000DPS);

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

  PIDConfigInit(&pid1200, 2.5, 0.0,0.8);
  PIDConfigInit(&pid2500, 1.5, 0.0,0);

  	HAL_UART_Receive_IT(&huart3, aRxBuffer,RX_BUFFER_SIZE);

	// servo motor turn
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	// motor backwheel move
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	// encoder monitor speed and distance
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
//	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

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

  /* creation of fastestPathTask */
  fastestPathTaskHandle = osThreadNew(runFastestPathTask, NULL, &fastestPathTask_attributes);

  /* creation of buzzerTask */
  buzzerTaskHandle = osThreadNew(runBuzzerTask, NULL, &buzzerTask_attributes);

  /* creation of FLTask */
  FLTaskHandle = osThreadNew(runFLTask, NULL, &FLTask_attributes);

  /* creation of FRTask */
  FRTaskHandle = osThreadNew(runFRTask, NULL, &FRTask_attributes);

  /* creation of BLTask */
  BLTaskHandle = osThreadNew(runBLTask, NULL, &BLTask_attributes);

  /* creation of BRTask */
  BRTaskHandle = osThreadNew(runBRTask, NULL, &BRTask_attributes);

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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|TRI_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : BUZZER_Pin TRI_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|TRI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
			__READ_COMMAND(cQueue, curCmd, rxMsg);
		}
	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] != 'L' && aRxBuffer[1] != 'R') { //FW
		manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
//		if (aRxBuffer[2] == '-' && aRxBuffer[3] == '-') manualMode = 1;
		__ADD_COMMAND(cQueue, 1, val);
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] != 'L' && aRxBuffer[1] != 'R') { //BW
		manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
//		if (aRxBuffer[2] == '-' && aRxBuffer[3] == '-') manualMode = 1;
		__ADD_COMMAND(cQueue, 2, val);
	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'L') { // FL
		manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		__ADD_COMMAND(cQueue, 3 + (manualMode ? 0 : 4), val);
	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'R') { // FR
		manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		__ADD_COMMAND(cQueue, 4 + (manualMode ? 0 : 4), val);
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'L') { // BL
		manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		__ADD_COMMAND(cQueue, 5 + (manualMode ? 0 : 4), val);
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'R') { // BR
		manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		__ADD_COMMAND(cQueue, 6 + (manualMode ? 0 : 4), val);
	}
	else if (aRxBuffer[0] == 'T' && aRxBuffer[1] == 'L') __ADD_COMMAND(cQueue, 11, val); // TL turn left max
	else if (aRxBuffer[0] == 'T' && aRxBuffer[1] == 'R') __ADD_COMMAND(cQueue, 12, val); // TR turn right max
	else if (aRxBuffer[0] == 'D' && aRxBuffer[1] == 'T') __ADD_COMMAND(cQueue, 13, val); // DT move until distance to obstacle
	else if (aRxBuffer[0] == 'X' && aRxBuffer[1] == 'X') __ADD_COMMAND(cQueue, 14, val); // XX alignment
	else if (aRxBuffer[0] == 'Z' && aRxBuffer[1] == 'Z') __ADD_COMMAND(cQueue, 15, val); // ZZ buzzer
	else if (aRxBuffer[0] == 'W' && aRxBuffer[1] == 'N') __ADD_COMMAND(cQueue, 16, val); // WN fastest path
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
		__READ_COMMAND(cQueue, curCmd, rxMsg);
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
int routineCount = 0;
uint8_t tempDir = 1 ;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (clickOnce) return;
	if (GPIO_Pin == SW1_Pin) {
		clickOnce = 1;

		// test routine before connecting to rpi
//		manualMode = 0;
//		if (routineCount == 0) { // DT20, IR Sensor (front) test
//			__ADD_COMMAND(cQueue, 13, 20);
////
//		} else if (routineCount == 1) { // test turn right
//			__ADD_COMMAND(cQueue, 9, 0); // BL00
//
//		} else if (routineCount == 2) { // test turn left
//			__ADD_COMMAND(cQueue, 10, 0); // BR00
//			clickOnce = 0;
//
//		} else if (routineCount == 3) { // test
//			clickOnce = 0;
//		}
//
//		routineCount = (routineCount + 1) % 4;
//
//		if (!__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
//			snprintf((char *)rxMsg, sizeof(rxMsg), "doing");
//			__READ_COMMAND(cQueue, curCmd, rxMsg);
//		}


		// pid test
//		manualMode = 0;
//		__ADD_COMMAND(cQueue, tempDir, targetD); // FW--
////		tempDir = tempDir == 1 ? 2 : 1;
//		targetD = (targetD + 5) % 85;
//		__READ_COMMAND(cQueue, curCmd, rxMsg);

		// test 5cm step forward
		manualMode = 0;
		__ADD_COMMAND(cQueue, 1, targetD); // FW--
		targetD = (targetD + 5) % 105;
		__READ_COMMAND(cQueue, curCmd, rxMsg);

		// turn test
//		manualMode = 0;
//		__ADD_COMMAND(cQueue, 9, 0); // BLxx
//		__READ_COMMAND(cQueue, curCmd, rxMsg);

		// align test
//		manualMode = 0;
//		__ADD_COMMAND(cQueue, 18, 0); // XX align task
//		__READ_COMMAND(cQueue, curCmd, rxMsg);

		// ADC test
//		manualMode = 0;
//		__ADD_COMMAND(cQueue, 17, 50);
//		__READ_COMMAND(cQueue, curCmd, rxMsg);

		// buzzer test
//		__ADD_COMMAND(cQueue, 19, 2);
//		__READ_COMMAND(cQueue, curCmd, rxMsg);

		// fastest path test
//		__ADD_COMMAND(cQueue, 20, 0);
//		__READ_COMMAND(cQueue, curCmd, rxMsg);

		//test turn 30cm
		manualMode = 0;
//		if (routineCount == 0) { // FL00
//			clickOnce = 1;
//			__ADD_COMMAND(cQueue, 7, 20);
//
//		} else
//			if (routineCount == 0) { // FR00
//			clickOnce = 1;
//			__ADD_COMMAND(cQueue, 8, 20);
//
//		} else
//			if (routineCount == 0) { // BL00
//			clickOnce = 1;
//			__ADD_COMMAND(cQueue, 9, 20);
//
//		} else
//			if (routineCount == 0) { // BR00
//			clickOnce = 1;
//			__ADD_COMMAND(cQueue, 10, 20);
//		}

//		routineCount = (routineCount + 1) %1;
//		__READ_COMMAND(cQueue, curCmd, rxMsg);
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
//PIDConfig curPIDConfig;

void StraightLineMove(const uint8_t speedMode) {
		__Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ); // polling
		dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? 1 : -1; // use only one of the wheel to determine car direction
		angleNow += ((gyroZ >= -4 && gyroZ <= 11) ? 0 : gyroZ); // / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
		if (speedMode == SPEED_MODE_FAST) {
			__PID_Angle_FAST(pid2500, angleNow, correction, dir, newDutyL, newDutyR);
		} else {
			__PID_Angle_SLOW(pid1200, angleNow, correction, dir, newDutyL, newDutyR);
		}
		__SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);
}

void StraightLineMoveSpeedScale(const uint8_t speedMode, float * speedScale) {
	__Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ); // polling
	dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? 1 : -1; // use only one of the wheel to determine car direction
	angleNow += ((gyroZ >= -4 && gyroZ <= 11) ? 0 : gyroZ); // / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
	if (speedMode == SPEED_MODE_FAST) {
		__PID_Angle_FAST(pid2500, angleNow, correction, dir, newDutyL, newDutyR);
	} else {
		__PID_Angle_SLOW(pid1200, angleNow, correction, dir, newDutyL, newDutyR);
	}
	__SET_MOTOR_DUTY(&htim8, newDutyL * (*speedScale), newDutyR * (*speedScale));
	}

float scale = 1;
void RobotMoveDist(float * targetDist, const uint8_t dir, const uint8_t speedMode) {
	angleNow = 0; gyroZ = 0; // reset angle for PID
	curDistTick = 0;

	__GET_TARGETTICK(*targetDist, targetDistTick);
//	targetDistTick = (*targetDist) / 20 * 1320;
	last_curTask_tick = HAL_GetTick();
	__SET_MOTOR_DIRECTION(dir);
	__SET_ENCODER_LAST_TICK(&htim2, lastDistTick_L);
	do {
		__GET_ENCODER_TICK_DELTA(&htim2, lastDistTick_L, dist_dL);
		curDistTick += dist_dL;

		if (curDistTick >= targetDistTick) break;

		if (HAL_GetTick() - last_curTask_tick >= 10) {
			if (speedMode == SPEED_MODE_SLOW) {
				StraightLineMove(speedMode);
			} else {
				scale = abs(curDistTick - targetDistTick) / 990; // start to slow down at last 990 ticks (15cm)
				scale = scale > 1 ? 1 : (scale < MIN_SPEED_SCALE ? MIN_SPEED_SCALE : scale);
				StraightLineMoveSpeedScale(speedMode, &scale);
			}
			last_curTask_tick = HAL_GetTick();
		}
	} while (1);
	__SET_MOTOR_DUTY(&htim8, 0, 0);

}

void RobotTurn(float * targetAngle) {
	angleNow = 0; gyroZ = 0;
	last_curTask_tick = HAL_GetTick();
	do {
	  if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
		  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
		  angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
		  if (abs(angleNow - *targetAngle) < 0.01) break;
		  last_curTask_tick = HAL_GetTick();
	  }
	} while(1);
	__SET_MOTOR_DUTY(&htim8, 0, 0);
	__RESET_SERVO_TURN(&htim1);
}



void FASTESTPATH_MOVE_DIST(float * targetDist, int * lastDistTick_L, uint16_t * posTickNow, uint16_t * dist_dL) {
//	__SET_ENCODER_LAST_TICKS(&htim2, *lastDistTick_L, &htim3, *lastDistTick_R);
	__SET_ENCODER_LAST_TICK(&htim2, *lastDistTick_L);
	*posTickNow = 0;
	__GET_TARGETTICK(*targetDist, targetDistTick);
//	targetDistTick = (*targetDist * DIST_M - DIST_C) / WHEEL_LENGTH * 1320;
	last_curTask_tick = HAL_GetTick();

	do {
	  __GET_ENCODER_TICK_DELTA(&htim2, *lastDistTick_L, *dist_dL);
//			  __GET_ENCODER_TICK_DELTA(&htim3, lastDistTick_R, dist_dR);

	  *posTickNow += *dist_dL; // use just left wheel to check traveled distance
	  if (*posTickNow >= targetDistTick) break;
//	  __SET_ENCODER_LAST_TICKS(&htim2, *lastDistTick_L, &htim3, *lastDistTick_R);
	  __SET_ENCODER_LAST_TICK(&htim2, *lastDistTick_L);
	  if (HAL_GetTick() - last_curTask_tick >= 10) {
		  StraightLineMove(SPEED_MODE_SLOW);
		  last_curTask_tick = HAL_GetTick();
	  }

	} while (1);
	__SET_MOTOR_DUTY(&htim8, 0, 0);
}

void FASTESTPATH_MOVE_DIST_OBSTACLE(const float targetDist) {
	obsDist_US = 255;
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
	last_curTask_tick = HAL_GetTick();
	do {
	  HAL_GPIO_WritePin(TRI_GPIO_Port, TRI_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	  __Delay_us_TIM4(&htim4, 10); // wait for 10us
	  HAL_GPIO_WritePin(TRI_GPIO_Port, TRI_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
	  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC2);
	  __SET_MOTOR_DIRECTION(obsDist_US >= targetDist);
	  if (HAL_GetTick() - last_curTask_tick >=10) {
		  StraightLineMove(SPEED_MODE_SLOW);
		  last_curTask_tick = HAL_GetTick();
	  }

	} while (abs(obsDist_US - targetDist) > 0.1);
	__SET_MOTOR_DUTY(&htim8, 0, 0);
	HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_2);
}

void FASTESTPATH_TURN_LEFT() {
	targetAngle = 87.5;
	__SET_SERVO_TURN(&htim1, 53.75);
	__SET_MOTOR_DIRECTION(1);
	angleNow = 0; gyroZ = 0;
	__SET_MOTOR_DUTY(&htim8, 2000, 2000);
	last_curTask_tick = HAL_GetTick();
	do {
	  if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
		  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
		  angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
		  if (abs(angleNow - targetAngle) < 0.01) break;
		  last_curTask_tick = HAL_GetTick();
	  }
	} while(1);
	__SET_MOTOR_DUTY(&htim8, 0, 0);
}

void FASTESTPATH_TURN_RIGHT() {
	targetAngle = -87;
	__SET_SERVO_TURN(&htim1, 109);
	__SET_MOTOR_DIRECTION(1);
	angleNow = 0; gyroZ = 0;
	__SET_MOTOR_DUTY(&htim8, 2000, 2000);
	last_curTask_tick = HAL_GetTick();
	do {
	  if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
		  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
		  angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
		  if (abs(angleNow - targetAngle) < 0.01) break;
		  last_curTask_tick = HAL_GetTick();
	  }
	} while(1);
	__SET_MOTOR_DUTY(&htim8, 0, 0);
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
	OLED_ShowString(0, 0, (char *) rxMsg);
	OLED_ShowString(0, 12, (char *) aRxBuffer);

	// wheel speed test
//	snprintf(ch, sizeof(ch), "angle:%-6d", (int)angleNow);
//	OLED_ShowString(0, 24, (char *) ch);
	snprintf(ch, sizeof(ch), "tdis:%-6d", (int)targetD);
	OLED_ShowString(0, 36, (char *) ch);
	snprintf(ch, sizeof(ch), "tick:%-6d", (int)targetDistTick);
	OLED_ShowString(0, 48, (char *) ch);
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
//	  	 case 0: // STOP handled in UART IRQ directly
//	  	  	  break;
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
			} else __READ_COMMAND(cQueue, curCmd, rxMsg);
			__PEND_CURCMD(curCmd);
			 break;
	  	 case 7: // FL
	  		 curTask = TASK_FL;
	  		__PEND_CURCMD(curCmd);
	  		 break;
	  	 case 8: // FR
	  		curTask = TASK_FR;
	  		__PEND_CURCMD(curCmd);
	  		break;
	  	 case 9: // BL
	  		curTask = TASK_BL;
	  		__PEND_CURCMD(curCmd);
	  		break;
	  	 case 10: //BR
	  		curTask = TASK_BR;
	  		__PEND_CURCMD(curCmd);
	  		break;
	  	 case 11: // TL
	  	 case 12: // TR
	  		 __SET_SERVO_TURN_MAX(&htim1, curCmd.index - 11 ? 1 : 0);
	  		 break;
	  	 case 13: // DT, move forward until distance to obstacle
	  		 curTask = TASK_ADC;
	  		 break;
	  	 case 14:
	  		 curTask = TASK_ALIGN;
	  		__PEND_CURCMD(curCmd);
	  		 break;
	  	 case 15:
	  		 curTask = TASK_BUZZER;
	  		__PEND_CURCMD(curCmd);
	  		break;
	  	 case 16:
	  		 curTask = TASK_FASTESTPATH;
	  		__PEND_CURCMD(curCmd);
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
	  		curTask = TASK_TURN;
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
	uint16_t dataPointL = 0; uint32_t IR_data_raw_acc_L = 0;
//	uint16_t dataPointR = 0; uint32_t IR_data_raw_acc_R = 0;
//	float targetDist = 0;

	uint8_t firstRead = 0;
//	float speedScale = 1;
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_ADC) osDelay(1000);
	  else {
//		  if (prevTask != TASK_ADC) {
			dataPointL = 0; IR_data_raw_acc_L = 0; obsDistL = 255;
//			dataPointR = 0; IR_data_raw_acc_R = 0; obsDistR = 255;
//			targetDist = curCmd.val + 7;
			angleNow = 0;
			gyroZ = 0;
			PIDConfigReset(&pid1200);
//			firstRead = 0;
//			__SET_CMD_CONFIG_WODUTY(cfgs[1], &htim1, targetAngle);
			last_curTask_tick = HAL_GetTick();
			__PEND_CURCMD(curCmd);

			do {
				__ADC_Read_Dist(&hadc1, dataPointL, IR_data_raw_acc_L, obsDistL, obsTickL);
//				__ADC_Read_Dist(&hadc2, dataPointR, IR_data_raw_acc_R, obsDistR, obsTickR);

//			  if (abs(obsDistL - targetDist) < 0.01) break;
//			  __SET_MOTOR_DIRECTION(obsDistL >= targetDist);

//			  if (!firstRead && obsDistL != 255) firstRead = 1;

//			  if (firstRead) {
//				  speedScale = abs(obsDistL - targetDist) / 20; // start slow down from 20cm to target distance
//				  speedScale = speedScale > 1 ? 1 : (speedScale < 0.6 ? 0.6 : speedScale);
//				  StraightLineMoveSpeedScale(&last_curTask_tick, &speedScale);
//			  }

			  osDelay(1);
			} while (1);
//		  }

		  __ON_TASK_END(&htim8, prevTask, curTask);
		  HAL_ADC_Stop(&hadc1);
//		  HAL_ADC_Stop(&hadc2);
//		  prevTargetDist = targetDist;
		  clickOnce = 0;

		if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, rxMsg);

		} else __READ_COMMAND(cQueue, curCmd, rxMsg);
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
			angleNow = 0; gyroZ = 0;
			PIDConfigReset(&pid1200);
			__SET_MOTOR_DUTY(&htim8, cfgs[curCmd.index].leftDuty, cfgs[curCmd.index].rightDuty);
		 } else __SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);

		  __CLEAR_CURCMD(curCmd);
		  __ACK_TASK_DONE(&huart3, rxMsg);

		  last_curTask_tick = HAL_GetTick();
		  do {
			  if (curTask != TASK_MOVE && curTask != TASK_MOVE_BACKWARD) break;

			  if (HAL_GetTick() - last_curTask_tick >= 10) {
				  StraightLineMove(SPEED_MODE_SLOW);
				  last_curTask_tick = HAL_GetTick();
			  }
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
		  __SET_CMD_CONFIG(cfgs[curCmd.index + (curCmd.val == 30 ? 4 : (curCmd.val == 40 ? 8 : 0))], &htim8, &htim1, targetAngle);
		  __PEND_CURCMD(curCmd);
		  RobotTurn(&targetAngle);

		  clickOnce = 0;
		  __ON_TASK_END(&htim8, prevTask, curTask);
		  PIDConfigReset(&pid1200);

		  __RESET_SERVO_TURN(&htim1);
		  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, rxMsg);
		}
		else __READ_COMMAND(cQueue, curCmd, rxMsg);

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
  /* Infinite loop */
  for(;;)
  {
	  if (manualMode || (curTask != TASK_MOVE && curTask != TASK_MOVE_BACKWARD)) osDelay(1000);
	  else {
		  __PEND_CURCMD(curCmd);
		  targetDist = (float) curCmd.val;
		  RobotMoveDist(&targetDist, curTask == TASK_MOVE ? DIR_FORWARD : DIR_BACKWARD, targetDist > 15 ? SPEED_MODE_FAST : SPEED_MODE_SLOW);

		  __ON_TASK_END(&htim8, prevTask, curTask);
		  clickOnce = 0;

		if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, rxMsg);
		} else __READ_COMMAND(cQueue, curCmd, rxMsg);
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
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_ALIGN) osDelay(1000);
	  else {
		  __ON_TASK_END(&htim8, prevTask, curTask);
		  clickOnce = 0;
//		  angleNow = 0;

		if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, rxMsg);
		} else __READ_COMMAND(cQueue, curCmd, rxMsg);

	  }

  }
  /* USER CODE END runAlignTask */
}

/* USER CODE BEGIN Header_runFastestPathTask */
/**
* @brief Function implementing the fastestPathTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runFastestPathTask */
void runFastestPathTask(void *argument)
{
  /* USER CODE BEGIN runFastestPathTask */
	int lastDistTick_L = 0, lastDistTick_R = 0;
	uint16_t posTickNow = 0, dist_dL = 0, dist_dR = 0;
	float targetDist = 0;
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_FASTESTPATH) osDelay(1000);
	  else {

		  // STEP 1: move forward until 30cm behind the obstacle
		  FASTESTPATH_MOVE_DIST_OBSTACLE(30);

		  // STEP 2: turn right (create ~2.8cm offset horizontally)
		  FASTESTPATH_TURN_RIGHT();

		  // STEP 3: move forward by 12.2cm (also clear previous 2.8 offset)
		  targetDist = 12.2;
		  FASTESTPATH_MOVE_DIST(&targetDist, &lastDistTick_L, &posTickNow, &dist_dL);

		  // STEP 4: turn left (create ~3.5cm offset vertically)
		  FASTESTPATH_TURN_LEFT();

		  // STEP 5: turn left (create ~3.5cm offset horizontally)
		  FASTESTPATH_TURN_LEFT();

		  // STEP 6: move left by 86.5cm (also clear previous ~3.5cm horizontal offset)
		  targetDist = 86.5;
		  FASTESTPATH_MOVE_DIST(&targetDist, &lastDistTick_L, &posTickNow, &dist_dL);

		  // STEP 7: turn left (also clear previous ~3.5cm vertical offset)
		  FASTESTPATH_TURN_LEFT();

		  // STEP 8: turn left (create ~3.5cm offset horizontally)
		  FASTESTPATH_TURN_LEFT();

		  // STEP 9: move forward by 11.5cm (also clear previous ~3.5cm offset)
		  targetDist = 11.5;
		  FASTESTPATH_MOVE_DIST(&targetDist, &lastDistTick_L, &posTickNow, &dist_dL);

		  // STEP 10: turn right (back to initial path)
		  FASTESTPATH_TURN_RIGHT();

		  // STEP 11: move back to carpack
		  FASTESTPATH_MOVE_DIST_OBSTACLE(10);


		  clickOnce = 0;
		  prevTask = curTask;
		  curTask = TASK_NONE;
		  __CLEAR_CURCMD(curCmd);
		  __ACK_TASK_DONE(&huart3, rxMsg);

	  }
  }
  /* USER CODE END runFastestPathTask */
}

/* USER CODE BEGIN Header_runBuzzerTask */
/**
* @brief Function implementing the buzzerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runBuzzerTask */
void runBuzzerTask(void *argument)
{
  /* USER CODE BEGIN runBuzzerTask */
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_BUZZER) osDelay(1000);
	  else {
		  while (curCmd.val > 0) {
			  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			  osDelay(100);
			  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			  osDelay(100);
			  curCmd.val--;
		  }
		  prevTask = curTask;
		  curTask = TASK_NONE;
		  clickOnce = 0;

		  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
			} else __READ_COMMAND(cQueue, curCmd, rxMsg);
	  }
  }
  /* USER CODE END runBuzzerTask */
}

/* USER CODE BEGIN Header_runFLTask */
/**
* @brief Function implementing the FLTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runFLTask */
void runFLTask(void *argument)
{
  /* USER CODE BEGIN runFLTask */
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_FL) osDelay(1000);
	  else {
		  switch(curCmd.val) {
		  case 30: // FL30
			  __SET_CMD_CONFIG(cfgs[CONFIG_FL30], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle);
			  osDelay(10);
			  targetDist = 4;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_SLOW);
			  osDelay(10);
			  break;
		  default: // FL20 or FL00
			  targetDist = 4;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_SLOW);
			  osDelay(10);
			  __SET_CMD_CONFIG(cfgs[CONFIG_FL20], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle);
			  osDelay(10);
			  targetDist = 7;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_SLOW);
			  osDelay(10);
			  break;
		  }


		  clickOnce = 0;
		  prevTask = curTask;
		  curTask = TASK_NONE;
		  PIDConfigReset(&pid1200);
		  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
		  } else __READ_COMMAND(cQueue, curCmd, rxMsg);
	  }
  }
  /* USER CODE END runFLTask */
}

/* USER CODE BEGIN Header_runFRTask */
/**
* @brief Function implementing the FRTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runFRTask */
void runFRTask(void *argument)
{
  /* USER CODE BEGIN runFRTask */
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_FR) osDelay(1000);
	  	  else {
	  		  switch(curCmd.val) {
	  		  case 30: // FR30
	  			  __SET_CMD_CONFIG(cfgs[CONFIG_FR30], &htim8, &htim1, targetAngle);
	  			  RobotTurn(&targetAngle);
				  osDelay(10);
				  targetDist = 4;
				  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_SLOW);
				  osDelay(10);
	  			  break;
	  		  default: // FR20 or FR00
	  			  targetDist = 3;
	  			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_SLOW);
	  			  osDelay(10);
	  			  __SET_CMD_CONFIG(cfgs[CONFIG_FR20], &htim8, &htim1, targetAngle);
	  			  RobotTurn(&targetAngle);
	  			  osDelay(10);
	  			  targetDist = 6.5;
	  			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_SLOW);
	  			  osDelay(10);
	  			  break;
	  		  }


	  		  clickOnce = 0;
	  		  prevTask = curTask;
	  		  curTask = TASK_NONE;
	  		  PIDConfigReset(&pid1200);
	  		  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
	  				__CLEAR_CURCMD(curCmd);
	  				__ACK_TASK_DONE(&huart3, rxMsg);
	  		  } else __READ_COMMAND(cQueue, curCmd, rxMsg);
	  	  }
  }
  /* USER CODE END runFRTask */
}

/* USER CODE BEGIN Header_runBLTask */
/**
* @brief Function implementing the BLTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runBLTask */
void runBLTask(void *argument)
{
  /* USER CODE BEGIN runBLTask */
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_BL) osDelay(1000);
	  else {
		  switch(curCmd.val) {
		  case 30: // BL30
			  __SET_CMD_CONFIG(cfgs[CONFIG_BL30], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle);
			  osDelay(10);
			  targetDist = 4.5;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_SLOW);
			  osDelay(10);
			  break;
		  default: // BL20 or BL00
			  targetDist = 6;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_SLOW);
			  osDelay(10);
			  __SET_CMD_CONFIG(cfgs[CONFIG_BL20], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle);
			  osDelay(10);
			  targetDist = 2.5;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_SLOW);
			  osDelay(10);
			  break;
		  }


		  clickOnce = 0;
		  prevTask = curTask;
		  curTask = TASK_NONE;
		  PIDConfigReset(&pid1200);
		  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
		  } else __READ_COMMAND(cQueue, curCmd, rxMsg);
	  }
  }
  /* USER CODE END runBLTask */
}

/* USER CODE BEGIN Header_runBRTask */
/**
* @brief Function implementing the BRTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runBRTask */
void runBRTask(void *argument)
{
  /* USER CODE BEGIN runBRTask */
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_BR) osDelay(1000);
	  else {
		  switch(curCmd.val) {
		  case 30: // BR30
			  __SET_CMD_CONFIG(cfgs[CONFIG_BR30], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle);
			  osDelay(10);
			  targetDist = 5;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_SLOW);
			  osDelay(10);
			  break;
		  default: // BR20 or BR00
			  targetDist = 7;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_SLOW);
			  osDelay(10);
			  __SET_CMD_CONFIG(cfgs[CONFIG_BR20], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle);
			  osDelay(10);
			  targetDist = 3;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_SLOW);
			  osDelay(10);
			  break;
		  }


		  clickOnce = 0;
		  prevTask = curTask;
		  curTask = TASK_NONE;
		  PIDConfigReset(&pid1200);
		  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
		  } else __READ_COMMAND(cQueue, curCmd, rxMsg);
	  }
  }
  /* USER CODE END runBRTask */
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

