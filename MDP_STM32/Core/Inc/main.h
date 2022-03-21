/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define PI 3.141592654
#define WHEEL_LENGTH 20
#define PPR 330
// distance calibration params
//#define DIST_M 1.14117166
//#define DIST_C 1.232534228
#define SPEED_MODE_T 0
#define SPEED_MODE_1 1
#define SPEED_MODE_2 2

#define DIST_M 1.150067316
#define DIST_C 0.965311399

#define INIT_DUTY_SPT_L 1200
#define INIT_DUTY_SPT_R 1200
#define DUTY_SPT_RANGE 600

#define INIT_DUTY_SP1_L 2300
#define INIT_DUTY_SP1_R 2300
#define DUTY_SP1_RANGE 700

#define INIT_DUTY_SP2_L 3000
#define INIT_DUTY_SP2_R 3000
#define DUTY_SP2_RANGE 700

#define DIR_FORWARD 1
#define DIR_BACKWARD 0

#define SERVO_LEFT_MAX 50
#define SERVO_CENTER 74
#define SERVO_RIGHT_MAX 115

#define IR_CONST_A 25644.81557
#define IR_CONST_B 260.4233354
#define IR_SAMPLE 100

#define MIN_SPEED_SCALE 0.4 // INIT_DUTY_SP1_L / INIT_DUTY_SP2_L

#define SERVO_TURN_TIME 300

#define __GET_TARGETTICK(dist, targetTick) ({ \
	targetTick = (((dist) * DIST_M - DIST_C) / WHEEL_LENGTH * 1320) - 10; \
})

#define __delay_us(_TIMER4, time) ({ \
	__HAL_TIM_SET_COUNTER(_TIMER4, 0); \
	while (__HAL_TIM_GET_COUNTER(_TIMER4) < time); \
})

#define __SET_MOTOR_DIRECTION(DIR) ({ \
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, ((DIR) ? GPIO_PIN_RESET : GPIO_PIN_SET)); \
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, ((DIR) ? GPIO_PIN_SET: GPIO_PIN_RESET)); \
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, ((DIR) ? GPIO_PIN_RESET: GPIO_PIN_SET)); \
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, ((DIR) ? GPIO_PIN_SET: GPIO_PIN_RESET)); \
})

#define __RESET_SERVO_TURN(_TIMER) ({ \
	(_TIMER)->Instance->CCR4 = SERVO_CENTER; \
	HAL_Delay(SERVO_TURN_TIME); \
})

#define __RESET_SERVO_TURN_FAST(_TIMER) ({ \
	(_TIMER)->Instance->CCR4 = SERVO_CENTER; \
	HAL_Delay(200); \
})

#define __SET_SERVO_TURN_MAX(_TIMER, _DIR) ({ \
	if (_DIR) (_TIMER)->Instance->CCR4 = SERVO_RIGHT_MAX; \
	else (_TIMER)->Instance->CCR4 = SERVO_LEFT_MAX; \
	HAL_Delay(SERVO_TURN_TIME); \
})

#define __SET_SERVO_TURN(_TIMER, AMT) ({ \
	(_TIMER)->Instance->CCR4 = ((AMT) > SERVO_RIGHT_MAX) ? SERVO_RIGHT_MAX : ((AMT) < SERVO_LEFT_MAX ? SERVO_LEFT_MAX : (AMT));\
	HAL_Delay(SERVO_TURN_TIME); \
})

#define __PID_Config_Reset(cfg) ({ \
	cfg.ek1 = 0; \
	cfg.ekSum = 0; \
})

#define __Gyro_Read_Z(_I2C, readGyroData, gyroZ) ({ \
	HAL_I2C_Mem_Read(_I2C,ICM20948__I2C_SLAVE_ADDRESS_1 << 1, ICM20948__USER_BANK_0__GYRO_ZOUT_H__REGISTER, I2C_MEMADD_SIZE_8BIT, readGyroData, 2, 0xFFFF); \
	gyroZ = readGyroData[0] << 8 | readGyroData[1]; \
})

#define __ADC_Read_Dist(_ADC, dataPoint, IR_data_raw_acc, obsDist, obsTick) ({ \
	HAL_ADC_Start(_ADC); \
	HAL_ADC_PollForConversion(_ADC,20); \
	IR_data_raw_acc += HAL_ADC_GetValue(_ADC); \
	dataPoint = (dataPoint + 1) % IR_SAMPLE; \
	if (dataPoint == IR_SAMPLE - 1) { \
		obsDist = IR_CONST_A / (IR_data_raw_acc / dataPoint - IR_CONST_B); \
		obsTick = IR_data_raw_acc / dataPoint; \
		IR_data_raw_acc = 0; \
	} \
})

#define __PID_SPEED_T(cfg, error, correction, dir, newDutyL, newDutyR) ({ \
	correction = (cfg).Kp * error + (cfg).Ki * (cfg).ekSum + (cfg).Kd * ((cfg).ek1 - error);\
	(cfg).ek1 = error; \
	(cfg).ekSum += error; \
	correction = correction > DUTY_SPT_RANGE ? DUTY_SPT_RANGE : (correction < -DUTY_SPT_RANGE ? -DUTY_SPT_RANGE : correction); \
	newDutyL = INIT_DUTY_SPT_L + correction*dir; \
	newDutyR = INIT_DUTY_SPT_R - correction*dir; \
})

#define __PID_SPEED_1(cfg, error, correction, dir, newDutyL, newDutyR) ({ \
	correction = (cfg).Kp * error + (cfg).Ki * (cfg).ekSum + (cfg).Kd * ((cfg).ek1 - error);\
	(cfg).ek1 = error; \
	(cfg).ekSum += error; \
	correction = correction > DUTY_SP1_RANGE ? DUTY_SP1_RANGE : (correction < -DUTY_SP1_RANGE ? -DUTY_SP1_RANGE : correction); \
	newDutyL = INIT_DUTY_SP1_L + correction*dir; \
	newDutyR = INIT_DUTY_SP1_R - correction*dir; \
})

#define __PID_SPEED_2(cfg, error, correction, dir, newDutyL, newDutyR) ({ \
	correction = (cfg).Kp * error + (cfg).Ki * (cfg).ekSum + (cfg).Kd * ((cfg).ek1 - error);\
	(cfg).ek1 = error; \
	(cfg).ekSum += error; \
	correction = correction > DUTY_SP2_RANGE ? DUTY_SP2_RANGE : (correction < -DUTY_SP2_RANGE ? -DUTY_SP2_RANGE : correction); \
	newDutyL = INIT_DUTY_SP2_L + correction*dir; \
	newDutyR = INIT_DUTY_SP2_R - correction*dir; \
})

/*
#define __PID_Speed(cfg, actual, target, newDutyL, newDutyR) ({ \
	cfg.ekSum += target - actual; \
	newDutyL *= 1 + (target - actual) / target * cfg.Kp; \
	newDutyR *= 1 + (target - actual)/ target * cfg.Kp; \
})
*/

#define __ON_TASK_END(_MTimer, prevTask, curTask) ({ \
	__SET_MOTOR_DUTY(_MTimer, 0, 0); \
	prevTask = curTask; \
	curTask = TASK_NONE; \
})

#define __ACK_TASK_DONE(_UART, msg) ({ \
	snprintf((char *)msg, sizeof(msg) - 1, "done!"); \
	HAL_UART_Transmit(_UART, (uint8_t *) "ACK|\r\n", 6, 0xFFFF); \
})

#define __SET_MOTOR_DUTY(_TIMER, DUTY_L, DUTY_R)({ \
	(_TIMER)->Instance->CCR1 = DUTY_L; \
	(_TIMER)->Instance->CCR2 = DUTY_R; \
})

#define __SET_CMD_CONFIG(cfg, _MTIMER, _STIMER, targetAngle) ({ \
	__SET_SERVO_TURN(_STIMER, (cfg).servoTurnVal); \
	targetAngle = (cfg).targetAngle; \
	__SET_MOTOR_DIRECTION((cfg).direction); \
	__SET_MOTOR_DUTY(_MTIMER, (cfg).leftDuty, (cfg).rightDuty); \
})

#define __SET_CMD_CONFIG_WODUTY(cfg, _STIMER, targetAngle) ({ \
	__SET_SERVO_TURN(_STIMER, (cfg).servoTurnVal); \
	targetAngle = (cfg).targetAngle; \
	__SET_MOTOR_DIRECTION((cfg).direction); \
}) \

#define __CLEAR_CURCMD(cmd) ({ \
	cmd.index = 100; \
	cmd.val = 0; \
})

#define __PEND_CURCMD(cmd) ({ \
	cmd.index = 99; \
})

#define __SET_ENCODER_LAST_TICK(_TIMER, LAST_TICK) ({ \
	LAST_TICK = __HAL_TIM_GET_COUNTER(_TIMER); \
})

#define __SET_ENCODER_LAST_TICK_L_R(_TIMER_L, LAST_TICK_L, _TIMER_R, LAST_TICK_R) ({ \
	__SET_ENCODER_LAST_TICKS(_TIMER_L, LAST_TICK_L); \
	__SET_ENCODER_LAST_TICKS(_TIMER_R, LAST_TICK_R); \
})

#define __GET_ENCODER_TICK_DELTA(_TIMER, LAST_TICK, _DIST) ({ \
	uint32_t CUR_TICK = __HAL_TIM_GET_COUNTER(_TIMER); \
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(_TIMER)) { \
		_DIST = (CUR_TICK <= LAST_TICK) ? LAST_TICK - CUR_TICK : (65535 - CUR_TICK) + LAST_TICK; \
	} else { \
		_DIST = (CUR_TICK >= LAST_TICK) ? CUR_TICK - LAST_TICK : (65535 - LAST_TICK) + CUR_TICK; \
	} \
	LAST_TICK = CUR_TICK; \
})

// FIFO
// write at head(add command), remove at tail (read command)
// head == tail implies empty queue
//
#define __READ_COMMAND(_CQ, CMD, msg) ({ \
	CMD = _CQ.buffer[_CQ.tail]; \
	_CQ.tail = (_CQ.tail + 1) % _CQ.size; \
	snprintf((char *)msg, sizeof(msg) - 1, "doing"); \
})

#define __COMMAND_QUEUE_IS_FULL(_CQ) (_CQ.head + 1) % _CQ.size == _CQ.tail

#define __COMMAND_QUEUE_IS_EMPTY(_CQ) (_CQ.head == _CQ.tail)

#define __ADD_COMMAND(_CQ, TASK_INDEX, TASK_VAL) ({ \
	_CQ.buffer[_CQ.head].index = TASK_INDEX; \
	_CQ.buffer[_CQ.head].val = TASK_VAL; \
	_CQ.head = (_CQ.head + 1) % _CQ.size; \
})


/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_SCL_Pin GPIO_PIN_5
#define OLED_SCL_GPIO_Port GPIOE
#define OLED_SDA_Pin GPIO_PIN_6
#define OLED_SDA_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_2
#define AIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_3
#define AIN1_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOA
#define OLED_RST_Pin GPIO_PIN_7
#define OLED_RST_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_8
#define OLED_DC_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_10
#define BUZZER_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_8
#define SW1_GPIO_Port GPIOD
#define SW1_EXTI_IRQn EXTI9_5_IRQn
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOC
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOC
#define TRI_Pin GPIO_PIN_4
#define TRI_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
