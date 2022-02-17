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
#define WHEEL_LENGTH 20.12
#define PPR 330
// distance calibration params
#define DIST_M 1.133145885
#define DIST_C 2.419816382
#define DIR_FORWARD 1
#define DIR_BACKWARD 0
#define SERVO_LEFT_MAX 50
#define SERVO_MIDDLE 74
#define SERVO_RIGHT_MAX 115

#define MAX_DUTY 1200

#define __ACK_TASK_DONE(_UART, msg) ({ \
	snprintf((char *)msg, sizeof(msg), "done"); \
	HAL_UART_Transmit(_UART, (uint8_t *) "ACK\r\n", 4, 0xFFFF); \
})

#define __SET_MOTOR_DUTY(_TIMER, DUTY_L, DUTY_R)({ \
	(_TIMER)->Instance->CCR1 = DUTY_L; \
	(_TIMER)->Instance->CCR2 = DUTY_R; \
})

#define __SET_MOTOR_DIRECTION(DIR) ({ \
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, ((DIR) ? GPIO_PIN_RESET : GPIO_PIN_SET)); \
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, ((DIR) ? GPIO_PIN_SET: GPIO_PIN_RESET)); \
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, ((DIR) ? GPIO_PIN_RESET: GPIO_PIN_SET)); \
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, ((DIR) ? GPIO_PIN_SET: GPIO_PIN_RESET)); \
})

#define __RESET_SERVO_TURN(_TIMER) ({ \
	(_TIMER)->Instance->CCR4 = SERVO_MIDDLE; \
	osDelay(500); \
})

#define __SET_SERVO_TURN_MAX(_TIMER, _DIR) ({ \
	if (_DIR) (_TIMER)->Instance->CCR4 = SERVO_RIGHT_MAX; \
	else (_TIMER)->Instance->CCR4 = SERVO_LEFT_MAX; \
	osDelay(500); \
})

#define __SET_SERVO_TURN(_TIMER, AMT) ({ \
	(_TIMER)->Instance->CCR4 = ((AMT) > SERVO_RIGHT_MAX) ? SERVO_RIGHT_MAX : ((AMT) < SERVO_LEFT_MAX ? SERVO_LEFT_MAX : (AMT));\
	osDelay(500); \
})

#define __SET_CMD_CONFIG(cfg, _MTIMER, _STIMER, targetAngle) ({ \
	__SET_SERVO_TURN(_STIMER, cfg.servoTurnVal); \
	targetAngle = cfg.targetAngle; \
	__SET_MOTOR_DIRECTION(cfg.direction); \
	__SET_MOTOR_DUTY(_MTIMER, cfg.leftDuty, cfg.rightDuty); \
})

#define __CLEAR_CURCMD(cmd) ({ \
	cmd.index = 100; \
	cmd.val = 0; \
})

#define __PEND_CURCMD(cmd) ({ \
	cmd.index = 99; \
})

#define __SET_ENCODER_LAST_TICKS(_TIMER_L, LAST_TICK_L, _TIMER_R, LAST_TICK_R) ({ \
	LAST_TICK_L = __HAL_TIM_GET_COUNTER(_TIMER_L); \
	LAST_TICK_R = __HAL_TIM_GET_COUNTER(_TIMER_R); \
})

#define __GET_ENCODER_TICK_DELTA(_TIMER, LAST_TICK, _DIST) ({ \
	int CUR_TICK = __HAL_TIM_GET_COUNTER(_TIMER); \
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(_TIMER)) { \
		_DIST = (CUR_TICK <= LAST_TICK) ? LAST_TICK - CUR_TICK : (65535 - CUR_TICK) + LAST_TICK; \
	} else { \
		_DIST = (CUR_TICK >= LAST_TICK) ? CUR_TICK - LAST_TICK : (65535 - LAST_TICK) + CUR_TICK; \
	} \
})

// FIFO
// write at head(add command), remove at tail (read command)
// head == tail implies empty queue
//
#define __READ_COMMAND(_CQ, CMD) ({ \
	CMD = _CQ.buffer[_CQ.tail]; \
	_CQ.tail = (_CQ.tail + 1) % _CQ.size; \
})

#define __COMMAND_QUEUE_IS_FULL(_CQ) (_CQ.head + 1) % _CQ.size == _CQ.tail

#define __COMMAND_QUEUE_IS_EMPTY(_CQ) (_CQ.head == _CQ.tail)

#define __ADD_COMMAND(_CQ, TASK_INDEX, TASK_VAL) ({ \
	_CQ.buffer[_CQ.head].index = TASK_INDEX; \
	_CQ.buffer[_CQ.head].val = TASK_VAL; \
	_CQ.head = (_CQ.head + 1) % _CQ.size; \
})


// Magnetometer read access
#define __Mag_Read(_I2C, readMagData, mag) ({ \
	HAL_I2C_Mem_Read(_I2C, AK09918__I2C_SLAVE_ADDRESS << 1, AK09916__XOUT_H__REGISTER, I2C_MEMADD_SIZE_8BIT, readMagData, 6,0xFFFF); \
	mag[0] = readMagData[1] << 8 | readMagData[0]; \
	mag[1] = readMagData[3] << 8 | readMagData[2]; \
	mag[2] = readMagData[5] << 8 | readMagData[4]; \
	HAL_I2C_Mem_Read(_I2C, AK09918__I2C_SLAVE_ADDRESS << 1, AK09916__ST2__REGISTER, I2C_MEMADD_SIZE_8BIT, readMagData, 6,0xFFFF); \
	mag[0] *= MAG_SENSITIVITY_SCALE_FACTOR; \
	mag[1] *= MAG_SENSITIVITY_SCALE_FACTOR; \
	mag[2] *= MAG_SENSITIVITY_SCALE_FACTOR; \
})

#define __Accel_Read(_I2C, readAccelData, accel) ({ \
	HAL_I2C_Mem_Read(_I2C, ICM20948__I2C_SLAVE_ADDRESS_1 << 1, ICM20948__USER_BANK_0__ACCEL_XOUT_H__REGISTER, I2C_MEMADD_SIZE_8BIT, readAccelData, 6, 10); \
	accel[0] = readAccelData[0] << 8 | readAccelData[1]; \
	accel[1] = readAccelData[2] << 8 | readAccelData[3]; \
	accel[2] = readAccelData[4] << 8 | readAccelData[5]; \
})

#define __Gyro_Read(_I2C, readGyroData, gyro) ({ \
	HAL_I2C_Mem_Read(_I2C,ICM20948__I2C_SLAVE_ADDRESS_1 << 1, ICM20948__USER_BANK_0__GYRO_XOUT_H__REGISTER, I2C_MEMADD_SIZE_8BIT, readGyroData, 6, 10); \
	gyro[0] = readGyroData[0] << 8 | readGyroData[1]; \
	gyro[1] = readGyroData[2] << 8 | readGyroData[3]; \
	gyro[2] = readGyroData[4] << 8 | readGyroData[5]; \
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
#define SW1_Pin GPIO_PIN_8
#define SW1_GPIO_Port GPIOD
#define SW1_EXTI_IRQn EXTI9_5_IRQn
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOC
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
