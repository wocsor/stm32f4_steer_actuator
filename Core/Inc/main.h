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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define CAN1_EN_Pin GPIO_PIN_1
#define CAN1_EN_GPIO_Port GPIOC
#define CAN2_EN_Pin GPIO_PIN_2
#define CAN2_EN_GPIO_Port GPIOC
#define TBD_Pin GPIO_PIN_3
#define TBD_GPIO_Port GPIOC

#define CH1_EN_Pin GPIO_PIN_2
#define CH1_EN_GPIO_Port GPIOA
#define CH2_EN_Pin GPIO_PIN_3
#define CH2_EN_GPIO_Port GPIOA
#define RELAY_EN_Pin GPIO_PIN_4
#define RELAY_EN_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define MAX_TIMEOUT 50U
#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
#define FAULT_COUNTER 7U
#define COUNTER_CYCLE 0xFU
#define LKA_COUNTER_CYCLE = 0x3FU
// CAN
#define CAN_IN 0x22e
#define CAN_OUT 0x260
/* USER CODE END Private defines */

extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef   TxHeader;
extern uint32_t              TxMailbox;

extern uint16_t torque_req;
extern uint8_t lka_req;

extern uint8_t state;

extern UART_HandleTypeDef huart2;

extern const uint8_t crc_poly;  // standard crc8 SAE J1850
extern uint8_t crc8_lut_1d[256];

uint8_t noCanTimer;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

