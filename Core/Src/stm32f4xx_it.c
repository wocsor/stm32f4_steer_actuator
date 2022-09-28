/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

// crc function from Panda 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"

const uint8_t crc_poly = 0xD5U; // CRC8
/* USER CODE BEGIN PFP */
static uint8_t crc_checksum(uint8_t *dat, int len, const uint8_t poly);

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */
// variables

// Torque request to be referenced in main loop
uint16_t torque_req = 0;
// LKA enabled, used in mainloop
uint8_t lka_req = 0;

uint8_t lka_counter = 0;
uint8_t lka_checksum = 0;
uint8_t eps_ok = 0;
uint8_t mode = 0;
uint16_t rel_input = 0;
uint16_t pos_input = 0;
uint16_t steer_torque_driver = 0;
uint16_t steer_torque_eps = 0;
uint8_t steer_override = 0;
uint8_t can1_count_out = 0;
uint8_t can1_count_in =  0;
uint8_t can2_count_out = 0;
uint32_t timeout = 0;
uint8_t state = 0;
uint8_t sent = 0;
uint8_t lka_state = 0;
/* USER CODE END EV */

 

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
   // clear interrupt
  CAN1->TSR |= CAN_TSR_RQCP0;
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {
    uint16_t address = CAN1->sFIFOMailBox[0].RIR >> 21;
    switch (address) {
      case CAN_IN: ;
        uint8_t dat[6] = {0};
        for (int i=0; i<6; i++) {
          // dat[i] = GET_BYTE(&CAN1->sFIFOMailBox[0], i);
        }
        uint8_t index = dat[1] & COUNTER_CYCLE;
        if(dat[0] == crc_checksum(dat, 6, crc_poly)) {
          if (((can1_count_in + 1U) & COUNTER_CYCLE) == index) {
            //if counter and checksum valid accept commands
            mode = ((dat[1] >> 4U) & 3U);
            if (mode != 0){
              lka_req = 1;
            } else {
              lka_req = 0;
            }
            pos_input = ((dat[3] & 0xFU) << 8U) | dat[2];
            rel_input = ((dat[5] << 8U) | dat[4]);
            // TODO: safety? scaling?
            torque_req = rel_input;

            

            can1_count_in++;
          }
          else {
            state = FAULT_COUNTER;
          }
          state = NO_FAULT;
          timeout = 0;
        }
        else {
          state = FAULT_BAD_CHECKSUM;
        }
        break;
      default: ;
    }
    // next
    // CAN1->RF0R |= CAN_RF0R_RFOM0;
  }
  
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 SCE interrupt.
  */
void CAN1_SCE_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_SCE_IRQn 0 */

  /* USER CODE END CAN1_SCE_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_SCE_IRQn 1 */

  /* USER CODE END CAN1_SCE_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
uint8_t led_state = 0;
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  // HAL_GPIO_WritePin(GPIOA, LED1_Pin, led_state);
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
   	HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
  }

  eps_ok = 1;
  state = 5;
  
  //send to EON
  uint8_t dat[7];

  dat[6] = (steer_torque_eps & 0xFF);
  dat[5] = (steer_torque_eps >> 8U);
  dat[4] = (steer_torque_driver & 0xFF);
  dat[3] = (steer_torque_driver >> 8U);
  dat[2] = eps_ok;
  dat[1] = ((state & 0xFU) << 4) | can1_count_out;
  dat[0] = crc_checksum(dat, 7, crc_poly);

  can1_count_out++;
  can1_count_out &= COUNTER_CYCLE;

  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, dat, &TxMailbox);

  CAN1->TSR |= CAN_TSR_ABRQ0;
  CAN1->MSR = CAN1->MSR;

  // if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, dat, &TxMailbox) != HAL_OK) {
  //   // do a thing
  // } else {
  //   // do another thing
  // }

  HAL_TIM_IRQHandler(&htim3);

}

/* USER CODE BEGIN 1 */
uint8_t crc_checksum(uint8_t *dat, int len, const uint8_t poly) {
  uint8_t crc = 0xFF;
  int i, j;
  for (i = len - 1; i >= 0; i--) {
    crc ^= dat[i];
    for (j = 0; j < 8; j++) {
      if ((crc & 0x80U) != 0U) {
        crc = (uint8_t)((crc << 1) ^ poly);
      }
      else {
        crc <<= 1;
      }
    }
  }
  return crc;
}
/* USER CODE END 1 */
