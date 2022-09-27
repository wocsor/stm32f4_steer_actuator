#include "main.h"

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static uint8_t crc_checksum(uint8_t, int, uint8_t);


void delay(int a);
void motor_set(int16_t amt, uint8_t enable);

CAN_TxHeaderTypeDef   TxHeader;
uint8_t               TxData[8];
uint32_t              TxMailbox;

const uint8_t crc_poly = 0xD5U; // CRC8 

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();

  // PWM channels init
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  int16_t CH1_DC = 0;

  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 0x446;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 2;

  TxData[0] = 50;  
  TxData[1] = 0xAA;

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);


  while (1)
  {
      if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
      	  HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
      }


    //HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
      {
        HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
        // Error_Handler ();
      } else {
          HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
      }

    HAL_Delay(100);

	  // HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
	  // HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);
	  // delay(500000);

    // while(CH1_DC < 32700)
    // {
    //     motor_set(CH1_DC, 1);
    //     CH1_DC += 64;
    //     HAL_Delay(1);
    // }

	  // HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
	  // HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
	  // delay(500000);

    // while(CH1_DC > -32700)
    // {
    //     motor_set(CH1_DC, 1);
    //     CH1_DC -= 70;
    //     HAL_Delay(1);
    // }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);


  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}



// crc function from Panda 

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



/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, CH1_EN_Pin|CH2_EN_Pin|RELAY_EN_Pin|LED1_Pin
                          |LED2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = CH1_EN_Pin|CH2_EN_Pin|RELAY_EN_Pin|LED1_Pin
                          |LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void delay (int a)
{
  volatile int i,j;
  for (i=0 ; i < a ; i++)
  {
    j++;
  }
  return;
}

void motor_set (int16_t amt, uint8_t enable)
{
  // differential signal with neutral at 50% duty cycle

  uint32_t duty = 32767;

  TIM2->CCR1 = duty + amt;
  TIM2->CCR2 = duty - amt; 

  // set the GPIO lines
  if (enable > 0){
    enable = 1;
  }
  HAL_GPIO_WritePin(GPIOA, CH1_EN_Pin, enable);
  HAL_GPIO_WritePin(GPIOA, CH2_EN_Pin, enable);

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
