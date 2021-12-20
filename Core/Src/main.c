/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void Toggle_GPIO_pin_Init(void);
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
	//Due to small ram (8KB) we must divide frame array by 32 (157440/32=4920)
	//The resulting problem with this is that the beginning of every 10th row will be over exposed.
	//uint8_t received_data[4920] = {0}; // size of one frame: 157440
	//uint8_t test[2000] = {0}; //2000 gives hard fault. 1000 does not. Seems like the RAM is the limitation.

	uint8_t received_data[4920] = {0};  //320*492 = 157440    157440/32 = 4920
	uint8_t empty_data[966] = {0};
	uint8_t test_pattern[6];
	test_pattern[0] = 0b10010000;
	test_pattern[1] = 0b11000001;
	test_pattern[2] = 0b00101010;
	test_pattern[3] = 0b10010010;
	test_pattern[4] = 0b00000101;
	test_pattern[5] = 0b00101000;
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
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  //---- TURN ON CAMERA LEDs ----
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  //---- ACTIVATION CLOCK PULSE -----
    HAL_SPI_DeInit(&hspi1);
    Toggle_GPIO_pin_Init();
    GPIOA -> ODR ^= GPIO_PIN_5;
    HAL_Delay(1);
    GPIOA -> ODR ^= GPIO_PIN_5;

    //---- REGISTER CONFIGURATION - 48-bit (4PP) -----
    MX_SPI1_Init();
    HAL_SPI_Transmit(&hspi1, test_pattern, 6, 1000);

    //---- WAIT FOR IDLE START-UP ----
    HAL_Delay(1);

    //---- 10 CLOCKS -----
    HAL_SPI_DeInit(&hspi1);
    Toggle_GPIO_pin_Init();
    for (uint8_t i = 0; i < 20; i++)
      {
  	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  	  HAL_Delay(1);
      }

    //---- 12 CLOCKS -----  IS this to slow?
    for (uint8_t i = 0; i < 24; i++)
      {
  	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  	  HAL_Delay(1);
      }

    //---- INITIAL PRE-SYNC MODE: RECEIVE 492 PP FROM CAMERA -----
    MX_DMA_Init();
    MX_SPI1_Init();
    HAL_SPI_Receive_DMA(&hspi1, received_data, 492);

    //---- SYNC MODE -----
    HAL_SPI_Receive_DMA(&hspi1, received_data, 984);

    //---- DELAY MODE -----
    HAL_SPI_Receive_DMA(&hspi1, received_data, 984);

    //---- READOUT MODE -----
    HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 1
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 2
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 3
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 4
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 5
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 6
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 7
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 8
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 9
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 10
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 11
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 12
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 13
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 14
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 15
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 16
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 17
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 18
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 19
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 20
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 21
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 22
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 23
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 24
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 25
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 26
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 27
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 28
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 29
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 30
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 31
	HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 32

    //---- END OF FRAME -----
    HAL_SPI_Receive_DMA(&hspi1, received_data, 8);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //---- REGISTER CONFIGURATION - 48-bit (4PP) -----
	  HAL_SPI_Transmit(&hspi1, test_pattern, 6, 1000);

	  //---- INTERFACE MODE -----
	  HAL_SPI_Transmit(&hspi1, empty_data, 966, 1000);

	  //---- SYNC and DELAY MODE -----
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 1968);

	  //---- READOUT MODE -----
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 1
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 2
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 3
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 4
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 5
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 6
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 7
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 8
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 9
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 10
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 11
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 12
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 13
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 14
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 15
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 16
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 17
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 18
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 19
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 20
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 21
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 22
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 23
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 24
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 25
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 26
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 27
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 28
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 29
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 30
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 31
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 4920); // 32

	  //---- END OF FRAME -----
	  HAL_SPI_Receive_DMA(&hspi1, received_data, 12);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_EN_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_EN_Pin LED_Pin */
  GPIO_InitStruct.Pin = LED_EN_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void Toggle_GPIO_pin_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
