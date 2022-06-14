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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
 TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
 enum{normOperation, emergency} beegState = normOperation;
// enum{INIT, waitCMDs, setVelo, setPos, set1Goal, setMultiGoal, mcuConnections, testCMD, setHome, reqStation, reqPos, reqMAXVelo} cmdState = INIT;
 enum{INIT, waitCMDs} cmdState = INIT;
 enum{mcuC, mcuDC} mcuConnex = mcuC;

 char TxDataBuffer[32] = {0};
 uint8_t RxDataBuffer[32] = {0};
 uint8_t checkSum = 0;
 uint8_t rxDataStart = 0;
 uint8_t rxDataCount = 0;
 uint8_t ACK_1[2] = { 0x58, 0b01110101 };
 uint8_t ACK_2[2] = { 70, 0b01101110 };
 uint8_t sendData[4] = {0};

 //test data for easier coding
 uint16_t posData = 0; //(max 16000)
 uint16_t veloData = 0; //(max 16000)
 // -------

 //for debugging
 uint8_t testGoalStartIDX = 0;
 // -------

 uint8_t uartVelo;
 uint16_t uartPos;
 uint8_t uartGoal[15];
 uint8_t goalAmount = 0;
 uint8_t goalIDX = 0;
 uint8_t runningFlag = 0;
 uint8_t reachedFlag = 0;
 uint8_t modeNo = 0;

 uint64_t _micro = 0;
 uint64_t timeElapsed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
//void checkSum();
void stateManagement();
uint8_t UARTReceiveIT();
uint64_t micros();
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
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim11);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Receive_IT(&huart2, &RxDataBuffer, 32);
	  stateManagement();

	  if(micros() - timeElapsed > 10000000){
		  timeElapsed = micros();
		  if(runningFlag == 1){
			  runningFlag = 0;
			  HAL_UART_Transmit_IT(&huart2, ACK_2, 2);
		  }
	  }

//	  HAL_UART_Receive_IT(&huart2, &testUART, 1);
//	  static GPIO_PinState B1State[2] = {0};
//	  B1State[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
//	  if(B1State[1] == GPIO_PIN_SET && B1State[0] == GPIO_PIN_RESET){ //falling edge detect
//		  static uint8_t testDig = 147;
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  sprintf(TxDataBuffer, "Xu");
//		  HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
//	  }
//	  B1State[1] = B1State[0];

	  HAL_Delay(200);
//
//	  sprintf(TxDataBuffer, "Got: [%c]\r\n", RxDataBuffer);
//	  HAL_UART_Transmit(&huart2, (uint16_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//void checkSum(){
//	for(int i = 0; i < strlen(uartRxData); i++){
//		checkSum += uartRxData[i];
//	}
//}
//
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	if (huart->Instance == USART2)
//	  {
//	    /* Transmit one byte with 100 ms timeout */
////	    HAL_UART_Transmit(&huart2, &testUART, 1, 100);
//
//	    /* Receive one byte in interrupt mode */
//	    HAL_UART_Receive_IT(&huart2, &RxDataBuffer, 1);
//	  }
//}

void stateManagement(){
//	static uint8_t rxDataStart = 0;
//	static uint8_t rxDataCount = 0;
//	static uint8_t cmdStorageShift = 0;
//	static uint8_t checkSum = 0;
	switch(beegState){
		case normOperation:
			switch(cmdState){
				case INIT:
					// reset pwm
					cmdState = waitCMDs;
					break;
				case waitCMDs:
					if(huart2.RxXferSize - huart2.RxXferCount != rxDataCount){
						if((uint8_t)RxDataBuffer[rxDataCount] == (uint8_t)(~checkSum)){
							checkSum = 0;
							switch(RxDataBuffer[rxDataStart]){
							case 0b10010001:
								modeNo = 1;
								break;
							case 0b10010010:
								modeNo = 2;
								HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
								break;
							case 0b10010011:
								modeNo = 3;
								HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
								break;
							case 0b10010100:
								modeNo = 4;
								uartVelo = RxDataBuffer[rxDataStart + 2]; // max 255
								HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
								break;
							case 0b10010101:
								modeNo = 5;
								uartPos = (RxDataBuffer[rxDataStart + 1] << 8) | RxDataBuffer[rxDataStart + 2]; // max 65535
								HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
								break;
							case 0b10010110:
								modeNo = 6;
								goalAmount = 1;
								uartGoal[0] = RxDataBuffer[rxDataStart + 2];
								HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
								break;
							case 0b10010111:
								modeNo = 7;
								goalAmount = RxDataBuffer[rxDataStart + 1];
								uint8_t j = 0;
								for(int i = 0; i < goalAmount; i += 2){
									uartGoal[0+i] = RxDataBuffer[rxDataStart+(2+j)] & 15; // low 8 bit (last 4 bit)
									uartGoal[1+i] = RxDataBuffer[rxDataStart+(2+j)] >> 4; // high 8 bit (first 4 bit)
									j++;
								}
								HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
								break;
							case 0b10011000:
								modeNo = 8;
								runningFlag = 1;
								HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
								break;
							case 0b10011001:
								modeNo = 9;
								HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
								sendData[0] = 153; // start-mode
								sendData[2] = 10; // set current goal
								sendData[3] = ~(sendData[0]+sendData[1]+sendData[2]);
								HAL_UART_Transmit_IT(&huart2, sendData, 4);
								break;
							case 0b10011010:
								modeNo = 10;
								HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
								posData = 10271; // data from zhong
								sendData[0] = 154; // start-mode
								sendData[1] = ((posData*65535)/16000) & 255; // set low byte posData
								sendData[2] = ((posData*65535)/16000) >> 8; // set high byte posData
								sendData[3] = ~(sendData[0]+sendData[1]+sendData[2]);
								HAL_UART_Transmit_IT(&huart2, sendData, 4);
								break;
							case 0b10011011:
								modeNo = 11;
								HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
								veloData = 2496;
								sendData[0] = 155;
								sendData[2] = ((veloData*255)/16000) & 255; // set low byte posData
								sendData[3] = ~(sendData[0]+sendData[1]+sendData[2]);
								HAL_UART_Transmit_IT(&huart2, sendData, 4);
								break;
							case 0b10011100:
								modeNo = 12;
								HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
								break;
							case 0b10011101:
								modeNo = 13;
								HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
								break;
							case 0b10011110:
								modeNo = 14;
								HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
								break;
							}
							rxDataStart = (rxDataCount + 1) % huart2.RxXferSize; // might have to redo
						}
						else{
							checkSum = checkSum + RxDataBuffer[rxDataCount];
						}
						rxDataCount = (rxDataCount + 1) % huart2.RxXferSize;
					}
					break;
				  }
	          	  break;
	    case emergency:
	    	break;
	    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim11) {
		_micro += 65535;
	}
}

uint64_t micros(){
	return _micro + htim11.Instance->CNT;
}

//uint8_t UARTReceiveIT(){
//	static uint32_t dataPos =0;
//	uint8_t data = 0;
//	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
//	{
//		data = RxDataBuffer[dataPos];
//		dataPos = (dataPos+1)%huart2.RxXferSize;
//	}
//	return data;
//}
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
