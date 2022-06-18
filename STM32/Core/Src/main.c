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

#include "PID.h"
#include "Kalman.h"
#include "arm_math.h"
#include "PIDVelocity.h"
#include "Trajectory.h"

#include "uartRingBufDMA.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ---------------------------------UART--------------------------------- //
#define UART huart2
#define DMA hdma_usart2_rx
/* Define the Size */
#define RxBuf_SIZE 20
#define TxBuf_SIZE 20
#define MainBuf_SIZE 40
// ---------------------------------UART--------------------------------- //
#define Endeff_ADDR 0b01000110
#define Endeff_TEST 0x45
/* Controller,Kalman parameters */
#define dt  0.001f
#define testDes 90.0f
#define Kalmanvar  1.0f
#define PID_KP  2.0f
#define PID_KI  0.02f
#define PID_KD  0.001f
#define PIDVELO_KP  8.0f
#define PIDVELO_KI  0.4f
#define PIDVELO_KD  1.2f
#define PID_LIM_MIN_INT -10000.0f
#define PID_LIM_MAX_INT  10000.0f
/* PWM MAX parameters */
#define AMAX 28.65f
#define JMAX 573.0f
/* PWM MAX parameters */
#define PWM_MAX 10000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
// ---------------------------------UART--------------------------------- //
uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];
uint8_t TxBuf[TxBuf_SIZE];

uint16_t oldPos = 0;
uint16_t newPos = 0;
uint16_t Head, Tail;

/* Timeout is in milliseconds */
int32_t TIMEOUT = 0;

uint8_t ACK_1[2] = { 0x58, 0b01110101 };
uint8_t ACK_2[2] = { 70, 0b01101110 };

// normOperation(MCCon) = MotorOn, EndEff On
// emergency = MotorOff, EndEff Off
// MCDisCon = MotorOff, EndEff Off
enum{MCDisCon ,normOperation, emergency} MainState = normOperation;
// idle = MotorOn, EndEff On(Do nothing)
// EndEff = MotorOn(PWM=0), EndEff On(Send Something and Shoot Laser)(LED On)
enum{init, idle, EndEff} RobotState = normOperation;
// Data Buffer

 uint8_t sendData[6] = {0};

 //for sending data to base sys
 uint8_t goalData = 0;
 uint16_t posData = 0; //(max 16000)
 uint16_t veloData = 0; //(max 16000)
 // -------

 uint16_t uartVelo = 0;
 uint16_t uartPos = 0;
 uint8_t uartGoal[15];
 uint8_t goalAmount = 0;
 uint8_t goalIDX = 0;
 uint8_t runningFlag = 0;
 uint8_t homingFlag = 0;
 uint8_t endEffFlag = 0;
 uint8_t modeNo = 0;

 uint64_t timeElapsed = 0;
 // ---------------------------------UART--------------------------------- //
 // ---------------------------------CTRL---------------------------------
/* Setup Microsec */
uint64_t _micro = 0;
/* Setup EncoderData */
int EncoderRawData[2] = {0};
int WrappingStep = 0;
int PositionRaw = 0;
float32_t PositionDeg[2] = {0};
float32_t VelocityDeg = 0;
/* Initialise Kalman Filter */
KalmanFilterVar KalmanVar = {
		{1.0,dt,0.5*dt*dt,0,1.0,dt,0,0,1.0}, // A
		{0.0,0.0,0.0}, // B
		{1.0,0.0,0.0}, // C
		{0.0}, // D
		{1000.0}, //Q
		{0.000001}, //R
		{((dt*dt*dt))/6.0,((dt*dt))/2.0,dt}, //G
		{0.0,0.0,0.0}, // STATE X
		{0.0,0.0,0.0}, // STATE X-1
		{0,0,0,0,0,0,0,0,0}, // STATE P
		{0,0,0,0,0,0,0,0,0}, // STATE P-1
		{0.0}, // Y
		{0.0}, // Z
		{0.0}, // S
		{0.0,0.0,0.0}, // K
		{1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0}, // I
		{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},	// Matrix At
		{0.0,0.0,0.0},	// Matrix Gt
		{0.0,0.0,0.0},	// Matrix GQ
		{0.0,0.0,0.0},	// Matrix Ct
		{0.0},	// Matrix Sinv
		{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},	// Matrix GQGt
		{0.0,0.0,0.0},	// Matrix CPk
		{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},	// Matrix APK
		{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},	// Matrix APKAt
		{0.0},	// Matrix CXk
		{0.0},	// Matrix CPkCt
		{0.0,0.0,0.0},	// Matrix PkCt
		{0.0,0.0,0.0},	// Matrix KYk
		{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},	// Matrix KC
		{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0} 	// Matrix IKC
};
/* Initialise PID controller */
PIDVelocityController PidVelo = {PIDVELO_KP,PIDVELO_KI,PIDVELO_KD,
								PID_LIM_MIN_INT,PID_LIM_MAX_INT};
PIDVelocityController PidPos = {PID_KP, PID_KI, PID_KD,
								PID_LIM_MIN_INT,PID_LIM_MAX_INT};
/* Simulate response using test system */
float setpoint = 0.0f;
float PWMCHECKER = 0.0f;
float PositionErrorControl = 0.3f;
/* Trajectory */
TrajectoryG traject = {AMAX,JMAX};
uint8_t flagT = 0;
static uint64_t StartTime =0;
static uint64_t CurrentTime =0;
static uint64_t CheckLoopStartTime =0;
static uint64_t CheckLoopStopTime =0;
static uint64_t CheckLoopDiffTime =0;
static uint8_t btncheck = 0;
uint8_t I2CEndEffectorReadFlag = 0;
uint8_t I2CEndEffectorWriteFlag = 0;
//static uint16_t len =1;
//static uint8_t dumdata[2] ={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint64_t Micros();
uint32_t PWMAbs(int32_t PWM);
void Drivemotor(int32_t PWM);
void EncoderRead();
float AbsVal(float number);
void ControllLoopAndErrorHandler();
void I2CWriteFcn(uint8_t *Wdata, uint16_t len, uint16_t MemAd);
void I2CReadFcn(uint8_t *Rdata, uint16_t len, uint16_t MemAd);

void stateManagement(uint8_t *Rxbuffer , uint16_t rxDataCurPos , uint16_t rxDataLastPos);

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //----UART-----//
  Ringbuf_Init();
  //  HAL_UART_Receive_DMA(&huart2, RxDataBuffer, 32);
  //----UART-----//
  KalmanMatrixInit(&KalmanVar);
  //////////////////////////
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT (&htim11);
  HAL_TIM_Base_Start_IT (&htim3);
  HAL_TIM_Base_Start_IT (&htim4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  EncoderRawData[0]=TIM2->CNT;
  EncoderRawData[1]=EncoderRawData[0];
  PositionRaw=EncoderRawData[0];
  PIDVelocityController_Init(&PidVelo);
  PIDVelocityController_Init(&PidPos);

  CoefficientAndTimeCalculation(&traject,0.0,testDes);

  btncheck = 0;
  /* USER CODE END 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  htim2.Init.Period = 11999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim11.Init.Prescaler = 99;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|PIN_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Pin_Relay1_Pin|Pin_Relay2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PIN_DIR_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|PIN_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Pin_Proxi_Pin */
  GPIO_InitStruct.Pin = Pin_Proxi_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pin_Proxi_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Pin_Relay1_Pin Pin_Relay2_Pin */
  GPIO_InitStruct.Pin = Pin_Relay1_Pin|Pin_Relay2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Pin_Emer_Pin */
  GPIO_InitStruct.Pin = Pin_Emer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pin_Emer_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void EncoderRead()
{
	static int32_t SignalThreshold = 0.6*12000;
	EncoderRawData[0] = TIM2->CNT;
	if(EncoderRawData[0]-EncoderRawData[1]<-SignalThreshold){
		WrappingStep+=12000;
	}
	else if(EncoderRawData[0]-EncoderRawData[1]>=SignalThreshold){
		WrappingStep-=12000;
	}
	PositionRaw = EncoderRawData[0] + WrappingStep;
//	PositionRad = (PositionRaw/12000.0)*2.0*3.14;
	PositionDeg[0] = (PositionRaw/12000.0)*360.0;
	if(PositionDeg[0] != PositionDeg[1])
	{
		VelocityDeg = ((PositionDeg[0] - PositionDeg[1])/dt);
	}
	else
	{
		VelocityDeg = VelocityDeg;
	}
	EncoderRawData[1] = EncoderRawData[0];
	PositionDeg[1] = PositionDeg[0];
}

uint32_t PWMAbs(int32_t PWM)
{
	if(PWM<0){
		return PWM*-1;
	}else{
		return PWM;
	}
}


void Drivemotor(int32_t PWM){
		if(PWM<=0 && PWM>=-PWM_MAX){
			htim1.Instance->CCR1=PWMAbs(PWM);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,0);
		}else if (PWM<-PWM_MAX){
			htim1.Instance->CCR1=PWM_MAX;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,0);
		}else if(PWM>=0 && PWM<=PWM_MAX){
			htim1.Instance->CCR1=PWMAbs(PWM);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,1);
		}else if(PWM>PWM_MAX){
			htim1.Instance->CCR1=PWM_MAX;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,1);
		}
}


void ControllLoopAndErrorHandler()
{
//	setpoint = 180.0;
//	PIDVelocityController_Update(&PidVelo, setpoint, KalmanVar.MatState_Data[1]);
//	PWMCHECKER = PidVelo.ControllerOut;
//	Drivemotor(2500.0);
	  if (flagT == 0)
	  {
	    StartTime = Micros();
	    flagT =1;
	  }
		CurrentTime = Micros();
		TrajectoryEvaluation(&traject,StartTime,CurrentTime);
	  if(AbsVal(testDes - PositionDeg[0]) < 0.5 && AbsVal(KalmanVar.MatState_Data[1]) < 1.0)
	  {
	    PWMCHECKER = 0.0;
	    Drivemotor(PWMCHECKER);
	  }
	  else
	  {
		PIDVelocityController_Update(&PidPos,traject.QX, PositionDeg[0]);
		PIDVelocityController_Update(&PidVelo, traject.QV + PidPos.ControllerOut  , KalmanVar.MatState_Data[1]);
		PWMCHECKER = PidVelo.ControllerOut;
		Drivemotor(PWMCHECKER);
	  }
}

void I2CWriteFcn(uint8_t *Wdata, uint16_t len, uint16_t MemAd) {
	if (I2CEndEffectorWriteFlag && hi2c1.State == HAL_I2C_STATE_READY) {
//		static uint8_t data;
//		data = Wdata[0];
		HAL_I2C_Master_Transmit_IT(&hi2c1, MemAd, Wdata, len);
//		HAL_I2C_Mem_Write_IT(&hi2c1, DevAddress, MemAddress, MemAddSize, pData, Size);
		I2CEndEffectorWriteFlag = 0;
	}
}
void I2CReadFcn(uint8_t *Rdata, uint16_t len, uint16_t MemAd) {
	if (I2CEndEffectorReadFlag && hi2c1.State == HAL_I2C_STATE_READY) {
//		static uint8_t data;
//		data = Rdata;
//		HAL_I2C_Mem_Read_IT(&hi2c1, DevAddress, MemAddress, MemAddSize, pData, Size)
//		HAL_I2C_Master_Receive_IT(&hi2c1, DevAddress, pData, Size);
		I2CEndEffectorReadFlag = 0;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	if(GPIO_Pin == GPIO_PIN_13)
//	{
//		btncheck++;
////		I2CWriteFcn(0x23,8,Endeff_ADDR);
//		len = 1;
//		dumdata[0] = 0x45;
//		I2CEndEffectorWriteFlag = 1;
//		I2CWriteFcn(dumdata,len,Endeff_ADDR);
////		HAL_I2C_Master_Transmit_IT(&hi2c1, Endeff_ADDR, 0b01000101, 1);
////		HAL_I2C_Mem_Write_IT(&hi2c1, Endeff_ADDR, Endeff_TEST, I2C_MEMADD_SIZE_16BIT, pData, Size);
//	}
	if(GPIO_Pin == GPIO_PIN_10)
	{
//		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState)
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	}
	if(GPIO_Pin == GPIO_PIN_5)
	{
//		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState)
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
		btncheck++;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim11) {
		_micro += 65535;
	}
	else if (htim == &htim3) {
		CheckLoopStartTime = Micros();
		EncoderRead();
		KalmanFilterFunction(&KalmanVar,PositionDeg[0]);
//		KalmanFilterFunction(&KalmanVar,VelocityDeg);
		ControllLoopAndErrorHandler();
		CheckLoopStopTime = Micros();
		CheckLoopStopTime = Micros();
		CheckLoopDiffTime = CheckLoopStopTime - CheckLoopStartTime;
	}
	else if (htim == &htim4) {

		}
}

uint64_t Micros(){
	return _micro + TIM11->CNT;
}

/* Initialize the Ring Buffer */
void Ringbuf_Init (void)
{
	memset(RxBuf, '\0', RxBuf_SIZE);
	memset(MainBuf, '\0', MainBuf_SIZE);

	Head = Tail = 0;
	oldPos = 0;
	newPos = 0;

  HAL_UARTEx_ReceiveToIdle_DMA(&UART, RxBuf, RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&DMA, DMA_IT_HT);
}

/* Resets the Ring buffer */
void Ringbuf_Reset (void)
{
	memset(MainBuf,'\0', MainBuf_SIZE);
	memset(RxBuf, '\0', RxBuf_SIZE);
	Tail = 0;
	Head = 0;
	oldPos = 0;
	newPos = 0;
}

uint8_t checkSum (uint8_t *buffertoCheckSum , int bufferSize)
{
	uint8_t sum = 0;
	for (int index = 0; index < bufferSize-1; ++index)
	{
		sum = sum + buffertoCheckSum[index];
	}
	if((uint8_t)(buffertoCheckSum[bufferSize-1])==(uint8_t)(~sum))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
		oldPos = newPos;  // Update the last position before copying new data

		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
		 * This is to maintain the circular buffer
		 * The old data in the main buffer will be overlapped
		 */
		if (oldPos+Size > MainBuf_SIZE)  // If the current position + new data size is greater than the main buffer
		{
			uint16_t datatocopy = MainBuf_SIZE-oldPos;  // find out how much space is left in the main buffer
			memcpy ((uint8_t *)MainBuf+oldPos, (uint8_t *)RxBuf, datatocopy);  // copy data in that remaining space

			oldPos = 0;  // point to the start of the buffer
			memcpy ((uint8_t *)MainBuf, (uint8_t *)RxBuf+datatocopy, (Size-datatocopy));  // copy the remaining data
			newPos = (Size-datatocopy);  // update the position
		}

		/* if the current position + new data size is less than the main buffer
		 * we will simply copy the data into the buffer and update the position
		 */
		else
		{
			memcpy ((uint8_t *)MainBuf+oldPos, (uint8_t *)RxBuf, Size);
			newPos = Size+oldPos;
		}

		/* Update the position of the Head
		 * If the current position + new size is less then the buffer size, Head will update normally
		 * Or else the head will be at the new position from the beginning
		 */
		if (Head+Size < MainBuf_SIZE) Head = Head+Size;
		else Head = Head+Size - MainBuf_SIZE;

		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&UART, (uint8_t *) RxBuf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&DMA, DMA_IT_HT);


	/****************** PROCESS (Little) THE DATA HERE *********************/
		if(checkSum(RxBuf, newPos-oldPos))
		{
		stateManagement(RxBuf,newPos,oldPos);
		}
}

void stateManagement(uint8_t *Rxbuffer , uint16_t rxDataCurPos , uint16_t rxDataLastPos)
{
	uint16_t rxDatalen = rxDataCurPos - rxDataLastPos;
	switch (MainState) {
		case emergency:
			//Do Some thing
			// Return to norm by IT GPIO
			break;
		case MCDisCon:
			if(Rxbuffer[0] == 0b10010010)
			{
				// Connect MC and Back to normal
				MainState = normOperation;
				HAL_UART_Transmit_IT(&UART, ACK_1, 2);
			}
			break;
		case normOperation:
			switch (Rxbuffer[0])
			{
				// Mode 1 Test Command
				case 0b10010001:
					// Do nothing
					HAL_UART_Transmit_IT(&UART, ACK_1, 2);
				break;
				// Mode 2 Connect MC
				case 0b10010010:
					// Start and Connect MC
					MainState = normOperation;
					HAL_UART_Transmit_IT(&UART, ACK_1, 2);
				break;
				// Mode 3 Disconnect MC
				case 0b10010011:
					// Disconnect MC
					MainState = MCDisCon;
					HAL_UART_Transmit_IT(&UART, ACK_1, 2);
				break;
				// Mode 4 Set Angular Velocity
				case 0b10010100:
					uartVelo = Rxbuffer[2];
					HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
					break;
				// Mode 5 Set Angular Position
				case 0b10010101:
					uartPos = (Rxbuffer[1] << 8) | Rxbuffer[2];
					HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
					break;
//				// Mode 6
//				case 0b10010110:
//					memset(uartGoal, 0, 15);
//					goalAmount = 1;
//					uartGoal[0] = RxDataBuffer[rxDataStart + 2];
//					HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
//					break;
//				// Mode 7
//				case 0b10010111:
//					memset(uartGoal, 0, 15);
//					goalAmount = RxDataBuffer[rxDataStart + 1];
//					for(int i = 0; i < ((goalAmount+1)/2); i++){
//						uartGoal[0+(i*2)] = RxDataBuffer[rxDataStart+(2+i)] & 15; // low 8 bit (last 4 bit)
//						uartGoal[1+(i*2)] = RxDataBuffer[rxDataStart+(2+i)] >> 4; // high 8 bit (first 4 bit)
//					}
//					HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
//					break;
				// Mode 8
				case 0b10011000:
					runningFlag = 1;
					HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
					break;
				// Mode 9
				case 0b10011001:
					goalData = 10;
					if(runningFlag == 1){
						memcpy(sendData, ACK_1, 2);
						sendData[2] = 153; // start-mode
						sendData[3] = 0;
						sendData[4] = goalData; // set current goal
						sendData[5] = (uint8_t)(~(sendData[2]+sendData[3]+sendData[4]));
					}
					else{
						memcpy(sendData, ACK_2, 2);
						sendData[2] = 153; // start-mode
						sendData[3] = 0;
						sendData[4] = goalData; // set current goal
						sendData[5] = (uint8_t)(~(sendData[2]+sendData[3]+sendData[4]));
					}
					HAL_UART_Transmit_IT(&huart2, sendData, 6);
					break;
				// Mode 10
				case 0b10011010:
					modeNo = 10;
					posData = 10271; // data from zhong
					if(runningFlag == 1){
						memcpy(sendData, ACK_1, 2);
						sendData[2] = 154; // start-mode
						sendData[3] = ((posData*65535)/16000) & 255; // set low byte posData
						sendData[4] = ((posData*65535)/16000) >> 8; // set high byte posData
						sendData[5] = (uint8_t)(~(sendData[2]+sendData[3]+sendData[4]));
					}
					else{
						memcpy(sendData, ACK_2, 2);
						sendData[2] = 154; // start-mode
						sendData[3] = ((posData*65535)/16000) & 255; // set low byte posData
						sendData[4] = ((posData*65535)/16000) >> 8; // set high byte posData
						sendData[5] = (uint8_t)(~(sendData[2]+sendData[3]+sendData[4]));
					}
					HAL_UART_Transmit_IT(&huart2, sendData, 6);
					break;
				// Mode 11
				case 0b10011011:
					veloData = 2496;
					if(runningFlag == 1){
						memcpy(sendData, ACK_1, 2);
						sendData[2] = 155;
						sendData[4] = ((veloData*255)/16000) & 255; // set low byte posData
						sendData[5] = (~(sendData[2]+sendData[3]+sendData[4]));
					}
					else{
						memcpy(sendData, ACK_2, 2);
						sendData[2] = 155;
						sendData[4] = ((veloData*255)/16000) & 255; // set low byte posData
						sendData[5] = (~(sendData[2]+sendData[3]+sendData[4]));
					}
					HAL_UART_Transmit_IT(&huart2, sendData, 6);
					break;
				// Mode 12
				case 0b10011100:
					endEffFlag = 1;
					HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
					break;
				// Mode 13
				case 0b10011101:
					endEffFlag = 0;
					HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
					break;
				// Mode 14
				case 0b10011110:
					homingFlag = 1;
					HAL_UART_Transmit_IT(&huart2, ACK_1, 2);
					break;
				}
	}
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
