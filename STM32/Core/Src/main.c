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
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Controller parameters */
#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_KD  0.25f

#define PID_TAU 0.02f

#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.01f

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 4.0f

/* Kalman Filter parameters */
#define dt  0.001f
#define var  1.0f
<<<<<<< HEAD
#define KalmanR 0.000001f

=======
>>>>>>> parent of 06c0c4d (Update แยกไฟล์ Function)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
<<<<<<< HEAD
/* Setup micro sec */
=======
/////////////////////////
arm_status Kalmanstatus;
// Matrix A
float32_t MatA_Data[9] = {1 , dt , 0.5*dt*dt,
						  0 , 1 , dt,
						  0 , 0 , 1};
arm_matrix_instance_f32 MatA;

// Matrix B
float32_t MatB_Data[3] = { 	0,
							0,
							0};
arm_matrix_instance_f32 MatB;

// Matrix C
float32_t MatC_Data[3] = {1,0,0};
arm_matrix_instance_f32 MatC;

// Matrix D
float32_t MatD_Data[1] = {0};
arm_matrix_instance_f32 MatD;

// Matrix Q
float32_t MatQ_Data[9] = {	((dt*dt*dt*dt)*var)/4 , ((dt*dt*dt)*var)/2 , ((dt*dt)*var)/2,
							((dt*dt*dt)*var)/2 , ((dt*dt)*var)   , dt,
							((dt*dt)*var)/2 , dt             , 1};
arm_matrix_instance_f32 MatQ;

// Matrix R
float32_t MatR_Data[1] = {0.000001};
arm_matrix_instance_f32 MatR;

// Matrix G
float32_t MatG_Data[9] = {	0 , 0 , ((dt*dt*dt))/6,
							0 , 0 , ((dt*dt))/2,
							0 , 0 , dt};
arm_matrix_instance_f32 MatG;

// Matrix State
float32_t MatState_Data[3] = { 	0,
								0,
								0};
arm_matrix_instance_f32 MatState;

// Matrix State Last
float32_t MatStateLast_Data[3] = { 	0,
									0,
									0};
arm_matrix_instance_f32 MatStateLast;

// Matrix Predict
float32_t MatPredict_Data[9] = {0 , 0 , 0,
								0 , 0 , 0,
								0 , 0 , 0};
arm_matrix_instance_f32 MatPredict;

// Matrix Predict Last
float32_t MatPredictLast_Data[9] = {0 , 0 , 0,
									0 , 0 , 0,
									0 , 0 , 0};
arm_matrix_instance_f32 MatPredictLast;

// Matrix Y
float32_t MatY_Data[1] = {0};
arm_matrix_instance_f32 MatY;

// Matrix Z
float32_t MatZ_Data[1] = {0};
arm_matrix_instance_f32 MatZ;

// Matrix S
float32_t MatS_Data[1] = {0};
arm_matrix_instance_f32 MatS;

// Matrix Kalman gain
float32_t MatK_Data[3] = { 	0,
							0,
							0};
arm_matrix_instance_f32 MatK;

// Matrix Iden
float32_t MatI_Data[9] = {1 , 0 , 0,
						  0 , 1 , 0,
						  0 , 0 , 1};
arm_matrix_instance_f32 MatI;

// Matrix Known Input
float32_t MatU_Data[1] = {0};
arm_matrix_instance_f32 MatU;

/* Matrix Buffer */
float32_t MatAt_Data[9] = {0};
arm_matrix_instance_f32 MatAt;
float32_t MatGt_Data[9] = {0};
arm_matrix_instance_f32 MatGt;
float32_t MatCt_Data[3] = {0};
arm_matrix_instance_f32 MatCt;
float32_t MatSinv_Data[1] = {0};
arm_matrix_instance_f32 MatSinv;
float32_t MatGQGt_Data[9] = {0};
arm_matrix_instance_f32 MatGQGt;
float32_t MatCPk_Data[3] = {0};
arm_matrix_instance_f32 MatCPk;
float32_t MatAPk_Data[9] = {0};
arm_matrix_instance_f32 MatAPk;
float32_t MatAPkAt_Data[9] = {0};
arm_matrix_instance_f32 MatAPkAt;
float32_t MatCXk_Data[1] = {0};
arm_matrix_instance_f32 MatCXk;
float32_t MatCPkCt_Data[1] = {0};
arm_matrix_instance_f32 MatCPkCt;
float32_t MatPkCt_Data[3] = {0};
arm_matrix_instance_f32 MatPkCt;
float32_t MatKYk_Data[3] = {0};
arm_matrix_instance_f32 MatKYk;
float32_t MatKC_Data[9] = {0};
arm_matrix_instance_f32 MatKC;
float32_t MatI_KC_Data[9] = {0};
arm_matrix_instance_f32 MatI_KC;

////////////////////////
>>>>>>> parent of 06c0c4d (Update แยกไฟล์ Function)
uint64_t _micro = 0;
/* Setup Encoder Data */
int EncoderRawData[2] = {0};
int WrappingStep = 0;
int PositionRaw = 0;;
float32_t PositionDeg = 0;
float32_t PositionRad = 0;
/* Setup Motor PWM */
int PWMC = 250;
uint8_t check = 0;
<<<<<<< HEAD
/* Initialise Kalman Filter */
KalmanFilterVar KalmanVar = {
		{1 , dt , 0.5*dt*dt,0 , 1 , dt,0 , 0 , 1},
		{0,0,0},
		{1,0,0},
		{0},
		{((dt*dt*dt*dt)*var)/4,((dt*dt*dt)*var)/2,((dt*dt)*var)/2,((dt*dt*dt)*var)/2,((dt*dt)*var),dt,((dt*dt)*var)/2,dt,1},
		{KalmanR},
		{0,0,((dt*dt*dt))/6,0, 0,((dt*dt))/2,0,0,dt},
		{0,0,0},
		{0,0,0},
		{0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,0},
		{0},
		{0},
		{0},
		{0,0,0},
		{1,0,0,0,1,0,0,0,1},
		{0},
		{0},
		{0},
		{0},
		{0},
		{0},
		{0},
		{0},
		{0},
		{0},
		{0},
		{0},
		{0},
		{0},
};
/* Initialise PID controller */
=======
 /* Initialise PID controller */
>>>>>>> parent of 06c0c4d (Update แยกไฟล์ Function)
PIDController pid = { PID_KP, PID_KI, PID_KD,
					  PID_TAU,
					  PID_LIM_MIN, PID_LIM_MAX,
		  PID_LIM_MIN_INT, PID_LIM_MAX_INT,
					  SAMPLE_TIME_S };
/* Simulate response using test system */
float setpoint = 1.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
uint64_t Micros();
void Drivemotor(int PWM);
void KalmanFilterFunction();
void KalmanMatrixInit();
void EncoderRead();
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
  /* USER CODE BEGIN 2 */
<<<<<<< HEAD
=======
  KalmanMatrixInit();
  //////////////////////////
>>>>>>> parent of 06c0c4d (Update แยกไฟล์ Function)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT (&htim11);
  HAL_TIM_Base_Start_IT (&htim3);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  EncoderRawData[0]=TIM2->CNT;
  EncoderRawData[1]=EncoderRawData[0];
  PositionRaw=EncoderRawData[0];
  /* Initialise PID controller */
  PIDController_Init(&pid);
  /* Initialise Kalman Filter */
  KalmanMatrixInit(&KalmanVar);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Drivemotor(PWMC);
	  static int timeStamp2 = 0;
	  if (Micros() - timeStamp2 > 2000000)
	  {
			timeStamp2 = Micros();
			PWMC = -1*PWMC;
	  }
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
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|PIN_DIR_Pin, GPIO_PIN_RESET);

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

}

/* USER CODE BEGIN 4 */
void KalmanMatrixInit()
{
	  arm_mat_init_f32(&MatA, 3, 3, MatA_Data);
	  arm_mat_init_f32(&MatB, 3, 1, MatB_Data);
	  arm_mat_init_f32(&MatC, 1, 3, MatC_Data);
	  arm_mat_init_f32(&MatD, 1, 1, MatD_Data);
	  arm_mat_init_f32(&MatQ, 3, 3, MatQ_Data);
	  arm_mat_init_f32(&MatR, 1, 1, MatR_Data);
	  arm_mat_init_f32(&MatG, 3, 3, MatG_Data);
	  arm_mat_init_f32(&MatState, 3, 1, MatState_Data);
	  arm_mat_init_f32(&MatStateLast, 3, 1, MatStateLast_Data);
	  arm_mat_init_f32(&MatPredict, 3, 3, MatPredict_Data);
	  arm_mat_init_f32(&MatPredictLast, 3, 3, MatPredictLast_Data);
	  arm_mat_init_f32(&MatY, 1, 1, MatY_Data);
	  arm_mat_init_f32(&MatZ, 1, 1, MatZ_Data);
	  arm_mat_init_f32(&MatS, 1, 1, MatS_Data);
	  arm_mat_init_f32(&MatK, 3, 1, MatK_Data);
	  arm_mat_init_f32(&MatI, 3, 3, MatI_Data);
	  arm_mat_init_f32(&MatAt, 3, 3, MatAt_Data);
	  arm_mat_init_f32(&MatGt, 3, 3, MatGt_Data);
	  arm_mat_init_f32(&MatCt, 3, 1, MatCt_Data);
	  arm_mat_init_f32(&MatGQGt, 3, 3, MatGQGt_Data);
	  arm_mat_init_f32(&MatSinv, 1, 1, MatSinv_Data);
	  arm_mat_init_f32(&MatCPk, 1, 3, MatCPk_Data);

	  arm_mat_init_f32(&MatAPk, 3, 3, MatAPk_Data);
	  arm_mat_init_f32(&MatAPkAt, 3, 3, MatAPkAt_Data);
	  arm_mat_init_f32(&MatCXk, 1, 1, MatCXk_Data);
	  arm_mat_init_f32(&MatCPkCt, 1, 1, MatCPkCt_Data);
	  arm_mat_init_f32(&MatPkCt, 3, 1, MatPkCt_Data);
	  arm_mat_init_f32(&MatKYk, 3, 1,MatKYk_Data);
	  arm_mat_init_f32(&MatKC, 3, 3, MatKC_Data);
	  arm_mat_init_f32(&MatI_KC, 3, 3, MatI_KC_Data);
	  // Get Transpose
	  arm_mat_trans_f32(&MatA, &MatAt);
	  arm_mat_trans_f32(&MatG, &MatGt);
	  arm_mat_trans_f32(&MatC, &MatCt);
	  // Get Buffer
	  arm_mat_mult_f32(&MatG, &MatQ, &MatGQGt);
	  arm_mat_mult_f32(&MatGQGt, &MatGt, &MatGQGt);
}

void KalmanFilterFunction()
{
	// 1.Prediction
	// Predicted State Estimate
	Kalmanstatus = arm_mat_mult_f32(&MatA, &MatStateLast, &MatState); // A*Xk-1 ,No B*u
	// Predicted error covariance
	Kalmanstatus = arm_mat_mult_f32(&MatA, &MatPredictLast, &MatAPk); // A*Pk-1
	Kalmanstatus = arm_mat_mult_f32(&MatAPk, &MatAt, &MatAPkAt); // A*Pk-1*At
	Kalmanstatus = arm_mat_add_f32(&MatAPkAt, &MatGQGt, &MatPredict); // A*Pk-1*At + GQGt
	// 2.Correction
	// Innovation residual
	MatZ_Data[0] = PositionDeg; // Sensor Input
	Kalmanstatus = arm_mat_mult_f32(&MatC, &MatState, &MatCXk); // C*Xk
	Kalmanstatus = arm_mat_sub_f32(&MatZ, &MatCXk, &MatY); // Zk - C*Xk
	// Innovation covariance
	Kalmanstatus = arm_mat_mult_f32(&MatC, &MatPredict, &MatCPk); // C*Pk
	Kalmanstatus = arm_mat_mult_f32(&MatCPk, &MatCt, &MatCPkCt); // C*Pk*Ct
	Kalmanstatus = arm_mat_add_f32(&MatCPkCt, &MatR, &MatS); // C*Pk*Ct + R
	Kalmanstatus = arm_mat_inverse_f32(&MatS, &MatSinv); // S inverse
	// Optimal Kalman gain
	Kalmanstatus = arm_mat_mult_f32(&MatPredict, &MatCt, &MatPkCt); // Pk*Ct
	Kalmanstatus = arm_mat_mult_f32(&MatPkCt, &MatSinv, &MatK); // Pk*Ct*Sinv
	// Corrected state estimate
	Kalmanstatus = arm_mat_mult_f32(&MatK, &MatY, &MatKYk); // K*Yk
	Kalmanstatus = arm_mat_add_f32(&MatKYk, &MatState, &MatStateLast); // Xk+K*Yk
	// Corrected estimate covariance
	Kalmanstatus = arm_mat_mult_f32(&MatK, &MatC, &MatKC); //K*C
	Kalmanstatus = arm_mat_sub_f32(&MatI, &MatKC, &MatI_KC); // I-K*C
	Kalmanstatus = arm_mat_mult_f32(&MatI_KC, &MatPredict, &MatPredictLast); // (I-K*C)*Pk
}

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
	PositionRad = (PositionRaw/12000.0)*2.0*3.14;
	PositionDeg = (PositionRaw/12000.0)*360.0;
	EncoderRawData[1] = EncoderRawData[0];
}

uint32_t aaabs(int x){

	if(x<0){
		return x*-1;
	}else{
		return x;
	}
}


void Drivemotor(int PWM){
	if(PWM<=0 && PWM>=-500){
		htim1.Instance->CCR1=aaabs(PWM);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,0);
	}else if (PWM<-500){
		htim1.Instance->CCR1=500;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,0);
	}else if(PWM>=0 && PWM<=500){
		htim1.Instance->CCR1=aaabs(PWM);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,1);
	}else if(PWM>500){
		htim1.Instance->CCR1=500;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim11) {
		_micro += 65535;
	}
	if (htim == &htim3) {
		EncoderRead();
		KalmanFilterFunction();
		}
}

uint64_t Micros(){
	return _micro + TIM11->CNT;
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
