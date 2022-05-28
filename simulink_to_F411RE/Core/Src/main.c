/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "math.h"
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
 TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
int8_t RxBuffer[64]={0};
uint8_t data[7]={73,109,64,99,0,0,126};

//float iniAngle = 0;
float finAngle = 360.0;
float veloMax = (2*math.pi*10)/60;
float accelMax = 0.5;
float jerkMax = 0.4;
float destAngle;
float timePeriod[7] = {0};
float b[7], c[7], d[7];
float posOut, veloOut, accelOut, jerkOut;
float timeElapsed;
float samplingTime = 0.001;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
//void uartprotocol();
void Drivemotor(int PWM);
void trajectoryGen(float toAngle);
void trajectoryEval();
void PID();
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart2,RxBuffer, 64);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);


  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_UART_Transmit_IT(&huart2, data, 7);
  HAL_TIM_Base_Start_IT(&htim5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uartprotocol();

	  trajectoryGen(destAngle);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim2.Init.Period = 3071;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 99;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 9999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t Posdata=0;
uint8_t PosdataPre=0;

int32_t valueuart=0;
uint8_t stateuart=0;
//void uartprotocol(){
//	static int8_t tempuart=0;
//	Posdata=huart2.RxXferSize-huart2.hdmarx->Instance->NDTR;
//	if(Posdata!=PosdataPre ){
//
//
//		switch(stateuart){
//			case 0:
//				if(RxBuffer[PosdataPre]==73){
//					stateuart=1;
//
//				}else{
//					stateuart=0;
//				}
//			break;
//			case 1:
//				if(RxBuffer[PosdataPre]==109){
//					stateuart=2;
//
//				}else{
//					stateuart=0;
//				}
//			break;
//			case 2:
//				if(RxBuffer[PosdataPre]==64){
//					stateuart=3;
//
//				}else{
//					stateuart=0;
//				}
//			break;
//			case 3:
//				if(RxBuffer[PosdataPre]==99){
//					stateuart=4;
//
//				}else{
//					stateuart=0;
//				}
//			break;
//			case 4:
//				tempuart=RxBuffer[PosdataPre];
//				stateuart=5;
//			break;
//			case 5:
//				if(RxBuffer[PosdataPre]==126){
//					valueuart=(int32_t)tempuart;
//					valueuart=(valueuart*500)/127;
////					htim1.Instance->CCR1=(valueuart*10000)/255;
//					Drivemotor(valueuart);
//					stateuart=0;
//				}else{
//					stateuart=0;
//				}
//			break;
//
//		}
//
//
//
//
//		PosdataPre=(PosdataPre+1)%64;
//	}
//
//
//
//}

uint16_t value=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim5){
		value=htim2.Instance->CNT;
		data[5]=(value&0b1111111100000000) >>8;
		data[4]=value&0b11111111;
		HAL_UART_Transmit_IT(&huart2, data, 7);
	}
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

void trajectoryGen(){
	totalTime = (accelMax/jerkMax) + (veloMax/accelMax) + (finAngle/veloMax);
	timePeriod[0] = accelMax/jerkMax;
	timePeriod[1] = (veloMax/accelMax);
	timePeriod[2] = (accelMax/jerkMax) + (veloMax/accelMax);
	timePeriod[3] = totalTime - timePeriod[2];
	timePeriod[4] = totalTime - timePeriod[1];
	timePeriod[5] = totalTime - timePeriod[0];
	timePeriod[6] = totalTime;

	b[0] = 0;
	b[1] = accelMax;
	b[2] = accelMax + (jerkMax*timePeriod[1]);
	b[3] = 0;
	b[4] = jerkMax*timePeriod[3];
	b[5] = -accelMax;
	b[6] = -accelMax - (jerkMax*timePeriod[5]);

	c[0] = 0;
	c[1] = ((jerkMax*timePeriod[0]^2)/2 + b[0]*timePeriod[0] + c[0]) - ((0*timePeriod[0]^2)/2 + b[1]*timePeriod[0]);
	c[2] = ((0*timePeriod[1]^2)/2 + b[1]*timePeriod[1] + c[1]) - ((-jerkMax*timePeriod[1]^2)/2 + b[2]*timePeriod[1]);
	c[3] = ((-jerkMax*timePeriod[2]^2)/2 + b[2]*timePeriod[2] + c[2]) - ((0*timePeriod[2]^2)/2 + b[3]*timePeriod[2]);
	c[4] = ((0*timePeriod[3]^2)/2 + b[3]*timePeriod[3] + c[3]) - ((-jerkMax*timePeriod[3]^2)/2 + b[4]*timePeriod[3]);
	c[5] = ((-jerkMax*timePeriod[4]^2)/2 + b[4]*timePeriod[4] + c[4]) - ((0*timePeriod[4]^2)/2 + b[5]*timePeriod[4]);
	c[6] = ((0*timePeriod[5]^2)/2 + b[5]*timePeriod[5] + c[5]) - ((jerkMax*timePeriod[5]^2)/2 + b[6]*timePeriod[5]);

	d[0] = 0;
	d[1] = ((jerkMax*timePeriod[0]^3)/6 + (b[0]*timePeriod[0]^2)/2 + c[0]*timePeriod[0] + d[0]) - ((0*timePeriod[0]^3)/6 + (b[1]*timePeriod[0]^2)/2 + c[1]*timePeriod[0]);
	d[2] = ((0*timePeriod[1]^3)/6 + (b[1]*timePeriod[1]^2)/2 + c[1]*timePeriod[1] + d[1]) - ((-jerkMax*timePeriod[1]^3)/6 + (b[2]*timePeriod[1]^2)/2 + c[2]*timePeriod[1]);
	d[3] = ((-jerkMax*timePeriod[2]^3)/6 + (b[2]*timePeriod[2]^2)/2 + c[2]*timePeriod[2] + d[2]) - ((0*timePeriod[2]^3)/6 + (b[3]*timePeriod[2]^2)/2 + c[3]*timePeriod[2]);
	d[4] = ((0*timePeriod[3]^3)/6 + (b[3]*timePeriod[3]^2)/2 + c[3]*timePeriod[3] + d[3]) - ((-jerkMax*timePeriod[3]^3)/6 + (b[4]*timePeriod[3]^2)/2 + c[4]*timePeriod[3]);
	d[5] = ((-jerkMax*timePeriod[4]^3)/6 + (b[4]*timePeriod[4]^2)/2 + c[4]*timePeriod[4] + d[4]) - ((0*timePeriod[4]^3)/6 + (b[5]*timePeriod[4]^2)/2 + c[5]*timePeriod[4]);
	d[6] = ((0*timePeriod[5]^3)/6 + (b[5]*timePeriod[5]^2)/2 + c[5]*timePeriod[5] + d[5]) - ((jerkMax*timePeriod[5]^3)/6 + (b[6]*timePeriod[5]^2)/2 + c[6]*timePeriod[5]);

//	iniAngle = (iniAngle + toAngle) % 360;
}

void trajectoryEval(){
	if (timeElapsed >= 0 && timeElapsed < timePeriod[0]){
		accelOut = jerkMax*timeElapsed + b[0];
	    jerkOut = jerkMax;
	    veloOut = jerkMax*timeElapsed^2/2 + b[0]*timeElapsed + c[0];
	    posOut = jerkMax*timeElapsed^3/6 + b[0]*timeElapsed^2/2 + c[0]*timeElapsed + d[0];
	}

	else if (timeElapsed >= timePeriod[0] && timeElapsed < timePeriod[1]){
		accelOut = 0*timeElapsed + b[1];
	    jerkOut = 0;
	    veloOut = 0*timeElapsed^2/2 + b[1]*timeElapsed + c[1];
	    posOut = 0*timeElapsed^3/6 + b[1]*timeElapsed^2/2 + c[1]*timeElapsed + d[1];
	}

	else if (timeElapsed >= timePeriod[1] && timeElapsed < timePeriod[2]){
		accelOut = -jerkMax*timeElapsed + b[2];
	    jerkOut = -jerkMax;
	    veloOut = -jerkMax*timeElapsed^2/2 + b[2]*timeElapsed + c[2];
	    posOut = -jerkMax*timeElapsed^3/6 + b[2]*timeElapsed^2/2 + c[2]*timeElapsed + d[2];
	}

	else if (timeElapsed >= timePeriod[2] && timeElapsed < timePeriod[3]){
		accelOut = 0*timeElapsed + b[3];
	    jerkOut = 0;
	    veloOut = 0*timeElapsed^2/2 + b[3]*timeElapsed + c[3];
	    posOut = 0*timeElapsed^3/6 + b[3]*timeElapsed^2/2 + c[3]*timeElapsed + d[3];
	}

	else if (timeElapsed >= timePeriod[3] && timeElapsed < timePeriod[4]){
		accelOut = -jerkMax*timeElapsed + b[4];
	    jerkOut = -jerkMax;
	    veloOut = -jerkMax*timeElapsed^2/2 + b[4]*timeElapsed + c[4];
	    posOut = -jerkMax*timeElapsed^3/6 + b[4]*timeElapsed^2/2 + c[4]*timeElapsed + d[4];
	}

	else if (timeElapsed >= timePeriod[4] && timeElapsed < timePeriod[5]){
		accelOut = 0*timeElapsed + b[5];
	    jerkOut = 0;
	    veloOut = 0*timeElapsed^2/2 + b[5]*timeElapsed + c[5];
	    posOut = 0*timeElapsed^3/6 + b[5]*timeElapsed^2/2 + c[5]*timeElapsed + d[5];
	}

	else{
		accelOut = jerkMax*timeElapsed + b[6];
	    jerkOut = jerkMax;
	    veloOut = jerkMax*timeElapsed^2/2 + b[6]*timeElapsed + c[6];
	    posOut = jerkMax*timeElapsed^3/6 + b[6]*timeElapsed^2/2 + c[6]*timeElapsed + d[6];
	}
}

void PIDController(){
	float ohHibub = 0;
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
