/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define  FACE1_DTYCYC						(TIM4->CCR1)
#define  FACE2_DTYCYC						(TIM4->CCR2)
#define  FACE3_DTYCYC						(TIM4->CCR3)
#define  FACE4_DTYCYC						(TIM4->CCR4)
#define  MOTION_ANGLE_0_360			0
#define  MOTION_ANGLE_90			    90
#define  MOTION_ANGLE_180				180
#define  MOTION_ANGLE_270				270
#define  TIMER_OVF_PERIOD				12
#define  USART_TIMEOUT				   100
#define  RXBUFFERSIZE                      1
#define  IMU_REG_ACCEL_DATA         ((uint8_t)0x3B)
#define  IMU_REG_GYRO_DATA          ((uint8_t)0x43)
#define  IMU_REG_TEMP_DATA          ((uint8_t)0x41)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t TIM_OVF = 0;
uint16_t 	MotionAng = 0;
uint8_t 		aTxBuffer[4];
uint8_t 		getMasterInput = 0;
uint8_t 		getTransferDirection = 0;
/* Buffer used for reception */
uint8_t 		aRxBuffer[4];
uint32_t 	ICValue = 0;
uint32_t 	Frequency = 0;
uint32_t 	XXICValue, YYICValue, ZZICValue;
uint8_t		i2ctemp[2] = { 0x06, 0xd2 };
uint8_t 		i2cAcc[6] = { 0, 2, 0, 3, 0, 0 };
uint8_t 		i2cGero[6] = { 0, 0, 0, 0, 0, 0 };
uint16_t	ACC_Xangle = 0;
uint16_t 	ACC_Yangle = 90;
uint16_t 	ACC_Zangle = 180;
uint16_t	Gyro_Xangle = 0;
uint16_t 	Gyro_Yangle = 0;
uint16_t 	Gyro_Zangle =0;
uint8_t 		increaseFlag = 0;
uint8_t		SimulationFace1Data[8] = {100,80,60,25,15,25,60,80};
uint8_t		SimulationFace2Data[8] = {60,80,100,80,60,25,15,25};
uint16_t	DAC1Val[8] 					 =	 {4095,3276,2457,1024,615,1024,2457,3276};
uint16_t	DAC2Val[8] 					 =  {615,1024,2457,3276,4095,3276,2457,1024};
uint8_t		SimulationFace3Data[8] = {15,25,60,80,100,80,60,25};
uint8_t		SimulationFace4Data[8] = {60,25,15,25,60,80,100,80};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void MotorX_Read(void);
void MotorY_Read(void);
void MotorZ_Read(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MotorX_Read(void)
{
	if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10) == 1)
			&& (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == 0)) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "X FORWARD \n", 11, 1000);
		if (ACC_Xangle != 360)
			ACC_Xangle++;
		if (ACC_Xangle == 360)
			ACC_Xangle = 0;

		i2cAcc[0] = ACC_Xangle >> 8;
		i2cAcc[1] = (uint8_t) ACC_Xangle;

	} else if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10) == 0)
			&& (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == 1)) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "X BACK  \n", 11, 1000);
		if (ACC_Xangle != 0)
			ACC_Xangle--;
		if (ACC_Xangle == 0)
			ACC_Xangle = 360;

		i2cAcc[0] = ACC_Xangle >> 8;
		i2cAcc[1] = (uint8_t) ACC_Xangle;
	} else {

	}
}
void MotorY_Read(void)
{
	if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == 1)
			&& (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == 0)) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "Y FORWARD \n", 11, 1000);
		if (ACC_Yangle != 360)
			ACC_Yangle++;
		if (ACC_Yangle == 360)
			ACC_Yangle = 0;

		i2cAcc[2] = ACC_Yangle >> 8;
		i2cAcc[3] = (uint8_t) ACC_Yangle;

	} else if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == 0)
			&& (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == 1)) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "Y BACK  \n", 11, 1000);
		if (ACC_Yangle != 0)
			ACC_Yangle--;
		if (ACC_Yangle == 0)
			ACC_Yangle = 360;

		i2cAcc[2] = ACC_Yangle >> 8;
		i2cAcc[3] = (uint8_t) ACC_Yangle;
	} else {

	}
}
void MotorZ_Read(void)
{
	if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == 1)
			&& (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == 0)) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "Z FORWARD \n", 11, 1000);
		if (ACC_Zangle != 360)
			ACC_Zangle++;
		if (ACC_Zangle == 360)
			ACC_Zangle = 0;

		i2cAcc[4] = ACC_Zangle >> 8;
		i2cAcc[5] = (uint8_t) ACC_Zangle;

	} else if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) == 0)
			&& (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == 1)) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "Z BACK  \n", 11, 1000);
		if (ACC_Zangle != 0)
			ACC_Zangle--;
		if (ACC_Zangle == 0)
			ACC_Zangle = 360;

		i2cAcc[4] = ACC_Zangle >> 8;
		i2cAcc[5] = (uint8_t) ACC_Zangle;
	} else {

	}
}
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
  MX_DAC_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2C_EnableListen_IT(&hi2c2);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	/*-------------------------------------------
	 * Start PWM for the Different Channels
	 * ------------------------------------------
	 */
	HAL_TIM_Base_Start_IT(&htim3);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	/*-------------------------------------------
	 * Set The Duty Cycle of Various PWM channels
	 * -------------------------------------------
	 */
	FACE1_DTYCYC = 90;
	FACE2_DTYCYC = 85;
	FACE3_DTYCYC = 25;
	FACE4_DTYCYC = 25;
	HAL_UART_Transmit(&huart2, (uint8_t*) "Hello Seif El-Didi && Islam Hany \n", 34,
						USART_TIMEOUT);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (TIM_OVF >= TIMER_OVF_PERIOD) {
			/*-------------------------------------------------------------
			 * Set Duty Cycle of PWM to Indicate LDR Voltage
			 * -------------------------------------------------------------
			 */
			FACE1_DTYCYC = SimulationFace1Data[MotionAng];
			FACE2_DTYCYC = SimulationFace2Data[MotionAng];
			FACE3_DTYCYC = SimulationFace3Data[MotionAng];
			FACE4_DTYCYC = SimulationFace4Data[MotionAng];
			/*-------------------------------------------------------------
			 * Set Dac Value to Simulate Solar Cells on Face 1 and 3
			 * -------------------------------------------------------------
			 */
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
					DAC1Val[MotionAng]);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
					DAC2Val[MotionAng]);
			MotionAng = (MotionAng == 7 ? 0 : MotionAng + 1);
			TIM_OVF = 0;

			/*-------------------------------------------------------------
			* tilting the CubeSat to change the Gyroscope readings
			* -------------------------------------------------------------
			*/
			Gyro_Zangle += ((45*65535)/360);
			i2cGero[4]	 = Gyro_Zangle>>8;
			i2cGero[5] 	 = Gyro_Zangle;

			if(Gyro_Zangle >= 65535)
				Gyro_Zangle = 0;

	  }
	  /********************** X Motor Read *************************/
	  MotorX_Read();
	  /*********************  Y Motor Read ***********************/
	  MotorY_Read();
	  	/******************** Z Motor Read ***********************/
	  MotorZ_Read();
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 222;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim3.Init.Prescaler = 65535-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PE10 PE11 PE12 PE13
                           PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	TIM_OVF ++;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {

    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
	HAL_I2C_EnableListen_IT(hi2c);
}
int flag = 0;

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
		uint16_t AddrMatchCode) {
	getTransferDirection = TransferDirection;
	if (TransferDirection == I2C_DIRECTION_TRANSMIT) // if the master wants to transmit the data
	{
		/*
		 * receive using sequential function.
		 *The I2C_FIRST_AND_LAST_FRAME implies that the slave
		 * will send a NACK after receiving "entered" num of bytes
		 */
		HAL_I2C_Slave_Sequential_Receive_IT(hi2c , aRxBuffer , 1 , I2C_FIRST_AND_LAST_FRAME);
	} else  // if the master requests the data from the slave
	{
		//HAL_UART_Transmit(&huart2, "SENDDDDDDD", 13, 1000);

		//HAL_I2C_Slave_Transmit(&hi2c3, tomaster,2, 100);
		//HAL_Delay(1)
		flag = 1;
		if (aRxBuffer[0] == IMU_REG_ACCEL_DATA) {
			HAL_I2C_Slave_Sequential_Transmit_IT(&hi2c2, i2cAcc, 6,
					I2C_FIRST_AND_NEXT_FRAME);

		}
		if (aRxBuffer[0] == IMU_REG_GYRO_DATA) {

			HAL_I2C_Slave_Sequential_Transmit_IT(&hi2c2, i2cGero, 6,
					I2C_FIRST_AND_NEXT_FRAME);
		}
		if (aRxBuffer[0] == IMU_REG_TEMP_DATA) {
			HAL_I2C_Slave_Sequential_Transmit_IT(&hi2c2, i2ctemp, 2,
					I2C_FIRST_AND_NEXT_FRAME);
		}

		/*	HAL_Delay(2);
		 HAL_I2C_DeInit(&hi2c3);
		 HAL_I2C_Init(&hi2c3);
		 */

		//HAL_I2C_EnableListen_IT(&hi2c3);
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
