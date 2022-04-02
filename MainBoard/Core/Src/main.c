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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_TIME 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

typedef struct uartStruct{
	unsigned char rxData[30];
	unsigned char rxBuffer[30];
	unsigned char txBuffer[30];
	uint16_t txBufferLength;
}Uart;

Uart PcUart;
Uart Motor1Uart;
Uart Motor2Uart;
Uart Flipper1Uart;
Uart Flipper2Uart;

struct flagStruct {
	struct {
		_Bool motorForward_bit;
		_Bool motorBackward_bit;
		_Bool UART_bit;
		_Bool CANBUS_bit;
		_Bool adminMode_bit;
		_Bool testMode_bit;
		_Bool normalMode_bit;
		_Bool ERROR_bit;
	} LED;
	struct {
		uint8_t rxIndex_bool;
		_Bool rxComplete_bool;
	} UART;
};

//flag pc_flag;
//flag motor1_flag;
//flag motor2_flag;

struct flagStruct flag = {{0,0,0,0,0,0,0,0},{0,0}};

typedef struct motorStruct{
	uint32_t RPM;
	_Bool direction;
}Motor;

Motor Motor1;
Motor Motor2;

typedef struct flipperStruct{
	uint32_t position;
	_Bool direction;
}Flipper;

Flipper Flipper1;
Flipper Flipper2;


uint16_t communicationUart = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

void RAKE_UART_Led_Controller(){
		HAL_GPIO_WritePin(GPIOD, MOTOR1_LED_Pin|MOTOR2_LED_Pin|FLIPPER1_LED_Pin|FLIPPER2_LED_Pin, GPIO_PIN_SET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(GPIOD, MOTOR1_LED_Pin|MOTOR2_LED_Pin|FLIPPER1_LED_Pin|FLIPPER2_LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(250);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM2) {
		communicationUart++;
	}
}

void RAKE_UART_Callback(Uart *uart, UART_HandleTypeDef *huart) {
	
		flag.LED.UART_bit = 1;

		//RX Buffer sifirlandi
		if(flag.UART.rxIndex_bool == 0) {
			for(uint8_t index; index < 30; index++) {
				uart->rxBuffer[index] = 0;
			}
		}
		if(uart->rxData[0] == 'S') {
			flag.UART.rxIndex_bool = 0;
		}

		//RX Buffer a veriler yazildi
		if(uart->rxData[0] != 'F'){
			uart->rxBuffer[flag.UART.rxIndex_bool++] = uart->rxData[0];
			//RAKE_UART_Led_Controller();
		} else {
			flag.UART.rxIndex_bool = 0;
			flag.UART.rxComplete_bool = 1;
			
		}

		HAL_UART_Receive_IT(huart, uart->rxData, 1);
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == UART4){
		RAKE_UART_Callback(&PcUart, &huart4);
	}else {
		flag.LED.UART_bit = 0;
	}
	if(huart->Instance == USART6){
		RAKE_UART_Callback(&Motor1Uart, &huart6);
	}else {
		flag.LED.UART_bit = 0;
	}
	if(huart->Instance == USART3){
		RAKE_UART_Callback(&Motor2Uart, &huart3);
	}else {
		flag.LED.UART_bit = 0;
	}
		if(huart->Instance == USART2){
		RAKE_UART_Callback(&Flipper1Uart, &huart2);
	}else {
		flag.LED.UART_bit = 0;
	}
		if(huart->Instance == UART5){
		RAKE_UART_Callback(&Flipper2Uart, &huart5);
	}else {
		flag.LED.UART_bit = 0;
	}
}

void RAKE_Rx_Motor_Speed(void) {
	if(flag.UART.rxComplete_bool == 1) {
		if(PcUart.rxBuffer[20] == 'C') {

			//String olarak alinan degerler int e cevirildi
			Motor1.RPM 	= 
															(PcUart.rxBuffer[2] - '0') * 100 +
															(PcUart.rxBuffer[3] - '0') * 10 +
															(PcUart.rxBuffer[4] - '0');
			
			Motor1.direction = 			(PcUart.rxBuffer[1] - '0');
			
			Motor2.RPM 	= 
															(PcUart.rxBuffer[7] - '0') * 100 +
															(PcUart.rxBuffer[8] - '0') * 10 +
															(PcUart.rxBuffer[9] - '0');
			
			Motor2.direction = 			(PcUart.rxBuffer[6] - '0');
			
			Flipper1.position = 
															(PcUart.rxBuffer[12] - '0') * 100 +
															(PcUart.rxBuffer[13] - '0') * 10 +
															(PcUart.rxBuffer[14] - '0');
			
			Flipper1.direction = 		(PcUart.rxBuffer[11] - '0');
			
			Flipper2.position = 
															(PcUart.rxBuffer[17] - '0') * 100 +
															(PcUart.rxBuffer[18] - '0') * 10 +
															(PcUart.rxBuffer[19] - '0');
			
			Flipper2.direction = 		(PcUart.rxBuffer[16] - '0');

			flag.UART.rxComplete_bool = 0;
		}
	}	
}
void RAKE_Tx_Motor_Speed(void) {
	if(communicationUart > TX_TIME){
		/*
			Data Type --> "SavvvCF"
			S = Start								(char)
			a = measured.direction 	(integer)
			vvv = measured.RPM			(char[3])
			C = Control							(char)
			F = Finish							(char)
		*/

		//RPM ve direction verileri motorlara gönderildi
		int hun = 0, ten = 0, one = 0;
		
		hun = (Motor1.RPM / 100);
		ten	= (Motor1.RPM % 100) / 10;
		one	= (Motor1.RPM % 10);
		
		Motor1Uart.txBufferLength = sprintf(Motor1Uart.txBuffer, "S%d%d%d%dCF",
															 Motor1.direction, hun, ten, one);
		
		HAL_UART_Transmit_IT(&huart6, Motor1Uart.txBuffer, Motor1Uart.txBufferLength);
		
		hun = (Motor2.RPM / 100);
		ten	= (Motor2.RPM % 100) / 10;
		one	= (Motor2.RPM % 10);
		
		Motor2Uart.txBufferLength = sprintf(Motor2Uart.txBuffer, "S%d%d%d%dCF",
															 Motor2.direction, hun, ten, one);
		
		HAL_UART_Transmit_IT(&huart3, Motor2Uart.txBuffer, Motor2Uart.txBufferLength);

		hun = (Flipper1.position / 100);
		ten	= (Flipper1.position % 100) / 10;
		one	= (Flipper1.position % 10);
		
		Flipper1Uart.txBufferLength = sprintf(Flipper1Uart.txBuffer, "S%d%d%d%dCF",
															 Flipper1.direction, hun, ten, one);
		
		HAL_UART_Transmit_IT(&huart2, Flipper1Uart.txBuffer, Flipper1Uart.txBufferLength);
		
		hun = (Flipper2.position / 100);
		ten	= (Flipper2.position % 100) / 10;
		one	= (Flipper2.position % 10);
		
		Flipper2Uart.txBufferLength = sprintf(Flipper2Uart.txBuffer, "S%d%d%d%dCF",
															 Flipper2.direction, hun, ten, one);
		
		HAL_UART_Transmit_IT(&huart5, Flipper2Uart.txBuffer, Flipper2Uart.txBufferLength);
		
		//Motorlardan gelen veriler istenilen data formatina cevirildi
		/*
		strcat(Motor1Uart.rxBuffer, Motor1Uart.rxBuffer);
		Motor1Uart.rxBuffer[12] = 'F';
		for (int c = 6 - 1; c < 12 - 1; c++) {
			Motor1Uart.rxBuffer[c] = Motor1Uart.rxBuffer[c+1];
		}
		Motor1Uart.rxBuffer[5] = '-';
    strcpy(&Motor1Uart.rxBuffer[10], &Motor1Uart.rxBuffer[10 + 1]);,
		*/
		//HAL_UART_Transmit_IT(&huart4, Motor1Uart.rxBuffer, sizeof(Motor1Uart.rxBuffer));
		
		communicationUart = 0;
	
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
  MX_TIM2_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	//Baslangicta UART dan veri alma islemi gerceklestirildi
	HAL_UART_Receive_IT(&huart4, PcUart.rxData, 1);
	HAL_UART_Receive_IT(&huart6, Motor1Uart.rxData, 1);
	HAL_UART_Receive_IT(&huart3, Motor2Uart.rxData, 1);
	HAL_UART_Receive_IT(&huart2, Flipper1Uart.rxData, 1);
	HAL_UART_Receive_IT(&huart5, Flipper2Uart.rxData, 1);	
	//Baslangicta TIM_Interrupt baslatildi
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		RAKE_Rx_Motor_Speed();
		RAKE_Tx_Motor_Speed();
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
  RCC_OscInitStruct.PLL.PLLN = 72;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MOTOR1_LED_Pin|MOTOR2_LED_Pin|FLIPPER1_LED_Pin|FLIPPER2_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR1_LED_Pin MOTOR2_LED_Pin FLIPPER1_LED_Pin FLIPPER2_LED_Pin */
  GPIO_InitStruct.Pin = MOTOR1_LED_Pin|MOTOR2_LED_Pin|FLIPPER1_LED_Pin|FLIPPER2_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
