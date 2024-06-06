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
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/sockets.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PORT	22

#define MAX(a,b) (((a)>(b))?(a):(b))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

osThreadId StartHandle;
/* USER CODE BEGIN PV */
osThreadId ClientHandle;
int sock, g;
struct sockaddr_in address;
err_t err = -1;

char uart_bufT[1000];
//char uart_bufR[100];
int uart_buf_len;

char spi_buf[30];
int state;
uint16_t d_in;

/* create an array for DAC's values:

		row 0 for DAC1: X1, Y1, Z1, T1
		row 1 for DAC2: X2, Y2, Z2, T2
		row 2 for DAC3: X3, Y3, Z3, T3

		X,Y,Z are voltage value in volt and T is time in ms

*/
double DAC[4][4] = {
		{0.0, 0.0, 0.0, 0.0},
		{0.1, 0.1, 0.1, 1.0},
		{-0.1, -0.1, -0.1, 1.0},
		{0.0, 0.0, 0.0, 1.0}
};
const double v_ref = 3.0;
const int max_dec = 65536;
int last_r = 3;

char help[] = "Correct format for communication with compensation coils driver:\r\n"
			  "\r\n"
			  "for getting help!\r\n"
			  "help\r\n"
			  "\r\n"
			  "for closing connection!\r\n"
			  "exit\r\n"
			  "\r\n"
			  "for sending values:\r\n"
			  "DAC: state: val1; val2; val3; time\r\n"
			  "or\r\n"
			  "DAC: state; val1; val2; val3; time\r\n"
			  "or\r\n"
			  "DAC: state val1; val2; val3; time\r\n"
			  "EXAMPLE:\r\n"
			  "DAC: 000: 1.2565; -2.30; -0.065; 150\r\n"
			  "\r\n"
			  "for sending more state:\r\n"
			  "DAC: state: val1; val2; val3; time; & state: val1; val2; val3; time; & state: val1; val2; val3; time\r\n"
			  "\r\n"
			  "all states:\r\n"
			  "state1: 000; state2: 001; state3: 010\r\n"
			  "values should be between: -3.0 v - +3.0 v\r\n"
			  "time should be more than of 1 ms\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
void StartThread(void const * argument);

/* USER CODE BEGIN PFP */
void AcceptanceNewClient(int * argument);
void SendSpiMesToDac(uint32_t);
void SetDAC(uint8_t channel, uint16_t value);
void SendToDAC(int r);
int ExtractMessage(char* msg);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();
/* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // DAC - Reset select.
  // If RSTSEL is low, input coding is binary;
  // if high = 2's complement
  HAL_GPIO_WritePin(RSTSEL_GPIO_Port, RSTSEL_Pin, GPIO_PIN_RESET);

  SetDAC(0, 0);
  SetDAC(1, 0);
  SetDAC(2, 0);
  SetDAC(3, 0);

  // DIR SET means: positive and DIR RESET means: negative
  HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Start */
  osThreadDef(Start, StartThread, osPriorityNormal, 0, 512);
  StartHandle = osThreadCreate(osThread(Start), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 19;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_24BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|ENABLE_Pin|LD3_Pin|DIR1_Pin
                          |DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DIR3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RSTSEL_GPIO_Port, RSTSEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TTL1_Pin */
  GPIO_InitStruct.Pin = TTL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(TTL1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TTL3_Pin */
  GPIO_InitStruct.Pin = TTL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TTL3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TTL2_Pin */
  GPIO_InitStruct.Pin = TTL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(TTL2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LDAC_Pin */
  GPIO_InitStruct.Pin = LDAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LDAC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR3_Pin */
  GPIO_InitStruct.Pin = DIR3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENABLE_Pin DIR1_Pin DIR2_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin|DIR1_Pin|DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RSTSEL_Pin */
  GPIO_InitStruct.Pin = RSTSEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RSTSEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*
	 * In new board version, there are only 3 input TTLs.
	 * TTL1, TTL2 are used to set state
	 * TTL3 is used to trigger state change
	 */

  HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
  if(GPIO_Pin==TTL3_Pin)
  {
	  if(HAL_GPIO_ReadPin(GPIOE, TTL1_Pin) == GPIO_PIN_RESET &&
		 HAL_GPIO_ReadPin(TTL2_GPIO_Port, TTL2_Pin) == GPIO_PIN_RESET
	  ){
		  // state 1 row 0 in the DAC's array
		  if(last_r != 0){
			  SendToDAC(0);
		  }
	  }else if(HAL_GPIO_ReadPin(GPIOE, TTL1_Pin) == GPIO_PIN_SET &&
			  HAL_GPIO_ReadPin(TTL2_GPIO_Port, TTL2_Pin) == GPIO_PIN_RESET
	  ){
		  // state 2 row 1 in the DAC's array
		  if(last_r != 1){
			  SendToDAC(1);
		  }
	  }else if(HAL_GPIO_ReadPin(GPIOE, TTL1_Pin) == GPIO_PIN_RESET &&
				 HAL_GPIO_ReadPin(TTL2_GPIO_Port, TTL2_Pin) == GPIO_PIN_SET
				 ){
		  // state 3 row 2 in the DAC's array
		  if(last_r != 2){
			  SendToDAC(2);
		  }
	  }
  }
}

void SendSpiMesToDac(uint32_t message){
	/*
	 * New function for new DAC converter on version 2 of board
	 */
	HAL_StatusTypeDef status;
	uint8_t dataToSend[3] = {
			(message >> 0) & 0xFF,
	        (message >> 8) & 0xFF,
	        (message >> 16)& 0xFF
	};
	status = HAL_SPI_Transmit(&hspi1, dataToSend, 1, 100);
	if (status != HAL_OK) {
	}
}

void SetDAC(uint8_t channel, uint16_t value){
	/*
	 * New function for new DAC converter on version 2 of board
	 */
	uint32_t message = 0x00000000;
	message = message | (value & 0xFFFF);
	message = message | (((uint32_t)channel & 0b11) << 17);
	SendSpiMesToDac(message);

	// LDAC load DACs, rising edge triggered loads all DAC register
	HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, GPIO_PIN_RESET);
}

void SendToDAC(int r)  // original Mehrdad's function
/*
 * r - state number (0-3) (depends on TTLs state)
 */
{
	uint32_t t0, t1, t;
	if(DAC[r][3] < 1){
		DAC[r][3] = 1;
	}

	int n = round(DAC[r][3]*58);	// 58 to apply values to DAC
	double dif1, dif2, dif3;

	// x? this part need for sending correct value in first loop
	// state = 1;
//	d_in = abs(round(((DAC[0][0])/v_ref) * max_dec));

	/*******************************
	spi_buf[0] = 0x00;
	spi_buf[1] = ((uint8_t*)&d_in)[1];
	spi_buf[2] = ((uint8_t*)&d_in)[0];

//	HAL_GPIO_WritePin(CS3_GPIO_Port, CS3_Pin, RESET);
//	HAL_GPIO_WritePin(GPIOE, CS1_Pin, RESET);
//	HAL_SPI_Transmit_IT(&hspi4, (uint8_t *)&spi_buf, 3);

//	while(state){}
 * *****************************************
 * changed to \|/ */
//	SetDAC(0, d_in);
//	SetDAC(1, d_in);
//	SetDAC(2, d_in);
	/*        /|\   */

	// x? this part need for sending correct value in first loop

	dif1 = fabs(DAC[r][0] - DAC[last_r][0])/n;
	dif2 = fabs(DAC[r][1] - DAC[last_r][1])/n;
	dif3 = fabs(DAC[r][2] - DAC[last_r][2])/n;

	DAC[3][0] = DAC[last_r][0];
	DAC[3][1] = DAC[last_r][1];
	DAC[3][2] = DAC[last_r][2];

	if(r == 3){
		n = 1;
	}

	t0 = HAL_GetTick();
	for(int i = 1; i <= n; i++){

		if(DAC[r][0] > DAC[last_r][0]){
			DAC[3][0] += dif1;
		}else{
			DAC[3][0] -= dif1;
		}
		if(DAC[r][1] > DAC[last_r][1]){
			DAC[3][1] += dif2;
		}else{
			DAC[3][1] -= dif2;
		}
		if(DAC[r][2] > DAC[last_r][2]){
			DAC[3][2] += dif3;
		}else{
			DAC[3][2] -= dif3;
		}

		for(int j = 0; j < 3; j++){

			  state = 1;

			  switch(j){
				  case 0:
					  if(DAC[3][j] >= 0){
						  HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
					  }else{
						  HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
					  }
					  break;
				  case 1:
					  if(DAC[3][j] >= 0){
						  HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
					  }else{
						  HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
					  }
					  break;
				  case 2:
					  if(DAC[3][j] >= 0){
						  HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_SET);
					  }else{
						  HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_RESET);
					  }
					  break;
			  }

			  if(fabs(DAC[3][j]) > v_ref){
				  d_in = 0xffff;
			  }else{
				  d_in = abs(round((DAC[3][j]/v_ref) * max_dec));
			  }

			  SetDAC(j, d_in);
		}
	}

	t1 = HAL_GetTick();
	t = t1 - t0;
	last_r = r;
}

void AcceptanceNewClient(int * argument)
{

	char msg[200] = {};
	int newVal = 0, state = 1;
	int conn = *argument;

	while(1)
	{
		if(newVal == 3){
			close(conn);
			osThreadTerminate(NULL);
//			osThreadSuspend(NULL);
		}else if(newVal == 2){
			last_r = 3;
			newVal = 0;
		}else{

			memset(msg, 0, sizeof msg);
			state = read(conn, (char*)msg, 200);
			newVal = ExtractMessage(msg);
			if(state <= 0){
				newVal = 3;
			}
		}
		osDelay(100);
	}
}

int ExtractMessage(char* msg){

	char temp[100] = {};
	int j = 0, k = 0, f1 = 1, f2 = 1, f3 = 0;
	double temp_dac[3][4];

	for(int a = 0; a < 3; a++){
		for(int b = 0; b < 4; b++){
			temp_dac[a][b] = DAC[a][b];
		}
	}
	for(int i = 0; i < strlen(msg); i++){
		if(msg[i] == ':'){
			if(strcmp(temp, "DAC") != 0){
				return 0;
			}else{

				f1 = 1;
				i++;	// for removing first space after DAC:
				j = 0;
				memset(temp, 0, sizeof temp);
				while(f1){
					i++;
					if(msg[i] == ' ' || msg[i] == ':' || msg[i] == ';' || i >= strlen(msg)){
						f2 = 1;
						if(strcmp(temp, "000") == 0){
							j = 0;
							memset(temp, 0, sizeof temp);
							while(f2){
								i++;
								if(msg[i] == ';'){
									temp_dac[0][k] = atof(temp);
									k++;
									j = 0;
									memset(temp, 0, sizeof temp);
								}else if(i >= strlen(msg)){
									if(k == 3){
										temp_dac[0][k] = atof(temp);
										k++;
										j = 0;
										memset(temp, 0, sizeof temp);
									}else{
										uart_buf_len = sprintf(uart_bufT, "the format is wrong: %s\r\n\n%s\r\n", (char*)temp, (char*)help);
										HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);
										return 0;
									}
								}else{
									temp[j] = msg[i];
									j++;
									continue;
								}
								if(k > 3){
									f2 = 0;
									f3 = 1;
									k = 0;
									for(int m = 0; m < 4; m++){
										DAC[0][m] = temp_dac[0][m];
									}
								}
							}
						}else if(strcmp(temp, "001") == 0){
							j = 0;
							memset(temp, 0, sizeof temp);
							while(f2){
								i++;
								if(msg[i] == ';'){

									temp_dac[1][k] = atof(temp);
									k++;
									j = 0;
									memset(temp, 0, sizeof temp);
								}else if(i >= strlen(msg)){

									if(k == 3){

										temp_dac[1][k] = atof(temp);
										k++;
										j = 0;
										memset(temp, 0, sizeof temp);
									}else{

										uart_buf_len = sprintf(uart_bufT, "the format is wrong: %s\r\n\n%s\r\n", (char*)temp, (char*)help);
										HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);
										return 0;
									}
								}else{
									temp[j] = msg[i];
									j++;
									continue;
								}
								if(k > 3){
									f2 = 0;
									f3 = 1;
									k = 0;

									for(int m = 0; m < 4; m++){
										DAC[1][m] = temp_dac[1][m];
									}
								}
							}
						}else if(strcmp(temp, "010") == 0){

							j = 0;
							memset(temp, 0, sizeof temp);

							while(f2){

								i++;

								if(msg[i] == ';'){

									temp_dac[2][k] = atof(temp);
									k++;
									j = 0;
									memset(temp, 0, sizeof temp);
								}else if(i >= strlen(msg)){

									if(k == 3){

										temp_dac[2][k] = atof(temp);
										k++;
										j = 0;
										memset(temp, 0, sizeof temp);
									}else{

										uart_buf_len = sprintf(uart_bufT, "the format is wrong: %s\r\n\n%s\r\n", (char*)temp, (char*)help);
										HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);
										return 0;
									}
								}else{

									temp[j] = msg[i];
									j++;
									continue;
								}
								if(k > 3){

									f2 = 0;
									f3 = 1;
									k = 0;

									for(int m = 0; m < 4; m++){
										DAC[2][m] = temp_dac[2][m];
									}
								}
							}
						}else{

							uart_buf_len = sprintf(uart_bufT, "the format is wrong: %s\r\n\n%s\r\n", (char*)temp, (char*)help);
							HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);
							return 0;
						}
					}else{

						temp[j] = msg[i];
						j++;
						continue;
					}

					while(f3){

						i++;
						if(msg[i] == '&'){

							f3 = 0;
							i++;	// for removing first space after $
						}else if(i >= strlen(msg)){

							f3 = 0;
							f1 = 0;

							return 2;
						}
					}
				}
			}
		}else{

			temp[j] = msg[i];

			if(strcmp(temp, "exit") == 0 || strcmp(temp, "Exit") == 0 || strcmp(temp, "EXIT") == 0){

				uart_buf_len = sprintf(uart_bufT, "Exit!\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

				return 3;
			}else if(strcmp(temp, "help") == 0 || strcmp(temp, "Help") == 0 || strcmp(temp, "HELP") == 0){

				uart_buf_len = sprintf(uart_bufT, "%s\r\n", (char*)help);
				HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);

				return 1;
			}
			j++;

			continue;
		}
	}

	uart_buf_len = sprintf(uart_bufT, "wrong msg: %s\r\n\n%s\r\n", (char*)temp, (char*)help);
	HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);
	return 0;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartThread */
/**
  * @brief  Function implementing the Start thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartThread */
void StartThread(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */

  sock = socket(AF_INET, SOCK_STREAM, 0);

  address.sin_family = AF_INET;
  address.sin_port = htons(PORT);
  address.sin_addr.s_addr = INADDR_ANY;

  err = bind(sock, (struct sockaddr *)&address, sizeof (address));
  err = listen(sock, 0);

  SendToDAC(3);

  // create the acceptance thread
  osThreadDef(Acceptance, AcceptanceNewClient, osPriorityLow, 0, configMINIMAL_STACK_SIZE *2);

  /* Infinite loop */
  for(;;)
  {
		g =  accept(sock, NULL, NULL);
		if(g < 0){
			osDelay(100);
			continue;
		}
		ClientHandle = osThreadCreate(osThread(Acceptance), &g);
//		osThreadTerminate(ClientHandle);

		uart_buf_len = sprintf(uart_bufT, "new connection...! \r\n"
							              "Connected to Compensation Coils Driver! \r\n\n%s\r\n", (char*)help);
		HAL_UART_Transmit(&huart3, (uint8_t*)uart_bufT, uart_buf_len, 100);
		write(g, (char*)uart_bufT, strlen(uart_bufT));
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		osDelay(100);
  }
  /* USER CODE END 5 */
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
