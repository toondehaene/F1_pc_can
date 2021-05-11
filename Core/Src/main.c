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
#include "packets.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void my_CAN_Transmit(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader,
		uint8_t aData[], uint16_t dataSize, uint32_t *pTxMailbox);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MY_TIMEOUT 500
#define BUFSIZE 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
CAN_TxHeaderTypeDef TxHeaderDASHsession, TxHeaderDASHlap, TxHeaderDASHcartelemetry, TxHeaderDASHcarstatus;
uint32_t TxMailbox;
CAN_FilterTypeDef sFilterConfig;
UART_HandleTypeDef huart2;
uint16_t myID = 0x0; // | 0xFF;
// for now using only the 3 MSB to give unique ID to our can nodes. The rest are 1 == low CAN priority

/* USER CODE BEGIN PV */
motion_packet my_motion_packet;
session_packet my_session_packet;
lap_data_packet my_lap_data_packet;
event_packet my_event_packet;
participants_packet my_participants_packet;
car_setups_packet my_car_setups_packet;
car_telemetry_packet my_car_telemetry_packet;
car_status_packet my_car_status_packet;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	uint8_t Rbuffer[BUFSIZE];

	uint8_t mcuready[] = "MCU_READY\n";
	uint32_t mask;

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
	MX_CAN_Init();
	/* USER CODE BEGIN 2 */
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //last argument is mask of activated ITs just OR| them together

	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterIdHigh = 0x0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0x0;	//11 highest bits of the mask are 1
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation = DISABLE;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		// ASK FOR UPDATE DATA WITH MCU_READY
		if (HAL_UART_Transmit(&huart2, (uint8_t*) mcuready,
				(uint16_t) sizeof(mcuready),
				MY_TIMEOUT) != HAL_OK) {
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //TRANSMIT FAILED, TOGGLE PIN14 LED
		} else {
			int Biter = 0;
			HAL_UART_Receive(&huart2, Rbuffer, sizeof(Rbuffer), MY_TIMEOUT); //RECEIVE NEW DATA
			memcpy(&mask, &Rbuffer[Biter], sizeof(mask)); // first 4 bytes of the Rbuffer is a mask which indicates what type of data is sent (update available)
			Biter += sizeof(mask);
			if ((mask & 0b00000001) >> 0) { // CASE: motion_packet
				memcpy(&my_motion_packet, &Rbuffer[Biter],
						sizeof(motion_packet));
				Biter += sizeof(motion_packet);
			}
			if ((mask & 0b00000010) >> 1) { // CASE: session_packet
				memcpy(&my_session_packet, &Rbuffer[Biter],
						sizeof(session_packet));
				Biter += sizeof(session_packet);
				my_CAN_Transmit(&hcan, &TxHeaderDASHsession,
						(uint8_t *) &my_session_packet,
						sizeof(my_session_packet), &TxMailbox);
			}
			if ((mask & 0b00000100) >> 2) { // CASE: lap_data_packet
				memcpy(&my_lap_data_packet, &Rbuffer[Biter],
						sizeof(lap_data_packet));
				Biter += sizeof(lap_data_packet);
				my_CAN_Transmit(&hcan, &TxHeaderDASHlap,
						(uint8_t *) &my_lap_data_packet,
						sizeof(my_lap_data_packet), &TxMailbox);
			}
			if ((mask & 0b00001000) >> 3) { // CASE: event_packet
				memcpy(&my_session_packet, &Rbuffer[Biter],
						sizeof(event_packet));
				Biter += sizeof(event_packet);
			}
			if ((mask & 0b00010000) >> 4) { // CASE: participants_packet
				memcpy(&my_participants_packet, &Rbuffer[Biter],
						sizeof(participants_packet));
				Biter += sizeof(participants_packet);
			}
			if ((mask & 0b00100000) >> 5) { // CASE: car_setups_packet
				memcpy(&my_car_setups_packet, &Rbuffer[Biter],
						sizeof(car_setups_packet));
				Biter += sizeof(car_setups_packet);
			}
			if ((mask & 0b01000000) >> 6) { // CASE: car_telemetry_packet
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15); //DEBUG
				memcpy(&my_car_telemetry_packet, &Rbuffer[Biter],
						sizeof(car_telemetry_packet));
				Biter += sizeof(car_telemetry_packet);
				my_CAN_Transmit(&hcan, &TxHeaderDASHcartelemetry,
						(uint8_t *) &my_car_telemetry_packet,
						sizeof(my_car_telemetry_packet), &TxMailbox);
			}
			if ((mask & 0b10000000) >> 7) { // CASE: car_status_packet
				memcpy(&my_car_status_packet, &Rbuffer[Biter],
						sizeof(car_status_packet));
				Biter += sizeof(car_status_packet);
				my_CAN_Transmit(&hcan, &TxHeaderDASHcarstatus,
						(uint8_t *) &my_car_status_packet,
						sizeof(my_car_status_packet), &TxMailbox);
			}
		} // READY FOR NEW WORK GO TO TOP OF WHILE

//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
//		HAL_Delay(200);
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 21;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */
	TxHeaderDASHsession.DLC = 8;		//data length in bytes, max 8
	TxHeaderDASHsession.IDE = CAN_ID_EXT;	// specifies if standard or extended IDs are used
	TxHeaderDASHsession.StdId = sessionID<<5|DASHBOARD; // first part of MY OWN ID so this specifies the SOURCE of the message
	TxHeaderDASHsession.ExtId = 0x0; // second part, ask Wouter Burns if 11 bit can IDs are enough
	TxHeaderDASHsession.RTR = CAN_RTR_DATA; // type of data, RTR means normal bytes, can also be commands etc

	TxHeaderDASHlap.DLC = 8;
	TxHeaderDASHlap.IDE = CAN_ID_EXT;
	TxHeaderDASHlap.StdId = lapID<<5|DASHBOARD;
	TxHeaderDASHlap.ExtId = 0;
	TxHeaderDASHlap.RTR = CAN_RTR_DATA;

	TxHeaderDASHcartelemetry.DLC = 8;
	TxHeaderDASHcartelemetry.IDE = CAN_ID_EXT;
	TxHeaderDASHcartelemetry.StdId = car_telemetryID<<5|DASHBOARD;
	TxHeaderDASHcartelemetry.ExtId = 0;
	TxHeaderDASHcartelemetry.RTR = CAN_RTR_DATA;

	TxHeaderDASHcarstatus.DLC = 8;
	TxHeaderDASHcarstatus.IDE = CAN_ID_EXT;
	TxHeaderDASHcarstatus.StdId = car_statusID<<5|DASHBOARD;
	TxHeaderDASHcarstatus.ExtId = 0;
	TxHeaderDASHcarstatus.RTR = CAN_RTR_DATA;
	HAL_CAN_Start(&hcan);
	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB13 PB14 PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/**
 * My fuction for transmitting structures of variable size over CAN
 */
void my_CAN_Transmit(CAN_HandleTypeDef *canheader, CAN_TxHeaderTypeDef *pHeader,
		uint8_t aData[], uint16_t dataSize, uint32_t *pTxMailbox) {
	if (dataSize > pHeader->DLC) {
		Error_Handler();
	}
	HAL_CAN_AddTxMessage(canheader, pHeader, aData, pTxMailbox);
}

/* USER CODE END 4 */

/**
 * @brief  This functioappn is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
