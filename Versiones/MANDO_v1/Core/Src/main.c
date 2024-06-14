/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c-lcd.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_MAP 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox[3];

uint8_t mapaACT = 1;
uint8_t TC = 0; //Traction control
uint8_t LC = 0; //Launch control
uint16_t rpm = 0; //revoluciones

int i = 0;

volatile int buttonUP=0;
volatile int buttonDOWN=0;
volatile int buttonCTRL=0;

char t_pantalla[33];
char t_mapa[2];
char t_rpm[6];

uint8_t highByte;
uint8_t lowByte;

void SEND_MESSAGE(){ //Mandar una secuencia de datos u otra segun el mapa activo
    TxHeader.DLC = 2; //Data length (envio 2 bytes)
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0x182; //ID (identificador del enviador)

    switch (mapaACT){
        case 1:
        	TxData[0] = 0x01;
        	break;
        case 2:
        	TxData[0] = 0x02;
        	break;
        case 3:
        	TxData[0] = 0x04;
        	break;
        case 4:
        	TxData[0] = 0x08;
        	break;
        case 5:
        	TxData[0] = 0x10;
        	break;
        }

    if (TC == 0 && LC == 0){
		TxData[1] = 0x00;
	} else if (TC == 1 && LC == 0){ //TC enabled
		TxData[1] = 0x01;
	} else if (TC == 0 && LC == 1){ //LC enabled
		TxData[1] = 0x02;
	} else if (TC == 1 && LC == 1){ //TC y LC enabled
		TxData[1] = 0x03;
	}

    	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox[0]);
    }

int debouncer(volatile int* button_int, GPIO_TypeDef* GPIO_port, uint16_t GPIO_number){
	static uint8_t button_count=0;
	static int counter=0;

	if (*button_int==1){
		if (button_count==0) {
			counter=HAL_GetTick();
			button_count++;
		}
		if (HAL_GetTick()-counter>=20){
			counter=HAL_GetTick();
			if (HAL_GPIO_ReadPin(GPIO_port, GPIO_number)!=1){
				button_count=1;
			}
			else{
				button_count++;
			}
			if (button_count==4){ //Periodo antirebotes
				button_count=0;
				*button_int=0;
				return 1;
			}
		}
	}
	return 0;
}


void MAPChange(){
	if (debouncer(&buttonUP, GPIOA, GPIO_PIN_1)){
		if (mapaACT < NUM_MAP){
			mapaACT++;
		} else {
			mapaACT = 1;
		}
		SEND_MESSAGE();
	}
	if (debouncer(&buttonDOWN, GPIOA, GPIO_PIN_2)){
		if (mapaACT > 1 ){
			mapaACT--;
		} else {
			mapaACT = NUM_MAP;
		}
		SEND_MESSAGE();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){ //Cambio de mapa activo segun el botón que se pulse
	if (GPIO_Pin == GPIO_PIN_1){ //Al pulsar el boton rojo se sube de mapa
		buttonUP = 1;
		buttonDOWN = 0;
		buttonCTRL = 0;
	}

	if (GPIO_Pin == GPIO_PIN_2){ //Al pulsar el boton amarillo se baja de mapa
		buttonDOWN = 1;
		buttonUP = 0;
		buttonCTRL = 0;
	}
	if (GPIO_Pin == GPIO_PIN_3){ //Al pulsar el boton azul
		buttonCTRL = 1;
		buttonDOWN = 0;
		buttonUP = 0;
	}
}



void RECEIVE_MESSAGE(){
	switch (RxData[0]){
	    case 0x81:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0);
	    	break;
	    case 0x82:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	    	break;
	    case 0x84:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	    	break;
	    case 0x88:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0);
	    	break;
	    case 0x90:
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 1);
	    	break;
	}

	switch (RxData[1]){
	    case 0x80: //TC y LC disabled
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 0);
	    	TC = 0;
	    	LC = 0;
	    	break;
	    case 0x81: //TC enabled
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 0);
	    	TC = 1;
	    	LC = 0;
	    	break;
	    case 0x82: //LC enabled
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 0);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);
	    	TC = 0;
	    	LC = 1;
	    	break;
	    case 0x83: //TC y LC enabled
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 1);
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);
	    	TC = 1;
	    	LC = 1;
	    	break;
	}
}

uint16_t combineBytes(uint8_t highByte, uint8_t lowByte) {
    return ((uint16_t)highByte << 8) | lowByte; //Desplaza los 2 primeros bytes a su posición correcta y añade los 2 últimos bytes
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){ //Para recibir mensajes del bus //<-- REVISAR hcan o hcan1
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if (RxHeader.StdId == 0x181){
		RECEIVE_MESSAGE();
	}
	if (RxHeader.StdId == 0x701){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_8); //Heartbeat
		//Activar las interrupciones
			HAL_NVIC_EnableIRQ(EXTI1_IRQn);
			HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	}
	if (RxHeader.StdId == 0x381){
		highByte = RxData[0];
		lowByte = RxData[1];
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){ //Heartbeat
	if(htim->Instance==TIM2){
	    TxHeader.DLC = 1; //Data length
	    TxHeader.IDE = CAN_ID_STD;
	    TxHeader.RTR = CAN_RTR_DATA;
	    TxHeader.StdId = 0x702; //ID (identificador del enviador)

	    if(i == 0){
	    	TxData[0] = 0x80;
	    	i ++;
	    } else {
	    	TxData[0] = 0x01;
	    }

	    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox[1]);
	}
	if(htim->Instance==TIM3){
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)){
			LC = 1;
		} else {
			if (TC == 0){
				TC = 1;
			} else {
				TC = 0;
			}
		}
		HAL_TIM_Base_Stop_IT(&htim3);
		SEND_MESSAGE();
	}
	if (htim->Instance == TIM4){
		lcd_update(t_pantalla);
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
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);
  lcd_init();


    //Activar la notificacion de que hay mensajes para recibir
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);

    //Al principio están desactivadas las interrrupciones de los botones
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (buttonUP==1 || buttonDOWN==1){
		  MAPChange();
	  }

	  if (buttonCTRL==1){
		  if (debouncer(&buttonCTRL, GPIOA, GPIO_PIN_3)){
				buttonUP = 0;
				buttonDOWN = 0;
				buttonCTRL = 0;
		  	HAL_TIM_Base_Start_IT(&htim3);
		  }
	  }


	  // Recombinar los bytes en un valor de 16 bits
	  rpm = combineBytes(highByte, lowByte);

	  sprintf(t_mapa, "%i", mapaACT);
	  sprintf(t_rpm, "%u", rpm);
	  if (TC == 1){
		  strcpy(t_pantalla, "TC:ON ");
	  } else {
		  strcpy(t_pantalla, "TC:OFF");
	  }
	  if (LC == 1){
		  strcat(t_pantalla, " LC:ON ");
	  } else {
		  strcat(t_pantalla, " LC:OFF");
	  }
	  strcat (t_pantalla, "   MAPA:");
	  strcat(t_pantalla, t_mapa);
	  strcat(t_pantalla, " RPM:");
	  strcat(t_pantalla, t_rpm);
	  strcat(t_pantalla, "    ");


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

      canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
      canfilterconfig.FilterBank = 18; //which filter bank to use from the assigned ones
      canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0; //Para guardar el mensaje entrante
      canfilterconfig.FilterIdHigh = 0x101<<5;
      canfilterconfig.FilterIdLow = 0x0000;
      canfilterconfig.FilterMaskIdHigh = 0x101<<5; //Escribo en la posicion 5
      canfilterconfig.FilterMaskIdLow = 0x0000;
      canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
      canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
      canfilterconfig.SlaveStartFilterBank = 20; //how many filters to assign to the CAN1 (master can)

      HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  htim2.Init.Prescaler = 41999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  htim3.Init.Prescaler = 41999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2999;
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
  // Limpiar cualquier interrupción pendiente
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_IT_UPDATE);

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
  htim4.Init.Prescaler = 41999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
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
