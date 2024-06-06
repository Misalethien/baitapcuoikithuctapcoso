/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "i2c-lcd.h"
//#include "spi.h"
#include "stdio.h"
#include "string.h"
#include "mfrc522.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


uint8_t CardID[5];
char szBuff[100];
short isOpen = 0;
int RFID_Success = 0;
short updateDoor = 0;
short updateLCD = 0;
short StateDoor = 0;
uint8_t CardIDSET[5] = {0x23,0xD9,0xAE,0xEC,0xB8};
char szBuff[100];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
int DoorClose();
int DoorOpen();
void BuzzerError(void);
void BuzzerSuccess(void);
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  //HAL_GPIO_WritePin(RC522_Rst_GPIO_Port, RC522_Rst_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
  TM_MFRC522_Init();	
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	lcd_init();
	lcd_goto_XY(1,2);
	lcd_send_string("-SMART BOX-");
	HAL_Delay(2000);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		if (TM_MFRC522_Check(CardID) == MI_OK) {
			int checkCard = 0;
			for (int k = 0; k < 5; k++)
			{
				if (CardID[k] != CardIDSET[k])
				{
					break;
				}
				else
				{
					checkCard++;
				}
			}
			if (checkCard == 5)
			{
				RFID_Success = 1;
			}
			else
			{
				RFID_Success = 2;
			}
			updateLCD = 1;
		}
		if (updateLCD || (RFID_Success != 0))
		{
			updateLCD = 0;
			sprintf(szBuff, "ID: 0x%02X%02X%02X%02X%02X", CardID[0], CardID[1], CardID[2], CardID[3], CardID[4]);
			lcd_goto_XY(1,0);
			lcd_send_string(szBuff);
			if (RFID_Success == 1)
			{
				if (StateDoor == 0)
				{
					lcd_goto_XY(2,0);
					lcd_send_string("Cua dang mo    ");
					BuzzerSuccess();
					HAL_Delay(1000);
					StateDoor = 1;
					updateDoor = 1;
					lcd_goto_XY(2,0);
					lcd_send_string("              ");
				}
				RFID_Success = 0;
			}
			else if (RFID_Success == 2)
			{
				lcd_goto_XY(2,0);
				lcd_send_string("The khong dung! ");
				BuzzerError();
				HAL_Delay(3000);
				RFID_Success = 0;
				lcd_goto_XY(2,0);
				lcd_send_string("                 ");
			}
		}
		if (updateDoor)
		{
			updateDoor = 0;
			lcd_goto_XY(1,0);
			lcd_send_string("  -SMART BOX-   ");
			lcd_goto_XY(2,0);
			lcd_send_string("                ");
			if (StateDoor == 0)
			{
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, 7);
			}
			else
			{
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, 18);
			}
		}
		if (StateDoor == 1)
		{
			if (isOpen == 0)
			{
				if (DoorOpen() == 1)
				{
					isOpen = 1;
					BuzzerSuccess();
					
				}
				else
				{
					lcd_goto_XY(2,0);	
					lcd_send_string("Xin moi mo cua..");
					HAL_Delay(500);
				}
			}
			else
			{
				if (DoorClose() == 1)
				{
					HAL_Delay(2000);
					lcd_goto_XY(2,0);	
					lcd_send_string("Cua dang khoa   ");
					BuzzerSuccess();
					HAL_Delay(1000);
					StateDoor = 0;
					updateDoor = 1;
					isOpen = 0;
				}
				else
				{
						lcd_goto_XY(2,0);	
						lcd_send_string("Xin moi dong cua");
						HAL_Delay(500);		
				}
			}
		}
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	//	HAL_Delay(1000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//------------------------------------------------------
void BuzzerError()
{
	for (int times = 0; times < 3; times++)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_Delay(200);
	}
}
void BuzzerSuccess()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_Delay(500);
}
int DoorClose()
{
	int Sensor = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
	if (Sensor == 0)
		return 1;
	else 
		return 0;
}
int DoorOpen()
{
	int Sensor = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
	if (Sensor == 1)
		return 1;
	else 
		return 0;
}
void TM_MFRC522_Init(void) {
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);						// Activate the RFID reader
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);					// not reset
	//TM_SPI_Init();

	TM_MFRC522_Reset();

	TM_MFRC522_WriteRegister(MFRC522_REG_T_MODE, 0x8D);
	TM_MFRC522_WriteRegister(MFRC522_REG_T_PRESCALER, 0x3E);
	TM_MFRC522_WriteRegister(MFRC522_REG_T_RELOAD_L, 30);           
	TM_MFRC522_WriteRegister(MFRC522_REG_T_RELOAD_H, 0);

	/* 48dB gain */
	TM_MFRC522_WriteRegister(MFRC522_REG_RF_CFG, 0x70);
	
	TM_MFRC522_WriteRegister(MFRC522_REG_TX_AUTO, 0x40);
	TM_MFRC522_WriteRegister(MFRC522_REG_MODE, 0x3D);

	TM_MFRC522_AntennaOn();		//Open the antenna
}

TM_MFRC522_Status_t TM_MFRC522_Check(uint8_t* id) {
	TM_MFRC522_Status_t status;
	//Find cards, return card type
	status = TM_MFRC522_Request(PICC_REQIDL, id);	
	if (status == MI_OK) {
		//Card detected
		//Anti-collision, return card serial number 4 bytes
		status = TM_MFRC522_Anticoll(id);	
	}
	TM_MFRC522_Halt();			//Command card into hibernation 

	return status;
}

TM_MFRC522_Status_t TM_MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID) {
	uint8_t i;
	for (i = 0; i < 5; i++) {
		if (CardID[i] != CompareID[i]) {
			return MI_ERR;
		}
	}
	return MI_OK;
}

//==================================================
//ham nay can thay doi vi cau truc F1 khac F4
void TM_MFRC522_WriteRegister(uint8_t addr, uint8_t val) {
	unsigned char addr_bits = (((addr<<1) & 0x7E));
	/* CS LOW */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);

	//Dinh dang dia chi:0XXXXXX0
	HAL_SPI_Transmit(&hspi2, &addr_bits, 1, 500);
	HAL_SPI_Transmit(&hspi2, &val, 1, 500);
	
	/* CS HIGH */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
}

uint8_t TM_MFRC522_ReadRegister(uint8_t addr) {
	
	unsigned char rx_bits;
  unsigned char addr_bits = (((addr<<1) & 0x7E) | 0x80);
	/* CS LOW */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
	//Dinh dang dia chi:1XXXXXX0
	HAL_SPI_Transmit(&hspi2, &addr_bits, 1, 500);
	HAL_SPI_Receive(&hspi2, &rx_bits, 1, 500);
	
	/* CS HIGH */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	
	return rx_bits; // return the rx bits, casting to an 8 bit int and chopping off the upper 24 bits
}

void TM_MFRC522_SetBitMask(uint8_t reg, uint8_t mask) {
	TM_MFRC522_WriteRegister(reg, TM_MFRC522_ReadRegister(reg) | mask);
}

void TM_MFRC522_ClearBitMask(uint8_t reg, uint8_t mask){
	TM_MFRC522_WriteRegister(reg, TM_MFRC522_ReadRegister(reg) & (~mask));
} 

void TM_MFRC522_AntennaOn(void) {
	uint8_t temp;

	temp = TM_MFRC522_ReadRegister(MFRC522_REG_TX_CONTROL);
	if (!(temp & 0x03)) {
		TM_MFRC522_SetBitMask(MFRC522_REG_TX_CONTROL, 0x03);
	}
}

void TM_MFRC522_AntennaOff(void) {
	TM_MFRC522_ClearBitMask(MFRC522_REG_TX_CONTROL, 0x03);
}

void TM_MFRC522_Reset(void) {
	TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_RESETPHASE);
}

TM_MFRC522_Status_t TM_MFRC522_Request(uint8_t reqMode, uint8_t* TagType) {
	TM_MFRC522_Status_t status;  
	uint16_t backBits;			//The received data bits

	TM_MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x07);		//TxLastBists = BitFramingReg[2..0]	???

	TagType[0] = reqMode;
	status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

	if ((status != MI_OK) || (backBits != 0x10)) {    
		status = MI_ERR;
	}

	return status;
}

TM_MFRC522_Status_t TM_MFRC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen) {
	TM_MFRC522_Status_t status = MI_ERR;
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits;
	uint8_t n;
	uint16_t i;

	switch (command) {
		case PCD_AUTHENT: {
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case PCD_TRANSCEIVE: {
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
			break;
	}

	TM_MFRC522_WriteRegister(MFRC522_REG_COMM_IE_N, irqEn | 0x80);
	TM_MFRC522_ClearBitMask(MFRC522_REG_COMM_IRQ, 0x80);
	TM_MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);

	TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_IDLE);

	//Writing data to the FIFO
	for (i = 0; i < sendLen; i++) {   
		TM_MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, sendData[i]);    
	}

	//Execute the command
	TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, command);
	if (command == PCD_TRANSCEIVE) {    
		TM_MFRC522_SetBitMask(MFRC522_REG_BIT_FRAMING, 0x80);		//StartSend=1,transmission of data starts  
	}   

	//Waiting to receive data to complete
	i = 2000;	//i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
	do {
		//CommIrqReg[7..0]
		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
		n = TM_MFRC522_ReadRegister(MFRC522_REG_COMM_IRQ);
		i--;
	} while ((i!=0) && !(n&0x01) && !(n&waitIRq));

	TM_MFRC522_ClearBitMask(MFRC522_REG_BIT_FRAMING, 0x80);			//StartSend=0

	if (i != 0)  {
		if (!(TM_MFRC522_ReadRegister(MFRC522_REG_ERROR) & 0x1B)) {
			status = MI_OK;
			if (n & irqEn & 0x01) {   
				status = MI_NOTAGERR;			
			}

			if (command == PCD_TRANSCEIVE) {
				n = TM_MFRC522_ReadRegister(MFRC522_REG_FIFO_LEVEL);
				lastBits = TM_MFRC522_ReadRegister(MFRC522_REG_CONTROL) & 0x07;
				if (lastBits) {   
					*backLen = (n - 1) * 8 + lastBits;   
				} else {   
					*backLen = n * 8;   
				}

				if (n == 0) {   
					n = 1;    
				}
				if (n > MFRC522_MAX_LEN) {   
					n = MFRC522_MAX_LEN;   
				}

				//Reading the received data in FIFO
				for (i = 0; i < n; i++) {   
					backData[i] = TM_MFRC522_ReadRegister(MFRC522_REG_FIFO_DATA);    
				}
			}
		} else {   
			status = MI_ERR;  
		}
	}

	return status;
}

TM_MFRC522_Status_t TM_MFRC522_Anticoll(uint8_t* serNum) {
	TM_MFRC522_Status_t status;
	uint8_t i;
	uint8_t serNumCheck = 0;
	uint16_t unLen;

	TM_MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x00);		//TxLastBists = BitFramingReg[2..0]

	serNum[0] = PICC_ANTICOLL;
	serNum[1] = 0x20;
	status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

	if (status == MI_OK) {
		//Check card serial number
		for (i = 0; i < 4; i++) {   
			serNumCheck ^= serNum[i];
		}
		if (serNumCheck != serNum[i]) {   
			status = MI_ERR;    
		}
	}
	return status;
} 

void TM_MFRC522_CalculateCRC(uint8_t*  pIndata, uint8_t len, uint8_t* pOutData) {
	uint8_t i, n;

	TM_MFRC522_ClearBitMask(MFRC522_REG_DIV_IRQ, 0x04);			//CRCIrq = 0
	TM_MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);			//Clear the FIFO pointer
	//Write_MFRC522(CommandReg, PCD_IDLE);

	//Writing data to the FIFO	
	for (i = 0; i < len; i++) {   
		TM_MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, *(pIndata+i));   
	}
	TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_CALCCRC);

	//Wait CRC calculation is complete
	i = 0xFF;
	do {
		n = TM_MFRC522_ReadRegister(MFRC522_REG_DIV_IRQ);
		i--;
	} while ((i!=0) && !(n&0x04));			//CRCIrq = 1

	//Read CRC calculation result
	pOutData[0] = TM_MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_L);
	pOutData[1] = TM_MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_M);
}

uint8_t TM_MFRC522_SelectTag(uint8_t* serNum) {
	uint8_t i;
	TM_MFRC522_Status_t status;
	uint8_t size;
	uint16_t recvBits;
	uint8_t buffer[9]; 

	buffer[0] = PICC_SElECTTAG;
	buffer[1] = 0x70;
	for (i = 0; i < 5; i++) {
		buffer[i+2] = *(serNum+i);
	}
	TM_MFRC522_CalculateCRC(buffer, 7, &buffer[7]);		//??
	status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

	if ((status == MI_OK) && (recvBits == 0x18)) {   
		size = buffer[0]; 
	} else {   
		size = 0;    
	}

	return size;
}

TM_MFRC522_Status_t TM_MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum) {
	TM_MFRC522_Status_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[12]; 

	//Verify the command block address + sector + password + card serial number
	buff[0] = authMode;
	buff[1] = BlockAddr;
	for (i = 0; i < 6; i++) {    
		buff[i+2] = *(Sectorkey+i);   
	}
	for (i=0; i<4; i++) {    
		buff[i+8] = *(serNum+i);   
	}
	status = TM_MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

	if ((status != MI_OK) || (!(TM_MFRC522_ReadRegister(MFRC522_REG_STATUS2) & 0x08))) {   
		status = MI_ERR;   
	}

	return status;
}

TM_MFRC522_Status_t TM_MFRC522_Read(uint8_t blockAddr, uint8_t* recvData) {
	TM_MFRC522_Status_t status;
	uint16_t unLen;

	recvData[0] = PICC_READ;
	recvData[1] = blockAddr;
	TM_MFRC522_CalculateCRC(recvData,2, &recvData[2]);
	status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

	if ((status != MI_OK) || (unLen != 0x90)) {
		status = MI_ERR;
	}

	return status;
}

TM_MFRC522_Status_t TM_MFRC522_Write(uint8_t blockAddr, uint8_t* writeData) {
	TM_MFRC522_Status_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[18]; 

	buff[0] = PICC_WRITE;
	buff[1] = blockAddr;
	TM_MFRC522_CalculateCRC(buff, 2, &buff[2]);
	status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

	if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {   
		status = MI_ERR;   
	}

	if (status == MI_OK) {
		//Data to the FIFO write 16Byte
		for (i = 0; i < 16; i++) {    
			buff[i] = *(writeData+i);   
		}
		TM_MFRC522_CalculateCRC(buff, 16, &buff[16]);
		status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {   
			status = MI_ERR;   
		}
	}

	return status;
}

void TM_MFRC522_Halt(void) {
	uint16_t unLen;
	uint8_t buff[4]; 

	buff[0] = PICC_HALT;
	buff[1] = 0;
	TM_MFRC522_CalculateCRC(buff, 2, &buff[2]);

	TM_MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}




/*
 * Ten ham:Write_MFRC5200
 * Chuc nang: Viet 1 byte du lieu vao thanh ghi MFRC522
 * Input:addr-> DIa chi ghi, val-> Gia tri de ghi
 * Tra ve: Khong
 */

/*
 * Ten ham:Write_MFRC5200
 * Chuc nang: Viet 1 byte du lieu vao thanh ghi MFRC522
 * Input:addr-> DIa chi ghi, val-> Gia tri de ghi
 * Tra ve: Khong
 */
//void Write_MFRC522(uchar addr, uchar val)
//{
//	uchar addr_bits = (((addr<<1) & 0x7E));
//	/* CS LOW */
//	HAL_GPIO_WritePin(MFRC522_CS_GPIO, MFRC522_CS_PIN,GPIO_PIN_RESET);

//	//Dinh dang dia chi:0XXXXXX0
//	HAL_SPI_Transmit(&hspi2, &addr_bits, 1, 500);
//	HAL_SPI_Transmit(&hspi2, &val, 1, 500);
//	
//	/* CS HIGH */
//	HAL_GPIO_WritePin(MFRC522_CS_GPIO, MFRC522_CS_PIN,GPIO_PIN_SET);
//}


///*
// * Ten ham:Read_MFRC522
// * Chuc nang:Doc 1 byte du lieu tu 1 thanh ghi MFRC522
// * Input:addr-> dia chi doc
// * Tra ve: Gia tri trong thanh ghi doc ve
// */
//uchar Read_MFRC522(uchar addr)
//{
//	uchar rx_bits;
//  uchar addr_bits = (((addr<<1) & 0x7E) | 0x80);
//	/* CS LOW */
//	HAL_GPIO_WritePin(MFRC522_CS_GPIO, MFRC522_CS_PIN,GPIO_PIN_RESET);
//	//Dinh dang dia chi:1XXXXXX0
//	HAL_SPI_Transmit(&hspi2, &addr_bits, 1, 500);
//	HAL_SPI_Receive(&hspi2, &rx_bits, 1, 500);
//	
//	/* CS HIGH */
//	HAL_GPIO_WritePin(MFRC522_CS_GPIO, MFRC522_CS_PIN, GPIO_PIN_SET);
//	
//	return (uchar) rx_bits; // return the rx bits, casting to an 8 bit int and chopping off the upper 24 bits
//	
//}


///*
// * Ten ham:SetBitMask
// * Chuc nang:Set bit trong mot thanh ghi MFRC522
// * Input:reg--Thanh ghi cai dat; mask--gia tri set
// * Tra ve: Khong
// */
//void SetBitMask(uchar reg, uchar mask)  
//{
//    uchar tmp;
//    tmp = Read_MFRC522(reg);
//    Write_MFRC522(reg, tmp | mask);  // set bit mask
//}


///*
// * Ten ham:ClearBitMask
// * Chuc nang:Reset bit trong thanh ghi MFRC522
// * Input:reg--Dia chi thanh ghi; mask--Gia tri bit can clear
// * Tra ve: Khong
// */
//void ClearBitMask(uchar reg, uchar mask)  
//{
//    uchar tmp;
//    tmp = Read_MFRC522(reg);
//    Write_MFRC522(reg, tmp & (~mask));  // clear bit mask
//} 


///*
// * Ten Ham:AntennaOn
// * Chuc Nang:Mo anten, nen co it nhat 1 ms
// * Input: khong
// * Tra ve: khong
// */
//void AntennaOn(void)
//{
//	uchar temp;

//	temp = Read_MFRC522(TxControlReg);
////	if (!(temp & 0x03))
////	{
////		SetBitMask(TxControlReg, 0x03);
////	}
//	SetBitMask(TxControlReg, 0x03);
//}


///*
// * Ten ham:AntennaOff
// * chuc nang:Dong Anten, nen co it nhat 1 ms
// * Input:khong
// * Tra ve: khong
// */
//void AntennaOff(void)
//{
//	ClearBitMask(TxControlReg, 0x03);
//}


///*
// * Ten ham:ResetMFRC522
// * Chuc nang:Khoi dong lai RC522
// * Input: Khong
// * Return: Khong
// */
//void MFRC522_Reset(void)
//{
//    Write_MFRC522(CommandReg, PCD_RESETPHASE);
//}

///*
// * Ten ham:MFRC522_SPI_Init
// * Chuc nang:Khoi tao SPI
// * Input: Khong
// * Tra va: Khong
// */
////void MFRC522_SPI_Init(void)
////{
////	GPIO_InitTypeDef GPIO_InitStruct;
////	SPI_InitTypeDef SPI_InitStruct;
////	
////	RCC_APB2PeriphClockCmd(MFRC522_SPI_GPIO_RCC | RCC_APB2Periph_AFIO,ENABLE);		// Enable clock GPIO
////	
////	if(MFRC522_SPI==SPI1)
////	{
////		RCC_APB2PeriphClockCmd(MFRC522_SPI_RCC,ENABLE);																// Enable clock SPI1
////	}
////	else
////	{
////		RCC_APB1PeriphClockCmd(MFRC522_SPI_RCC,ENABLE);																// Enable clock SPI2 or SPI3
////	}
////	
////	GPIO_InitStruct.GPIO_Pin = MFRC522_SCK_PIN | MFRC522_MISO_PIN | MFRC522_MOSI_PIN;	// SCK, MISO, MOSI
////	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
////	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
////	GPIO_Init(MFRC522_SPI_GPIO, &GPIO_InitStruct);																	// init GPIO SPI

////	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
////	SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
////	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
////	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
////	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
////	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
////	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
////	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
////	SPI_InitStruct.SPI_CRCPolynomial = 7;
////	SPI_Init(MFRC522_SPI, &SPI_InitStruct);

////	SPI_Cmd(MFRC522_SPI, ENABLE);
////}

///*
// * Ten ham:InitMFRC522
// * Chuc nang:Khoi tao RC522
// * Input: Khong
// * Tra va: Khong
// */
//void MFRC522_Init(void)
//{
//	/* Khoi tao SPI */
////	GPIO_InitTypeDef GPIO_InitStructure;
////	/* GPIOD Periph clock enable */
////  RCC_APB2PeriphClockCmd(MFRC522_CS_RCC | MFRC522_RST_RCC, ENABLE);

////  /* Configure CS is output pushpull mode */
////  GPIO_InitStructure.GPIO_Pin = MFRC522_CS_PIN;				// Set digital pin 10 as OUTPUT to connect it to the RFID /ENABLE pin 
////  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
////  GPIO_Init(MFRC522_CS_GPIO, &GPIO_InitStructure);
////	
////  GPIO_InitStructure.GPIO_Pin = MFRC522_RST_PIN;			// Set digital pin 10 , Not Reset and Power-down
////	GPIO_Init(MFRC522_RST_GPIO, &GPIO_InitStructure);
////	
//	HAL_GPIO_WritePin(MFRC522_CS_GPIO,MFRC522_CS_PIN,GPIO_PIN_SET);						// Activate the RFID reader
//	HAL_GPIO_WritePin(MFRC522_RST_GPIO,MFRC522_RST_PIN,GPIO_PIN_SET);					// not reset

//		// spi config
//	//MFRC522_SPI_Init();
//	
//	MFRC522_Reset();
//	 	
//	//Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
//	Write_MFRC522(TModeReg, 0x8D);		//Tauto=1; f(Timer) = 6.78MHz/TPreScaler
//	Write_MFRC522(TPrescalerReg, 0x3E);	//TModeReg[3..0] + TPrescalerReg
//	Write_MFRC522(TReloadRegL, 30);           
//	Write_MFRC522(TReloadRegH, 0);
//	
//	Write_MFRC522(TxAutoReg, 0x40);		//100%ASK
//	Write_MFRC522(ModeReg, 0x3D);		//CRC Gia tri ban dau 0x6363	???

//	//ClearBitMask(Status2Reg, 0x08);		//MFCrypto1On=0
//	//Write_MFRC522(RxSelReg, 0x86);		//RxWait = RxSelReg[5..0]
//	//Write_MFRC522(RFCfgReg, 0x7F);   		//RxGain = 48dB

//	AntennaOn();		//Mo Anten
//}

///*
// * Ten ham:MFRC522_ToCard
// * Chuc nang:truyen thong giua RC522 va the ISO14443
// * Input:command--lenh gui den MF522,
// *			 sendData--Du lieu gui den the bang MFRC522, 
// *			 sendLen--Chieu dai du lieu gui 
// *			 backData--Du lieu nhan duoc tro lai
// *			 backLen--Tra ve do dai bit cua du lieu
// * Tra ve: MI_OK neu thanh cong
// */
//uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData, uint *backLen)
//{
//    uchar status = MI_ERR;
//    uchar irqEn = 0x00;
//    uchar waitIRq = 0x00;
//    uchar lastBits;
//    uchar n;
//    uint i;

//    switch (command)
//    {
//        case PCD_AUTHENT:		//Xac nhan the gan
//		{
//			irqEn = 0x12;
//			waitIRq = 0x10;
//			break;
//		}
//		case PCD_TRANSCEIVE:	// Gui du lieu FIFO
//		{
//			irqEn = 0x77;
//			waitIRq = 0x30;
//			break;
//		}
//		default:
//			break;
//    }
//   
//    Write_MFRC522(CommIEnReg, irqEn|0x80);	//Yeu cau ngat
//    ClearBitMask(CommIrqReg, 0x80);			//Clear tat ca cac bit yeu cau ngat
//    SetBitMask(FIFOLevelReg, 0x80);			//FlushBuffer=1, Khoi tao FIFO
//    
//	Write_MFRC522(CommandReg, PCD_IDLE);	//NO action; Huy bo lenh hien hanh	???

//	// Ghi du lieu vao FIFO
//    for (i=0; i<sendLen; i++)
//    {   
//		Write_MFRC522(FIFODataReg, sendData[i]);    
//	}

//	//chay
//	Write_MFRC522(CommandReg, command);
//    if (command == PCD_TRANSCEIVE)
//    {    
//		SetBitMask(BitFramingReg, 0x80);		//StartSend=1,transmission of data starts  
//	}   
//    
//	//Cho doi de nhan duoc du lieu day du
//	i = 2000;	//i tuy thuoc tan so thach anh, thoi gian toi da cho the M1 la 25ms
//    do 
//    {
//		//CommIrqReg[7..0]
//		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
//        n = Read_MFRC522(CommIrqReg);
//        i--;
//    }
//    while ((i!=0) && !(n&0x01) && !(n&waitIRq));

//    ClearBitMask(BitFramingReg, 0x80);			//StartSend=0
//	
//    if (i != 0)
//    {    
//        if(!(Read_MFRC522(ErrorReg) & 0x1B))	//BufferOvfl Collerr CRCErr ProtecolErr
//        {
//            status = MI_OK;
//            if (n & irqEn & 0x01)
//            {   
//				status = MI_NOTAGERR;			//??   
//			}

//            if (command == PCD_TRANSCEIVE)
//            {
//               	n = Read_MFRC522(FIFOLevelReg);
//              	lastBits = Read_MFRC522(ControlReg) & 0x07;
//                if (lastBits)
//                {   
//					*backLen = (n-1)*8 + lastBits;   
//				}
//                else
//                {   
//					*backLen = n*8;   
//				}

//                if (n == 0)
//                {   
//					n = 1;    
//				}
//                if (n > MAX_LEN)
//                {   
//					n = MAX_LEN;   
//				}
//				
//				//Doc FIFO trong cac du lieu nhan duoc
//                for (i=0; i<n; i++)
//                {   
//					backData[i] = Read_MFRC522(FIFODataReg);    
//				}
//            }
//        }
//        else
//        {   
//			status = MI_ERR;  
//		}
//        
//    }
//	
//    //SetBitMask(ControlReg,0x80);           //timer stops
//    //Write_MFRC522(CommandReg, PCD_IDLE); 

//    return status;
//}

///*
// * Ten ham:MFRC522_Request
// * Chuc nang:Phat hien the, doc loai the
// * Input:reqMode--Phat hien co the,
// *			 TagType--Loai the tra ve
// *			 	0x4400 = Mifare_UltraLight
// *				0x0400 = Mifare_One(S50)
// *				0x0200 = Mifare_One(S70)
// *				0x0800 = Mifare_Pro(X)
// *				0x4403 = Mifare_DESFire
// * Return: MI_OK neu thanh cong
// */
//uchar MFRC522_Request(uchar reqMode, uchar *TagType)
//{
//	uchar status;  
//	uint backBits;			//cac bit du lieu nhan duoc

//	Write_MFRC522(BitFramingReg, 0x07);		//TxLastBists = BitFramingReg[2..0]	???
//	
//	TagType[0] = reqMode;
//	status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

//	if ((status != MI_OK) || (backBits != 0x10))
//	{    
//		status = MI_ERR;
//	}
//   
//	return status;
//}


///*
// * Ten ham:MFRC522_Anticoll
// * Chuc nang:Phat hien chong va cham, chon the va doc so serial the
// * Input:serNum--Tra ve serial the 4 byte, byte 5 la ma checksum
// * Tra ve: MI_OK neu thanh cong
// */
//uchar MFRC522_Anticoll(uchar *serNum)
//{
//    uchar status;
//    uchar i;
//	uchar serNumCheck=0;
//    uint unLen;
//    

//    //ClearBitMask(Status2Reg, 0x08);		//TempSensclear
//    //ClearBitMask(CollReg,0x80);			//ValuesAfterColl
//	Write_MFRC522(BitFramingReg, 0x00);		//TxLastBists = BitFramingReg[2..0]
// 
//    serNum[0] = PICC_ANTICOLL;
//    serNum[1] = 0x20;
//    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

//    if (status == MI_OK)
//	{
//		//Kiem tra so serial the
//		for (i=0; i<4; i++)
//		{   
//		 	serNumCheck ^= serNum[i];
//		}
//		if (serNumCheck != serNum[i])
//		{   
//			status = MI_ERR;    
//		}
//    }

//    //SetBitMask(CollReg, 0x80);		//ValuesAfterColl=1

//    return status;
//} 


///*
// * Ten Ham:CalulateCRC
// * Chuc nang:MFRC522 tinh toan RC522
// * Input:pIndata--Du lieu CRC vao can tinh toan,len--Chieu dai du lieu,pOutData--Ket qua tinh toan CRC
// * Tra ve: Khong
// */
//void CalulateCRC(uchar *pIndata, uchar len, uchar *pOutData)
//{
//    uchar i, n;

//    ClearBitMask(DivIrqReg, 0x04);			//CRCIrq = 0
//    SetBitMask(FIFOLevelReg, 0x80);			//Con tro FIFO
//    //Write_MFRC522(CommandReg, PCD_IDLE);

//	//Ghi du lieu vao FIFO
//    for (i=0; i<len; i++)
//    {   
//		Write_MFRC522(FIFODataReg, *(pIndata+i));   
//	}
//    Write_MFRC522(CommandReg, PCD_CALCCRC);

//	// Cho cho viec tinh toan CRC hoan tat
//    i = 0xFF;
//    do 
//    {
//        n = Read_MFRC522(DivIrqReg);
//        i--;
//    }
//    while ((i!=0) && !(n&0x04));			//CRCIrq = 1

//	//Doc ket qua tinh toan CRC
//    pOutData[0] = Read_MFRC522(CRCResultRegL);
//    pOutData[1] = Read_MFRC522(CRCResultRegM);
//}


///*
// * Ten ham:MFRC522_SelectTag
// * Chuc nang:Lua chon the, doc dung luong bo nho the
// * Input:serNum--So serial the
// * Tra ve:Dung luong the tra ve thanh cong
// */
//uchar MFRC522_SelectTag(uchar *serNum)
//{
//	uchar i;
//	uchar status;
//	uchar size;
//	uint recvBits;
//	uchar buffer[9]; 

//	//ClearBitMask(Status2Reg, 0x08);			//MFCrypto1On=0

//    buffer[0] = PICC_SElECTTAG;
//    buffer[1] = 0x70;
//    for (i=0; i<5; i++)
//    {
//    	buffer[i+2] = *(serNum+i);
//    }
//	CalulateCRC(buffer, 7, &buffer[7]);		//??
//    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
//    
//    if ((status == MI_OK) && (recvBits == 0x18))
//    {   
//		size = buffer[0]; 
//	}
//    else
//    {   
//		size = 0;    
//	}

//    return size;
//}


///*
// * Ten Ham:MFRC522_Auth
// * Chuc nang:Xac nhan mat khau the
// * Input:authMode--Che do xac thuc mat khau
//                 0x60 = Xac nhan phim A
//                 0x61 = Xac nhan phim B
//             BlockAddr--Cac khoi dia chi
//             Sectorkey--Khu vuc mat khau
//             serNum--So serial the, 4 byte
// * Tra ve:MI_OK neu thanh cong
// */
//uchar MFRC522_Auth(uchar authMode, uchar BlockAddr, uchar *Sectorkey, uchar *serNum)
//{
//    uchar status;
//    uint recvBits;
//    uchar i;
//	uchar buff[12]; 

//	//Xac nhan lenh + Khoi dia chi + mat khau + so nhanh
//    buff[0] = authMode;
//    buff[1] = BlockAddr;
//    for (i=0; i<6; i++)
//    {    
//		buff[i+2] = *(Sectorkey+i);   
//	}
//    for (i=0; i<4; i++)
//    {    
//		buff[i+8] = *(serNum+i);   
//	}
//    status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

//    if ((status != MI_OK) || (!(Read_MFRC522(Status2Reg) & 0x08)))
//    {   
//		status = MI_ERR;   
//	}
//    
//    return status;
//}


///*
// * Ten ham:MFRC522_Read
// * Chuc nang: Doc khoi du lieu
// * Input:blockAddr--Cac khoi dia chi;recvData--Khoi du lieu doc ra
// * Tra ve: MI_OK neu thanh cong
// */
//uchar MFRC522_Read(uchar blockAddr, uchar *recvData)
//{
//    uchar status;
//    uint unLen;

//    recvData[0] = PICC_READ;
//    recvData[1] = blockAddr;
//    CalulateCRC(recvData,2, &recvData[2]);
//    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

//    if ((status != MI_OK) || (unLen != 0x90))
//    {
//        status = MI_ERR;
//    }
//    
//    return status;
//}


///*
// * Ten ham:MFRC522_Write
// * Chuc nang:Viet khoi du lieu
// * Input:blockAddr--cac khoi dia chi;writeData--du lieu ghi
// * Tra ve: MI_OK neu thanh cong
// */
//uchar MFRC522_Write(uchar blockAddr, uchar *writeData)
//{
//    uchar status;
//    uint recvBits;
//    uchar i;
//	uchar buff[18]; 
//    
//    buff[0] = PICC_WRITE;
//    buff[1] = blockAddr;
//    CalulateCRC(buff, 2, &buff[2]);
//    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

//    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
//    {   
//		status = MI_ERR;   
//	}
//        
//    if (status == MI_OK)
//    {
//        for (i=0; i<16; i++)		//16 byte FIFO ghi du lieu vao
//        {    
//        	buff[i] = *(writeData+i);   
//        }
//        CalulateCRC(buff, 16, &buff[16]);
//        status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);
//        
//		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
//        {   
//			status = MI_ERR;   
//		}
//    }
//    
//    return status;
//}


///*
// * Ten ham:MFRC522_Halt
// * CHuc nang: Dua the vao ngu dong
// * Input: Khong
// * Tra ve: Khong
// */
//void MFRC522_Halt(void)
//{
//	uint unLen;
//	uchar buff[4]; 

//	buff[0] = PICC_HALT;
//	buff[1] = 0;
//	CalulateCRC(buff, 2, &buff[2]);
// 
//	MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
