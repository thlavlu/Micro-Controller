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
#include"SPI_multiple.h"
#include "stdio.h"
#include "stdint.h"
#include <stdbool.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RTC_READ_ADDRESS   			0xD1  // DS1307 READ ID
#define RTC_WRITE_ADDRESS  			0xD0  // DS1307 WRITE ID
#define RTC_CONTROL_ADDRESS 	    0x07 // CONTROL REGISTER ADDRESS
#define RTC_ADDRESS_FOR_SECOND 	    0x00 // ADDRESS FOR SECOND
#define PCF8574A_ADDRESS   			0x3F // 0x27u

#define I2C_PORT 	GPIOA //GPIOA // PINs BELONG TO THE PORT
#define SCL     	GPIO_PIN_0 //GPIO_PIN_0 // CLOCK PINs as OUTPUT
#define SDA     	GPIO_PIN_1 //GPIO_PIN_1 //DATA PIN as OUTPUT
#define SCL_IN     	GPIO_PIN_2 //GPIO_PIN_2 // CLOCK PINs as INPUT
#define SDA_IN     	GPIO_PIN_3 //GPIO_PIN_3 //DATA PIN as INPUT

#define RS_BIT 0 // Register select bit
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit


typedef struct
{
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t weekDay;
  uint8_t date;
  uint8_t month;
  uint8_t year;
}rtc_t;
bool b;
uint8_t hr_u, hr_d, mn_d, mn_u, sc_d, sc_u, dt_u, dt_d, mt_u, mt_d, yr_u, yr_d, min, hour;
	rtc_t rtc;


uint32_t M_RTC[8];
uint8_t Data_0[8] = {0b00000, 0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110};
uint8_t Data_1[8] = {0b00000, 0b00100, 0b01100, 0b10100, 0b00100, 0b00100, 0b00100, 0b11111};
uint8_t Data_2[8] = {0b00000, 0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111};
uint8_t Data_3[8] = {0b00000, 0b01110, 0b10001, 0b00001, 0b00110, 0b00001, 0b10001, 0b01110};
uint8_t Data_4[8] = {0b00000, 0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010};
uint8_t Data_5[8] = {0b00000, 0b11111, 0b10000, 0b10000, 0b11110, 0b00001, 0b10001, 0b01110};
uint8_t Data_6[8] = {0b00000, 0b01110, 0b10001, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110};
uint8_t Data_7[8] = {0b00000, 0b11111, 0b00001, 0b00010, 0b00100, 0b00100, 0b01000, 0b01000};
uint8_t Data_8[8] = {0b00000, 0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110};
uint8_t Data_9[8] = {0b00000, 0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b10001, 0b01110};
uint8_t Data_k[8] = {0b00, 0b10, 0b10, 0b00, 0b00, 0b10, 0b10, 0b00};
uint8_t Data_sc[8] = {0b00000, 0b00100, 0b00100, 0b00000, 0b00000, 0b00100, 0b00100, 0b00000};
uint8_t Blink[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
uint8_t  Blink_D1[8] = {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000};

uint8_t min, hour, num_cycle = 0;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t backlight_state = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SPI_7219_INIT(int n) {
	for (int k = 1; k <= n; k++) {
		SPI_7219_SEND(0x09, 0x00, k);       //  no decoding
		SPI_7219_SEND(0x0A, 0x0F, k);       //  brightness intensity
		SPI_7219_SEND(0x0B, 0x07, k);       //  scan limit = 8 LEDs
		SPI_7219_SEND(0x0C, 0x01, k);       //  power down =0,normal mode = 1
		SPI_7219_SEND(0x0F, 0x00, k);       //  no test display
	}
}

void display_digit(uint8_t digit, uint8_t shift) {
    // Holding the digits 0 to 9.
    const uint8_t* digit_data[] = {Data_0, Data_1, Data_2, Data_3, Data_4, Data_5, Data_6, Data_7, Data_8, Data_9};
    //uint8_t shift;
    if (digit >= 0 && digit <= 9) {
    	if (shift == 4 ){
        	for (int i = 0; i < 8; i++) {
        	SPI_7219_SEND(i + 1, digit_data[digit][i], 4);   //4th display
        	}

    	} else if (shift == 3){
        	for (int i = 0; i < 8; i++) {
        	SPI_7219_SEND(i + 1, digit_data[digit][i]<<2, 3);  // 3rd display
        	}

    	} else if (shift == 2){
        	for (int i = 0; i < 8; i++) {
        	SPI_7219_SEND(i + 1, Data_k[i]<<6|digit_data[digit][i], 2);  //2nd display
        	}

    	} else if (shift == 1){
            for (int i = 0; i < 8; i++) {
            	SPI_7219_SEND(i + 1, Data_0[i]<<2, 1); //first display
          }

    	}

    } else {
        // Handle the default case.
        for (int i = 0; i < 8; i++) {
           SPI_7219_SEND(i + 1, Data_0[i] << shift, 3 - shift);
        }
    }
}

void display_rtc(uint8_t hr_u, uint8_t hr_d, uint8_t mn_u, uint8_t mn_d) {
    // Display hours tens and units digits.
    display_digit(hr_d, 4);  //4th display
    display_digit(hr_u, 3); // 3rd display
    // Display minutes tens and units digits.
    display_digit(mn_d, 2); //2nd display
    display_digit(mn_u, 1); //first display
}
// Tiny delay
void I2C_dly(void)
{
}

void I2C_Start(void)
{
  HAL_GPIO_WritePin(I2C_PORT, SDA, 1);  // SDA = 1 i2c start bit sequence
  I2C_dly();
  HAL_GPIO_WritePin(I2C_PORT, SCL, 1);  //SCL = 1
  I2C_dly();
  HAL_GPIO_WritePin(I2C_PORT, SDA, 0);  // SDA = 0
  I2C_dly();
  HAL_GPIO_WritePin(I2C_PORT, SCL, 0);  //  SCL = 0;
  I2C_dly();
}

void I2C_Stop(void)
{
  HAL_GPIO_WritePin(I2C_PORT, SDA, 0);  // SDA = 0 i2c stop bit sequence
  I2C_dly();
  HAL_GPIO_WritePin(I2C_PORT, SCL, 1);  //SCL = 1
  I2C_dly();
  HAL_GPIO_WritePin(I2C_PORT, SDA, 1);  // SDA = 1
  I2C_dly();
}

unsigned char I2C_Read(char ack)
{
char x, d=0;
HAL_GPIO_WritePin(I2C_PORT, SDA, 1);  // SDA = 1
  for(x=0; x<8; x++) {
    d <<= 1;
    do {
    	HAL_GPIO_WritePin(I2C_PORT, SCL, 1);  //SCL = 1
    }
    while(HAL_GPIO_ReadPin(I2C_PORT, SCL_IN)==0);    // wait for any SCL clock stretching
    I2C_dly();
    if(HAL_GPIO_ReadPin(I2C_PORT, SDA_IN)) d |= 1;
    HAL_GPIO_WritePin(I2C_PORT, SCL, 0);  //SCL = 0
  }
  if(ack){
	  HAL_GPIO_WritePin(I2C_PORT, SDA, 0);  //SDA = 0;
  }
  else{
	  HAL_GPIO_WritePin(I2C_PORT, SDA, 1);  //SDA = 1;
  }

  HAL_GPIO_WritePin(I2C_PORT, SCL, 1);  //SCL = 1;
  I2C_dly();             // send (N)ACK bit
  HAL_GPIO_WritePin(I2C_PORT, SCL, 0);  //SCL = 0;
  HAL_GPIO_WritePin(I2C_PORT, SDA, 1);  //SDA = 1;
  return d;
}

bool I2C_Write(unsigned char d)
{
char x;
static bool b;
  for(x=8; x; x--) {
    if(d&0x80){
    	HAL_GPIO_WritePin(I2C_PORT, SDA, 1);  //SDA = 1;
    }
    else{
    	HAL_GPIO_WritePin(I2C_PORT, SDA, 0);  //SDA = 0;
    }
    HAL_GPIO_WritePin(I2C_PORT, SCL, 1);  //SCL = 1;
    d <<= 1;
    HAL_GPIO_WritePin(I2C_PORT, SCL, 0);  //SCL = 0;
  }
  HAL_GPIO_WritePin(I2C_PORT, SDA, 1);  //SDA = 1;
  HAL_GPIO_WritePin(I2C_PORT, SCL, 1);  //SCL = 1;
  I2C_dly();
  b = HAL_GPIO_ReadPin(I2C_PORT, SDA_IN);  //READIN SDA_IN;
  HAL_GPIO_WritePin(I2C_PORT, SCL, 0);  //SCL = 0;
  return b;
}

//We can initialize the RTC with the code below
void RTC_Init(void)
{

    I2C_Start();                            // Start I2C communication

    I2C_Write(RTC_WRITE_ADDRESS);        // Connect to DS1307 by sending its ID on I2c Bus
    I2C_Write(RTC_CONTROL_ADDRESS);// Select the Ds1307 ControlRegister to configure Ds1307

    I2C_Write(0x00);                        // Write 0x00 to Control register to disable SQW-Out

    I2C_Stop();                             // Stop I2C communication after initializing DS1307
}

// Set Date and Time
void RTC_SetDateTime(rtc_t *rtc)
{
    I2C_Start();                          // Start I2C communication

    I2C_Write(RTC_WRITE_ADDRESS);      // connect to DS1307 by sending its ID on I2c Bus
    I2C_Write(RTC_ADDRESS_FOR_SECOND); // Request sec RAM address at 00H

    I2C_Write(rtc->sec);                    // Write sec from RAM address 00H
    I2C_Write(rtc->min);                    // Write min from RAM address 01H
    I2C_Write(rtc->hour);                    // Write hour from RAM address 02H
    I2C_Write(rtc->weekDay);                // Write weekDay on RAM address 03H
    I2C_Write(rtc->date);                    // Write date on RAM address 04H
    I2C_Write(rtc->month);                    // Write month on RAM address 05H
    I2C_Write(rtc->year);                    // Write year on RAM address 06h

    I2C_Stop();                              // Stop I2C communication after Setting the Date
}


// Get Date and Time
void RTC_GetDateTime(rtc_t *rtc)
{
    I2C_Start();                            // Start I2C communication

    I2C_Write(RTC_WRITE_ADDRESS);        // connect to DS1307 by sending its ID on I2c Bus
    I2C_Write(RTC_ADDRESS_FOR_SECOND); // Request Sec RAM address at 00H

    I2C_Start();                            // Start I2C communication
    I2C_Write(RTC_READ_ADDRESS);            // connect to DS1307(Read mode) by sending its ID

    rtc->sec = I2C_Read(1);                // read second and return Positive ACK
    rtc->min = I2C_Read(1);                 // read minute and return Positive ACK
    rtc->hour= I2C_Read(1);               // read hour and return Negative/No ACK
    rtc->weekDay = I2C_Read(1);           // read weekDay and return Positive ACK
    rtc->date= I2C_Read(1);              // read Date and return Positive ACK
    rtc->month=I2C_Read(1);            // read Month and return Positive ACK
    rtc->year =I2C_Read(0);             // read Year and return Negative/No ACK

    I2C_Stop();                              // Stop I2C communication after reading the Date
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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  RTC_Init();
    rtc.hour = 0x17; //
    rtc.min =  0x30;
    rtc.sec =  0x00;

    rtc.date = 0x24; //
    rtc.month = 0x09;
    rtc.year = 0x23;
    rtc.weekDay = 6;

    RTC_SetDateTime(&rtc);


	SPI_7219_INIT(4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		RTC_GetDateTime(&rtc);
		/* Display the Time and Date continuously */
		hr_u=0x0f&rtc.hour;
		hr_d=0x0f&rtc.hour>>4;
		mn_u=0x0f&rtc.min;
		mn_d=0x0f&rtc.min>>4;
		display_rtc(hr_u, hr_d, mn_u, mn_d);
		HAL_Delay(1000);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim3.Init.Prescaler = 0;
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2048;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 35156;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  huart3.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
