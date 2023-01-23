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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

#include "oled.h"
#include "uart.h"
#include "sensor_bmp180.h"
#include "sensor_BH1750.h"
#include "sensor_DHT11.h"

#include "u8g2.h"
#include "u8x8.h"

#include "icon.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_SCAN_DELAY 20

#define KEY_UP 			1
#define KEY_3 			2
#define KEY_2 			3
#define KEY_1 			4
#define KEY_RIGHT 	5
#define KEY_6 			6
#define KEY_5 			7
#define KEY_4 			8
#define KEY_LEFT 		9
#define KEY_9 			10
#define KEY_8 			11
#define KEY_7 			12
#define KEY_DOWN 		13
#define KEY_ENTER 	14
#define KEY_0 			15
#define KEY_CANCEL 	16

#define UI_PAGE_MAX 		1
#define UI_SETTING_MAX 	2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

float  		environment_pressure 						=	 0.0;
float 		environment_temperature_BMP180 	=	 0.0;
float 		environment_temperature_DHT11 	=	 0.0;
uint16_t	environment_humidity 						=	 0;
uint16_t  environment_light						  	=  0;

float			limit_pressure		 = 1200;
float			limit_temperature	 = 15.0;
uint16_t 	limit_humidity		 = 60;
uint16_t 	limit_light				 = 1000;

char str_buffer[32];

uint8_t UI_page = 0;
uint8_t UI_subpage = 0;
uint8_t UI_setting_selected = 0;
uint8_t UI_setting_subselected = 0;
uint8_t UI_SUBSELECTED_MAX = 0;

const char UI_setting_texts[3][8] = {
		"Alarm",
		"Wi-Fi",
		"Upload",
	};

/* Definitions for tasks */

/*
osThreadId_t ;
const osThreadAttr_t _attributes = {
	.name = "",
	.stack_size = 128 * ,
	.priority = (osPriority_t) osPriorityNormal,
};
*/


osThreadId_t OLEDTaskHandle;
const osThreadAttr_t OLEDTask_attributes = {
  .name = "OLEDTask",
  .stack_size = 128 * 16,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t uartDebugTaskHandle;
const osThreadAttr_t uartDebugTask_attributes = {
	.name = "uartDebug",
	.stack_size = 128 * 8,
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t getSensorDataTaskHandle;
const osThreadAttr_t getSensorDataTask_attributes = {
	.name = "sensorData",
	.stack_size = 128 * 8,
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t get1WireDataTaskHandle;
const osThreadAttr_t get1WireDataTask_attributes = {
	.name = "1WireData",
	.stack_size = 128 * 8,
	.priority = (osPriority_t) osPriorityRealtime,	//Need high priority for 1-wire communication.
};

osThreadId_t keyboardServiceTaskHandle;
const osThreadAttr_t keyboardServiceTaskHandle_attributes = {
	.name = "keyboardServ",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal2,
};

osThreadId_t buzzerTaskHandle;
const osThreadAttr_t buzzerTaskHandle_attributes = {
	.name = "buzzer",
	.stack_size = 128 * 2,
	.priority = (osPriority_t) osPriorityNormal1,
};

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);


void StartOLEDTask(void *argument);
void StartUartDebugTask(void *argument);
void StartGetSensorDataTask(void *argument);
void StartGet1WireDataTask(void *argument);
void StartKeyboardServiceTask(void *argument);
void StartBuzzerTask(void *argument);


uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr);
uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,void *arg_ptr);
void u8g2_Init(u8g2_t *u8g2);
void u8g2_Draw(u8g2_t *u8g2);

void draw_inputBox(u8g2_t *u8g2, uint8_t x, uint8_t y, uint8_t w, uint8_t h, float value);


/* USER CODE BEGIN PFP */

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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
	SET_BIT(PWR->CR, PWR_CR_CWUF_Msk);
	SET_BIT(PWR->CR, PWR_CR_CSBF_Msk);
	
	OLED_Init();

	HAL_I2C_Init(&hi2c1);
	BMP180_Init();
//	BH1750_Send_Cmd(POWER_ON_CMD);
//	BH1750_Send_Cmd(RESET_REGISTER);
//	BH1750_Send_Cmd(CONT_H_MODE);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
	
  OLEDTaskHandle 						=   osThreadNew(StartOLEDTask, NULL, &OLEDTask_attributes);
	uartDebugTaskHandle 			= 	osThreadNew(StartUartDebugTask, NULL, &uartDebugTask_attributes);
	getSensorDataTaskHandle 	= 	osThreadNew(StartGetSensorDataTask, NULL, &getSensorDataTask_attributes);
	get1WireDataTaskHandle	  = 	osThreadNew(StartGet1WireDataTask, NULL, &get1WireDataTask_attributes);
	keyboardServiceTaskHandle = 	osThreadNew(StartKeyboardServiceTask, NULL, &keyboardServiceTaskHandle_attributes);
	buzzerTaskHandle					=		osThreadNew(StartBuzzerTask, NULL, &buzzerTaskHandle_attributes);
	
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
	
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
void SystemClock_Config(void)			//PLL 72MHz
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

//void SystemClock_Config(void)			//HSI 8MHz
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_4|OLED_DC_Pin|OLED_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCDC_EN_GPIO_Port, DCDC_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*Configure GPIO pins : PB4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB14 PB3 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_14|GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_CS_Pin OLED_DC_Pin OLED_RES_Pin */
  GPIO_InitStruct.Pin = OLED_CS_Pin|OLED_DC_Pin|OLED_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DCDC_EN_Pin */
  GPIO_InitStruct.Pin = DCDC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DCDC_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void StartOLEDTask(void *argument)
{
	//char buffer[32];
	u8g2_t u8g2;
	OLED_RES_Set();	
	u8g2_Init(&u8g2);
  for(;;)
  {
		u8g2_FirstPage(&u8g2);
		do{
			u8g2_Draw(&u8g2);
		}while(u8g2_NextPage(&u8g2));
		
    osDelay(1);
  }

}

void StartUartDebugTask(void *argument){
	
	char buffer[48];
	
	for(;;){
		UartSend(&huart1, "\nEnvironment Status:\n");
		
		sprintf(buffer, "Pressure: %f\n", environment_pressure);
		UartSend(&huart1, buffer);
		
		sprintf(buffer, "Temperature(BMP180): %f\n", environment_temperature_BMP180);
		UartSend(&huart1, buffer);
		
		sprintf(buffer, "Temperature(DHT11): %f\n", environment_temperature_DHT11);
		UartSend(&huart1, buffer);
		
		sprintf(buffer, "Humidity: %d\n", environment_humidity);
		UartSend(&huart1, buffer);
		
		sprintf(buffer, "Light: %d\n", environment_light);
		UartSend(&huart1, buffer);
		
//		sprintf(buffer, "%lu\n", (unsigned long)HAL_RCC_GetHCLKFreq());
//		UartSend(&huart1, buffer); 
		
		UartSend(&huart1, "\n");
		
		osDelay(1000);
	}
}

void StartGetSensorDataTask(void *argument){
	
	uint8_t lightData_Raw[2];
	
	for(;;){
		
		environment_pressure = BMP180_GetPressure();
		
		environment_temperature_BMP180 = BMP180_GetTemperature();
		
		if(BH1750_Send_Cmd(CONT_H_MODE) == HAL_OK){
			HAL_Delay(200);
			if(BH1750_Read_Dat(&lightData_Raw) == HAL_OK){
				environment_light = BH1750_Dat_To_Lux(&lightData_Raw);
			}
		}
		
		osDelay(1000);
	}

}

void StartGet1WireDataTask(void *argument){
	
	osDelay(1000);
	
	for(;;){
		DHT11_Read_Data(&environment_humidity, &environment_temperature_DHT11);

		osDelay(2000);
	}

}

void StartKeyboardServiceTask(void *argument) {
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1);	//to avoid unexpected poweroff after WKUP.
	
	uint8_t col;
	uint8_t key;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	for(;;){
		col = 0;
		key = 0;

		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1){
			//Enter standby mode.
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1);
			HAL_Delay(100);
			HAL_GPIO_WritePin(DCDC_EN_GPIO_Port, DCDC_EN_Pin, GPIO_PIN_RESET);	//disable DCDC
			SET_BIT(PWR->CR, PWR_CR_CWUF_Msk);
			HAL_PWR_EnterSTANDBYMode();
		}
		
		GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0){
			HAL_Delay(KEY_SCAN_DELAY);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0) col = 1;
		}
		else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0){
			HAL_Delay(KEY_SCAN_DELAY);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0) col = 2;
		}
		else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 0){
			HAL_Delay(KEY_SCAN_DELAY);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == 0) col = 3;
		}
		else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == 0){
			HAL_Delay(KEY_SCAN_DELAY);
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == 0) col = 4;
		}
		
		if(col != 0){
			GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
			
			switch(col){
				case 1:
					if		 (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == 1) key = 1;
					else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == 1) key = 2;
					else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)  == 1) key = 3;
					else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)  == 1) key = 4;
					break;
				case 2:
					if		 (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == 1) key = 5;
					else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == 1) key = 6;
					else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)  == 1) key = 7;
					else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)  == 1) key = 8;
					break;
				case 3:
					if		 (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == 1) key = 9;
					else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == 1) key = 10;
					else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)  == 1) key = 11;
					else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)  == 1) key = 12;
					break;
				case 4:
					if		 (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == 1) key = 13;
					else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == 1) key = 14;
					else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)  == 1) key = 15;
					else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)  == 1) key = 16;
					break;
				default:
					break;
			}
			
			switch(key){
				case KEY_LEFT:
					if(UI_page != 0){
						if(UI_subpage == 0) UI_page--;
					}
					if(UI_page == 1){
						if(UI_subpage == 1){
							switch(UI_setting_subselected){
								case 0:
									if(limit_temperature >= 1) limit_temperature--;
									break;
								
								case 1:
									if(limit_light >= 100) limit_light -= 100;
									break;
								
								case 2:
									if(limit_pressure >= 100) limit_pressure -=100;
									break;
								
								case 3:
									if(limit_humidity >= 1) limit_humidity--;
									break;
							
								default:
									break;
							}
						
						}
					}
					break;
				
				case KEY_RIGHT:
					if(UI_page != UI_PAGE_MAX){
						if(UI_subpage == 0) UI_page++;
					}
					if(UI_page == 1){
						if(UI_subpage == 1){
							switch(UI_setting_subselected){
								case 0:
									if(limit_temperature <= 99) limit_temperature++;
									break;
								
								case 1:
									if(limit_light <= 65525) limit_light += 100;
									break;
								
								case 2:
									if(limit_pressure <= 2900) limit_pressure += 100;
									break;
								
								case 3:
									if(limit_humidity <= 99) limit_humidity++;
									break;
							
								default:
									break;
							}
						
						}
					}
					break;
				
				case KEY_UP:
					if(UI_page == 1){
						if(UI_subpage == 0){
							if(UI_setting_selected != 0) UI_setting_selected--;
						}
						else{
							if(UI_setting_subselected != 0) UI_setting_subselected--;
						}						
					}
					break;

				case KEY_DOWN:
					if(UI_page == 1){
						if(UI_subpage == 0){
							if(UI_setting_selected != UI_SETTING_MAX) UI_setting_selected++;
						}
						else{
							if(UI_setting_subselected != UI_SUBSELECTED_MAX) UI_setting_subselected++;
						}
					}
					break;
					
				case KEY_ENTER:
					if(UI_page == 1){
						UI_subpage = UI_setting_selected + 1;
						switch(UI_subpage){
							case 1:
								UI_SUBSELECTED_MAX = 3;
								break;
							
							case 2:
								UI_SUBSELECTED_MAX = 3;
								break;
							
							case 3:
								UI_SUBSELECTED_MAX = 3;
								break;
							
							default:
								break;
						}
					}
					break;
					
				case KEY_CANCEL:
					if(UI_page == 1){
						if(UI_subpage != 0){
							UI_subpage = 0;
						}
					}
					break;
			
				default:
					break;
			}
			
			HAL_Delay(200);
		}	
		
		osDelay(10);
	}
}

void StartBuzzerTask(void *argument) {
	
	osDelay(1000);
	
	for(;;){
		
		if(environment_light > limit_light || 
			 environment_pressure > limit_pressure ||
			 environment_humidity > limit_humidity ||
			 ((environment_temperature_DHT11 + environment_temperature_BMP180) / 2) > limit_temperature){
		
			for(uint8_t i = 0; i < 64; i++){
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
				delay_us(500000 / 294);
			}
			for(uint8_t i = 0; i < 64; i++){
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
				delay_us(500000 / 350);
			}
			for(uint8_t i = 0; i < 96; i++){
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
				delay_us(500000 / 441);
			}
			
		}

		osDelay(1000);
	}


}

uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,void *arg_ptr) {
	uint8_t i;
	uint8_t *data = NULL;
	
	switch (msg){
		case U8X8_MSG_BYTE_SEND: 
			data = (uint8_t*)arg_ptr;
			if(arg_int == 1){
				OLED_WR_Byte(data[0], OLED_CMD);
			}
			else{
				for(i = 0; i < arg_int; i++) OLED_WR_Byte(data[i], OLED_DATA);	
			}
			break;
		
		case U8X8_MSG_BYTE_INIT:
			OLED_Init();
			break;
		
		case U8X8_MSG_BYTE_SET_DC:
			arg_int ? OLED_DC_Set() : OLED_DC_Clr();
			break;
		
		case U8X8_MSG_BYTE_START_TRANSFER: 
			OLED_CS_Set();
			break;
		
		case U8X8_MSG_BYTE_END_TRANSFER: 
			OLED_CS_Clr();
			break;
		
		default:
			return 0;
		
	}
	return 1;
}



uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr) {
	switch (msg){
		case U8X8_MSG_GPIO_AND_DELAY_INIT:
			OLED_Init();
			HAL_Delay(10);
			break;
		
		case U8X8_MSG_DELAY_MILLI:
			HAL_Delay(arg_int);
			break;
		
		case U8X8_MSG_GPIO_CS:
			arg_int ? OLED_CS_Set() : OLED_CS_Clr();
			break;
		
		case U8X8_MSG_GPIO_DC: 
			arg_int ? OLED_DC_Set() : OLED_DC_Clr();
			break;
		
		case U8X8_MSG_GPIO_RESET:
			//arg_int ? OLED_RES_Set() : OLED_RES_Clr();
			break;
		
	}
	return 1;
}

void u8g2_Init(u8g2_t *u8g2) {
	u8g2_Setup_ssd1306_128x64_noname_f(u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay);
	u8g2_InitDisplay(u8g2);
	u8g2_SetPowerSave(u8g2, 0);

}


void u8g2_Draw(u8g2_t *u8g2) {
	
	switch(UI_page){
		case 0:
			u8g2_DrawXBMP(u8g2, 96, 32, 32, 32, icon_background);
		
			u8g2_DrawXBMP(u8g2, 4, 0, 16, 16, icon_temp);
			u8g2_DrawXBMP(u8g2, 64, 0, 16, 16, icon_temp);
			u8g2_DrawXBMP(u8g2, 4, 16, 16, 16, icon_light);
			u8g2_DrawXBMP(u8g2, 4, 32, 16, 16, icon_pres);
			u8g2_DrawXBMP(u8g2, 4, 48, 16, 16, icon_humi);
			
			u8g2_SetFontDirection(u8g2, 1);
			u8g2_SetFontDirection(u8g2, 0);
			u8g2_SetFont(u8g2, u8g2_font_4x6_mf);
			u8g2_DrawStr(u8g2, 16, 6, "1");
			u8g2_DrawStr(u8g2, 76, 6, "2");
			
			u8g2_SetFont(u8g2, u8g2_font_6x10_tf);

			sprintf(str_buffer, "%.2f", environment_temperature_BMP180);
			u8g2_DrawStr(u8g2, 24, 12, str_buffer);

			sprintf(str_buffer, "%.2f", environment_temperature_DHT11);
			u8g2_DrawStr(u8g2, 84, 12, str_buffer);
			
			sprintf(str_buffer, "%dlux", environment_light);
			u8g2_DrawStr(u8g2, 24, 28, str_buffer);
			
			sprintf(str_buffer, "%.2fhPa", environment_pressure);
			u8g2_DrawStr(u8g2, 24, 44, str_buffer);
			
			sprintf(str_buffer, "%d%%", environment_humidity);
			u8g2_DrawStr(u8g2, 24, 60, str_buffer);
			break;
		
		case 1:
			switch(UI_subpage){
				case 0:
					u8g2_DrawXBMP(u8g2, 96, 32, 32, 32, icon_background_setting);
					u8g2_SetFont(u8g2, u8g2_font_6x10_tf);
					for(int i = 0; i < 3; i++){
						u8g2_DrawStr(u8g2, 16, 16 * (i + 1), UI_setting_texts[i]);
						if(i == UI_setting_selected){
							u8g2_DrawStr(u8g2, 4, 16 * (i + 1), ">");
						}
					}
					break;
					
				case 1:	//alarm
					u8g2_DrawStr(u8g2, 4, 16 * (UI_setting_subselected + 1) - 4, ">");
					u8g2_DrawXBMP(u8g2, 96, 32, 32, 32, icon_alarm);
					u8g2_DrawXBMP(u8g2, 16, 0, 16, 16, icon_temp);
					u8g2_DrawXBMP(u8g2, 16, 16, 16, 16, icon_light);
					u8g2_DrawXBMP(u8g2, 16, 32, 16, 16, icon_pres);
					u8g2_DrawXBMP(u8g2, 16, 48, 16, 16, icon_humi);
				
					sprintf(str_buffer, "%.2f", limit_temperature);
					u8g2_DrawStr(u8g2, 36, 12, str_buffer);	
				
					sprintf(str_buffer, "%dlux", limit_light);
					u8g2_DrawStr(u8g2, 36, 28, str_buffer);
					
					sprintf(str_buffer, "%.2fhPa", limit_pressure);
					u8g2_DrawStr(u8g2, 36, 44, str_buffer);
					
					sprintf(str_buffer, "%d%%", limit_humidity);
					u8g2_DrawStr(u8g2, 36, 60, str_buffer);
					break;
				
				case 2:	//wifi
					
					break;
				
				case 3:	//upload
					
					break;
					
				default:
					break;
			}
			break;
	
		default:
			break;
	}

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
	
	UartSend(&huart1, "!!!Error_Handle!!!\n");
	
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
