#include "sensor_DHT11.h"

#include <stdio.h>
#include "uart.h"

char dbgbuffer[32];

extern UART_HandleTypeDef huart1;


//void delay_us(uint32_t us)
//{
//	uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
//	while(delay--);
//}

#define CPU_FREQUENCY_MHZ	72
volatile void delay_us(__IO uint32_t delay)
{
	int last, curr, val;
	int temp;

	while (delay != 0){
		temp = delay > 900 ? 900 : delay;
		last = SysTick->VAL;
		curr = last - CPU_FREQUENCY_MHZ * temp;
		if (curr >= 0){
			do{ val = SysTick->VAL; }while ((val < last) && (val >= curr));
		}
		else{
			curr += CPU_FREQUENCY_MHZ * 1000;
			do{ val = SysTick->VAL; }
			while ((val <= last) || (val > curr));
		}
		delay -= temp;
	}
}

uint8_t DHT11_Read_Bit(void) {
	uint8_t retry=0;	
	
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) && retry < 100){
		retry++;
		delay_us(1);
	}
	
	retry = 0;
	while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) && retry < 100){
		retry++;
		delay_us(1);
	}
	
	delay_us(4);
	
	return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
}

uint8_t DHT11_Read_Byte(void) {
	uint8_t i, temp = 0;
	for(i = 0; i < 8; i++){
		temp <<= 1;
		temp |= DHT11_Read_Bit();
	}
	return temp;
}

uint8_t DHT11_Read_Data(float *humi, float *temp) {
	uint8_t i;
	uint8_t data[5];
	
	DHT11_Start();
	
	if(DHT11_Check()){

		for(i = 0; i < 5; i++) data[i] = DHT11_Read_Byte();
		
		DHT11_Mode_Set_Output();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		
		sprintf(dbgbuffer, "[%x][%x][%x][%x][%x]", data[0], data[1], data[2], data[3], data[4]);
		UartSend(&huart1, dbgbuffer);
		
		if(data[4] == (data[0] + data[1] + data[2] + data[3])){
			UartSend(&huart1, "[CheckSum OK]\n");
			
			*humi = (float)(data[0]);
			*temp = (float)(data[2]) + (float)(data[3]) / 100;
			
			return 1; 
		}
	}
	return 0;
}

void DHT11_Start(void){
	DHT11_Mode_Set_Output();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	delay_us(30);
}

uint8_t DHT11_Check(void) {
	uint8_t retry = 0;
	
	DHT11_Mode_Set_Input();
	
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) && retry < 100) {
		retry++;
		delay_us(1);
	}
	
	if(retry >= 100) return 0;	
	else retry = 0;	
	
	while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) && retry < 100){
		retry++;
		delay_us(1);
	}
	
	if(retry >= 100) return 0;
	
	return 1;
}

static void DHT11_Mode_Set_Output(void) {
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin 		= 	GPIO_PIN_8;
  GPIO_InitStruct.Mode  	= 	GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull 		=   GPIO_PULLUP;
  GPIO_InitStruct.Speed 	= 	GPIO_SPEED_FREQ_HIGH;
	
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
 
static void DHT11_Mode_Set_Input(void) {
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin 		= 	GPIO_PIN_8;
  GPIO_InitStruct.Mode		= 	GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull 		=   GPIO_PULLUP;
  GPIO_InitStruct.Speed   =   GPIO_SPEED_FREQ_HIGH;
	
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}