#ifndef __SENSOR_DHT11_H__
#define __SENSOR_DHT11_H__

#include "main.h"

static void DHT11_Mode_Set_Output(void);
static void DHT11_Mode_Set_Input(void);

uint8_t DHT11_Read_Data(uint8_t *humi, float *temp);
uint8_t DHT11_Read_Bit(void);
uint8_t DHT11_Read_Byte(void);

void DHT11_Start(void);
uint8_t DHT11_Check(void);

volatile void delay_us(__IO uint32_t delay);

#endif