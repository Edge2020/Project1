#ifndef __SENSOR_BMP180_H__
#define __SENSOR_BMP180_H__

#include "main.h"

#define BMP180_ADDR_W  	0xEE
#define BMP180_MADDR_W 	0xF4

#define BMP180_ADDR_R  	0xEF
#define BMP180_MADDR_R 	0xF6
#define BMP180_AC_BE   	0xAA

extern I2C_HandleTypeDef hi2c1;

typedef struct {
	int16_t			AC1;
	int16_t	  	AC2;
	int16_t	  	AC3;
	uint16_t  	AC4;
	uint16_t  	AC5;
	uint16_t  	AC6;
	int16_t	 		B1;
	int16_t 		B2;
	int16_t 		MB;
	int16_t 		MC;
	int16_t 		MD;
	int32_t 		B5;
} BMP180_Calibration_TypeDef;

void BMP180_Init();
float BMP180_GetPressure();
float BMP180_GetTemperature();
int32_t BMP180_GetUT(void);
int32_t BMP180_GetUP(void);

#endif