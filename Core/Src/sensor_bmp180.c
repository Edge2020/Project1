#include "sensor_bmp180.h"

BMP180_Calibration_TypeDef BMP180_CalibrationData;
uint32_t BMP180_CMD_PRESSURE    = 0xF4;
uint32_t BMP180_CMD_TEMPERATURE = 0x2E;
int osrs = 3;

void BMP180_Init()
{
	uint8_t buffer[22];
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR_R, BMP180_AC_BE, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer, 22, 3000);
	BMP180_CalibrationData.AC1 = (buffer[0]  << 8)  |  buffer[1];
	BMP180_CalibrationData.AC2 = (buffer[2]  << 8)  |  buffer[3];
	BMP180_CalibrationData.AC3 = (buffer[4]  << 8)  |  buffer[5];
	BMP180_CalibrationData.AC4 = (buffer[6]  << 8)  |  buffer[7];
	BMP180_CalibrationData.AC5 = (buffer[8]  << 8)  |  buffer[9];
	BMP180_CalibrationData.AC6 = (buffer[10]  << 8) |  buffer[11];
	BMP180_CalibrationData.B1  = (buffer[12]  << 8) |  buffer[13];
	BMP180_CalibrationData.B2  = (buffer[14]  << 8) |  buffer[15];
	BMP180_CalibrationData.MB  = (buffer[16]  << 8) |  buffer[17];
	BMP180_CalibrationData.MC  = (buffer[18]  << 8) |  buffer[19];
	BMP180_CalibrationData.MD  = (buffer[20]  << 8) |  buffer[21];
}

float BMP180_GetPressure(){
	uint8_t data[3];
	int32_t B3, B6, X3, p;
	uint32_t B4, B7;	
	
	//get data
	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR_W, BMP180_MADDR_W, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&BMP180_CMD_PRESSURE, 1, 1000);
	HAL_Delay(26);
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR_R, BMP180_MADDR_R, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, 3, 1000);
 
	B6 = BMP180_CalibrationData.B5 - 4000;
	X3 = ((BMP180_CalibrationData.B2 * ((B6 * B6) >> 12)) >> 11) + ((BMP180_CalibrationData.AC2 * B6) >> 11);
	B3 = (((((int32_t)BMP180_CalibrationData.AC1) * 4 + X3) << osrs) + 2) >> 2;
	X3 = (((BMP180_CalibrationData.AC3 * B6) >> 13) + ((BMP180_CalibrationData.B1 * ((B6 * B6) >> 12)) >> 16) + 2) >> 2;
	B4 = (BMP180_CalibrationData.AC4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)((data[0] << 16) | (data[1] <<8) | data[2] >> (8 - osrs)) - B3) * (50000 >> osrs);
	
	if (B7 < 0x80000000){
		p = (B7 << 1) / B4; 
	}
	else{
		p = (B7 / B4) << 1;
	}
	
	p = p + (((((p >> 8) * (p >> 8) * 3038) >> 16) + ((-7357 * p) >> 16) + 3791) >> 4);
	
	return p / 100.0;
}

float BMP180_GetTemperature(){
	uint8_t data[2];
	
	//get data
	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR_W, BMP180_MADDR_W, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&BMP180_CMD_TEMPERATURE, 1, 1000);
	HAL_Delay(5);
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR_R, BMP180_MADDR_R, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, 2, 1000);
	
	uint16_t UT = (data[0] << 8) | data[1];
	BMP180_CalibrationData.B5  = (((int32_t)UT - (int32_t)BMP180_CalibrationData.AC6) * (int32_t)BMP180_CalibrationData.AC5) >> 15;
	BMP180_CalibrationData.B5 += ((int32_t)BMP180_CalibrationData.MC << 11) / (BMP180_CalibrationData.B5 + BMP180_CalibrationData.MD);
	
	return (BMP180_CalibrationData.B5 + 8) / 160.0;
}