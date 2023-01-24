#include "esp8266.h"

uint8_t buffer_rx[UART2_BUFFER_SIZE];
uint8_t buffer_tx[UART2_BUFFER_SIZE];
uint8_t rx;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void wifi_send(uint8_t *str) {

	memset(buffer_tx, '\0', UART2_BUFFER_SIZE);
	
	strcpy((char *)buffer_tx, (char *)str);
	
	HAL_UART_Receive_IT(&huart2, &rx, 1);
	
	HAL_UART_Transmit_IT(&huart2, buffer_tx, strlen((char *)buffer_tx));

}