#include "esp8266.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void wifi_send(char *str) {

	UartSend(&huart2, str);

	while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);
}