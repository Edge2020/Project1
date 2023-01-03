#ifndef __UART_H__
#define __UART_H__

#include <string.h>

#include "main.h"

#define UART_TIMEOUT (1000)

#define UartSend(_HUART, _STR) HAL_UART_Transmit(_HUART, _STR, strlen(_STR), UART_TIMEOUT)

#endif