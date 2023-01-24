#ifndef __ESP8266_H__
#define __ESP8266_H__

#include "main.h"
#include "uart.h"

#define UART2_BUFFER_SIZE 256 

void wifi_send(uint8_t *str);

#endif