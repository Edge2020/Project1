#ifndef __FLASH_H__
#define __FLASH_H__

#include "main.h"

#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"

uint8_t writeData(uint32_t addr, uint32_t *data, uint32_t size);
void readData(uint32_t addr, uint32_t *sto, uint32_t size);
uint32_t eraseFlash(uint32_t addr, uint32_t page_cnt);

#endif