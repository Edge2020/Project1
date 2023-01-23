#include "flash.h"

uint8_t writeData(uint32_t addr, uint32_t *data, uint32_t size) {
	uint8_t isOK = 1;
	
	HAL_FLASH_Unlock();
	
	if(eraseFlash(addr, ((size / 0x400) + 1)) != 0xFFFFFFFF) isOK = 0;
	
	if(isOK){
		for(uint32_t i = 0; i < size && isOK == 1; i++){
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4 * i, data[i]) != HAL_OK) isOK = 0;
		}
	}

	HAL_FLASH_Lock();
	
	return isOK;
}

void readData(uint32_t addr, uint32_t *sto, uint32_t size) {
	
	for(uint32_t i = 0; i < size; i++) sto[i] = *(__IO uint32_t *)(addr + 4 * i);
	
	
	
}

uint32_t eraseFlash(uint32_t addr, uint32_t page_cnt) {
	FLASH_EraseInitTypeDef erase;
	uint32_t err;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.NbPages = page_cnt;
	erase.PageAddress = addr;

	HAL_FLASHEx_Erase(&erase, &err);

	return err;
}