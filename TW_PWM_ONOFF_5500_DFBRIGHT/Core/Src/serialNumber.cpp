#include "serialNumber.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// Put the serial number at the end of the flash.
// IF WE EVER NEED TO USE THOSE LAST 128KB, THIS ALL GOES TO HELL
#define BASE_ADDRESS 0x08060000
#define BASE_SECTOR 7


void flash_write(uint32_t address, uint32_t data)
{
    HAL_FLASH_Unlock();
    FLASH_Erase_Sector(BASE_SECTOR, VOLTAGE_RANGE_1);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);
    HAL_FLASH_Lock();
}

void setSerialNumber(uint32_t serial)
{
    flash_write(BASE_ADDRESS, serial);
}

uint32_t getSerialNumber()
{
    return *(uint32_t *)BASE_ADDRESS;
}
