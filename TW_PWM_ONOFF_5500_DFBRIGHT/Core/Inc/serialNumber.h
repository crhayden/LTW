#ifndef __SERIAL_NUMBER_H__
#define __SERIAL_NUMBER_H__

#include "stdint.h"

void setSerialNumber(uint32_t serial);  // Set the serial number in a dword in flash
uint32_t getSerialNumber();             // Get the stored serial number from flash

#endif
