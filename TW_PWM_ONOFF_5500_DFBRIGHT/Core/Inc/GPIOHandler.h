#ifndef __GPIO_HANDLER_H__
#define __GPIO_HANDLER_H__

#include "stdint.h"

extern uint16_t brightness_LD1;//variable for duty% setting, LD1 --> Usable range of 4500 - 8100....< 3000 is off, 8100 max
extern uint16_t brightness_LD2;//variable for duty% setting, LD2 --> Usable range of 4500 - 8100....< 3000 is off, 8100 max

void setGPIOState(uint16_t mask);   // Set the GPIO bits per mask
uint16_t getGPIOOutputState(void);  // Get the last mask we output
uint16_t getGPIOInputState(void);   // Get a mask of digital inputs


#endif
