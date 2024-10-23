#ifndef __RESET_HANDLER_H__
#define __RESET_HANDLER_H__

#include "stdint.h"
#include "stm32f4xx.h"
#include "stdbool.h"

enum RESET_STATE
{
    RESET_STATE_UNINIT,
    RESET_STATE_IDLE,
    RESET_STATE_RESETTING,
};

class ResetHandler
{
public:
    ResetHandler();
    void initializeCold(int resetPin, GPIO_TypeDef *resetPort, uint32_t resetTime);
    void reset();
    void run();

private:
    RESET_STATE state;
    int rPin;                           // GPIO pin we are watching
    GPIO_TypeDef *rPort;                // GPIO Port we are watching
    uint32_t rTime;                     // Time to wait for reset in ms
    uint32_t resetStartTime;            // Time at which the reset was requested
};

extern ResetHandler usbResetHandler;
extern ResetHandler modemResetHandler;
void runResetHandlers();


#endif
