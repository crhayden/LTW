#include "resetHandler.h"
#include "stm32f4xx.h"

ResetHandler usbResetHandler;
ResetHandler modemResetHandler;

ResetHandler::ResetHandler()
{
    state = RESET_STATE_UNINIT;
}

void ResetHandler::initializeCold(int resetPin, GPIO_TypeDef *resetPort, uint32_t resetTime)
{
    rPin = resetPin;
    rPort = resetPort;
    rTime = resetTime;
    HAL_GPIO_WritePin(rPort, rPin, GPIO_PIN_SET);
    state = RESET_STATE_IDLE;
}

void ResetHandler::reset()
{
    resetStartTime = HAL_GetTick();
    state = RESET_STATE_RESETTING;
    HAL_GPIO_WritePin(rPort, rPin, GPIO_PIN_RESET);
}

void  ResetHandler::run()
{
    if (state == RESET_STATE_RESETTING && HAL_GetTick() - resetStartTime > rTime)
    {
        HAL_GPIO_WritePin(rPort, rPin, GPIO_PIN_SET);
        state = RESET_STATE_IDLE;
    }
}

void runResetHandlers()
{
    usbResetHandler.run();
    modemResetHandler.run();
}
