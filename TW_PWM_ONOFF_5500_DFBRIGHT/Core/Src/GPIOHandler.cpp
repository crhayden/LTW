#include "GPIOHandler.h"
#include "kisPackets.h"
#include "gpio.h"
#include "tim.h"


static uint16_t gpioOutputState = 0; //SOLENOID IS ON DEFAULT
extern TIM_HandleTypeDef htim3;

void setGPIOState(uint16_t mask)
{
    gpioOutputState = mask;
    if ((mask & FE_RUN) == FE_RUN)
    {
        HAL_GPIO_WritePin(FE_RUN_GPIO_Port, FE_RUN_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(FE_RUN_GPIO_Port, FE_RUN_Pin, GPIO_PIN_RESET);
    }
    if ((mask & EM_STOP) == EM_STOP)
    {
        HAL_GPIO_WritePin(EM_STOP_GPIO_Port, EM_STOP_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(EM_STOP_GPIO_Port, EM_STOP_Pin, GPIO_PIN_RESET);
    }
    if ((mask & LD_1) == LD_1)
    {
    	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,brightness_LD1);
    	HAL_Delay(10);
    }
    else
    {
    	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
    	HAL_Delay(10);
    }
    if ((mask & LD_2) == LD_2)
    {
    	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,brightness_LD2);
		HAL_Delay(10);
    }
    else
    {
    	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
		HAL_Delay(10);
    }
    // The solenoid's pin is inverted (0 = ON)
    if ((mask & SOLENOID) == SOLENOID)
    {
        HAL_GPIO_WritePin(SOLENOID_GPIO_Port, SOLENOID_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(SOLENOID_GPIO_Port, SOLENOID_Pin, GPIO_PIN_SET);
    }
}

uint16_t getGPIOInputState(void)
{
    uint16_t inputMask = 0;
    // TODO UPDATE PIN NAMES
    if (HAL_GPIO_ReadPin(YDIAG0_GPIO_Port, YDIAG0_Pin))
    {
        inputMask += YDIAG0;
    }
    if (HAL_GPIO_ReadPin(YDIAG1_GPIO_Port, YDIAG1_Pin))
    {
        inputMask += YDIAG1;
    }
    if (HAL_GPIO_ReadPin(Y_FRONT__GPIO_Port, Y_FRONT__Pin))
    {
        inputMask += Y_FRONT;
    }
    if (HAL_GPIO_ReadPin(Y_BACK__GPIO_Port, Y_BACK__Pin))
    {
        inputMask += Y_BACK;
    }
    if (HAL_GPIO_ReadPin(FDIAG0_GPIO_Port, FDIAG0_Pin))
    {
        inputMask += FDIAG0;
    }
    if (HAL_GPIO_ReadPin(FDIAG1_GPIO_Port, FDIAG1_Pin))
    {
        inputMask += FDIAG1;
    }
    if (HAL_GPIO_ReadPin(OPTO_LT_LS_GPIO_Port, OPTO_LT_LS_Pin))
    {
        inputMask += OPTO_LT;
    }
    if (HAL_GPIO_ReadPin(OPTO_RT_LS_GPIO_Port, OPTO_RT_LS_Pin))
    {
        inputMask += OPTO_RT;
    }
    if (HAL_GPIO_ReadPin(INTER_LS_GPIO_Port, INTER_LS_Pin))
    {
        inputMask += INTER_LS;
    }
    if (HAL_GPIO_ReadPin(HOME__GPIO_Port, HOME__Pin))
    {
        inputMask += HOME;
    }
    if (HAL_GPIO_ReadPin(TAG_DOWN__GPIO_Port, TAG_DOWN__Pin))
    {
        inputMask += TAG_DOWN;
    }
    if (HAL_GPIO_ReadPin(LIMIT__GPIO_Port, LIMIT__Pin))
    {
        inputMask += LIMIT;
    }
    if (HAL_GPIO_ReadPin(TAG_UP__GPIO_Port, TAG_UP__Pin))
    {
        inputMask += TAG_UP;
    }
    if (HAL_GPIO_ReadPin(FE_WRN_S_GPIO_Port, FE_WRN_S_Pin))
    {
        inputMask += FE_WRN;
    }
    if (HAL_GPIO_ReadPin(FE_STAT_S_GPIO_Port, FE_STAT_S_Pin))
    {
        inputMask += FE_STAT;
    }

    return (inputMask);
}

uint16_t getGPIOOutputState(void)
{
    return (gpioOutputState);
}
