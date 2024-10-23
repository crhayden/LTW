/**
  ******************************************************************************
  * @file    motor.c
  * @brief   This file provides code for the configuration
  *          of the MOTOR instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "motor.h"

void configSteppers(void)
{
	HAL_GPIO_WritePin(GPIOA, STEPPER_RESET__Pin, SET);//TURN ON VCC_IO
	HAL_Delay(100);

	TMC2130InitStallGuard();
	HAL_Delay(100);
}
