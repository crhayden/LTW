/**
  ******************************************************************************
  * @file    motor.h
  * @brief   This file contains all the function prototypes for
  *          the motor.c file
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

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tmc2130.h"

/* Function Prototypes -------------------------------------------------------*/
void configSteppers(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ADDRLED_H_ */
