/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.c
  * @brief   This file provides code for the configuration
  *          of the motor instances.
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <tmc2130.h>
#include "spi.h"

/* Declarations --------------------------------------------------------------*/



void TMC2130InitStallGuard(void)
{
    static uint8_t GCONF[5]       = {0x80,0x00,0x00,0x00,0x80};
    static uint8_t GCONF_FLIPPER[5]       = {0x80,0x00,0x00,0x00,0x04};
    static uint8_t COOLCONF[5]    = {0xED,0x00,0x00,0x00,0x00};
    static uint8_t TCOOLTHRS[5]   = {0x94,0x00,0x00,0x00,0x40};
    static uint8_t SHORCONF[5]    = {0x89,0x00,0x01,0x06,0x06};
    static uint8_t DRVCONF[5]     = {0x8A,0x00,0x08,0x04,0x00};
    static uint8_t TPWRDOWN[5]    = {0x91,0x00,0x00,0x00,0x0A};
    static uint8_t PWMCONF[5]     = {0xF0,0xC4,0x0C,0x00,0x1E};
    static uint8_t FLIP_CHOP_CONF[5]   = {0xEC,0x34,0x41,0x01,0x53};
	static uint8_t YAXIS_CHOP_CONF[5]   = {0xEC,0x38,0x41,0x01,0x53};
	static uint8_t ZAXIS_CHOP_CONF[5]   = {0xEC,0x38,0x41,0x01,0x53};


    //stall guard configuration
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, GCONF, 5, 100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, COOLCONF, 5, 100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, TCOOLTHRS, 5, 100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, SHORCONF, 5, 100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, DRVCONF, 5, 100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, TPWRDOWN, 5, 100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, ZAXIS_CHOP_CONF, 5, 100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, PWMCONF, 5, 100);
	HAL_GPIO_WritePin(ZAXIS_CS__GPIO_Port, ZAXIS_CS__Pin, SET);
	HAL_Delay(100);

    //stall guard configuration
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, GCONF, 5, 100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, COOLCONF, 5, 100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, TCOOLTHRS, 5, 100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, SHORCONF, 5, 100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, DRVCONF, 5, 100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, TPWRDOWN, 5, 100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, YAXIS_CHOP_CONF, 5, 100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, PWMCONF, 5, 100);
	HAL_GPIO_WritePin(YAXIS_CS__GPIO_Port, YAXIS_CS__Pin, SET);
	HAL_Delay(100);

    //stall guard configuration
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, GCONF_FLIPPER, 5, 100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, COOLCONF, 5, 100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, TCOOLTHRS, 5, 100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, SHORCONF, 5, 100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, DRVCONF, 5, 100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, TPWRDOWN, 5, 100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, FLIP_CHOP_CONF, 5, 100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, RESET);
	HAL_SPI_Transmit(&hspi4, PWMCONF, 5, 100);
	HAL_GPIO_WritePin(FAXIS_CS__GPIO_Port, FAXIS_CS__Pin, SET);
	HAL_Delay(100);
}



