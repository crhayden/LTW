/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void JumpToBootloader();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define YAXIS_CS__Pin GPIO_PIN_3
#define YAXIS_CS__GPIO_Port GPIOE
#define FAXIS_EN__Pin GPIO_PIN_5
#define FAXIS_EN__GPIO_Port GPIOE
#define FAXIS_DIR_Pin GPIO_PIN_6
#define FAXIS_DIR_GPIO_Port GPIOE
#define OSC32K_IN_Pin GPIO_PIN_14
#define OSC32K_IN_GPIO_Port GPIOC
#define OSC32K_O_Pin GPIO_PIN_15
#define OSC32K_O_GPIO_Port GPIOC
#define OSC24M_I_Pin GPIO_PIN_0
#define OSC24M_I_GPIO_Port GPIOH
#define OSC24M_O_Pin GPIO_PIN_1
#define OSC24M_O_GPIO_Port GPIOH
#define VBUS_SNSF_Pin GPIO_PIN_0
#define VBUS_SNSF_GPIO_Port GPIOC
#define EN_USB2_Pin GPIO_PIN_1
#define EN_USB2_GPIO_Port GPIOC
#define EN_USB3_Pin GPIO_PIN_2
#define EN_USB3_GPIO_Port GPIOC
#define EM_STOP_Pin GPIO_PIN_3
#define EM_STOP_GPIO_Port GPIOC
#define STEPPER_RESET__Pin GPIO_PIN_1
#define STEPPER_RESET__GPIO_Port GPIOA
#define YDIAG0_Pin GPIO_PIN_2
#define YDIAG0_GPIO_Port GPIOA
#define YDIAG1_Pin GPIO_PIN_3
#define YDIAG1_GPIO_Port GPIOA
#define ZDIAG0_Pin GPIO_PIN_4
#define ZDIAG0_GPIO_Port GPIOA
#define ZDIAG1_Pin GPIO_PIN_5
#define ZDIAG1_GPIO_Port GPIOA
#define ZAXIS_STEP_Pin GPIO_PIN_6
#define ZAXIS_STEP_GPIO_Port GPIOA
#define YAXIS_STEP_Pin GPIO_PIN_7
#define YAXIS_STEP_GPIO_Port GPIOA
#define LD_2_Pin GPIO_PIN_9
#define LD_2_GPIO_Port GPIOE
#define FAXIS_CS__Pin GPIO_PIN_10
#define FAXIS_CS__GPIO_Port GPIOE
#define SPI_SCK_Pin GPIO_PIN_12
#define SPI_SCK_GPIO_Port GPIOE
#define SPI_MISO_Pin GPIO_PIN_13
#define SPI_MISO_GPIO_Port GPIOE
#define SPI_MOSI_Pin GPIO_PIN_14
#define SPI_MOSI_GPIO_Port GPIOE
#define ZAXIS_CS__Pin GPIO_PIN_15
#define ZAXIS_CS__GPIO_Port GPIOE
#define FE_RUN_Pin GPIO_PIN_10
#define FE_RUN_GPIO_Port GPIOB
#define SOLENOID_Pin GPIO_PIN_12
#define SOLENOID_GPIO_Port GPIOB
#define MCU_5V_LSR_Pin GPIO_PIN_13
#define MCU_5V_LSR_GPIO_Port GPIOB
#define MCU_GALVO_Pin GPIO_PIN_14
#define MCU_GALVO_GPIO_Port GPIOB
#define MCU_24V_LASER_Pin GPIO_PIN_15
#define MCU_24V_LASER_GPIO_Port GPIOB
#define Y_BACK__Pin GPIO_PIN_8
#define Y_BACK__GPIO_Port GPIOD
#define ZAXIS_EN__Pin GPIO_PIN_12
#define ZAXIS_EN__GPIO_Port GPIOD
#define ZAXIS_DIR_Pin GPIO_PIN_13
#define ZAXIS_DIR_GPIO_Port GPIOD
#define MODEM_RESET__Pin GPIO_PIN_14
#define MODEM_RESET__GPIO_Port GPIOD
//#define LD_1_Pin GPIO_PIN_6
//#define LD_1_GPIO_Port GPIOC
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_OTG_FS_N_Pin GPIO_PIN_11
#define USB_OTG_FS_N_GPIO_Port GPIOA
#define USB_OTG_FS_P_Pin GPIO_PIN_12
#define USB_OTG_FS_P_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define TO_PC_Pin GPIO_PIN_10
#define TO_PC_GPIO_Port GPIOC
#define FROM_PC_Pin GPIO_PIN_11
#define FROM_PC_GPIO_Port GPIOC
#define OPTO_RT_LS_Pin GPIO_PIN_0
#define OPTO_RT_LS_GPIO_Port GPIOD
#define OPTO_LT_LS_Pin GPIO_PIN_1
#define OPTO_LT_LS_GPIO_Port GPIOD
#define INTER_LS_Pin GPIO_PIN_2
#define INTER_LS_GPIO_Port GPIOD
#define HOME__Pin GPIO_PIN_3
#define HOME__GPIO_Port GPIOD
#define TAG_DOWN__Pin GPIO_PIN_4
#define TAG_DOWN__GPIO_Port GPIOD
#define LIMIT__Pin GPIO_PIN_5
#define LIMIT__GPIO_Port GPIOD
#define TAG_UP__Pin GPIO_PIN_6
#define TAG_UP__GPIO_Port GPIOD
#define Y_FRONT__Pin GPIO_PIN_7
#define Y_FRONT__GPIO_Port GPIOD
#define FDIAG0_Pin GPIO_PIN_3
#define FDIAG0_GPIO_Port GPIOB
#define FDIAG1_Pin GPIO_PIN_4
#define FDIAG1_GPIO_Port GPIOB
#define FE_WRN_S_Pin GPIO_PIN_5
#define FE_WRN_S_GPIO_Port GPIOB
#define FE_STAT_S_Pin GPIO_PIN_6
#define FE_STAT_S_GPIO_Port GPIOB
#define FAXIS_STEP_Pin GPIO_PIN_9
#define FAXIS_STEP_GPIO_Port GPIOB
#define YAXIS_EN__Pin GPIO_PIN_0
#define YAXIS_EN__GPIO_Port GPIOE
#define YAXIS_DIR_Pin GPIO_PIN_1
#define YAXIS_DIR_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
