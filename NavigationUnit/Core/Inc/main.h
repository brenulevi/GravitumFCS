/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NU_IMU_ACCEL_INT_Pin GPIO_PIN_13
#define NU_IMU_ACCEL_INT_GPIO_Port GPIOC
#define NU_IMU_GYRO_INT_Pin GPIO_PIN_14
#define NU_IMU_GYRO_INT_GPIO_Port GPIOC
#define NU_BARO_INT_Pin GPIO_PIN_15
#define NU_BARO_INT_GPIO_Port GPIOC
#define NU_MAG_INT_Pin GPIO_PIN_0
#define NU_MAG_INT_GPIO_Port GPIOC
#define NU_SPI1_CS_FLASH_Pin GPIO_PIN_4
#define NU_SPI1_CS_FLASH_GPIO_Port GPIOA
#define NU_GPS_LNA_EN_Pin GPIO_PIN_4
#define NU_GPS_LNA_EN_GPIO_Port GPIOC
#define NU_GPS_NRST_Pin GPIO_PIN_5
#define NU_GPS_NRST_GPIO_Port GPIOC
#define NU_GPS_PSS_Pin GPIO_PIN_0
#define NU_GPS_PSS_GPIO_Port GPIOB
#define NU_SPI2_CS_IMU_Pin GPIO_PIN_12
#define NU_SPI2_CS_IMU_GPIO_Port GPIOB
#define NU_LED_4_Pin GPIO_PIN_14
#define NU_LED_4_GPIO_Port GPIOB
#define NU_LED_3_Pin GPIO_PIN_15
#define NU_LED_3_GPIO_Port GPIOB
#define NU_LED_2_Pin GPIO_PIN_6
#define NU_LED_2_GPIO_Port GPIOC
#define NU_LED_1_Pin GPIO_PIN_7
#define NU_LED_1_GPIO_Port GPIOC
#define NU_GPIO_F_Pin GPIO_PIN_15
#define NU_GPIO_F_GPIO_Port GPIOA
#define NU_GPIO_E_Pin GPIO_PIN_12
#define NU_GPIO_E_GPIO_Port GPIOC
#define NU_GPIO_D_Pin GPIO_PIN_2
#define NU_GPIO_D_GPIO_Port GPIOD
#define NU_GPIO_C_Pin GPIO_PIN_3
#define NU_GPIO_C_GPIO_Port GPIOB
#define NU_GPIO_B_Pin GPIO_PIN_4
#define NU_GPIO_B_GPIO_Port GPIOB
#define NU_GPIO_A_Pin GPIO_PIN_5
#define NU_GPIO_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
