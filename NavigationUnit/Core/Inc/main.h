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
#define NU_LED3_Pin GPIO_PIN_13
#define NU_LED3_GPIO_Port GPIOC
#define NU_LED2_Pin GPIO_PIN_14
#define NU_LED2_GPIO_Port GPIOC
#define NU_LED1_Pin GPIO_PIN_15
#define NU_LED1_GPIO_Port GPIOC
#define NU_SPI1_FLASH_CS_Pin GPIO_PIN_4
#define NU_SPI1_FLASH_CS_GPIO_Port GPIOC
#define NU_BARO_INT_Pin GPIO_PIN_5
#define NU_BARO_INT_GPIO_Port GPIOC
#define NU_MAG_INT_Pin GPIO_PIN_0
#define NU_MAG_INT_GPIO_Port GPIOB
#define NU_IMU_GYRO_INT_Pin GPIO_PIN_1
#define NU_IMU_GYRO_INT_GPIO_Port GPIOB
#define NU_IMU_ACCEL_INT_Pin GPIO_PIN_2
#define NU_IMU_ACCEL_INT_GPIO_Port GPIOB
#define NU_SPI2_IMU_CS_Pin GPIO_PIN_12
#define NU_SPI2_IMU_CS_GPIO_Port GPIOB
#define NU_GPIOA_Pin GPIO_PIN_14
#define NU_GPIOA_GPIO_Port GPIOB
#define NU_GPIOB_Pin GPIO_PIN_15
#define NU_GPIOB_GPIO_Port GPIOB
#define NU_GPIOC_Pin GPIO_PIN_6
#define NU_GPIOC_GPIO_Port GPIOC
#define NU_GPIOD_Pin GPIO_PIN_7
#define NU_GPIOD_GPIO_Port GPIOC
#define NU_GPIOE_Pin GPIO_PIN_8
#define NU_GPIOE_GPIO_Port GPIOC
#define NU_GPIOF_Pin GPIO_PIN_11
#define NU_GPIOF_GPIO_Port GPIOA
#define NU_GPIOG_Pin GPIO_PIN_12
#define NU_GPIOG_GPIO_Port GPIOA
#define NU_GPS_PSS_Pin GPIO_PIN_12
#define NU_GPS_PSS_GPIO_Port GPIOC
#define NU_GPS_NRST_Pin GPIO_PIN_2
#define NU_GPS_NRST_GPIO_Port GPIOD
#define NU_GPS_LNA_EN_Pin GPIO_PIN_3
#define NU_GPS_LNA_EN_GPIO_Port GPIOB
#define NU_LED4_Pin GPIO_PIN_9
#define NU_LED4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
