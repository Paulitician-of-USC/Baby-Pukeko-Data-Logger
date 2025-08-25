/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define MT2_NSS_Pin GPIO_PIN_15
#define MT2_NSS_GPIO_Port GPIOA
#define LSM_NSS_Pin GPIO_PIN_9
#define LSM_NSS_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define MS_NSS_Pin GPIO_PIN_4
#define MS_NSS_GPIO_Port GPIOA
#define H3_NSS_Pin GPIO_PIN_6
#define H3_NSS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
typedef enum STATE_MACHINE{
	STATE_INIT,
	STATE_INIT_SUCCESS,
	STATE_MOD_CONFIG,
	STATE_READ_LOG,
	STATE_ARM,
	STATE_BOOST,
	STATE_BURNOUT,
	STATE_APOGEE,
	STATE_LANDED
}STATE_MACHINE;

typedef struct{
	uint8_t G_THRESHOLD;
	uint8_t THRESHOLD_COUNT;
	uint8_t WipeFlash;
}Parameters;

typedef struct{
    uint8_t	flight_num;  // NEW: first byte
    uint32_t time_ms;
    uint16_t packet_num;
    uint8_t  flag;
    float    temp_c;
    float    pressure_pa;
    float    altitude_m;
    float    Hacc_x;
    float    Hacc_y;
    float    Hacc_z;
    float 	 Lacc_x;
    float 	 Lacc_y;
    float 	 Lacc_z;
    float    gyro_x;
    float    gyro_y;
    float    gyro_z;
} __attribute__((packed)) LogPacket;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
