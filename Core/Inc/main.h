/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
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
#define RX_BUFFER_SIZE 128
#define RX_OVERTIME_MS 10
typedef struct {
  __IO uint8_t rxStartFlag;
  __IO uint8_t rxEndFlag;
  uint8_t rxData[2];
  uint8_t rxBuf[RX_BUFFER_SIZE];
  __IO uint8_t rxBufIndex;
  __IO uint8_t rxCounter;
  uint8_t rxSaveBuf[RX_BUFFER_SIZE];
  __IO uint32_t rxTick;
} uart_ctrl_t;

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
void RGB(uint8_t r, uint8_t g, uint8_t b);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_1_Pin GPIO_PIN_2
#define KEY_1_GPIO_Port GPIOE
#define KEY_2_Pin GPIO_PIN_3
#define KEY_2_GPIO_Port GPIOE
#define LED_R_Pin GPIO_PIN_6
#define LED_R_GPIO_Port GPIOG
#define LED_G_Pin GPIO_PIN_7
#define LED_G_GPIO_Port GPIOG
#define LED_B_Pin GPIO_PIN_8
#define LED_B_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
