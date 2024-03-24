/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f072b_discovery.h"
#include "stm32f072b_discovery_gyroscope.h"
#include "tsl_user.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* User can use this section to tailor TSCx/TSCx instance used and associated
   resources */
/* Definition for TSCx clock resources */
#define TSCx                                        TSC
#define TSCx_CLK_ENABLE()                           __HAL_RCC_TSC_CLK_ENABLE()
#define TSCx_TS1_IO_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define TSCx_TS2_IO_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define TSCx_TS3_IO_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define TSCx_TS1_CAPACITOR_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define TSCx_TS2_CAPACITOR_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define TSCx_TS3_CAPACITOR_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()

#define TSCx_FORCE_RESET()                          __HAL_RCC_TSC_FORCE_RESET()
#define TSCx_RELEASE_RESET()                        __HAL_RCC_TSC_RELEASE_RESET()

/* Definition for TSCx IO Pins */
#define TSCx_TS1_IO_PIN                             GPIO_PIN_2
#define TSCx_TS1_IO_GPIO_PORT                       GPIOA
#define TSCx_TS1_IO_AF                              GPIO_AF3_TSC

#define TSCx_TS2_IO_PIN                             GPIO_PIN_6
#define TSCx_TS2_IO_GPIO_PORT                       GPIOA
#define TSCx_TS2_IO_AF                              GPIO_AF3_TSC

#define TSCx_TS3_IO_PIN                             GPIO_PIN_0
#define TSCx_TS3_IO_GPIO_PORT                       GPIOB
#define TSCx_TS3_IO_AF                              GPIO_AF3_TSC

/* Definition for TSCx Sampling Capacitor Pins */
#define TSCx_TS1_SAMPLING_PIN                       GPIO_PIN_3
#define TSCx_TS1_SAMPLING_GPIO_PORT                 GPIOA
#define TSCx_TS1_SAMPLING_AF                        GPIO_AF3_TSC

#define TSCx_TS2_SAMPLING_PIN                       GPIO_PIN_7
#define TSCx_TS2_SAMPLING_GPIO_PORT                 GPIOA
#define TSCx_TS2_SAMPLING_AF                        GPIO_AF3_TSC

#define TSCx_TS3_SAMPLING_PIN                       GPIO_PIN_1
#define TSCx_TS3_SAMPLING_GPIO_PORT                 GPIOB
#define TSCx_TS3_SAMPLING_AF                        GPIO_AF3_TSC

/* Definition for TSCx's NVIC */
/* Not used in this example. Keep it for reference.
#define TSCx_EXTI_IRQn                              TSC_IRQn
#define TSCx_EXTI_IRQHandler                        TSC_IRQHandler */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NCS_MEMS_SPI_Pin GPIO_PIN_0
#define NCS_MEMS_SPI_GPIO_Port GPIOC
#define MEMS_INT1_Pin GPIO_PIN_1
#define MEMS_INT1_GPIO_Port GPIOC
#define MEMS_INT2_Pin GPIO_PIN_2
#define MEMS_INT2_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define EXT_RESET_Pin GPIO_PIN_5
#define EXT_RESET_GPIO_Port GPIOC
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_6
#define LD3_GPIO_Port GPIOC
#define LD6_Pin GPIO_PIN_7
#define LD6_GPIO_Port GPIOC
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD5_Pin GPIO_PIN_9
#define LD5_GPIO_Port GPIOC
#define USBF4_DM_Pin GPIO_PIN_11
#define USBF4_DM_GPIO_Port GPIOA
#define USBF4_DP_Pin GPIO_PIN_12
#define USBF4_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
