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
#include "stm32f1xx_hal.h"

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
#define Percepcion_RangeUS016_Pin GPIO_PIN_0
#define Percepcion_RangeUS016_GPIO_Port GPIOA
#define Energetico_BatVoltage_Pin GPIO_PIN_1
#define Energetico_BatVoltage_GPIO_Port GPIOA
#define Locomocion_MotorPWM_Derecha_Pin GPIO_PIN_2
#define Locomocion_MotorPWM_Derecha_GPIO_Port GPIOA
#define Locomocion_MotorDIR_Derecha1_Pin GPIO_PIN_3
#define Locomocion_MotorDIR_Derecha1_GPIO_Port GPIOA
#define Locomocion_MotorDIR_Derecha2_Pin GPIO_PIN_4
#define Locomocion_MotorDIR_Derecha2_GPIO_Port GPIOA
#define Locomocion_MotorDIR_Izquierda1_Pin GPIO_PIN_6
#define Locomocion_MotorDIR_Izquierda1_GPIO_Port GPIOA
#define Locomocion_MotorDIR_Izquierda2_Pin GPIO_PIN_7
#define Locomocion_MotorDIR_Izquierda2_GPIO_Port GPIOA
#define Percepcion_ColorOUT_Pin GPIO_PIN_0
#define Percepcion_ColorOUT_GPIO_Port GPIOB
#define Recoleccion_Servo1_Pin GPIO_PIN_1
#define Recoleccion_Servo1_GPIO_Port GPIOB
#define Indicadores_LedRed_Pin GPIO_PIN_10
#define Indicadores_LedRed_GPIO_Port GPIOB
#define Indicadores_LedGreen_Pin GPIO_PIN_11
#define Indicadores_LedGreen_GPIO_Port GPIOB
#define Indicadores_LedBlue_Pin GPIO_PIN_12
#define Indicadores_LedBlue_GPIO_Port GPIOB
#define Locomocion_MotorPWM_Izquierda_Pin GPIO_PIN_8
#define Locomocion_MotorPWM_Izquierda_GPIO_Port GPIOA
#define Comunicacion_BluetoothTX_Pin GPIO_PIN_9
#define Comunicacion_BluetoothTX_GPIO_Port GPIOA
#define Comunicacion_BluetoothRX_Pin GPIO_PIN_10
#define Comunicacion_BluetoothRX_GPIO_Port GPIOA
#define Recoleccion_Servo2_Pin GPIO_PIN_3
#define Recoleccion_Servo2_GPIO_Port GPIOB
#define Recoleccion_Servo3_Pin GPIO_PIN_4
#define Recoleccion_Servo3_GPIO_Port GPIOB
#define Pantalla_SCL_Pin GPIO_PIN_6
#define Pantalla_SCL_GPIO_Port GPIOB
#define Pantalla_SDA_Pin GPIO_PIN_7
#define Pantalla_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
