/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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
#define M1HALLA_Pin GPIO_PIN_2
#define M1HALLA_GPIO_Port GPIOE
#define M1HALLA_EXTI_IRQn EXTI2_IRQn
#define M1HALLB_Pin GPIO_PIN_3
#define M1HALLB_GPIO_Port GPIOE
#define M1HALLB_EXTI_IRQn EXTI3_IRQn
#define M2HALLA_Pin GPIO_PIN_4
#define M2HALLA_GPIO_Port GPIOE
#define M2HALLA_EXTI_IRQn EXTI4_IRQn
#define M2HALLB_Pin GPIO_PIN_5
#define M2HALLB_GPIO_Port GPIOE
#define M2HALLB_EXTI_IRQn EXTI9_5_IRQn
#define M3HALLA_Pin GPIO_PIN_6
#define M3HALLA_GPIO_Port GPIOE
#define M3HALLA_EXTI_IRQn EXTI9_5_IRQn
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define M3HALLB_Pin GPIO_PIN_7
#define M3HALLB_GPIO_Port GPIOF
#define M3HALLB_EXTI_IRQn EXTI9_5_IRQn
#define M4HALLA_Pin GPIO_PIN_8
#define M4HALLA_GPIO_Port GPIOF
#define M4HALLA_EXTI_IRQn EXTI9_5_IRQn
#define M4HALLB_Pin GPIO_PIN_9
#define M4HALLB_GPIO_Port GPIOF
#define M4HALLB_EXTI_IRQn EXTI9_5_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define Motor_shield_Latch_Pin GPIO_PIN_6
#define Motor_shield_Latch_GPIO_Port GPIOA
#define M1PWM_Pin GPIO_PIN_7
#define M1PWM_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define Motor_shield_Data_Pin GPIO_PIN_12
#define Motor_shield_Data_GPIO_Port GPIOF
#define Motor_shield_Enable_Pin GPIO_PIN_13
#define Motor_shield_Enable_GPIO_Port GPIOF
#define Motor_shield_Clock_Pin GPIO_PIN_14
#define Motor_shield_Clock_GPIO_Port GPIOF
#define LORA_RX_Pin GPIO_PIN_7
#define LORA_RX_GPIO_Port GPIOE
#define LORA_TX_Pin GPIO_PIN_8
#define LORA_TX_GPIO_Port GPIOE
#define M3PWM_Pin GPIO_PIN_9
#define M3PWM_GPIO_Port GPIOE
#define Sensor_Back_Trig_Pin GPIO_PIN_10
#define Sensor_Back_Trig_GPIO_Port GPIOE
#define M4PWM_Pin GPIO_PIN_11
#define M4PWM_GPIO_Port GPIOE
#define Sensor_Left_Trig_Pin GPIO_PIN_12
#define Sensor_Left_Trig_GPIO_Port GPIOE
#define M2PWM_Pin GPIO_PIN_13
#define M2PWM_GPIO_Port GPIOE
#define Sensor_Front_Trig_Pin GPIO_PIN_14
#define Sensor_Front_Trig_GPIO_Port GPIOE
#define Sensor_Right_Trig_Pin GPIO_PIN_15
#define Sensor_Right_Trig_GPIO_Port GPIOE
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define Com_port_tx_Pin GPIO_PIN_8
#define Com_port_tx_GPIO_Port GPIOD
#define Com_port_rx_Pin GPIO_PIN_9
#define Com_port_rx_GPIO_Port GPIOD
#define LoRa_RESET_Pin GPIO_PIN_13
#define LoRa_RESET_GPIO_Port GPIOD
#define Sensor_Front_Servo_Pin GPIO_PIN_14
#define Sensor_Front_Servo_GPIO_Port GPIOD
#define STEPPER_MOTOR_Pin GPIO_PIN_15
#define STEPPER_MOTOR_GPIO_Port GPIOD
#define SD_CARD_CS_Pin GPIO_PIN_2
#define SD_CARD_CS_GPIO_Port GPIOG
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define Sensor_Front_Echo_Pin GPIO_PIN_6
#define Sensor_Front_Echo_GPIO_Port GPIOC
#define Sensor_Back_Echo_Pin GPIO_PIN_7
#define Sensor_Back_Echo_GPIO_Port GPIOC
#define Sensor_Left_Echo_Pin GPIO_PIN_8
#define Sensor_Left_Echo_GPIO_Port GPIOC
#define Sensor_Right_Echo_Pin GPIO_PIN_9
#define Sensor_Right_Echo_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define T_CAMERA_RX_Pin GPIO_PIN_10
#define T_CAMERA_RX_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define NEO_M8N_PPS_Pin GPIO_PIN_11
#define NEO_M8N_PPS_GPIO_Port GPIOC
#define NEO_M8N_TX_Pin GPIO_PIN_12
#define NEO_M8N_TX_GPIO_Port GPIOC
#define NEO_M8N_RX_Pin GPIO_PIN_2
#define NEO_M8N_RX_GPIO_Port GPIOD
#define SIM800H_PWRKY_Pin GPIO_PIN_4
#define SIM800H_PWRKY_GPIO_Port GPIOD
#define SIM800H_TX_Pin GPIO_PIN_5
#define SIM800H_TX_GPIO_Port GPIOD
#define SIM800H_RX_Pin GPIO_PIN_6
#define SIM800H_RX_GPIO_Port GPIOD
#define SIM800H_RESET_Pin GPIO_PIN_7
#define SIM800H_RESET_GPIO_Port GPIOD
#define D0_RX_Pin GPIO_PIN_9
#define D0_RX_GPIO_Port GPIOG
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define D1_TX_Pin GPIO_PIN_14
#define D1_TX_GPIO_Port GPIOG
#define T_CAMERA_TX_Pin GPIO_PIN_6
#define T_CAMERA_TX_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
