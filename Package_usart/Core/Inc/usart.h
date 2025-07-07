/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart5;

extern UART_HandleTypeDef huart7;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define UART_BUFFER_SIZE 128

// USART1 – T-Camera
extern UART_HandleTypeDef huart1;
extern uint8_t uart1_rx_char;
extern char uart1_rx_buffer[UART_BUFFER_SIZE];
extern uint8_t uart1_rx_index;
extern volatile uint8_t uart1_rx_ready;

// USART2 – SIM800H
extern UART_HandleTypeDef huart2;
extern uint8_t uart2_rx_char;
extern char uart2_rx_buffer[UART_BUFFER_SIZE];
extern uint8_t uart2_rx_index;
extern volatile uint8_t uart2_rx_ready;

// USART3 – PC log
extern UART_HandleTypeDef huart3;
extern uint8_t uart3_rx_char;
extern char uart3_rx_buffer[UART_BUFFER_SIZE];
extern uint8_t uart3_rx_index;
extern volatile uint8_t uart3_rx_ready;

// UART5 – NEO-M8N GPS
extern UART_HandleTypeDef huart5;
extern uint8_t uart5_rx_char;
extern char uart5_rx_buffer[UART_BUFFER_SIZE];
extern uint8_t uart5_rx_index;
extern volatile uint8_t uart5_rx_ready;

// USART6 – MotorShield
extern UART_HandleTypeDef huart6;
extern uint8_t uart6_rx_char;
extern char uart6_rx_buffer[UART_BUFFER_SIZE];
extern uint8_t uart6_rx_index;
extern volatile uint8_t uart6_rx_ready;

// UART7 – LoRa (Bluetooth BLE)
extern UART_HandleTypeDef huart7;
extern uint8_t uart7_rx_char;
extern char uart7_rx_buffer[UART_BUFFER_SIZE];
extern uint8_t uart7_rx_index;
extern volatile uint8_t uart7_rx_ready;

/* USER CODE END Private defines */

void MX_UART5_Init(void);
void MX_UART7_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void UartDispatcher_Process(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

