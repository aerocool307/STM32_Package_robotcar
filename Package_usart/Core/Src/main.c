/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "hall.h"
#include "ultrasonic.h"
#include "servo.h"
#include "fatfs_sd.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* --- UART fogadó bufferek --- */
#define RX_BUFFER_SIZE 64
uint8_t rx_char;
char uart_rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
volatile uint8_t uart_rx_ready = 0;
uint8_t bluetooth_rx_byte;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Green_led_blink(void);
void Hall_GPIO_Init(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_FATFS_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_UART7_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  Servo_Init(); // A PWM indítása
  Ultrasonic_Init(); // Az IC bemenetek indítása

  HAL_UART_Receive_IT(&huart3, &rx_char, 1);  // UART RX IT indítása

      char welcome[] = "USART3 ready. Type a command:\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)welcome, strlen(welcome), HAL_MAX_DELAY);

  Motor_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Green_led_blink();
	  //hall szenzorok
	 // Hall_read_usart3();

	  Ultrasonic_SendDistanceUART(0); // Front szenzor
	      Ultrasonic_SendDistanceUART(1); // Hátsó
	      Ultrasonic_SendDistanceUART(2); // Bal
	      Ultrasonic_SendDistanceUART(3); // Jobb

	      HAL_Delay(1000);


	  HAL_UART_Receive_IT(&huart3, &rx_char, 1);         // UART3 fogadás (PC/parancs)
	  HAL_UART_Receive_IT(&huart2, &bluetooth_rx_byte, 1); // UART2 fogadás (Bluetooth)

	  //com-port message
	      /*
	  if (uart_rx_ready)
	          {
	              uart_rx_ready = 0;

	              // Feldolgozás: enter utáni parancs
	              if (strcmp(uart_rx_buffer, "hello") == 0)
	              {
	                  HAL_UART_Transmit(&huart3, (uint8_t*)"Hello to you too!\r\n", 20, HAL_MAX_DELAY);
	              }
	              else if (strcmp(uart_rx_buffer, "start") == 0)
	              {
	                  HAL_UART_Transmit(&huart3, (uint8_t*)"Start parancs elfogadva.\r\n", 27, HAL_MAX_DELAY);
	              }
	              else
	              {
	                  HAL_UART_Transmit(&huart3, (uint8_t*)"Ismeretlen parancs.\r\n", 22, HAL_MAX_DELAY);
	              }
	          }
*/
	 /* for (uint8_t i = 0; i < 4; i++) {
	          float dist = Ultrasonic_ReadDistance(i);
	          printf("Szenzor %d: %.2f cm\r\n", i, dist); // @suppress("Float formatting support")
	      }
	      HAL_Delay(250);
*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Green_led_blink(void)
{
	// PB0 villog
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(500);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // UART3 – parancsalapú kézi vezérlés
    if (huart->Instance == USART3)
    {
        if (rx_char != '\r' && rx_char != '\n' && rx_index < RX_BUFFER_SIZE - 1)
        {
            uart_rx_buffer[rx_index++] = rx_char;
        }
        else
        {
            uart_rx_buffer[rx_index] = '\0';  // lezárjuk a stringet
            rx_index = 0;
            uart_rx_ready = 1;

            // közvetlen feldolgozás (opcionális):
            MotorControl_HandleInput((uint8_t *)uart_rx_buffer[0]);  // vagy főciklusban
        }

        HAL_UART_Receive_IT(&huart3, &rx_char, 1);
    }

    // UART2 – Bluetooth + PID vezérlés
    if (huart->Instance == USART2)
    {
        MotorControl_HandleBluetooth(bluetooth_rx_byte);
        HAL_UART_Receive_IT(&huart2, &bluetooth_rx_byte, 1);
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        case GPIO_PIN_0: Hall_UpdateCounter(0); break;
        case GPIO_PIN_1: Hall_UpdateCounter(1); break;
        case GPIO_PIN_2: Hall_UpdateCounter(2); break;
        case GPIO_PIN_3: Hall_UpdateCounter(3); break;
        case GPIO_PIN_4: Hall_UpdateCounter(4); break;
        case GPIO_PIN_5: Hall_UpdateCounter(5); break;
        case GPIO_PIN_6: Hall_UpdateCounter(6); break;
        case GPIO_PIN_7: Hall_UpdateCounter(7); break;
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  printf(">> ERROR HANDLER <<\r\n");
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	printf("Wrong parameters value: file %s on line %d\r\n", file, line);
		 while (1)
		   {
			 printf(">> ERROR ASSERT <<\r\n");
		     // Hiba esetén itt megáll a program
		   }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
