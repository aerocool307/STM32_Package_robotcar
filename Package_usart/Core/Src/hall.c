
/* hall.c */

#include "hall.h"
#include "stm32f7xx_hal.h"
#include <string.h>
#include <stdio.h>

GPIO_TypeDef* HALL_GPIO_PORTS[HALL_COUNT] = {
    HALL_GPIO_PORT_0, HALL_GPIO_PORT_1, HALL_GPIO_PORT_2, HALL_GPIO_PORT_3,
    HALL_GPIO_PORT_4, HALL_GPIO_PORT_5, HALL_GPIO_PORT_6, HALL_GPIO_PORT_7
};

uint16_t HALL_PINS[HALL_COUNT] = {
    HALL_GPIO_PIN_0, HALL_GPIO_PIN_1, HALL_GPIO_PIN_2, HALL_GPIO_PIN_3,
    HALL_GPIO_PIN_4, HALL_GPIO_PIN_5, HALL_GPIO_PIN_6, HALL_GPIO_PIN_7
};

extern UART_HandleTypeDef huart3;


uint8_t hall_values[HALL_COUNT];
char uart_buffer[64];

// minden motorhoz külön számláló és időbélyeg
volatile uint32_t hall_pulse_count[MOTOR_COUNT] = {0};
uint32_t last_pulse_count[MOTOR_COUNT] = {0};
uint32_t last_time_ms[MOTOR_COUNT] = {0};


void Hall_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	    __HAL_RCC_GPIOE_CLK_ENABLE();
	    __HAL_RCC_GPIOF_CLK_ENABLE();

	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_PULLUP;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	    for (int i = 0; i < HALL_COUNT; i++)
	    {
	        GPIO_InitStruct.Pin = HALL_PINS[i];
	        HAL_GPIO_Init(HALL_GPIO_PORTS[i], &GPIO_InitStruct);
	    }

}

void Hall_ReadAll(uint8_t* values)
{
	for (int i = 0; i < HALL_COUNT; i++)
	    {
	        values[i] = HAL_GPIO_ReadPin(HALL_GPIO_PORTS[i], HALL_PINS[i]);
	    }
}

void Hall_DebugPrint(void)
{
	Hall_ReadAll(hall_values);

	    int len = snprintf(uart_buffer, sizeof(uart_buffer), "HALL: ");
	    for (int i = 0; i < HALL_COUNT; i++)
	    {
	        len += snprintf(uart_buffer + len, sizeof(uart_buffer) - len, "%d ", hall_values[i]);
	    }
	    uart_buffer[len++] = '\r';
	    uart_buffer[len++] = '\n';

	    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
}

void Hall_UpdateCounter(uint8_t hall_index)
{
    if (hall_index >= HALL_COUNT) return;

    // Minden pár Hall (A/B) egy motorhoz tartozik:
    uint8_t motor_index = hall_index / 2;  // 0–3
    hall_pulse_count[motor_index]++;
}

float Get_Hall_Speed(uint8_t motor_index)
{
    if (motor_index >= MOTOR_COUNT) return -1;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - last_time_ms[motor_index];
    if (elapsed < 100) return -1;  // min. 100ms mérési idő

    uint32_t pulses = hall_pulse_count[motor_index] - last_pulse_count[motor_index];

    last_pulse_count[motor_index] = hall_pulse_count[motor_index];
    last_time_ms[motor_index] = now;

    float revolutions = (float)pulses / IMPULSES_PER_REV;
    float rpm = revolutions * (60000.0f / (float)elapsed);
    return rpm;
}
