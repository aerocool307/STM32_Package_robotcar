
/*
 * hall.c
 *
 */

#include "hall.h"
#include "stm32f7xx_hal.h"


GPIO_TypeDef* HALL_GPIO_PORTS[HALL_COUNT] = {
    GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOD, GPIOD
};

uint16_t HALL_PINS[HALL_COUNT] = {
    GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5,
    GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_7, GPIO_PIN_3
};

void HALL_DebugPrint(void)
{

}

void Hall_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // GPIOE és GPIOD engedélyezése
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    // GPIOE
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // GPIOD
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void Hall_ReadAll(uint8_t* values)
{
    for (int i = 0; i < HALL_COUNT; i++)
    {
        values[i] = HAL_GPIO_ReadPin(HALL_GPIO_PORTS[i], HALL_PINS[i]);
    }
}
