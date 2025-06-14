/*
 * ultasonic.c
 *
 */


// ultrasonic.c

#include "ultrasonic.h"

extern TIM_HandleTypeDef htim3;
 TIM_HandleTypeDef *ultrasonic_timer = &htim3;

void Ultrasonic_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Trigger pin-ek: kimenet
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = US1_TRIG_PIN;
    HAL_GPIO_Init(US1_TRIG_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = US2_TRIG_PIN;
    HAL_GPIO_Init(US2_TRIG_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = US3_TRIG_PIN;
    HAL_GPIO_Init(US3_TRIG_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = US4_TRIG_PIN;
    HAL_GPIO_Init(US4_TRIG_GPIO, &GPIO_InitStruct);

    // Echo pin-ek: bemenet
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Pin = US1_ECHO_PIN;
    HAL_GPIO_Init(US1_ECHO_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = US2_ECHO_PIN;
    HAL_GPIO_Init(US2_ECHO_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = US3_ECHO_PIN;
    HAL_GPIO_Init(US3_ECHO_GPIO, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = US4_ECHO_PIN;
    HAL_GPIO_Init(US4_ECHO_GPIO, &GPIO_InitStruct);
}

float Ultrasonic_ReadDistance(uint8_t index)
{
    GPIO_TypeDef *trig_gpio = NULL;
    uint16_t trig_pin = 0;
    GPIO_TypeDef *echo_gpio = NULL;
    uint16_t echo_pin = 0;

    switch (index)
    {
    case 0:
        trig_gpio = US1_TRIG_GPIO; trig_pin = US1_TRIG_PIN;
        echo_gpio = US1_ECHO_GPIO; echo_pin = US1_ECHO_PIN;
        break;
    case 1:
        trig_gpio = US2_TRIG_GPIO; trig_pin = US2_TRIG_PIN;
        echo_gpio = US2_ECHO_GPIO; echo_pin = US2_ECHO_PIN;
        break;
    case 2:
        trig_gpio = US3_TRIG_GPIO; trig_pin = US3_TRIG_PIN;
        echo_gpio = US3_ECHO_GPIO; echo_pin = US3_ECHO_PIN;
        break;
    case 3:
        trig_gpio = US4_TRIG_GPIO; trig_pin = US4_TRIG_PIN;
        echo_gpio = US4_ECHO_GPIO; echo_pin = US4_ECHO_PIN;
        break;
    default:
        return -1;
    }

    // Trigger jel 10µs
    HAL_GPIO_WritePin(trig_gpio, trig_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(trig_gpio, trig_pin, GPIO_PIN_SET);
    HAL_Delay(0.01);  // kb. 10µs
    HAL_GPIO_WritePin(trig_gpio, trig_pin, GPIO_PIN_RESET);

    // Várakozás ECHO HIGH-ra
    uint32_t timeout = 30000;
    while (HAL_GPIO_ReadPin(echo_gpio, echo_pin) == GPIO_PIN_RESET && timeout--);

    // Időmérés indítása
    __HAL_TIM_SET_COUNTER(ultrasonic_timer, 0);
    HAL_TIM_Base_Start(ultrasonic_timer);

    timeout = 30000;
    while (HAL_GPIO_ReadPin(echo_gpio, echo_pin) == GPIO_PIN_SET && timeout--);

    HAL_TIM_Base_Stop(ultrasonic_timer);
    uint32_t time = __HAL_TIM_GET_COUNTER(ultrasonic_timer);

    // Távolság számítása (cm) – hangsebesség alapján: t (us) / 58
    return (float)time / 58.0f;
}
