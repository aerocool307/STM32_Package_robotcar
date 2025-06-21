/*
 * servo.c
 *
 */

#include "ultrasonic.h"
#include "main.h"
#include "servo.h"



void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // Indítsd el a PWM-et
}

void Servo_SetAngle(uint8_t angle) {
    // Átalakítás szervó PWM kitöltésre (0.5ms – 2.5ms: 0°–180°)
    uint32_t pulse = ((angle * (2500 - 500)) / 180) + 500; // µs

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse);
    HAL_Delay(300); // Idő a mozgáshoz
}
