
/* servo.c */


#include "servo.h"
#include "tim.h"  // TIM4 például itt definiált
#include "main.h"
#include <stdio.h>  // ha kell
#include "usart.h"  // ha myprintf uart-on küld

extern void myprintf(const char *fmt, ...);  // ha nem lenne deklarálva máshol
SERVO_Handle_t front_servo;
// Inicializálás: csak elmenti a paramétereket, PWM nem indul
void Servo_Init(void)
{
    front_servo.htim = &htim4;
    front_servo.channel = TIM_CHANNEL_3;

    // PWM-et nem indítjuk folyamatosan, csak amikor kell
}

// Szög beállítása: -90 és +90 fok között
void Servo_SetAngle(SERVO_Handle_t *servo, int8_t angle)
{
    if (angle < -20) angle = -20;
    if (angle > 20) angle = 20;

    uint16_t pulse_width_us = 1500 + (angle * 500) / 90;
    // Biztonsági határolás
    if (pulse_width_us < 1000) pulse_width_us = 1000;
    if (pulse_width_us > 2000) pulse_width_us = 2000;

    HAL_TIM_PWM_Start(servo->htim, servo->channel);// PWM indítása
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_width_us);
    HAL_Delay(400);  // pozíció elérésére idő
    HAL_TIM_PWM_Stop(servo->htim, servo->channel); // PWM leállítása
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    myprintf("Angle: %d°, PWM Pulse Width: %u us\r\n", angle, pulse_width_us);
}



