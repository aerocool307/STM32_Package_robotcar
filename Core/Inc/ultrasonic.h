/*
 * ultrasonic.h
 *
 *  Created on: Jun 9, 2025
 *      Author: Qwerty
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#include "stm32f7xx_hal.h"

#define ULTRASONIC_COUNT 4

// Módosítsd az alábbi GPIO portokat és pineket a valós bekötés szerint
//Front
#define US1_TRIG_GPIO GPIOA
#define US1_TRIG_PIN  GPIO_PIN_3
#define US1_ECHO_GPIO GPIOE
#define US1_ECHO_PIN  GPIO_PIN_8
//Back
#define US2_TRIG_GPIO GPIOA
#define US2_TRIG_PIN  GPIO_PIN_4
#define US2_ECHO_GPIO GPIOE
#define US2_ECHO_PIN  GPIO_PIN_10
//Left
#define US3_TRIG_GPIO GPIOA
#define US3_TRIG_PIN  GPIO_PIN_2
#define US3_ECHO_GPIO GPIOE
#define US3_ECHO_PIN  GPIO_PIN_12
//Right
#define US4_TRIG_GPIO GPIOE
#define US4_TRIG_PIN  GPIO_PIN_15
#define US4_ECHO_GPIO GPIOE
#define US4_ECHO_PIN  GPIO_PIN_14

void Ultrasonic_Init(void);
float Ultrasonic_ReadDistance(uint8_t index);

#endif /* INC_ULTRASONIC_H_ */
