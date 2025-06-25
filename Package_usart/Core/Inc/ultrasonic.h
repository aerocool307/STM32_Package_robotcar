
/* ultrasonic.h */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#include "stm32f7xx_hal.h"


//Front
#define US1_TRIG_GPIO GPIOE
#define US1_TRIG_PIN  GPIO_PIN_14
#define US1_ECHO_GPIO GPIOC
#define US1_ECHO_PIN  GPIO_PIN_6
#define US1_SERVO_GPIO GPIOD
#define US1_SERVO_PIN  GPIO_PIN_14
//Back
#define US2_TRIG_GPIO GPIOE
#define US2_TRIG_PIN  GPIO_PIN_10
#define US2_ECHO_GPIO GPIOC
#define US2_ECHO_PIN  GPIO_PIN_7
//Left
#define US3_TRIG_GPIO GPIOE
#define US3_TRIG_PIN  GPIO_PIN_12
#define US3_ECHO_GPIO GPIOC
#define US3_ECHO_PIN  GPIO_PIN_8
//Right
#define US4_TRIG_GPIO GPIOE
#define US4_TRIG_PIN  GPIO_PIN_15
#define US4_ECHO_GPIO GPIOC
#define US4_ECHO_PIN  GPIO_PIN_9

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void Ultrasonic_Init(void);
float Ultrasonic_ReadDistance(uint8_t index);
float Ultrasonic_SingleMeasure(uint8_t index);
void Ultrasonic_SendDistanceUART(uint8_t index);

#endif /* INC_ULTRASONIC_H_ */
