/*
 * servo.h
 *
 *  Created on: Jun 18, 2025
 *      Author: Qwerty
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

extern TIM_HandleTypeDef htim4; // ha pl. a PD14 (TIM4_CH3) lábon van a szervo

void Servo_Init(void);
void Servo_SetAngle(uint8_t angle);

#endif /* INC_SERVO_H_ */
