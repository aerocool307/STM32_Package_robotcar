
/* servo.h */


#ifndef INC_SERVO_H_
#define INC_SERVO_H_
#include "stm32f7xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} SERVO_Handle_t;

// Globálisan használt példány (pl. front szervó)
extern SERVO_Handle_t front_servo;

void Servo_Init(void);  // AppInit-féle függvény servo.c-ben
void Servo_SetAngle(SERVO_Handle_t *servo, int8_t angle);

#endif /* INC_SERVO_H_ */

