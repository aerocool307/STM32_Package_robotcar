
/* hall.h */

#ifndef INC_HALL_H_
#define INC_HALL_H_

#include "stm32f7xx_hal.h"
#include <stdint.h>

#define HALL_COUNT 8
#define MOTOR_COUNT 4
#define IMPULSES_PER_REV 11  // 1 Hall jel alapján

#define HALL_GPIO_PORT_0   GPIOE
#define HALL_GPIO_PIN_0    GPIO_PIN_2

#define HALL_GPIO_PORT_1   GPIOE
#define HALL_GPIO_PIN_1    GPIO_PIN_3

#define HALL_GPIO_PORT_2   GPIOE
#define HALL_GPIO_PIN_2    GPIO_PIN_4

#define HALL_GPIO_PORT_3   GPIOE
#define HALL_GPIO_PIN_3    GPIO_PIN_5

#define HALL_GPIO_PORT_4   GPIOE
#define HALL_GPIO_PIN_4    GPIO_PIN_6

#define HALL_GPIO_PORT_5   GPIOF
#define HALL_GPIO_PIN_5    GPIO_PIN_7

#define HALL_GPIO_PORT_6   GPIOF
#define HALL_GPIO_PIN_6    GPIO_PIN_8

#define HALL_GPIO_PORT_7   GPIOF
#define HALL_GPIO_PIN_7    GPIO_PIN_9


void Hall_GPIO_Init(void);
void Hall_ReadAll(uint8_t* values);
void Hall_DebugPrint(void);// Összes Hall-szenzor állapotának kiírása UART-ra (debug célra)
float Get_Hall_Speed(uint8_t motor_index);  // Motor index: 0=M1, 1=M2, 2=M3, 3=M4
void Hall_UpdateCounter(uint8_t hall_index);  // EXTI-nél hívjuk meg

#endif /* INC_HALL_H_ */
