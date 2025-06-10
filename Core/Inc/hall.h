
/*
 * hall.h
 *
 */

#ifndef INC_HALL_H_
#define INC_HALL_H_


#include "stm32f7xx_hal.h"

#define HALL_COUNT 8
// Hall-szenzor lábak deklarációja extern formában

extern GPIO_TypeDef* HALL_GPIO_PORTS[HALL_COUNT];
extern uint16_t HALL_PINS[HALL_COUNT];

// Hall-szenzor bemeneti pinek – igazítsd az aktuális kapcsolásodhoz
/*
#define HALL1_GPIO_Port GPIOE
#define HALL1_Pin       GPIO_PIN_2

#define HALL2_GPIO_Port GPIOE
#define HALL2_Pin       GPIO_PIN_3

#define HALL3_GPIO_Port GPIOE
#define HALL3_Pin       GPIO_PIN_4

#define HALL4_GPIO_Port GPIOE
#define HALL4_Pin       GPIO_PIN_5

#define HALL5_GPIO_Port GPIOE
#define HALL5_Pin       GPIO_PIN_6

#define HALL6_GPIO_Port GPIOE
#define HALL6_Pin       GPIO_PIN_7

#define HALL7_GPIO_Port GPIOD
#define HALL7_Pin       GPIO_PIN_7

#define HALL8_GPIO_Port GPIOD
#define HALL8_Pin       GPIO_PIN_3
*/

// Összes Hall-szenzor állapotának kiírása UART-ra (debug célra)
void HALL_DebugPrint(void);

void Hall_GPIO_Init(void);
void Hall_ReadAll(uint8_t* values);

#endif /* INC_HALL_H_ */
