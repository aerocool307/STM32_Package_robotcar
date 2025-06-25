
/* motor.h */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_


#include "stm32f7xx_hal.h"

// STM32 GPIO defination
#define MOTORLATCH_GPIO_Port   GPIOA
#define MOTORLATCH_Pin         GPIO_PIN_6

#define MOTORCLK_GPIO_Port     GPIOF
#define MOTORCLK_Pin           GPIO_PIN_14

#define MOTORENABLE_GPIO_Port  GPIOF
#define MOTORENABLE_Pin        GPIO_PIN_13

#define MOTORDATA_GPIO_Port    GPIOF
#define MOTORDATA_Pin          GPIO_PIN_12

// Bit positions in shift register (74HC595)
#define MOTOR_A1_BIT  2 // Motor 1 IN1
#define MOTOR_A2_BIT  3 // Motor 1 IN2
#define MOTOR_B1_BIT  1 // Motor 2 IN1
#define MOTOR_B2_BIT  4 // Motor 2 IN2
#define MOTOR_C1_BIT  5 // Motor 3 IN1
#define MOTOR_C2_BIT  7 // Motor 3 IN2
#define MOTOR_D1_BIT  0 // Motor 4 IN1
#define MOTOR_D2_BIT  6 // Motor 4 IN2

// Hall szenzor adatai
#define HALL_COUNT 8
// Motorvezérlési struktúra
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim14;

// Motor irány enum
typedef enum {
    FORWARD,
    BACKWARD,
    RELEASE
} MotorDirection;

// Motor azonosítók
typedef enum {
    MOTOR_1,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4
} MotorID;

// Enum a motor állapothoz
typedef uint16_t MotorState;

// Get hall speed motor indexek
#define MOTOR_1_INDEX 0
#define MOTOR_2_INDEX 1
#define MOTOR_3_INDEX 2
#define MOTOR_4_INDEX 3

// Function prototypes
void Motor_Init(void);
void Motor_output(uint8_t output);
void Motor_Set(MotorID motor, MotorDirection command, uint16_t speed);
void Motor_SetSpeed(uint8_t motor_index, uint16_t speed);
void MotorControl_HandleInput(uint8_t byte);



#endif /* INC_MOTOR_H_ */




