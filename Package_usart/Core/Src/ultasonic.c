
/* ultasonic.c */

#include "ultrasonic.h"
#include "main.h"
#include "servo.h"
#include <stdio.h> // sprintf-hez
#include <string.h>

extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart3;

#define NUM_SENSORS 4

extern void myprintf(const char *fmt, ...);
//myprintf("Distance: %.2f cm\r\n", distance);
static const uint32_t tim_channels[NUM_SENSORS] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4
};

static volatile uint32_t ic_start[NUM_SENSORS] = {0};
static volatile uint32_t ic_end[NUM_SENSORS] = {0};
static volatile uint8_t  capture_done[NUM_SENSORS] = {0};
static uint8_t polarity_state[NUM_SENSORS] = {0, 0, 0, 0}; // 0 = rising, 1 = falling

// TRIG lábak GPIO portjai és pinek (statikusan)
GPIO_TypeDef* trig_ports[NUM_SENSORS] = {GPIOE, GPIOE, GPIOE, GPIOE};
uint16_t trig_pins[NUM_SENSORS]       = {GPIO_PIN_14, GPIO_PIN_10, GPIO_PIN_12, GPIO_PIN_15};

// ECHO lábak GPIO portjai és pinek (statikusan)
GPIO_TypeDef* echo_ports[NUM_SENSORS] = {GPIOC, GPIOC, GPIOC, GPIOC};
uint16_t echo_pins[NUM_SENSORS]       = {GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9};



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

    uint8_t index;

    if (htim->Instance != TIM8) return;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) index = 0;
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) index = 1;
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) index = 2;
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) index = 3;
    else return;

    uint32_t channel = tim_channels[index];

    if (polarity_state[index] == 0) // Rising edge
        {
            ic_start[index] = HAL_TIM_ReadCapturedValue(htim, channel);
            HAL_TIM_IC_Stop_IT(htim, channel);
            TIM_IC_InitTypeDef sConfigIC = {0};
            sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
            sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
            sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
            sConfigIC.ICFilter = 0;
            HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, channel);
            HAL_TIM_IC_Start_IT(htim, channel);
            polarity_state[index] = 1;
        }
        else // Falling edge
        {
            ic_end[index] = HAL_TIM_ReadCapturedValue(htim, channel);
            HAL_TIM_IC_Stop_IT(htim, channel);
            TIM_IC_InitTypeDef sConfigIC = {0};
            sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
            sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
            sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
            sConfigIC.ICFilter = 0;
            HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, channel);
            HAL_TIM_IC_Start_IT(htim, channel);
            capture_done[index] = 1;
            polarity_state[index] = 0;
        }
}

void Ultrasonic_Init(void)
{
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_4);

}

float Ultrasonic_ReadDistance(uint8_t index)
{
	if (index > 3) return -1;

	    if (index == 0) {
	    	myprintf("Servo -> Left\n");
	        // Front szenzor – mozdítsuk el jobbra, balra, majd előre
	        float dist_left, dist_center, dist_right;

	        Servo_SetAngle(30);
	        dist_left = Ultrasonic_SingleMeasure(index);

	        Servo_SetAngle(90);
	        dist_center = Ultrasonic_SingleMeasure(index);

	        Servo_SetAngle(150);
	        dist_right = Ultrasonic_SingleMeasure(index);

	        Servo_SetAngle(90); // Vissza alapállásba

	        // Válasszuk ki a legkisebb távolságot, vagy átlagolhatunk is
	        return (dist_left + dist_center + dist_right) / 3.0f;
	    } else {

	        return Ultrasonic_SingleMeasure(index);
	    }
}

float Ultrasonic_SingleMeasure(uint8_t index)
{

    uint32_t channel = tim_channels[index];

    capture_done[index] = 0;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim8, channel, TIM_INPUTCHANNELPOLARITY_RISING);

    HAL_GPIO_WritePin(trig_ports[index], trig_pins[index], GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(trig_ports[index], trig_pins[index], GPIO_PIN_SET);
    HAL_Delay(0.01);  // 10 µs
    HAL_GPIO_WritePin(trig_ports[index], trig_pins[index], GPIO_PIN_RESET);

    uint32_t timeout = HAL_GetTick();
    while (!capture_done[index]) {
        if (HAL_GetTick() - timeout > 100) return -1;
    }

    uint32_t diff = (ic_end[index] >= ic_start[index])
                    ? (ic_end[index] - ic_start[index])
                    : (0xFFFF - ic_start[index] + ic_end[index]);

    return (float)diff * 0.0343f / 2.0f;
}

void Ultrasonic_SendDistanceUART(uint8_t index)
{

    float distance = Ultrasonic_ReadDistance(index);
    //HAL_UART_Transmit(&huart3, (uint8_t*)distance, 20, HAL_MAX_DELAY);
    char msg[64];

    if (distance < 0) {
        sprintf(msg, "Sensor %d: Timeout\r\n", index);
    } else {
        sprintf(msg, "Sensor %d: %.2f cm\r\n", index, distance);
    }

    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    //char msg[64];
    float value = 12.345f;

    sprintf(msg, "Float ertek: %.2f\r\n", value);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}
