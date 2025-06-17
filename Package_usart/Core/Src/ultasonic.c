/*
 * ultasonic.c
 *
 */


// ultrasonic.c

#include "ultrasonic.h"
#include "main.h"

extern TIM_HandleTypeDef htim5;

#define NUM_SENSORS 4

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
GPIO_TypeDef* trig_ports[NUM_SENSORS] = {GPIOA, GPIOE, GPIOE, GPIOE};
uint16_t trig_pins[NUM_SENSORS]       = {GPIO_PIN_4, GPIO_PIN_10, GPIO_PIN_12, GPIO_PIN_15};

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint8_t index;

    if (htim->Instance != TIM5) return;

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
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);
}

float Ultrasonic_ReadDistance(uint8_t index)
{
    if (index > 3) return -1;

    uint32_t channel = tim_channels[index];

    capture_done[index] = 0;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim5, channel, TIM_INPUTCHANNELPOLARITY_RISING);

    // Trigger kiadása
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
