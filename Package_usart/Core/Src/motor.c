
/* motor.c */

#include "motor.h"
#include "main.h"
#include "tim.h"
#include "hall.h"
#include <string.h>
#include <stdio.h>

#define CMD_BUFFER_SIZE 64

static uint8_t latch_state = 0;
static char cmd_buffer[CMD_BUFFER_SIZE];
static uint8_t cmd_index = 0;
// Globális PID példány minden motorhoz
PIDController pid_m1, pid_m2, pid_m3, pid_m4;

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim14;

// Globális állapotváltozó a motorhoz
uint8_t motor_state = 0x00; // vagy MOTOR_STOP ha definiálva van

// Shift regiszter GPIO-k inicializálása
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE(); // MOTORLATCH
    __HAL_RCC_GPIOF_CLK_ENABLE(); // CLK, ENABLE, DATA

    GPIO_InitStruct.Pin = MOTORLATCH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTORLATCH_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MOTORCLK_Pin | MOTORENABLE_Pin | MOTORDATA_Pin;
    HAL_GPIO_Init(MOTORCLK_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(MOTORENABLE_GPIO_Port, MOTORENABLE_Pin, GPIO_PIN_SET);//Engedélyezés
    Motor_output(0x00);

    // Indítsuk el a PWM kimeneteket
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);  // Motor1
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // Motor2
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Motor3
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // Motor4

    // Opcionálisan: kezdeti kitöltési tényező
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

    PID_Init(&pid_m1, 1.0f, 0.1f, 0.05f, 0, 100);
    PID_Init(&pid_m2, 1.0f, 0.1f, 0.05f, 0, 100);
    PID_Init(&pid_m3, 1.0f, 0.1f, 0.05f, 0, 100);
    PID_Init(&pid_m4, 1.0f, 0.1f, 0.05f, 0, 100);
}

// Motor kimenetek bitenkénti beállítása (shift regiszter frissítés)
void Motor_output(uint8_t output)
{
	motor_state = output;
    HAL_GPIO_WritePin(MOTORLATCH_GPIO_Port, MOTORLATCH_Pin, GPIO_PIN_RESET);

    for (int i = 7; i >= 0; i--)
    {
       HAL_GPIO_WritePin(MOTORCLK_GPIO_Port, MOTORCLK_Pin, GPIO_PIN_RESET);
       HAL_GPIO_WritePin(MOTORDATA_GPIO_Port, MOTORDATA_Pin,
                        (motor_state & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
       HAL_GPIO_WritePin(MOTORCLK_GPIO_Port, MOTORCLK_Pin, GPIO_PIN_SET);
     }

     HAL_GPIO_WritePin(MOTORLATCH_GPIO_Port, MOTORLATCH_Pin, GPIO_PIN_SET);

}

// Egy motor irányának és sebességének beállítása
void Motor_Set(MotorID motor, MotorDirection command, MotorState speed)
{
    // Bitpárok
	uint8_t a, b;
    uint32_t pwm_channel = 0;

    // Motor választás
    switch (motor)
    {
        case MOTOR_1:
        	a = MOTOR_A1_BIT;
        	b = MOTOR_A2_BIT;
            pwm_channel = TIM_CHANNEL_1;
            break;
        case MOTOR_2:
        	a = MOTOR_B1_BIT;
        	b = MOTOR_B2_BIT;
            pwm_channel = TIM_CHANNEL_3;
            break;
        case MOTOR_3:
        	a = MOTOR_C1_BIT;
        	b = MOTOR_C2_BIT;
            pwm_channel = TIM_CHANNEL_1;
            break;
        case MOTOR_4:
        	a = MOTOR_D1_BIT;
        	b = MOTOR_D2_BIT;
            pwm_channel = TIM_CHANNEL_2;
            break;
        default:
            return;
    }

    // Irány beállítása
    switch (command) {
            case FORWARD:
                latch_state |= (1 << a);
                latch_state &= ~(1 << b);
                break;
            case BACKWARD:
                latch_state &= ~(1 << a);
                latch_state |= (1 << b);
                break;
            case RELEASE:
                latch_state &= ~(1 << a);
                latch_state &= ~(1 << b);
                speed = 0;
                break;
    }

    Motor_output(latch_state);

    // PWM beállítás (htim1 és htim14)
    if (motor == MOTOR_1)
        __HAL_TIM_SET_COMPARE(&htim14, pwm_channel, speed); // PA7 - TIM14 CH1
    else
        __HAL_TIM_SET_COMPARE(&htim1, pwm_channel, speed);  // Többi motor: TIM1
}

// Motor sebességének beállítása
void Motor_SetSpeed(uint8_t motor_index, uint16_t speed)
{
    switch (motor_index) {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, speed);//PA7 (M1)
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);//PE13 (M2)
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);//PE9 (M3)
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);//PE11 (M4)
            break;
    }
}

// UART-ból jövő parancs feldolgozása
void MotorControl_HandleInput(uint8_t byte)
{
    if (byte == '\n' || byte == '\r') {
        cmd_buffer[cmd_index] = '\0';

        if (strcmp(cmd_buffer, "fwd") == 0) {
            Motor_Set(MOTOR_1, FORWARD, 32);
            Motor_Set(MOTOR_2, FORWARD, 32);
            Motor_Set(MOTOR_3, FORWARD, 32);
            Motor_Set(MOTOR_4, FORWARD, 32);
        } else if (strcmp(cmd_buffer, "rev") == 0) {
            Motor_Set(MOTOR_1, BACKWARD, 32);
            Motor_Set(MOTOR_2, BACKWARD, 32);
            Motor_Set(MOTOR_3, BACKWARD, 32);
            Motor_Set(MOTOR_4, BACKWARD, 32);
        } else if (strcmp(cmd_buffer, "stop") == 0) {
            Motor_Set(MOTOR_1, RELEASE, 0);
            Motor_Set(MOTOR_2, RELEASE, 0);
            Motor_Set(MOTOR_3, RELEASE, 0);
            Motor_Set(MOTOR_4, RELEASE, 0);
        } else if (strcmp(cmd_buffer, "left") == 0) {
            Motor_Set(MOTOR_1, BACKWARD, 32);
            Motor_Set(MOTOR_2, BACKWARD, 32);
            Motor_Set(MOTOR_3, FORWARD, 32);
            Motor_Set(MOTOR_4, FORWARD, 32);
        } else if (strcmp(cmd_buffer, "right") == 0) {
            Motor_Set(MOTOR_1, FORWARD, 32);
            Motor_Set(MOTOR_2, FORWARD, 32);
            Motor_Set(MOTOR_3, BACKWARD, 32);
            Motor_Set(MOTOR_4, BACKWARD, 32);
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "CMD: %s\r\n", cmd_buffer);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        cmd_index = 0;
    } else {
        if (cmd_index < CMD_BUFFER_SIZE - 1) {
            cmd_buffer[cmd_index++] = byte;
        }
    }
}

void PID_Init(PIDController* pid, float Kp, float Ki, float Kd, float min, float max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->output = 0;
    pid->min_output = min;
    pid->max_output = max;
}

float PID_Compute(PIDController* pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;

    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < pid->min_output) pid->output = pid->min_output;

    pid->prev_error = error;

    return pid->output;
}

// Bluetooth input handler (pl. UART2-ből hívod meg)
void MotorControl_HandleBluetooth(uint8_t byte)
{
    static char bt_cmd[64];
    static uint8_t idx = 0;

    if (byte == '\n' || byte == '\r') {
        bt_cmd[idx] = '\0';

        // példa: "spd 100" → célsebesség 100 RPM
        int spd = 0;
        if (sscanf(bt_cmd, "spd %d", &spd) == 1) {
            float dt = 0.1f;  // időalap
            float actual_speed = Get_Hall_Speed();
            float pwm = PID_Compute(&pid_m1, spd, actual_speed, dt);
            Motor_Set(MOTOR_1, FORWARD, (uint16_t)pwm);
        }

        idx = 0;
    } else if (idx < sizeof(bt_cmd) - 1) {
        bt_cmd[idx++] = byte;
    }
    /*
    //pid szabályozás
    float actual_rpm = Get_Hall_Speed(0);  // 0 = MOTOR_1
    float pwm = PID_Compute(&pid_m1, target_rpm, actual_rpm, dt);
    Motor_Set(MOTOR_1, FORWARD, (uint16_t)pwm);
    */
}
