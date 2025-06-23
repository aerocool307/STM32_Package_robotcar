
/* control.c */

#include "control.h"
#include "motor.h"
#include "hall.h"
#include "pid.h"

#define MOTOR_COUNT 4

static PIDController pid[MOTOR_COUNT];
static float target_rpm[MOTOR_COUNT];

void Control_Init(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        PID_Init(&pid[i], 1.0f, 0.1f, 0.05f, 0.0f, 100.0f);
        target_rpm[i] = 0;
    }
}

void Control_SetTargetSpeed(uint8_t motor_index, float rpm)
{
    if (motor_index < MOTOR_COUNT)
        target_rpm[motor_index] = rpm;
}

void Control_Update(float dt)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        float actual = Get_Hall_Speed(i);
        if (actual >= 0) { // érvényes mért érték
            float pwm = PID_Compute(&pid[i], target_rpm[i], actual, dt);
            Motor_SetSpeed(i + 1, (uint16_t)pwm); // +1, mert Motor_SetSpeed 1-től indexel
        }
    }
}

void App_Init()
{
    PID_Init(&pid_m1, 1.0f, 0.1f, 0.05f, 0, 100);
    PID_Init(&pid_m2, 1.0f, 0.1f, 0.05f, 0, 100);
    PID_Init(&pid_m3, 1.0f, 0.1f, 0.05f, 0, 100);
    PID_Init(&pid_m4, 1.0f, 0.1f, 0.05f, 0, 100);
}

void App_Update(float dt)
{
    float actual = Get_Hall_Speed(0);
    float pwm = PID_Compute(&pid_m1, target_rpm, actual, dt);
    Motor_SetSpeed(1, (uint16_t)pwm);
}

