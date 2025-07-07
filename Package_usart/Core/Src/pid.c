
/* pid.c */

#include "pid.h"
#include "control.h"
#include "hall.h"
#include "motor.h"

PIDController pid[MOTOR_COUNT];


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

void Motor_PID_Update(void)
{

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
    	static uint32_t last_tick = 0;
    	uint32_t now = HAL_GetTick();
    	float dt = (now - last_tick) / 1000.0f;  // dt másodpercben
    	if (dt < 0.001f) dt = 0.001f;  // minimális dt
    	last_tick = now;

        float measured_rpm = Get_Hall_Speed(i);
        float output = PID_Compute(&pid[i], target_rpm[i], measured_rpm, dt);

        Motor_SetPWM(i, output);
    }
}
