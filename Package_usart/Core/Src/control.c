
/* control.c */

#include "control.h"
#include "main.h"
#include "motor.h"
#include "hall.h"
#include "pid.h"
#include "obstacle.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

float target_rpm[MOTOR_COUNT];
extern PIDController pid[MOTOR_COUNT];

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

void Control_Update(void)
{
	static uint32_t last_tick = 0;
	uint32_t now = HAL_GetTick();
	float dt = (now - last_tick) / 1000.0f;
	last_tick = now;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        float actual = Get_Hall_Speed(i);
        if (actual >= 0) { // érvényes mért érték
            float pwm = PID_Compute(&pid[i], target_rpm[i], actual, dt);
            // PWM korlátozás
            if (pwm < 0) pwm = 0;
            if (pwm > 1000) pwm = 1000;

            // Iránykezelés, ha negatív célsebességet is használsz
            if (target_rpm[i] >= 0) {
                Motor_Set(i + 1, FORWARD, (uint16_t)pwm);
            } else {
                Motor_Set(i + 1, BACKWARD, (uint16_t)pwm);
            }
        }
    }

}

void Control_ParseCommand(const char* msg)
{

    float val = 0.0f;
    int spd = 0;

    if (sscanf(msg, "kp %f", &val) == 1) {
        pid[0].Kp = val;
        myprintf("PID Kp beallitva: %.2f\r\n", val);

    } else if (sscanf(msg, "m1 %d", &spd) == 1) {
        Control_SetTargetSpeed(0, spd);
        myprintf("Speed beallitva Motor_1: %d\r\n", spd);

    } else if (sscanf(msg, "m2 %d", &spd) == 1) {
        Control_SetTargetSpeed(1, spd);
        myprintf("Speed beallitva Motor_2: %d\r\n", spd);

    } else if (sscanf(msg, "m3 %d", &spd) == 1) {
        Control_SetTargetSpeed(2, spd);
        myprintf("Speed beallitva Motor_3: %d\r\n", spd);

    } else if (sscanf(msg, "m4 %d", &spd) == 1) {
        Control_SetTargetSpeed(3, spd);
        myprintf("Speed beallitva Motor_4: %d\r\n", spd);

    } else if (sscanf(msg, "all %d", &spd) == 1) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            Control_SetTargetSpeed(i, (float)spd);
        }
        myprintf("All Motor Speed beallitva: %d\r\n", spd);
    } else if (strcmp(msg, "start") == 0) {
        myprintf("[CMD] START parancs\r\n");
        for (int i = 0; i < MOTOR_COUNT; i++) {
            Control_SetTargetSpeed(i, 150);  // pl. 150 RPM célérték
            myprintf("Motor %d: indul \r\n", i+1);
        }

    } else if (strcmp(msg, "stop") == 0) {
        myprintf("[CMD] STOP parancs\r\n");
        for (int i = 0; i < MOTOR_COUNT; i++) {
            Control_SetTargetSpeed(i, 0);
            Motor_Set(i + 1, STOP, 0);
            myprintf("Motor %d: leallitva\r\n", i+1);
        }

    } else if (strcmp(msg, "status") == 0) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            float actual = Get_Hall_Speed(i);
            float pwm = PID_Compute(&pid[i], target_rpm[i], actual, 0.1f);
            myprintf("M%d | Target: %.2f RPM | Actual: %.2f RPM | PWM: %.2f\r\n",
                     i + 1, target_rpm[i], actual, pwm);
        }
        myprintf("[CMD] STATUS: robot mukodik, tavolsag = %.2f cm\r\n", Obstacle_GetFrontDistance());
    } else {
        myprintf("Ismeretlen parancs: %s\r\n", msg);
    }

}
