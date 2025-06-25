
/* control.c */

#include "control.h"
#include "main.h"
#include "motor.h"
#include "hall.h"
#include "pid.h"
#include <stdio.h>
#include <string.h>

#define MOTOR_COUNT 4

static PIDController pid[MOTOR_COUNT];
static float target_rpm[MOTOR_COUNT];

extern void myprintf(const char *fmt, ...);


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



// Bluetooth input handler (pl. UART2-ből hívod meg)
void MotorControl_HandleBluetooth(uint8_t byte)
{
    static char bt_cmd[64];
    static uint8_t idx = 0;

    if (byte == '\n' || byte == '\r') {
        bt_cmd[idx] = '\0';

        float val = 0.0f;
        int spd = 0;

        if (sscanf(bt_cmd, "kp %f", &val) == 1) { // @suppress("Float formatting support")
            pid[0].Kp = val;  // pl. motor 1 hangolása
        }else if (sscanf(bt_cmd, "m1 %d", &spd) == 1) {// példa: "m1 100" → célsebesség motor1-en 100 RPM
            Control_SetTargetSpeed(0, spd);
        } else if (sscanf(bt_cmd, "m2 %d", &spd) == 1) {
            Control_SetTargetSpeed(1, spd);
        } else if (sscanf(bt_cmd, "m3 %d", &spd) == 1) {
            Control_SetTargetSpeed(2, spd);
        } else if (sscanf(bt_cmd, "m4 %d", &spd) == 1) {
            Control_SetTargetSpeed(3, spd);
        }else if (sscanf(bt_cmd, "all %d", &spd) == 1) {
        	for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        		Control_SetTargetSpeed(i, (float)spd);
        	}
        }else if (strcmp(bt_cmd, "status") == 0) {
            for (int i = 0; i < MOTOR_COUNT; i++) {
                float actual = Get_Hall_Speed(i);
                float pwm = PID_Compute(&pid[i], target_rpm[i], actual, 0.1f);  // vagy csak olvasd a PID kimenetet, ha tárolod

                myprintf("M%d | Target: %.2f RPM | Actual: %.2f RPM | PWM: %.2f\r\n",
                         i + 1, target_rpm[i], actual, pwm);
            }
        }

        idx = 0;
       } else if (idx < sizeof(bt_cmd) - 1) {
            bt_cmd[idx++] = byte;
            }

}

/*
void App_Init()
{
    PID_Init(&pid_m1, 1.0f, 0.1f, 0.05f, 0, 100);
    PID_Init(&pid_m2, 1.0f, 0.1f, 0.05f, 0, 100);
    PID_Init(&pid_m3, 1.0f, 0.1f, 0.05f, 0, 100);
    PID_Init(&pid_m4, 1.0f, 0.1f, 0.05f, 0, 100);
}
*/
/*
void App_Update(float dt)
{
    float actual = Get_Hall_Speed(0);
    float pwm = PID_Compute(&pid_m1, pid_target_rpm, actual, dt);
    Motor_SetSpeed(1, (uint16_t)pwm);
}
*/
