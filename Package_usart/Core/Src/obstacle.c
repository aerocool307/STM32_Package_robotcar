
/* obstacle.c */

#include "obstacle.h"
#include "ultrasonic.h"
#include "motor.h"
#include "control.h"
#include "main.h"
#include <stdio.h>

#define OBSTACLE_THRESHOLD_CM 15.0f
#define SENSOR_COUNT 4

static float front_dist = 0;
static bool blocked = false;

extern void myprintf(const char *fmt, ...);

void Obstacle_Init(void)
{
    // ha szükséges, ide jöhet például kalibráció, LED stb.
	// Ultrahang inicializálás, ha külön kellene
}

void Obstacle_Update(void)
{
	front_dist = Ultrasonic_ReadDistance(0); // index = 0 legyen a front szenzor
	    myprintf("FRONT_DISTANCE: %.2f cm\r\n", front_dist);

	    if (front_dist > 5.0f && front_dist < OBSTACLE_THRESHOLD_CM) {
	        blocked = true;
	        myprintf("[OBS] Blokkolva! Tavolsag: %.2f cm\r\n", front_dist);
	    } else {
	        blocked = false;
	    }
}

bool Obstacle_IsBlocked(void)
{
	if (blocked)
	    {
	        myprintf("⛔ Mozgas leallitva akadaly miatt! %.2f cm\r\n", front_dist);
	    }
	return blocked;
}

void Obstacle_Handle(void)
{
    if (Obstacle_IsBlocked()) {
        for (int i = 0; i < 4; i++) {
            Control_SetTargetSpeed(i, 0);  // Célsebességet 0-ra állítjuk
            Motor_Set(i + 1, STOP, 0);     // Közvetlenül is leállítható
        }
    }
}

void Obstacle_Check(void)
{
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        float dist = Ultrasonic_ReadDistance(i);

        if (dist > 0 && dist < OBSTACLE_THRESHOLD_CM) {
            myprintf("\xE2\x9A\xA0 Akadaly a(z) %d. szenzoron: %.2f cm --> STOP\r\n", i, dist);

            for (uint8_t m = 0; m < 4; m++) {
                Motor_Set(m + 1, STOP, 0);
                Control_SetTargetSpeed(m, 0);
            }
            return;
        }
    }
}
float Obstacle_GetFrontDistance(void)
{
    return front_dist;
}
