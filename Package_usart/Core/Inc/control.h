
/* control.h */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include <stdint.h>

#define MOTOR_COUNT 4

extern float target_rpm[MOTOR_COUNT];

extern void myprintf(const char *fmt, ...);

void Control_Init(void);
void Control_SetTargetSpeed(uint8_t motor_index, float rpm);
void Control_Update(void);  // időalap, pl. 0.1f másodperc
void Control_ParseCommand(const char* msg);


#endif /* INC_CONTROL_H_ */
