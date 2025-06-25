
/* control.h */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include <stdint.h>

void Control_Init(void);
void Control_SetTargetSpeed(uint8_t motor_index, float rpm);
void Control_Update(void);  // időalap, pl. 0.1f másodperc
//void App_Init();
//void App_Update(float dt);
void MotorControl_HandleBluetooth(uint8_t byte);  // PID vezérléshez



#endif /* INC_CONTROL_H_ */
