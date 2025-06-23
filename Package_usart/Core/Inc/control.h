
/* control.h */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include <stdint.h>

void Control_Init(void);
void Control_SetTargetSpeed(uint8_t motor_index, float rpm);
void Control_Update(float dt);  // időalap, pl. 0.1f másodperc

#endif /* INC_CONTROL_H_ */
