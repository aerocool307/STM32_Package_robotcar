
/* obstacle.h */

#ifndef INC_OBSTACLE_H_
#define INC_OBSTACLE_H_

#include <stdint.h>
#include <stdbool.h>

void Obstacle_Init(void);
void Obstacle_Update(void);
bool Obstacle_IsBlocked(void);  // TRUE ha valami túl közel van
void Obstacle_Handle(void);     // STOP parancs kiadása akadály esetén
void Obstacle_Check(void);
float Obstacle_GetFrontDistance(void);


#endif /* INC_OBSTACLE_H_ */
