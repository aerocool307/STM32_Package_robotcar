
/* sd_card.h */

#ifndef INC_SD_CARD_H_
#define INC_SD_CARD_H_

#include "fatfs.h"
#include "ff.h"

void SD_Card_Init(void);
void SD_Card_ReadTestFile(void);
void SD_Card_WriteTestFile(void);

#endif /* INC_SD_CARD_H_ */
