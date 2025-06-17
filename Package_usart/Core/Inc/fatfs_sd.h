/*
 * fatfs_sd.h
 *
 */

#ifndef INC_FATFS_SD_H_
#define INC_FATFS_SD_H_

#include "stdint.h"

void SD_Select(void);
void SD_Deselect(void);
uint8_t SD_SPI_TxRx(uint8_t data);

uint8_t SD_ReadSingleBlock(uint8_t *buff, uint32_t sector);
uint8_t SD_WriteSingleBlock(const uint8_t *buff, uint32_t sector);

#endif /* INC_FATFS_SD_H_ */
