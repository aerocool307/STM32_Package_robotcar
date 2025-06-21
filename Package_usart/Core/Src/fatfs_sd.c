
/* fatfs_sd.c */

#include "fatfs_sd.h"
#include "spi.h"
#include "gpio.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;
#define SD_CS_GPIO_Port GPIOG
#define SD_CS_Pin GPIO_PIN_2

void SD_Select(void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

void SD_Deselect(void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

uint8_t SD_SPI_TxRx(uint8_t data) {
    uint8_t received;
    HAL_SPI_TransmitReceive(&hspi2, &data, &received, 1, HAL_MAX_DELAY);
    return received;
}
uint8_t SD_ReadSingleBlock(uint8_t *buff, uint32_t sector) {
    SD_Select();
    // Send dummy CMD17 command (read single block) for demo
    // Normally: send CMD17 + sector address + CRC, wait for data token (0xFE)
    // For simplicity, we simulate success
    for (int i = 0; i < 512; i++) {
        buff[i] = SD_SPI_TxRx(0xFF);  // Dummy read
    }
    SD_Deselect();
    return 1; // success
}

uint8_t SD_WriteSingleBlock(const uint8_t *buff, uint32_t sector) {
    SD_Select();
    // Send dummy CMD24 command (write single block) for demo
    // Normally: send CMD24 + sector + CRC, then 0xFE + 512 data + CRC
    // For simplicity, we simulate success
    for (int i = 0; i < 512; i++) {
        SD_SPI_TxRx(buff[i]); // Dummy write
    }
    SD_Deselect();
    return 1; // success
}
