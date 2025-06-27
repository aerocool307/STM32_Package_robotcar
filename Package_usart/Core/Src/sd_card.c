
/* sd_card.c */


#include "sd_card.h"
#include "main.h"   // myprintf miatt

extern void myprintf(const char *fmt, ...);

void SD_Card_Init(void)
{
    myprintf("\r\n~ SD card init by aerocool ~\r\n");

    HAL_Delay(1000);

    FATFS FatFs;
    FRESULT fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK) {
        myprintf("f_mount error (%i)\r\n", fres);
        return;
    }

    DWORD free_clusters, free_sectors, total_sectors;
    FATFS* getFreeFs;

    fres = f_getfree("", &free_clusters, &getFreeFs);
    if (fres != FR_OK) {
        myprintf("f_getfree error (%i)\r\n", fres);
        return;
    }

    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
    free_sectors = free_clusters * getFreeFs->csize;

    myprintf("SD stats:\r\n%10lu KiB total.\r\n%10lu KiB free.\r\n",
             total_sectors / 2, free_sectors / 2);
}

void SD_Card_ReadTestFile(void)
{
    FIL fil;
    FRESULT fres = f_open(&fil, "test.txt", FA_READ);
    if (fres != FR_OK) {
        myprintf("f_open (read) error (%i)\r\n", fres);
        return;
    }

    BYTE readBuf[30];
    TCHAR* rres = f_gets((TCHAR*)readBuf, sizeof(readBuf), &fil);
    if (rres != 0) {
        myprintf("Read from 'test.txt': %s\r\n", readBuf);
    } else {
        myprintf("f_gets error (%i)\r\n", fres);
    }

    f_close(&fil);
}

void SD_Card_WriteTestFile(void)
{
    FIL fil;
    FRESULT fres = f_open(&fil, "write.txt", FA_WRITE | FA_CREATE_ALWAYS);
    if (fres != FR_OK) {
        myprintf("f_open (write) error (%i)\r\n", fres);
        return;
    }

    BYTE writeBuf[] = "a new file is made!";
    UINT bytesWrote;
    fres = f_write(&fil, writeBuf, sizeof(writeBuf) - 1, &bytesWrote);
    if (fres == FR_OK) {
        myprintf("Wrote %i bytes to 'write.txt'\r\n", bytesWrote);
    } else {
        myprintf("f_write error (%i)\r\n", fres);
    }

    f_close(&fil);
}
