
/* uart_handler.c */

#include "uart_handler.h"
#include "main.h"

void myprintf(const char *fmt, ...);

void Process_Command_From_TCamera(const char *cmd)
{
    myprintf("[T-CAMERA] Üzenet: %s\r\n", cmd);
    // ide jöhet pl. kamera ki-/bekapcsolás, kép készítés parancs stb.
}

void Process_Command_From_SIM800H(const char *cmd)
{
    myprintf("[SIM800H] Üzenet: %s\r\n", cmd);
    // pl. SMS feldolgozás, hívásparancsok, HTTP AT parancsok
}

void Process_Command_From_PC(const char *cmd)
{
    myprintf("[PC] Parancs: %s\r\n", cmd);
    // PC terminálról jövő vezérlés, debug stb.
}

void Process_Command_From_MotorShield(const char *cmd)
{
    myprintf("[MOTOR SHIELD] Válasz/parancs: %s\r\n", cmd);
    // pl. visszajelzés, belső diagnosztika
}

void Process_Command_From_GPS(const char *cmd)
{
    myprintf("[GPS] NMEA: %s\r\n", cmd);
    // ha "$GPRMC" vagy "$GPGGA" -> pozíció kiemelés stb.
}

void Process_Command_From_LoRa(const char *cmd)
{
    myprintf("[LoRa] Üzenet: %s\r\n", cmd);
    // parancs: "start", "stop", "speed 30", "mode auto" stb.
}

