
/* uart_handler.h */

#ifndef INC_UART_HANDLER_H_
#define INC_UART_HANDLER_H_

#include <stdint.h>

void Process_Command_From_TCamera(const char *cmd);//USART1
void Process_Command_From_SIM800H(const char *cmd);//USART2
void Process_Command_From_PC(const char *cmd);//USART3
void Process_Command_From_MotorShield(const char *cmd);//USART6
void Process_Command_From_GPS(const char *cmd);//UART5
void Process_Command_From_LoRa(const char *cmd);//UART7

#endif /* INC_UART_HANDLER_H_ */
