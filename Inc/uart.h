#ifndef UART_H
#define UART_H

#include "stm32l4xx_hal.h"

#define UART_DBG 					/*Enable UART debug port*/
#define MAX_UART_TX_LEN  (29 + 3)	/*reserve 3 bytes for cr+lf+0x00*/
#define MAX_UART_RX_LEN  (16)

extern void ODM_UART_IRQHandler(UART_HandleTypeDef *huart);
extern void odmPrintf(char *buf);
extern void uartPrintf(char *fmt, ...);
extern void send_char_to_uart(char ch);
extern int odmGetStr(char *buf, int maxLen); /*return length of str*/
extern void uart_rx_enable(void);
extern void uart_rx_disable(void);
extern void uartPrintfBlock(char *fmt, ...); /* for ASSERT() */
#endif //UART_H
