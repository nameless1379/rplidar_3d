#ifndef _LIDAR_UART_H_
#define _LIDAR_UART_H_

#define UART_TO_HOST   &UARTD1

#define  HOST_TRANSMIT_FREQ 400U


void uart_host_init(void);

#endif
