/* Author: Luca Obwegs */
#ifndef COMM_H
#define COMM_H

#include "main.h"

#define COMM_LINE_SIZE 64U

/* Line-based UART protocol on LPUART1 (ST-LINK virtual COM port, 115200 8N1).
   Received bytes are collected in the RX interrupt; complete lines ('\r' or '\n'
   terminated) are queued for the main loop. Outgoing messages are queued and
   sent with DMA so the control loop never blocks. */
HAL_StatusTypeDef comm_init(void);
/* Copies the next received line (without terminator) into line; returns 0 if
   there is none. */
uint8_t comm_take_line(char line[COMM_LINE_SIZE]);
/* Queues a message; returns 0 if the queue is full or the text is too long. */
uint8_t comm_send(const char *text);
/* Starts the next DMA transfer if the UART is idle. */
void comm_poll(void);
/* Blocking send for the boot fault report before the main loop runs. */
void comm_send_blocking(const char *text);

#endif
