/* Author: Luca Obwegs */
#include "comm.h"

#include <string.h>

#define RX_QUEUE_LENGTH 4U
#define TX_QUEUE_LENGTH 4U
#define TX_MESSAGE_SIZE 128U

extern UART_HandleTypeDef hlpuart1;

static uint8_t rx_byte;
static char rx_line[COMM_LINE_SIZE];
static uint8_t rx_length;
static char rx_queue[RX_QUEUE_LENGTH][COMM_LINE_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;

static char tx_queue[TX_QUEUE_LENGTH][TX_MESSAGE_SIZE];
static uint16_t tx_lengths[TX_QUEUE_LENGTH];
static uint8_t tx_head;
static uint8_t tx_tail;
static volatile uint8_t tx_busy;

HAL_StatusTypeDef comm_init(void)
{
  rx_length = 0U;
  rx_head = rx_tail = 0U;
  tx_head = tx_tail = 0U;
  tx_busy = 0U;
  return HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1U);
}

uint8_t comm_take_line(char line[COMM_LINE_SIZE])
{
  if (rx_tail == rx_head)
    return 0U;
  memcpy(line, rx_queue[rx_tail], COMM_LINE_SIZE);
  rx_tail = (uint8_t)((rx_tail + 1U) % RX_QUEUE_LENGTH);
  return 1U;
}

uint8_t comm_send(const char *text)
{
  size_t length = strlen(text);
  uint8_t next = (uint8_t)((tx_head + 1U) % TX_QUEUE_LENGTH);
  if (length == 0U || length >= TX_MESSAGE_SIZE || next == tx_tail)
    return 0U;
  memcpy(tx_queue[tx_head], text, length);
  tx_lengths[tx_head] = (uint16_t)length;
  tx_head = next;
  return 1U;
}

void comm_poll(void)
{
  if (tx_busy || tx_tail == tx_head)
    return;
  tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)tx_queue[tx_tail], tx_lengths[tx_tail]) !=
      HAL_OK)
  {
    /* Telemetry is best effort: drop the message rather than block the loop. */
    tx_busy = 0U;
  }
  tx_tail = (uint8_t)((tx_tail + 1U) % TX_QUEUE_LENGTH);
}

void comm_send_blocking(const char *text)
{
  (void)HAL_UART_Transmit(&hlpuart1, (const uint8_t *)text, (uint16_t)strlen(text), 100U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance != LPUART1)
    return;
  char byte = (char)rx_byte;
  if (byte == '\n' || byte == '\r')
  {
    uint8_t next = (uint8_t)((rx_head + 1U) % RX_QUEUE_LENGTH);
    /* A full queue drops the line; the teleop app repeats its commands. */
    if (rx_length > 0U && next != rx_tail)
    {
      memcpy(rx_queue[rx_head], rx_line, rx_length);
      rx_queue[rx_head][rx_length] = '\0';
      rx_head = next;
    }
    rx_length = 0U;
  }
  else if (rx_length < COMM_LINE_SIZE - 1U)
  {
    rx_line[rx_length++] = byte;
  }
  else
  {
    rx_length = 0U;
  }
  (void)HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1U);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance == LPUART1)
    tx_busy = 0U;
}
