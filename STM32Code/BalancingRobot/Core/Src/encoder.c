/* Author: Luca Obwegs */
#include "encoder.h"

#include "main.h"
#include "robot_config.h"

#include <stdio.h>
#include <string.h>

#define AS5600_I2C_ADDRESS_7BIT 0x36U
#define AS5600_REG_STATUS 0x0BU /* STATUS, RAW ANGLE high, RAW ANGLE low */
#define AS5600_STATUS_MD 0x20U  /* magnet detected */
#define AS5600_STATUS_MH 0x08U  /* magnet too strong */

extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c4;

typedef enum
{
  ENCODER_RX_IDLE = 0,
  ENCODER_RX_BUSY,
  ENCODER_RX_DONE,
  ENCODER_RX_ERROR
} EncoderRxState;

static volatile uint8_t rx_state[ENCODER_COUNT];
static volatile uint8_t rx_data[ENCODER_COUNT][3];
static volatile uint32_t rx_error[ENCODER_COUNT];
static uint16_t angle[ENCODER_COUNT];
static uint32_t consecutive_errors[ENCODER_COUNT];
static uint32_t magnet_consecutive[ENCODER_COUNT];
static char failure_text[64];
static uint8_t failure_pending;

static I2C_HandleTypeDef *bus_handle(uint8_t bus)
{
  return bus == 0U ? &hi2c3 : &hi2c4;
}

static int32_t bus_index(const I2C_HandleTypeDef *i2c)
{
  return i2c == &hi2c3 ? 0 : (i2c == &hi2c4 ? 1 : -1);
}

static unsigned bus_number(uint8_t bus)
{
  return bus == 0U ? 3U : 4U;
}

static void bus_delay(void)
{
  /* About 5-10 us at 170 MHz: a slow, safe bit-banged clock. */
  for (volatile uint32_t i = 0U; i < 200U; i++)
  {
  }
}

/* Releases an AS5600 that still holds SDA low after an aborted transfer (an MCU
   reset does not reset the sensor): up to 9 SCL pulses, a STOP, then re-init. */
static void bus_recover(uint8_t bus)
{
  I2C_HandleTypeDef *i2c = bus_handle(bus);
  GPIO_TypeDef *scl_port = bus == 0U ? scl_left_enc_GPIO_Port : scl_right_enc_GPIO_Port;
  GPIO_TypeDef *sda_port = bus == 0U ? sda_left_enc_GPIO_Port : sda_right_enc_GPIO_Port;
  uint16_t scl_pin = bus == 0U ? scl_left_enc_Pin : scl_right_enc_Pin;
  uint16_t sda_pin = bus == 0U ? sda_left_enc_Pin : sda_right_enc_Pin;
  GPIO_InitTypeDef gpio = {0};

  /* Stop the EV/ER interrupts first so an in-flight transfer cannot complete
     into a peripheral that is being reset; HAL_I2C_Init re-enables them. */
  HAL_NVIC_DisableIRQ(bus == 0U ? I2C3_EV_IRQn : I2C4_EV_IRQn);
  HAL_NVIC_DisableIRQ(bus == 0U ? I2C3_ER_IRQn : I2C4_ER_IRQn);
  rx_state[bus] = ENCODER_RX_IDLE;
  (void)HAL_I2C_DeInit(i2c);
  HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = scl_pin;
  HAL_GPIO_Init(scl_port, &gpio);
  gpio.Pin = sda_pin;
  HAL_GPIO_Init(sda_port, &gpio);
  bus_delay();
  for (uint32_t i = 0U; i < 9U && HAL_GPIO_ReadPin(sda_port, sda_pin) == GPIO_PIN_RESET; i++)
  {
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);
    bus_delay();
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
    bus_delay();
  }
  HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);
  bus_delay();
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET);
  bus_delay();
  HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
  bus_delay();
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
  bus_delay();

  /* HAL_I2C_Init runs the CubeMX MSP init again (alternate function, pull-ups). */
  (void)HAL_I2C_Init(i2c);
  (void)HAL_I2CEx_ConfigAnalogFilter(i2c, I2C_ANALOGFILTER_ENABLE);
  (void)HAL_I2CEx_ConfigDigitalFilter(i2c, 0U);
}

static void read_failed(uint8_t bus, HAL_StatusTypeDef result, uint32_t error)
{
  bus_recover(bus);
  if (++consecutive_errors[bus] >= ROBOT_ENCODER_MAX_CONSECUTIVE_ERRORS && !failure_pending)
  {
    (void)snprintf(failure_text, sizeof(failure_text),
                   "encoder_i2c%u,hal=%u,err=0x%02lX,fails=%lu", bus_number(bus),
                   (unsigned)result, (unsigned long)error,
                   (unsigned long)consecutive_errors[bus]);
    failure_pending = 1U;
  }
}

/* Interrupt context (I2C3/I2C4 EV/ER): only flag the result. */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  int32_t bus = bus_index(hi2c);
  if (bus < 0 || rx_state[bus] != ENCODER_RX_BUSY)
    return;
  rx_state[bus] = ENCODER_RX_DONE;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  int32_t bus = bus_index(hi2c);
  if (bus < 0 || rx_state[bus] != ENCODER_RX_BUSY)
    return;
  rx_error[bus] = HAL_I2C_GetError(hi2c);
  rx_state[bus] = ENCODER_RX_ERROR;
}

void encoder_reset_errors(void)
{
  memset(consecutive_errors, 0, sizeof(consecutive_errors));
  memset(magnet_consecutive, 0, sizeof(magnet_consecutive));
  failure_pending = 0U;
}

uint8_t encoder_start_read(uint8_t bus)
{
  I2C_HandleTypeDef *i2c = bus_handle(bus);
  if (rx_state[bus] == ENCODER_RX_BUSY)
  {
    /* The previous read did not finish within a whole control tick. */
    read_failed(bus, HAL_TIMEOUT, HAL_I2C_GetError(i2c));
    return 0U;
  }
  /* A stuck BUSY line would make HAL wait 25 ms; recover instead. */
  if ((i2c->Instance->ISR & I2C_ISR_BUSY) != 0U)
  {
    read_failed(bus, HAL_BUSY, HAL_I2C_GetError(i2c));
    return 0U;
  }
  rx_error[bus] = HAL_I2C_ERROR_NONE;
  rx_state[bus] = ENCODER_RX_BUSY;
  HAL_StatusTypeDef result =
      HAL_I2C_Mem_Read_IT(i2c, AS5600_I2C_ADDRESS_7BIT << 1, AS5600_REG_STATUS,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)rx_data[bus], 3U);
  if (result != HAL_OK)
  {
    rx_state[bus] = ENCODER_RX_IDLE;
    read_failed(bus, result, HAL_I2C_GetError(i2c));
    return 0U;
  }
  return 1U;
}

uint8_t encoder_collect_read(uint8_t bus)
{
  uint8_t state = rx_state[bus];
  if (state == ENCODER_RX_IDLE)
    return 0U; /* The start already failed and was counted. */
  if (state != ENCODER_RX_DONE)
  {
    I2C_HandleTypeDef *i2c = bus_handle(bus);
    if (state == ENCODER_RX_BUSY)
      read_failed(bus, HAL_TIMEOUT, HAL_I2C_GetError(i2c));
    else
      read_failed(bus, HAL_ERROR, rx_error[bus]);
    return 0U;
  }
  uint8_t data[3] = {rx_data[bus][0], rx_data[bus][1], rx_data[bus][2]};
  rx_state[bus] = ENCODER_RX_IDLE;
  consecutive_errors[bus] = 0U;
  /* ML (weak field) is accepted: the angle is still valid at max AGC gain. A
     single bad MD/MH status (bus bit error or a short field dropout while the
     wheel wobbles) only skips the sample; it fails after repeated errors. */
  if ((data[0] & (AS5600_STATUS_MD | AS5600_STATUS_MH)) != AS5600_STATUS_MD)
  {
    if (++magnet_consecutive[bus] >= ROBOT_ENCODER_MAX_CONSECUTIVE_ERRORS && !failure_pending)
    {
      (void)snprintf(failure_text, sizeof(failure_text),
                     "encoder_magnet%u_%s,status=0x%02X,fails=%lu", bus_number(bus),
                     !(data[0] & AS5600_STATUS_MD) ? "missing" : "strong",
                     (unsigned)data[0], (unsigned long)magnet_consecutive[bus]);
      failure_pending = 1U;
    }
    return 0U;
  }
  magnet_consecutive[bus] = 0U;
  angle[bus] = (uint16_t)(((data[1] & 0x0FU) << 8) | data[2]);
  return 1U;
}

uint16_t encoder_angle(uint8_t bus)
{
  return angle[bus];
}

const char *encoder_take_failure(void)
{
  if (!failure_pending)
    return NULL;
  failure_pending = 0U;
  return failure_text;
}
