/* Author: Luca Obwegs */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Two AS5600 magnetic wheel encoders: bus 0 = left (I2C3), bus 1 = right (I2C4).
   A read is started on one control tick (interrupt driven) and collected on the
   next one. A failed read recovers the bus; after
   ROBOT_ENCODER_MAX_CONSECUTIVE_ERRORS failures in a row the encoder is reported
   as failed through encoder_take_failure(). */
#define ENCODER_COUNT 2U

void encoder_reset_errors(void);
/* Returns 1 if the read was started. */
uint8_t encoder_start_read(uint8_t bus);
/* Returns 1 if a valid sample arrived; encoder_angle() then holds it. */
uint8_t encoder_collect_read(uint8_t bus);
/* Raw 12-bit angle of the last valid sample. */
uint16_t encoder_angle(uint8_t bus);
/* Description of a latched encoder failure, or NULL. Clears the failure. */
const char *encoder_take_failure(void);

#endif
