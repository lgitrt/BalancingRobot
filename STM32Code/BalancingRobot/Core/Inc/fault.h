/* Author: Luca Obwegs */
#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>

/* Latched application fault. The first reason wins; a reset clears it. */
void fault_latch(const char *reason);
uint8_t fault_active(void);
const char *fault_reason(void);

#endif
