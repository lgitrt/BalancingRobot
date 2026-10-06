/* Author: Luca Obwegs */
#include "fault.h"

#include <stddef.h>

static volatile uint8_t fault_latched;
static const char *volatile fault_text = "none";

void fault_latch(const char *reason)
{
  if (fault_latched)
    return;
  fault_text = reason != NULL ? reason : "unknown";
  fault_latched = 1U;
}

uint8_t fault_active(void)
{
  return fault_latched;
}

const char *fault_reason(void)
{
  return fault_text;
}
