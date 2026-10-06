/* Author: Luca Obwegs */
#ifndef STEPPER_H
#define STEPPER_H

#include "main.h"

/* Two A4988 stepper drivers: STEP from a PWM channel (TIM2 CH1 left, TIM1 CH1
   right, 1 MHz timer clock), DIR and the shared active-low ENABLE from GPIOs.
   Speeds are robot-forward in m/s. */
void stepper_init(void);
void stepper_enable(void);
/* Stops the step outputs and disables both drivers. */
void stepper_disable(void);
/* Keeps the drivers as they are but stops stepping (STEP compare = 0). */
void stepper_hold(void);
/* Applies both wheel speeds; the realised (quantised) speeds are returned by
   stepper_left_speed()/stepper_right_speed(). Latches a fault on a timer error. */
void stepper_set_speeds(float left_m_s, float right_m_s);
float stepper_left_speed(void);
float stepper_right_speed(void);

#endif
