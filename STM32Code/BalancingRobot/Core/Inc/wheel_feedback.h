/* Author: Luca Obwegs */
#ifndef WHEEL_FEEDBACK_H
#define WHEEL_FEEDBACK_H

#include <stdint.h>

/* Wheel position/velocity for the controllers, robot-forward in m and m/s.
   "steps": integrated step commands. "kf": per-wheel Kalman filter (wheel_kf)
   that fuses the step commands with the AS5600 encoders and detects lost steps.
   While balancing the encoders are always read; if they fail the feedback falls
   back to the step count and the robot keeps balancing. */
void wheelfb_select_kf(uint8_t use_kf);
uint8_t wheelfb_kf_selected(void);
/* Zeroes position and velocity (called when arming). */
void wheelfb_zero(void);
/* Stops the encoder reads (called when the motors are disabled). */
void wheelfb_stop(void);
/* Start of a control tick: consumes the encoder sample started on the previous
   tick. Returns the controller velocity, reduced if a stalled wheel had to be
   restarted from its measured speed. */
float wheelfb_begin_tick(uint8_t armed, float controller_velocity_m_s);
/* End of a control tick with the wheel speeds that were applied. */
void wheelfb_end_tick(uint8_t armed, float left_m_s, float right_m_s);
float wheelfb_position(void);
float wheelfb_velocity(void);
/* Status message (ACTIVE/FALLBACK) to send, or NULL. */
const char *wheelfb_take_event(void);

#endif
