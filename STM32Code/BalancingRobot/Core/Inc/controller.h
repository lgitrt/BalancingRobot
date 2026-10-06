/* Author: Luca Obwegs */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>

typedef enum
{
  CONTROLLER_PID = 0,
  CONTROLLER_LQR
} ControllerType;

typedef struct
{
  float position_m;          /* wheel position estimate */
  float velocity_m_s;        /* wheel velocity estimate */
  float requested_speed_m_s; /* teleop forward speed */
  float pitch_rad;           /* estimator_control_pitch() */
  float pitch_reference_rad; /* estimator_pitch_reference() */
  float pitch_rate_rad_s;    /* estimator_control_rate() */
  float trim_integral_scale; /* gain of the LQR position integral (1 = full) */
} ControllerInput;

/* Balance controllers. Both command the wheel acceleration, which is
   integrated into the common wheel speed (stepper motors follow speed commands).
   - LQR: full state feedback on position error, velocity, pitch and pitch rate
     plus a slow position integral that learns the balance point.
   - PID: PD on pitch plus velocity feedback and a bounded position correction. */
void controller_set_type(ControllerType type);
ControllerType controller_type(void);
const char *controller_name(void);
void controller_set_position_hold(uint8_t enabled);
uint8_t controller_position_hold(void);
/* Clears the reference, integrators and the wheel speed (called when arming). */
void controller_reset(void);
/* One armed control tick: advances the position reference by the requested
   speed, returns the clamped acceleration and updates controller_velocity(). */
float controller_update(const ControllerInput *input);
float controller_velocity(void);
void controller_set_velocity(float velocity_m_s);

#endif
