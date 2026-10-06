/* Author: Luca Obwegs */
#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "imu.h"

#include <stdint.h>

typedef enum
{
  ESTIMATOR_COMPLEMENTARY = 0,
  ESTIMATOR_KALMAN,
  ESTIMATOR_OFFSET_KF
} EstimatorMode;

/* Pitch estimation from the IMU.
   - complementary: gyro integration corrected by the accelerometer angle, with
     gyro bias tracking while balancing.
   - kalman: 2-state Kalman filter [pitch, gyro bias].
   - offsetkf: 4-state Kalman filter [pitch, pitch rate, gyro bias, balance
     point] that also uses the commanded wheel acceleration. The controller then
     balances around the estimated balance point instead of a fixed trim.
   The complementary/Kalman pitch always runs; it is used for the fall and arming
   checks and the telemetry. */
void estimator_set_mode(EstimatorMode mode);
EstimatorMode estimator_mode(void);
const char *estimator_mode_name(void);
/* motion_accel_m_s2: the controller acceleration of the previous tick (what the
   wheels execute now), 0 while disarmed. Latches a fault on a numerical error. */
HAL_StatusTypeDef estimator_update(const ImuSample *sample, float motion_accel_m_s2,
                                   uint8_t armed);
/* Pitch for the fall/arming checks and the telemetry. */
float estimator_pitch(void);
/* Pitch, setpoint and filtered pitch rate for the balance controller. */
float estimator_control_pitch(void);
float estimator_pitch_reference(void);
float estimator_control_rate(void);
/* The offset KF already tracks the balance point, so the LQR position integral
   is slowed down while it is active. */
float estimator_integral_scale(void);

#endif
