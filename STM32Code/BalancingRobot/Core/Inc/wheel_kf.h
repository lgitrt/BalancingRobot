/* Author: Luca Obwegs */
#ifndef WHEEL_KF_H
#define WHEEL_KF_H

#include <stdint.h>

/* Two-state wheel Kalman filter, x = [angle rad, speed rad/s], robot-forward.
   The commanded change of the step rate is the model input, the angle-corrected
   AS5600 reading is the measurement. A measurement that is delay_s old is
   modelled as H = [1, -delay_s]. */

#define WHEEL_ENCODER_COUNTS_PER_REV 4096.0f

typedef struct
{
  float q_rad2_s3;       /* white-acceleration process noise density */
  float r_rad2;          /* angle measurement variance */
  float delay_s;         /* age of the measurement when it is applied */
  float gate_sigma;      /* innovation gate in standard deviations */
  float gate_min_rad;    /* lower bound of the gate (quantisation, residual error) */
  float sample_period_s; /* encoder sample period, used to re-estimate the speed after a stall */
  float initial_speed_var;
  uint8_t max_rejects;   /* consecutive gated samples before the filter resynchronises */
} WheelKfConfig;

typedef enum
{
  WHEEL_KF_ACCEPTED = 0,
  WHEEL_KF_REJECTED,
  WHEEL_KF_RESYNC
} WheelKfUpdate;

typedef struct
{
  WheelKfConfig config;
  float angle_rad;
  float speed_rad_s;
  float p00, p01, p11;
  float innovation_rad;
  float previous_measurement_rad;
  uint8_t rejects;
  uint32_t accepted;
  uint32_t rejected;
  uint32_t resyncs;
} WheelKf;

typedef struct
{
  float coefficients[4]; /* a1, b1, a2, b2 in bus counts: magnet eccentricity harmonics */
  float sign;            /* +1 if bus counts increase when the wheel turns robot-forward */
  float previous_counts;
  float unwrapped_counts;
} WheelEncoder;

void wheel_kf_init(WheelKf *kf, const WheelKfConfig *config, float angle_rad);
void wheel_kf_predict(WheelKf *kf, float speed_step_rad_s, float dt_s);
WheelKfUpdate wheel_kf_update(WheelKf *kf, float measured_angle_rad);

float wheel_encoder_corrected_counts(const float coefficients[4], uint16_t raw_counts);
void wheel_encoder_init(WheelEncoder *encoder, const float coefficients[4], float sign,
                        uint16_t raw_counts);
float wheel_encoder_update(WheelEncoder *encoder, uint16_t raw_counts);
float wheel_encoder_angle_rad(const WheelEncoder *encoder);

#endif
