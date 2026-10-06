/* Author: Luca Obwegs */
#include "wheel_kf.h"

#include <math.h>
#include <string.h>

#define WHEEL_TWO_PI 6.28318530718f

void wheel_kf_init(WheelKf *kf, const WheelKfConfig *config, float angle_rad)
{
  memset(kf, 0, sizeof(*kf));
  kf->config = *config;
  kf->angle_rad = angle_rad;
  kf->previous_measurement_rad = angle_rad;
  kf->p00 = config->r_rad2;
  kf->p11 = config->initial_speed_var;
}

void wheel_kf_predict(WheelKf *kf, float speed_step_rad_s, float dt_s)
{
  float dt2 = dt_s * dt_s;
  float q = kf->config.q_rad2_s3;
  /* The step generator changes its rate at the tick, then runs at constant speed. */
  kf->speed_rad_s += speed_step_rad_s;
  kf->angle_rad += kf->speed_rad_s * dt_s;
  kf->p00 += 2.0f * dt_s * kf->p01 + dt2 * kf->p11 + q * dt2 * dt_s / 3.0f;
  kf->p01 += dt_s * kf->p11 + 0.5f * q * dt2;
  kf->p11 += q * dt_s;
}

WheelKfUpdate wheel_kf_update(WheelKf *kf, float measured_angle_rad)
{
  float d = kf->config.delay_s;
  float previous = kf->previous_measurement_rad;
  kf->previous_measurement_rad = measured_angle_rad;
  float y = measured_angle_rad - (kf->angle_rad - d * kf->speed_rad_s);
  float s = kf->p00 - 2.0f * d * kf->p01 + d * d * kf->p11 + kf->config.r_rad2;
  float limit = fmaxf(kf->config.gate_sigma * sqrtf(s), kf->config.gate_min_rad);
  kf->innovation_rad = y;
  if (fabsf(y) > limit)
  {
    if (++kf->rejects < kf->config.max_rejects)
    {
      kf->rejected++;
      return WHEEL_KF_REJECTED;
    }
    /* Persistent disagreement is real motion the model did not predict (stall,
       lost steps, manual turning): restart from the last two measurements. */
    float ts = kf->config.sample_period_s;
    float r = kf->config.r_rad2;
    kf->speed_rad_s = (measured_angle_rad - previous) / ts;
    kf->angle_rad = measured_angle_rad + d * kf->speed_rad_s;
    kf->p11 = 2.0f * r / (ts * ts);
    kf->p01 = r / ts + d * kf->p11;
    kf->p00 = r + 2.0f * d * r / ts + d * d * kf->p11;
    kf->rejects = 0U;
    kf->resyncs++;
    return WHEEL_KF_RESYNC;
  }
  kf->accepted++;
  kf->rejects = 0U;
  float ph0 = kf->p00 - d * kf->p01;
  float ph1 = kf->p01 - d * kf->p11;
  float k0 = ph0 / s;
  float k1 = ph1 / s;
  kf->angle_rad += k0 * y;
  kf->speed_rad_s += k1 * y;
  kf->p00 -= k0 * ph0;
  kf->p01 -= k0 * ph1;
  kf->p11 -= k1 * ph1;
  return WHEEL_KF_ACCEPTED;
}

float wheel_encoder_corrected_counts(const float coefficients[4], uint16_t raw_counts)
{
  float theta = WHEEL_TWO_PI * (float)raw_counts / WHEEL_ENCODER_COUNTS_PER_REV;
  float error = coefficients[0] * cosf(theta) + coefficients[1] * sinf(theta) +
                coefficients[2] * cosf(2.0f * theta) + coefficients[3] * sinf(2.0f * theta);
  return (float)raw_counts - error;
}

void wheel_encoder_init(WheelEncoder *encoder, const float coefficients[4], float sign,
                        uint16_t raw_counts)
{
  memcpy(encoder->coefficients, coefficients, sizeof(encoder->coefficients));
  encoder->sign = sign;
  encoder->previous_counts = wheel_encoder_corrected_counts(coefficients, raw_counts);
  encoder->unwrapped_counts = encoder->previous_counts;
}

float wheel_encoder_update(WheelEncoder *encoder, uint16_t raw_counts)
{
  float counts = wheel_encoder_corrected_counts(encoder->coefficients, raw_counts);
  float delta = counts - encoder->previous_counts;
  if (delta > 0.5f * WHEEL_ENCODER_COUNTS_PER_REV)
    delta -= WHEEL_ENCODER_COUNTS_PER_REV;
  else if (delta < -0.5f * WHEEL_ENCODER_COUNTS_PER_REV)
    delta += WHEEL_ENCODER_COUNTS_PER_REV;
  encoder->previous_counts = counts;
  encoder->unwrapped_counts += delta;
  return wheel_encoder_angle_rad(encoder);
}

float wheel_encoder_angle_rad(const WheelEncoder *encoder)
{
  return encoder->sign * encoder->unwrapped_counts * (WHEEL_TWO_PI / WHEEL_ENCODER_COUNTS_PER_REV);
}
