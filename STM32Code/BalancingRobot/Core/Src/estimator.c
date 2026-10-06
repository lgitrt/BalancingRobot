/* Author: Luca Obwegs */
#include "estimator.h"

#include "fault.h"
#include "robot_config.h"

#include <math.h>
#include <string.h>

#define BALANCE_TRIM_RAD (ROBOT_BALANCE_TRIM_DEG * DEG_TO_RAD)

static EstimatorMode mode = ROBOT_ESTIMATOR_OFFSET_KF_DEFAULT ? ESTIMATOR_OFFSET_KF :
                            ROBOT_ESTIMATOR_KALMAN_DEFAULT ? ESTIMATOR_KALMAN :
                                                             ESTIMATOR_COMPLEMENTARY;

/* Complementary / 2-state Kalman filter. */
static uint8_t pitch_initialized;
static float pitch_rad;
static float gyro_bias;
static float p00, p01, p11;

/* Offset KF x = [pitch, pitch rate, gyro bias, balance point], rad and rad/s. */
static uint8_t offset_kf_initialized;
static float offset_kf_x[4];
static float offset_kf_p[4][4];
static float offset_kf_accel_m_s2;

static uint8_t rate_filter_initialized;
static float filtered_rate_rad_s;

/* Complementary filter (or 2-state Kalman filter): fuses the gyro rate with the
   motion-compensated accelerometer angle. */
static HAL_StatusTypeDef estimate_pitch(float gyro_rate, float accel_angle, float norm_error,
                                        float motion_accel, uint8_t armed)
{
  const float dt = ROBOT_CONTROL_DT;
  if (!isfinite(gyro_rate) || !isfinite(accel_angle) || !isfinite(pitch_rad) ||
      !isfinite(norm_error))
    return HAL_ERROR;
  if (!pitch_initialized)
  {
    gyro_bias = 0.0f;
    p00 = DEG_TO_RAD * DEG_TO_RAD;
    p01 = 0.0f;
    p11 = 0.25f * DEG_TO_RAD * DEG_TO_RAD;
    pitch_initialized = 1U;
  }
  float prediction = pitch_rad + (gyro_rate - gyro_bias) * dt;
  float innovation = remainderf(accel_angle - prediction, 2.0f * ROBOT_PI);
  float correction = 0.0f;

  if (mode != ESTIMATOR_KALMAN)
  {
    if (!armed)
    {
      correction = ROBOT_COMPLEMENTARY_IDLE_GAIN * innovation;
    }
    else
    {
      correction = ROBOT_COMPLEMENTARY_ARMED_KP_PER_S * dt * innovation;
      /* Gyro bias drift after the boot calibration acts like a trim error. */
      if (norm_error <= ROBOT_KALMAN_MAX_NORM_ERROR_G)
      {
        const float max_bias = ROBOT_COMPLEMENTARY_MAX_BIAS_DEG_S * DEG_TO_RAD;
        gyro_bias = fmaxf(-max_bias, fminf(max_bias, gyro_bias -
            ROBOT_COMPLEMENTARY_ARMED_KI_PER_S2 * dt * innovation));
      }
    }
    pitch_rad = prediction + correction;
    return isfinite(pitch_rad) && isfinite(gyro_bias) ? HAL_OK : HAL_ERROR;
  }

  const float qa = ROBOT_KALMAN_ANGLE_NOISE * DEG_TO_RAD;
  const float qb = ROBOT_KALMAN_BIAS_NOISE * DEG_TO_RAD;
  float pp00 = p00 - 2.0f * dt * p01 + dt * dt * p11 + qa * qa * dt +
               qb * qb * dt * dt * dt / 3.0f;
  float pp01 = p01 - dt * p11 - qb * qb * dt * dt / 2.0f;
  float pp11 = p11 + qb * qb * dt;
  /* The norm alone misses horizontal acceleration; the command also reduces trust. */
  float motion = armed ? fabsf(motion_accel) / GRAVITY_M_S2 : 0.0f;
  float noise_scale = 1.0f + 100.0f * norm_error * norm_error + 100.0f * motion * motion;
  float std = ROBOT_KALMAN_ACCEL_STD_DEG * DEG_TO_RAD;
  float r = std * std * noise_scale;
  float s = pp00 + r;
  if (!isfinite(s) || s <= 0.0f || !isfinite(pp01) || !isfinite(pp11) || pp00 < 0.0f ||
      pp11 < 0.0f)
    return HAL_ERROR;
  float k0 = pp00 / s;
  float k1 = pp01 / s;
  uint8_t rejected = (uint8_t)(norm_error > ROBOT_KALMAN_MAX_NORM_ERROR_G ||
      fabsf(innovation) > ROBOT_KALMAN_MAX_INNOVATION_DEG * DEG_TO_RAD ||
      innovation * innovation > ROBOT_KALMAN_INNOVATION_SIGMA *
                                ROBOT_KALMAN_INNOVATION_SIGMA * s);
  if (rejected)
  {
    p00 = pp00;
    p01 = pp01;
    p11 = pp11;
  }
  else
  {
    correction = k0 * innovation;
    if (fabsf(correction) > ROBOT_KALMAN_MAX_CORRECTION_DEG * DEG_TO_RAD)
    {
      /* Increase R instead of clipping the state with inconsistent covariance. */
      r = fmaxf(r, pp00 * (fabsf(innovation) /
                           (ROBOT_KALMAN_MAX_CORRECTION_DEG * DEG_TO_RAD) - 1.0f));
      s = pp00 + r;
      k0 = pp00 / s;
      k1 = pp01 / s;
      correction = k0 * innovation;
    }
    gyro_bias += k1 * innovation;
    /* Joseph covariance update preserves symmetry and avoids cancellation. */
    float a = 1.0f - k0;
    p00 = a * a * pp00 + k0 * k0 * r;
    p01 = a * (pp01 - k1 * pp00) + k0 * k1 * r;
    p11 = pp11 - 2.0f * k1 * pp01 + k1 * k1 * (pp00 + r);
  }
  pitch_rad = prediction + correction;
  return isfinite(pitch_rad) && isfinite(gyro_bias) && isfinite(p00) && isfinite(p01) &&
                 isfinite(p11) ? HAL_OK : HAL_ERROR;
}

static float filter_pitch_rate(float rate)
{
  if (!rate_filter_initialized)
  {
    filtered_rate_rad_s = rate;
    rate_filter_initialized = 1U;
  }
  else
  {
    const float tau = 1.0f / (2.0f * ROBOT_PI * ROBOT_CONTROL_RATE_FILTER_HZ);
    const float alpha = ROBOT_CONTROL_DT / (tau + ROBOT_CONTROL_DT);
    filtered_rate_rad_s += alpha * (rate - filtered_rate_rad_s);
  }
  return filtered_rate_rad_s;
}

static void offset_kf_reset(float gyro_rate)
{
  const float pitch_std = ROBOT_OFFSET_KF_INIT_PITCH_STD_DEG * DEG_TO_RAD;
  const float rate_std = ROBOT_OFFSET_KF_INIT_RATE_STD_DEG * DEG_TO_RAD;
  const float bias_std = ROBOT_OFFSET_KF_INIT_BIAS_STD_DEG * DEG_TO_RAD;
  const float offset_std = ROBOT_OFFSET_KF_INIT_OFFSET_STD_DEG * DEG_TO_RAD;
  memset(offset_kf_p, 0, sizeof(offset_kf_p));
  offset_kf_p[0][0] = pitch_std * pitch_std;
  offset_kf_p[1][1] = rate_std * rate_std;
  offset_kf_p[2][2] = bias_std * bias_std;
  offset_kf_p[3][3] = offset_std * offset_std;
  offset_kf_x[0] = pitch_rad;
  offset_kf_x[1] = gyro_rate - gyro_bias;
  offset_kf_x[2] = gyro_bias;
  offset_kf_x[3] = BALANCE_TRIM_RAD;
  offset_kf_accel_m_s2 = 0.0f;
  offset_kf_initialized = 1U;
}

/* Scalar measurement update (Joseph form); returns 0 if the innovation gate
   rejected it. */
static uint8_t offset_kf_measure(const float h[4], float residual, float variance,
                                 uint8_t gated)
{
  float ph[4];
  float s = variance;
  for (uint32_t i = 0U; i < 4U; i++)
  {
    ph[i] = 0.0f;
    for (uint32_t j = 0U; j < 4U; j++)
      ph[i] += offset_kf_p[i][j] * h[j];
    s += h[i] * ph[i];
  }
  if (!(s > 0.0f) || (gated && residual * residual > ROBOT_OFFSET_KF_INNOVATION_SIGMA *
                                                         ROBOT_OFFSET_KF_INNOVATION_SIGMA * s))
    return 0U;
  float gain[4];
  for (uint32_t i = 0U; i < 4U; i++)
  {
    gain[i] = ph[i] / s;
    offset_kf_x[i] += gain[i] * residual;
  }
  for (uint32_t i = 0U; i < 4U; i++)
  {
    for (uint32_t j = 0U; j < 4U; j++)
      offset_kf_p[i][j] += -gain[i] * ph[j] - ph[i] * gain[j] + gain[i] * s * gain[j];
  }
  for (uint32_t i = 0U; i < 4U; i++)
  {
    for (uint32_t j = i + 1U; j < 4U; j++)
    {
      float mean = 0.5f * (offset_kf_p[i][j] + offset_kf_p[j][i]);
      offset_kf_p[i][j] = offset_kf_p[j][i] = mean;
    }
  }
  return 1U;
}

/* Model: theta_ddot = alpha * (theta - d) - damping * theta_dot - beta * a.
   The gyro reads rate + bias; the accelerometer angle reads
   pitch - (a + h * pitch_ddot) / g, so it also sees the balance point d. */
static HAL_StatusTypeDef offset_kf_update(float gyro_rate, float accel_angle, float norm_error,
                                          float motion_accel, uint8_t armed)
{
  const float dt = ROBOT_CONTROL_DT;
  const float g = GRAVITY_M_S2;
  if (!armed || !offset_kf_initialized)
  {
    offset_kf_reset(gyro_rate);
    return HAL_OK;
  }
  /* Same first-order wheel lag as the simulation, driven by the last command. */
  offset_kf_accel_m_s2 += (motion_accel - offset_kf_accel_m_s2) * dt /
                          (ROBOT_MOTOR_TIME_CONSTANT_S + dt);
  const float a = offset_kf_accel_m_s2;
  const float f[4][4] = {
      {1.0f, dt, 0.0f, 0.0f},
      {ROBOT_PLANT_ALPHA * dt, 1.0f - ROBOT_PLANT_DAMPING * dt, 0.0f, -ROBOT_PLANT_ALPHA * dt},
      {0.0f, 0.0f, 1.0f, 0.0f},
      {0.0f, 0.0f, 0.0f, 1.0f},
  };
  float x[4];
  for (uint32_t i = 0U; i < 4U; i++)
  {
    x[i] = 0.0f;
    for (uint32_t j = 0U; j < 4U; j++)
      x[i] += f[i][j] * offset_kf_x[j];
  }
  x[1] -= ROBOT_PLANT_BETA * a * dt;
  memcpy(offset_kf_x, x, sizeof(x));
  float fp[4][4];
  for (uint32_t i = 0U; i < 4U; i++)
  {
    for (uint32_t j = 0U; j < 4U; j++)
    {
      fp[i][j] = 0.0f;
      for (uint32_t k = 0U; k < 4U; k++)
        fp[i][j] += f[i][k] * offset_kf_p[k][j];
    }
  }
  for (uint32_t i = 0U; i < 4U; i++)
  {
    for (uint32_t j = 0U; j < 4U; j++)
    {
      float sum = 0.0f;
      for (uint32_t k = 0U; k < 4U; k++)
        sum += fp[i][k] * f[j][k];
      offset_kf_p[i][j] = sum;
    }
  }
  const float torque = ROBOT_OFFSET_KF_TORQUE_NOISE;
  const float bias_walk = ROBOT_OFFSET_KF_BIAS_WALK_DEG * DEG_TO_RAD;
  const float offset_walk = ROBOT_OFFSET_KF_OFFSET_WALK_DEG * DEG_TO_RAD;
  offset_kf_p[1][1] += torque * torque * dt;
  offset_kf_p[2][2] += bias_walk * bias_walk * dt;
  offset_kf_p[3][3] += offset_walk * offset_walk * dt;

  const float gyro_std = ROBOT_OFFSET_KF_GYRO_STD_DEG * DEG_TO_RAD;
  const float h_gyro[4] = {0.0f, 1.0f, 1.0f, 0.0f};
  (void)offset_kf_measure(h_gyro, gyro_rate - (offset_kf_x[1] + offset_kf_x[2]),
                          gyro_std * gyro_std, 0U);
  if (norm_error <= ROBOT_KALMAN_MAX_NORM_ERROR_G)
  {
    const float k = ROBOT_IMU_HEIGHT_M / g;
    const float h_accel[4] = {1.0f - k * ROBOT_PLANT_ALPHA, k * ROBOT_PLANT_DAMPING, 0.0f,
                              k * ROBOT_PLANT_ALPHA};
    const float accel_std = ROBOT_OFFSET_KF_ACCEL_STD_DEG * DEG_TO_RAD;
    float predicted = -(1.0f - ROBOT_IMU_HEIGHT_M * ROBOT_PLANT_BETA) / g * a;
    for (uint32_t i = 0U; i < 4U; i++)
      predicted += h_accel[i] * offset_kf_x[i];
    (void)offset_kf_measure(h_accel, accel_angle - predicted, accel_std * accel_std, 1U);
  }
  for (uint32_t i = 0U; i < 4U; i++)
  {
    if (!isfinite(offset_kf_x[i]) || !isfinite(offset_kf_p[i][i]) || offset_kf_p[i][i] < 0.0f)
      return HAL_ERROR;
  }
  return HAL_OK;
}

void estimator_set_mode(EstimatorMode new_mode)
{
  mode = new_mode;
  pitch_initialized = 0U;
  rate_filter_initialized = 0U;
  offset_kf_initialized = 0U;
}

EstimatorMode estimator_mode(void)
{
  return mode;
}

const char *estimator_mode_name(void)
{
  return mode == ESTIMATOR_OFFSET_KF ? "offsetkf" :
         mode == ESTIMATOR_KALMAN ? "kalman" : "complementary";
}

HAL_StatusTypeDef estimator_update(const ImuSample *sample, float motion_accel_m_s2,
                                   uint8_t armed)
{
  float norm_error = fabsf(sample->accel_norm_mg / 1000.0f - 1.0f);
  /* The previous tick's command is the acceleration the wheels execute now. */
  float compensated_angle = sample->accel_pitch_rad +
      ROBOT_ACCEL_MOTION_COMPENSATION * motion_accel_m_s2 / GRAVITY_M_S2;
  if (estimate_pitch(sample->rate_rad_s, compensated_angle, norm_error, motion_accel_m_s2,
                     armed) != HAL_OK)
  {
    fault_latch("estimator");
    return HAL_ERROR;
  }
  if (offset_kf_update(sample->rate_rad_s, sample->accel_pitch_rad, norm_error,
                       motion_accel_m_s2, armed) != HAL_OK)
  {
    if (mode == ESTIMATOR_OFFSET_KF)
    {
      fault_latch("offset_kf");
      return HAL_ERROR;
    }
    /* Not used by the controller: restart it instead of stopping a good controller. */
    offset_kf_reset(sample->rate_rad_s);
  }
  float bias = mode == ESTIMATOR_OFFSET_KF ? offset_kf_x[2] : gyro_bias;
  (void)filter_pitch_rate(sample->rate_rad_s - bias);
  return HAL_OK;
}

float estimator_pitch(void)
{
  return pitch_rad;
}

float estimator_control_pitch(void)
{
  return mode == ESTIMATOR_OFFSET_KF ? offset_kf_x[0] - offset_kf_x[3] : pitch_rad;
}

float estimator_pitch_reference(void)
{
  return mode == ESTIMATOR_OFFSET_KF ? 0.0f : BALANCE_TRIM_RAD;
}

float estimator_control_rate(void)
{
  return filtered_rate_rad_s;
}

float estimator_integral_scale(void)
{
  return mode == ESTIMATOR_OFFSET_KF ? ROBOT_OFFSET_KF_POSITION_INTEGRAL_SCALE : 1.0f;
}
