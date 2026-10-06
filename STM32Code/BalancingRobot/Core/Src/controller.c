/* Author: Luca Obwegs */
#include "controller.h"

#include "robot_config.h"

#include <math.h>

static ControllerType active_type = CONTROLLER_LQR;
static uint8_t position_hold = ROBOT_POSITION_HOLD_ENABLED;
static float reference_position_m;
/* LQR position integral, expressed as a pitch setpoint offset. */
static float position_trim_rad;
static float pitch_integral;
static float velocity_m_s;

static float clampf(float value, float limit)
{
  return fmaxf(-limit, fminf(limit, value));
}

void controller_set_type(ControllerType type)
{
  active_type = type;
  pitch_integral = 0.0f;
}

ControllerType controller_type(void)
{
  return active_type;
}

const char *controller_name(void)
{
  return active_type == CONTROLLER_LQR ? "lqr" : "pid";
}

void controller_set_position_hold(uint8_t enabled)
{
  position_hold = enabled ? 1U : 0U;
}

uint8_t controller_position_hold(void)
{
  return position_hold;
}

void controller_reset(void)
{
  reference_position_m = 0.0f;
  position_trim_rad = 0.0f;
  pitch_integral = 0.0f;
  velocity_m_s = 0.0f;
}

static float lqr_update(const ControllerInput *in, float position_error)
{
  const float dt = ROBOT_CONTROL_DT;
  float velocity_error = in->velocity_m_s - in->requested_speed_m_s;
  float pitch_error = in->pitch_rad - (in->pitch_reference_rad + position_trim_rad);
  float acceleration = -(ROBOT_LQR_K_POSITION * position_error +
                         ROBOT_LQR_K_VELOCITY * velocity_error +
                         ROBOT_LQR_K_PITCH * pitch_error +
                         ROBOT_LQR_K_PITCH_RATE * in->pitch_rate_rad_s);
  /* Integral state: a lasting position error means the balance point is off. */
  if (position_hold && fabsf(acceleration) < ROBOT_MAX_ACCEL_M_S2)
  {
    position_trim_rad = clampf(position_trim_rad - in->trim_integral_scale *
                                   (ROBOT_LQR_K_INTEGRAL / ROBOT_LQR_K_PITCH) *
                                   position_error * dt,
                               ROBOT_POSITION_HOLD_MAX_TRIM_DEG * DEG_TO_RAD);
  }
  return acceleration;
}

static float pid_update(const ControllerInput *in, float position_offset)
{
  const float dt = ROBOT_CONTROL_DT;
  float position_correction = position_hold ?
      clampf(-ROBOT_POSITION_HOLD_KP * position_offset, ROBOT_POSITION_HOLD_MAX_SPEED_M_S) :
      0.0f;
  float velocity_error = in->velocity_m_s - (in->requested_speed_m_s + position_correction);
  float pitch_error = in->pitch_rad - in->pitch_reference_rad;
  float next_integral = clampf(pitch_integral + pitch_error * dt, 0.5f);
  /* Positive velocity feedback: to slow down, the wheels first have to move
     under the centre of mass so that the robot leans back. */
  float acceleration = ROBOT_PID_PITCH_KP * pitch_error +
                       ROBOT_PID_PITCH_KI * next_integral +
                       ROBOT_PID_PITCH_KD * in->pitch_rate_rad_s +
                       ROBOT_PID_VELOCITY_KP * velocity_error;
  /* Anti-windup: only integrate while the output is not saturated. */
  if (fabsf(acceleration) < ROBOT_MAX_ACCEL_M_S2)
    pitch_integral = next_integral;
  return acceleration;
}

float controller_update(const ControllerInput *in)
{
  const float dt = ROBOT_CONTROL_DT;
  reference_position_m += in->requested_speed_m_s * dt;
  float position_offset = in->position_m - reference_position_m;
  float acceleration;
  if (active_type == CONTROLLER_LQR)
  {
    float position_error = position_hold ?
        clampf(position_offset, ROBOT_POSITION_HOLD_MAX_ERROR_M) : 0.0f;
    acceleration = lqr_update(in, position_error);
  }
  else
  {
    acceleration = pid_update(in, position_offset);
  }
  acceleration = clampf(acceleration, ROBOT_MAX_ACCEL_M_S2);
  velocity_m_s = clampf(velocity_m_s + acceleration * dt, ROBOT_MAX_SPEED_M_S);
  return acceleration;
}

float controller_velocity(void)
{
  return velocity_m_s;
}

void controller_set_velocity(float new_velocity_m_s)
{
  velocity_m_s = new_velocity_m_s;
}
