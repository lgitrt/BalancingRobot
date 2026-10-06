/* Author: Luca Obwegs */
#include "wheel_feedback.h"

#include "encoder.h"
#include "robot_config.h"
#include "wheel_kf.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

typedef enum
{
  PREFLIGHT_START = 0, /* start the first encoder read */
  PREFLIGHT_BASELINE,  /* collect it and initialise the filters */
  PREFLIGHT_RUNNING
} PreflightState;

static WheelKf wheel_kf[ENCODER_COUNT];
static WheelEncoder wheel_encoder[ENCODER_COUNT];
static uint8_t kf_selected = ROBOT_WHEELFB_KF_DEFAULT;
static uint8_t kf_active;
static uint8_t encoders_on;
static uint8_t was_armed;
static uint8_t preflight;
static uint8_t read_pending;
static uint8_t sample_ticks;
static uint8_t resync_pending[ENCODER_COUNT];
static float applied_rad_s[ENCODER_COUNT];
static float step_position_m;
static float step_velocity_m_s;
static float estimated_position_m;
static float estimated_velocity_m_s;
/* KF position minus step position at the hand-over, so the estimate does not jump. */
static float kf_offset_m;
static char event_text[128];
static uint8_t event_pending;

static void set_event(const char *text)
{
  (void)snprintf(event_text, sizeof(event_text), "%s\r\n", text);
  event_pending = 1U;
}

static void filters_init(void)
{
  static const float coefficients[ENCODER_COUNT][4] = {ROBOT_ENCODER_LEFT_ANGLE_ERR_COUNTS,
                                                       ROBOT_ENCODER_RIGHT_ANGLE_ERR_COUNTS};
  static const float signs[ENCODER_COUNT] = {ROBOT_ENCODER_LEFT_SIGN, ROBOT_ENCODER_RIGHT_SIGN};
  const WheelKfConfig config = {
      .q_rad2_s3 = ROBOT_ENCODER_KF_Q_RAD2_S3,
      .r_rad2 = ROBOT_ENCODER_KF_R_RAD2,
      .delay_s = ROBOT_ENCODER_KF_DELAY_S,
      .gate_sigma = ROBOT_ENCODER_KF_GATE_SIGMA,
      .gate_min_rad = ROBOT_ENCODER_KF_GATE_MIN_RAD,
      .sample_period_s = (float)ROBOT_ENCODER_SAMPLE_TICKS * ROBOT_CONTROL_DT,
      .initial_speed_var = ROBOT_ENCODER_KF_INITIAL_SPEED_VAR,
      .max_rejects = ROBOT_ENCODER_KF_MAX_REJECTS};
  for (uint8_t w = 0U; w < ENCODER_COUNT; w++)
  {
    wheel_encoder_init(&wheel_encoder[w], coefficients[w], signs[w], encoder_angle(w));
    wheel_kf_init(&wheel_kf[w], &config, wheel_encoder_angle_rad(&wheel_encoder[w]));
  }
}

/* First armed tick of a balancing session. */
static void session_start(void)
{
  encoders_on = 1U;
  preflight = PREFLIGHT_START;
  read_pending = 0U;
  sample_ticks = 0U;
  kf_active = 0U;
  kf_offset_m = 0.0f;
  memset(resync_pending, 0, sizeof(resync_pending));
  memset(applied_rad_s, 0, sizeof(applied_rad_s));
  encoder_reset_errors();
  step_position_m = estimated_position_m;
  step_velocity_m_s = 0.0f;
}

static void fallback(const char *reason)
{
  if (!encoders_on)
    return;
  encoders_on = 0U;
  preflight = PREFLIGHT_START;
  read_pending = 0U;
  kf_active = 0U;
  /* Continue from the last estimate so position hold does not jump. */
  step_position_m = estimated_position_m;
  (void)snprintf(event_text, sizeof(event_text),
                 "WHEELFB,FALLBACK,feedback=steps,reason=%s,motors=STILL_ARMED\r\n", reason);
  event_pending = 1U;
}

static void check_encoder_failure(void)
{
  const char *failure = encoder_take_failure();
  if (failure != NULL)
    fallback(failure);
}

static void set_estimate(void)
{
  if (!kf_active)
    return;
  estimated_velocity_m_s =
      ROBOT_WHEEL_RADIUS_M * 0.5f * (wheel_kf[0].speed_rad_s + wheel_kf[1].speed_rad_s);
  estimated_position_m =
      ROBOT_WHEEL_RADIUS_M * 0.5f * (wheel_kf[0].angle_rad + wheel_kf[1].angle_rad) -
      kf_offset_m;
}

/* A stalled stepper cannot jump back to the commanded rate: restart the slipped
   wheel from the speed its encoder sees, never reversed and never faster than
   its command, and pull the common controller speed down with it. */
static float reramp(float controller_velocity_m_s)
{
  uint8_t any = 0U;
  float best = 0.0f;
  for (uint8_t w = 0U; w < ENCODER_COUNT; w++)
  {
    WheelKf *kf = &wheel_kf[w];
    float cmd = applied_rad_s[w];
    uint8_t resync = resync_pending[w];
    resync_pending[w] = 0U;
    if (!resync && fabsf(cmd - kf->speed_rad_s) <= ROBOT_WHEELFB_STALL_SPEED_RAD_S)
      continue;
    float measured = kf->speed_rad_s;
    float restart = 0.0f;
    if (measured * cmd > 0.0f)
      restart = fabsf(measured) < fabsf(cmd) ? measured : cmd;
    if (resync)
      kf->angle_rad += kf->config.delay_s * (restart - measured);
    kf->speed_rad_s = restart;
    applied_rad_s[w] = restart;
    /* Each wheel command is the controller speed -/+ the turn offset. */
    float candidate = controller_velocity_m_s + (restart - cmd) * ROBOT_WHEEL_RADIUS_M;
    if (!any || fabsf(candidate) < fabsf(best))
      best = candidate;
    any = 1U;
  }
  return any ? best : controller_velocity_m_s;
}

static void baseline(void)
{
  uint8_t valid = encoder_collect_read(0U);
  valid &= encoder_collect_read(1U);
  check_encoder_failure();
  if (!encoders_on)
    return;
  if (!valid)
  {
    preflight = PREFLIGHT_START;
    return;
  }
  filters_init();
  float mean_angle = 0.0f;
  for (uint8_t w = 0U; w < ENCODER_COUNT; w++)
  {
    wheel_kf[w].speed_rad_s = applied_rad_s[w];
    wheel_kf[w].angle_rad += ROBOT_ENCODER_KF_DELAY_S * applied_rad_s[w];
    mean_angle += 0.5f * wheel_kf[w].angle_rad;
  }
  /* Hand over without a jump: the KF position starts at the step position. */
  kf_offset_m = ROBOT_WHEEL_RADIUS_M * mean_angle - step_position_m;
  preflight = PREFLIGHT_RUNNING;
  sample_ticks = 0U;
  kf_active = kf_selected;
  set_event(kf_active ? "WHEELFB,ACTIVE,feedback=kf" :
                        "WHEELFB,ACTIVE,feedback=steps,encoders=logging");
  set_estimate();
}

void wheelfb_select_kf(uint8_t use_kf)
{
  kf_selected = use_kf ? 1U : 0U;
}

uint8_t wheelfb_kf_selected(void)
{
  return kf_selected;
}

void wheelfb_zero(void)
{
  estimated_position_m = 0.0f;
  estimated_velocity_m_s = 0.0f;
  step_position_m = 0.0f;
  step_velocity_m_s = 0.0f;
}

void wheelfb_stop(void)
{
  encoders_on = 0U;
  was_armed = 0U;
  kf_active = 0U;
  preflight = PREFLIGHT_START;
  read_pending = 0U;
}

float wheelfb_begin_tick(uint8_t armed, float controller_velocity_m_s)
{
  if (armed && !was_armed)
    session_start();
  was_armed = armed;
  if (!armed || !encoders_on)
    return controller_velocity_m_s;
  if (preflight == PREFLIGHT_START)
  {
    uint8_t started = encoder_start_read(0U);
    started &= encoder_start_read(1U);
    check_encoder_failure();
    if (encoders_on)
      preflight = started ? PREFLIGHT_BASELINE : PREFLIGHT_START;
    return controller_velocity_m_s;
  }
  if (preflight == PREFLIGHT_BASELINE)
  {
    baseline();
    return controller_velocity_m_s;
  }
  if (read_pending)
  {
    read_pending = 0U;
    uint8_t valid = encoder_collect_read(0U);
    valid &= encoder_collect_read(1U);
    check_encoder_failure();
    if (!encoders_on)
      return controller_velocity_m_s;
    if (valid)
    {
      for (uint8_t w = 0U; w < ENCODER_COUNT; w++)
      {
        float measured = wheel_encoder_update(&wheel_encoder[w], encoder_angle(w));
        if (wheel_kf_update(&wheel_kf[w], measured) == WHEEL_KF_RESYNC)
          resync_pending[w] = 1U;
      }
      if (kf_active)
        controller_velocity_m_s = reramp(controller_velocity_m_s);
      else
        memset(resync_pending, 0, sizeof(resync_pending));
    }
  }
  set_estimate();
  return controller_velocity_m_s;
}

void wheelfb_end_tick(uint8_t armed, float left_m_s, float right_m_s)
{
  const float dt = ROBOT_CONTROL_DT;
  step_velocity_m_s = (left_m_s + right_m_s) * 0.5f;
  if (!armed)
  {
    estimated_velocity_m_s = step_velocity_m_s;
    estimated_position_m += step_velocity_m_s * dt;
    step_position_m = estimated_position_m;
    return;
  }
  step_position_m += step_velocity_m_s * dt;
  estimated_velocity_m_s = step_velocity_m_s;
  estimated_position_m = step_position_m;
  if (!encoders_on)
    return;
  const float applied[ENCODER_COUNT] = {left_m_s / ROBOT_WHEEL_RADIUS_M,
                                        right_m_s / ROBOT_WHEEL_RADIUS_M};
  for (uint8_t w = 0U; w < ENCODER_COUNT; w++)
  {
    if (preflight == PREFLIGHT_RUNNING)
      wheel_kf_predict(&wheel_kf[w], applied[w] - applied_rad_s[w], dt);
    applied_rad_s[w] = applied[w];
  }
  if (preflight != PREFLIGHT_RUNNING)
    return;
  set_estimate();
  if (++sample_ticks >= ROBOT_ENCODER_SAMPLE_TICKS)
  {
    sample_ticks = 0U;
    (void)encoder_start_read(0U);
    (void)encoder_start_read(1U);
    check_encoder_failure();
    read_pending = encoders_on;
  }
}

float wheelfb_position(void)
{
  return estimated_position_m;
}

float wheelfb_velocity(void)
{
  return estimated_velocity_m_s;
}

const char *wheelfb_take_event(void)
{
  if (!event_pending)
    return NULL;
  event_pending = 0U;
  return event_text;
}
