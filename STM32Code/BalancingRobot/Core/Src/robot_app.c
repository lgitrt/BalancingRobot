/* Author: Luca Obwegs */
#include "robot_app.h"

#include "comm.h"
#include "controller.h"
#include "estimator.h"
#include "fault.h"
#include "imu.h"
#include "robot_config.h"
#include "stepper.h"
#include "wheel_feedback.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim6;

/* TIM6 runs at 500 Hz; the main loop does one control step per tick. */
static volatile uint32_t pending_ticks;
static volatile uint32_t tick_count;

static uint8_t armed;
static uint8_t fault_reported;
static float motion_accel_m_s2;
static float requested_speed_m_s;
static float requested_turn_rad_s;
static uint32_t last_command_tick;
static uint32_t telemetry_ticks;

static void reply(const char *text)
{
  (void)comm_send(text);
}

static void motors_disable(void)
{
  stepper_disable();
  wheelfb_stop();
  armed = 0U;
  motion_accel_m_s2 = 0.0f;
}

static void arm(void)
{
  float pitch = estimator_pitch();
  if (fault_active())
  {
    reply("ARM,REJECTED,fault\r\n");
    return;
  }
  if (fabsf(pitch) >= ROBOT_ARM_MAX_PITCH_RAD ||
      fabsf(pitch - ROBOT_BALANCE_TRIM_DEG * DEG_TO_RAD) >= ROBOT_ARM_MAX_PITCH_RAD)
  {
    reply("ARM,REJECTED,hold_upright\r\n");
    return;
  }
  wheelfb_zero();
  controller_reset();
  armed = 1U;
  telemetry_ticks = 0U;
  stepper_enable();
}

static void reply_estimator(void)
{
  char text[48];
  (void)snprintf(text, sizeof(text), "ESTIMATOR,mode=%s\r\n", estimator_mode_name());
  reply(text);
}

static void command_estimator(const char *argument)
{
  if (argument != NULL)
  {
    EstimatorMode mode;
    if (armed || fault_active())
    {
      reply("ESTIMATOR,REJECTED,stop_first\r\n");
      return;
    }
    if (strcmp(argument, "complementary") == 0)
      mode = ESTIMATOR_COMPLEMENTARY;
    else if (strcmp(argument, "kalman") == 0)
      mode = ESTIMATOR_KALMAN;
    else if (strcmp(argument, "offsetkf") == 0)
      mode = ESTIMATOR_OFFSET_KF;
    else
    {
      reply("ESTIMATOR,REJECTED,use_complementary_kalman_or_offsetkf\r\n");
      return;
    }
    estimator_set_mode(mode);
  }
  reply_estimator();
}

static void command_wheelfb(const char *argument)
{
  if (argument != NULL)
  {
    if (armed)
    {
      reply("WHEELFB,REJECTED,stop_first\r\n");
      return;
    }
    if (strcmp(argument, "kf") == 0)
      wheelfb_select_kf(1U);
    else if (strcmp(argument, "steps") == 0)
      wheelfb_select_kf(0U);
    else
    {
      reply("WHEELFB,REJECTED,use_kf_or_steps\r\n");
      return;
    }
  }
  reply(wheelfb_kf_selected() ? "WHEELFB,mode=kf\r\n" : "WHEELFB,mode=steps\r\n");
}

static void command_hold(const char *argument)
{
  if (argument != NULL)
  {
    if (armed || fault_active())
    {
      reply("HOLD,REJECTED,stop_first\r\n");
      return;
    }
    if (strcmp(argument, "on") == 0)
      controller_set_position_hold(1U);
    else if (strcmp(argument, "off") == 0)
      controller_set_position_hold(0U);
    else
    {
      reply("HOLD,REJECTED,use_on_or_off\r\n");
      return;
    }
  }
  reply(controller_position_hold() ? "HOLD,enabled=1\r\n" : "HOLD,enabled=0\r\n");
}

/* "v <m/s>" forward speed or "t <rad/s>" turn rate. The values expire after
   ROBOT_COMMAND_TIMEOUT_TICKS, so the sender has to repeat them. */
static void command_motion(char kind, const char *number)
{
  char *end = NULL;
  float value = strtof(number, &end);
  if (end == number)
    return;
  while (*end == ' ')
    end++;
  if (*end != '\0' || !isfinite(value))
    return;
  if (kind == 'v')
    requested_speed_m_s = fmaxf(-ROBOT_MAX_SPEED_M_S, fminf(ROBOT_MAX_SPEED_M_S, value));
  else
    requested_turn_rad_s =
        fmaxf(-ROBOT_MAX_TURN_RATE_RAD_S, fminf(ROBOT_MAX_TURN_RATE_RAD_S, value));
  last_command_tick = tick_count;
}

/* Returns the text after "<name>" or "<name> ", NULL if there is none, or
   sets *match to 0 if the line is a different command. */
static const char *command_argument(const char *line, const char *name, uint8_t *match)
{
  size_t length = strlen(name);
  *match = 0U;
  if (strncmp(line, name, length) != 0)
    return NULL;
  if (line[length] == '\0')
  {
    *match = 1U;
    return NULL;
  }
  if (line[length] != ' ')
    return NULL;
  *match = 1U;
  return &line[length + 1U];
}

static void process_command(const char *line)
{
  uint8_t match;
  const char *argument;
  if (strcmp(line, "arm") == 0)
    arm();
  else if (strcmp(line, "disarm") == 0 || strcmp(line, "stop") == 0)
  {
    motors_disable();
    requested_speed_m_s = 0.0f;
    requested_turn_rad_s = 0.0f;
  }
  else if (strcmp(line, "pid") == 0)
    controller_set_type(CONTROLLER_PID);
  else if (strcmp(line, "lqr") == 0)
    controller_set_type(CONTROLLER_LQR);
  else if ((line[0] == 'v' || line[0] == 't') && line[1] == ' ')
    command_motion(line[0], &line[2]);
  else if ((argument = command_argument(line, "estimator", &match)), match)
    command_estimator(argument);
  else if ((argument = command_argument(line, "wheelfb", &match)), match)
    command_wheelfb(argument);
  else if ((argument = command_argument(line, "hold", &match)), match)
    command_hold(argument);
}

static void send_telemetry(void)
{
  char text[96];
  (void)snprintf(text, sizeof(text), "T,%ld,%ld,%ld,%ld,%u,%u\r\n",
                 (long)(estimator_pitch() * 10000.0f), (long)(wheelfb_position() * 10000.0f),
                 (long)(wheelfb_velocity() * 10000.0f),
                 (long)(controller_velocity() * 10000.0f), (unsigned)controller_type(),
                 (unsigned)armed);
  reply(text);
}

static void control_step(void)
{
  ImuSample sample;
  if (imu_read(&sample) != HAL_OK)
  {
    fault_latch("imu");
    return;
  }
  if (estimator_update(&sample, motion_accel_m_s2, armed) != HAL_OK)
    return;

  if ((uint32_t)(tick_count - last_command_tick) >= ROBOT_COMMAND_TIMEOUT_TICKS)
  {
    requested_speed_m_s = 0.0f;
    requested_turn_rad_s = 0.0f;
  }
  if (fabsf(estimator_pitch()) >= ROBOT_FALL_ANGLE_RAD)
  {
    fault_latch("fall");
    return;
  }

  if (!armed)
  {
    (void)wheelfb_begin_tick(0U, 0.0f);
    stepper_hold();
    controller_set_velocity(0.0f);
    motion_accel_m_s2 = 0.0f;
    wheelfb_end_tick(0U, 0.0f, 0.0f);
    return;
  }

  controller_set_velocity(wheelfb_begin_tick(1U, controller_velocity()));
  const ControllerInput input = {
      .position_m = wheelfb_position(),
      .velocity_m_s = wheelfb_velocity(),
      .requested_speed_m_s = requested_speed_m_s,
      .pitch_rad = estimator_control_pitch(),
      .pitch_reference_rad = estimator_pitch_reference(),
      .pitch_rate_rad_s = estimator_control_rate(),
      .trim_integral_scale = estimator_integral_scale()};
  /* The estimators use this acceleration as the wheel motion of the next tick. */
  motion_accel_m_s2 = controller_update(&input);
  float wheel_offset = requested_turn_rad_s * ROBOT_WHEEL_SEPARATION_M * 0.5f;
  stepper_set_speeds(controller_velocity() - wheel_offset,
                     controller_velocity() + wheel_offset);
  wheelfb_end_tick((uint8_t)!fault_active(), stepper_left_speed(), stepper_right_speed());
}

HAL_StatusTypeDef Robot_App_Init(void)
{
  stepper_init();
  if (!fault_active())
    (void)imu_init();
  if (comm_init() != HAL_OK)
    fault_latch("uart_rx");
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    fault_latch("tim6");
  if (fault_active())
  {
    char text[40];
    stepper_disable();
    if (strcmp(fault_reason(), "imu_id") == 0)
      (void)snprintf(text, sizeof(text), "FAULT,imu_id,0x%02X\r\n", imu_who_am_i());
    else
      (void)snprintf(text, sizeof(text), "FAULT,%s\r\n", fault_reason());
    comm_send_blocking(text);
    fault_reported = 1U;
    return HAL_ERROR;
  }
  return HAL_OK;
}

void Robot_App_Run(void)
{
  char line[COMM_LINE_SIZE];
  while (comm_take_line(line))
    process_command(line);
  const char *event = wheelfb_take_event();
  if (event != NULL)
    reply(event);
  comm_poll();

  uint32_t ticks;
  __disable_irq();
  ticks = pending_ticks;
  pending_ticks = 0U;
  __enable_irq();
  if (ticks == 0U)
  {
    __WFI();
    return;
  }
  if (ticks > 1U)
    fault_latch("timing");

  if (!fault_active())
    control_step();
  if (fault_active())
  {
    motors_disable();
    if (!fault_reported)
    {
      char text[40];
      (void)snprintf(text, sizeof(text), "FAULT,%s\r\n", fault_reason());
      fault_reported = comm_send(text);
    }
  }

  /* Also sent while disarmed so the teleop app shows the state and the pitch. */
  if (++telemetry_ticks >= ROBOT_TELEMETRY_TICKS)
  {
    telemetry_ticks = 0U;
    send_telemetry();
  }
  (void)HAL_IWDG_Refresh(&hiwdg);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
  if (timer->Instance == TIM6)
  {
    pending_ticks++;
    tick_count++;
  }
}
