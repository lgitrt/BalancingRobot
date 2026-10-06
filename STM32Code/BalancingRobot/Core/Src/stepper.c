/* Author: Luca Obwegs */
#include "stepper.h"

#include "fault.h"
#include "robot_config.h"

#include <math.h>

#define STEP_PULSE_US 3U

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint32_t channel;
  GPIO_TypeDef *direction_port;
  uint16_t direction_pin;
  float motor_sign;
  int8_t direction;
  float speed_m_s;
  uint8_t output_running;
} StepperWheel;

static StepperWheel left_wheel = {
    .timer = &htim2, .channel = TIM_CHANNEL_1,
    .direction_port = dir_left_GPIO_Port, .direction_pin = dir_left_Pin,
    .motor_sign = ROBOT_LEFT_MOTOR_SIGN};
static StepperWheel right_wheel = {
    .timer = &htim1, .channel = TIM_CHANNEL_1,
    .direction_port = dir_right_GPIO_Port, .direction_pin = dir_right_Pin,
    .motor_sign = ROBOT_RIGHT_MOTOR_SIGN};

static float meters_per_step(void)
{
  return (2.0f * ROBOT_PI * ROBOT_WHEEL_RADIUS_M * (ROBOT_STEP_ANGLE_DEG / 360.0f)) /
         (ROBOT_MICROSTEPS * ROBOT_GEAR_RATIO);
}

static void wheel_stop_output(StepperWheel *wheel)
{
  __HAL_TIM_SET_COMPARE(wheel->timer, wheel->channel, 0U);
  if (wheel->output_running)
  {
    (void)HAL_TIM_PWM_Stop(wheel->timer, wheel->channel);
    wheel->output_running = 0U;
  }
}

static void wheel_apply_speed(StepperWheel *wheel, float speed_m_s)
{
  float signed_speed = speed_m_s * wheel->motor_sign;
  float max_step_rate = fminf(ROBOT_MAX_STEP_RATE_HZ, ROBOT_MAX_SPEED_M_S / meters_per_step());
  float step_rate = fabsf(signed_speed) / meters_per_step();
  if (step_rate > max_step_rate)
  {
    step_rate = max_step_rate;
    signed_speed = copysignf(step_rate * meters_per_step(), signed_speed);
  }
  if (step_rate < 1.0f)
  {
    __HAL_TIM_SET_COMPARE(wheel->timer, wheel->channel, 0U);
    wheel->speed_m_s = 0.0f;
    return;
  }

  uint32_t period_counts = (uint32_t)(1000000.0f / step_rate + 0.5f);
  if (period_counts < STEP_PULSE_US + 2U)
    period_counts = STEP_PULSE_US + 2U;
  if (period_counts > 65536U)
  {
    __HAL_TIM_SET_COMPARE(wheel->timer, wheel->channel, 0U);
    wheel->speed_m_s = 0.0f;
    return;
  }

  int8_t direction = signed_speed >= 0.0f ? 1 : -1;
  if (direction != wheel->direction)
  {
    /* Change DIR only while the output is stopped so no step uses the old level. */
    wheel_stop_output(wheel);
    HAL_GPIO_WritePin(wheel->direction_port, wheel->direction_pin,
                      direction > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    wheel->direction = direction;
    __HAL_TIM_SET_COUNTER(wheel->timer, 0U);
  }

  __HAL_TIM_SET_AUTORELOAD(wheel->timer, period_counts - 1U);
  __HAL_TIM_SET_COMPARE(wheel->timer, wheel->channel, STEP_PULSE_US);
  if (!wheel->output_running)
  {
    if (HAL_TIM_GenerateEvent(wheel->timer, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
    {
      fault_latch("timer");
      stepper_disable();
      return;
    }
    __HAL_TIM_SET_COUNTER(wheel->timer, 0U);
    if (HAL_TIM_PWM_Start(wheel->timer, wheel->channel) != HAL_OK)
    {
      fault_latch("pwm");
      stepper_disable();
      return;
    }
    wheel->output_running = 1U;
  }
  wheel->speed_m_s = (float)direction * (1000000.0f / (float)period_counts) *
                     meters_per_step() * wheel->motor_sign;
}

void stepper_init(void)
{
  stepper_disable();
  __HAL_TIM_ENABLE_OCxPRELOAD(right_wheel.timer, right_wheel.channel);
  __HAL_TIM_ENABLE_OCxPRELOAD(left_wheel.timer, left_wheel.channel);
  if (HAL_TIM_PWM_Start(right_wheel.timer, right_wheel.channel) == HAL_OK)
    right_wheel.output_running = 1U;
  else
    fault_latch("pwm_right");
  if (HAL_TIM_PWM_Start(left_wheel.timer, left_wheel.channel) == HAL_OK)
    left_wheel.output_running = 1U;
  else
    fault_latch("pwm_left");
}

void stepper_enable(void)
{
  HAL_GPIO_WritePin(motor_enable1_GPIO_Port, motor_enable1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor_enable2_GPIO_Port, motor_enable2_Pin, GPIO_PIN_RESET);
}

void stepper_disable(void)
{
  wheel_stop_output(&right_wheel);
  wheel_stop_output(&left_wheel);
  right_wheel.speed_m_s = 0.0f;
  left_wheel.speed_m_s = 0.0f;
  HAL_GPIO_WritePin(motor_enable1_GPIO_Port, motor_enable1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor_enable2_GPIO_Port, motor_enable2_Pin, GPIO_PIN_SET);
}

void stepper_hold(void)
{
  __HAL_TIM_SET_COMPARE(right_wheel.timer, right_wheel.channel, 0U);
  __HAL_TIM_SET_COMPARE(left_wheel.timer, left_wheel.channel, 0U);
  right_wheel.speed_m_s = 0.0f;
  left_wheel.speed_m_s = 0.0f;
}

void stepper_set_speeds(float left_m_s, float right_m_s)
{
  wheel_apply_speed(&left_wheel, left_m_s);
  if (!fault_active())
    wheel_apply_speed(&right_wheel, right_m_s);
}

float stepper_left_speed(void)
{
  return left_wheel.speed_m_s;
}

float stepper_right_speed(void)
{
  return right_wheel.speed_m_s;
}
