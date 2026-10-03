/* Author: Luca Obwegs */
#include "robot_app.h"
#include "main.h"
#include "robot_config.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ROBOT_STEP_PULSE_US 3U
#define ROBOT_RX_BUFFER_SIZE 64U
#define ROBOT_COMMAND_TIMEOUT_TICKS 250U
#define LSM6DS0_REG_WHO_AM_I 0x0FU
#define LSM6DS0_REG_CTRL_REG1_G 0x10U
#define LSM6DS0_REG_OUT_X_L_G 0x18U
#define LSM6DS0_REG_CTRL_REG4_G 0x1EU
#define LSM6DS0_REG_CTRL_REG6_XL 0x20U
#define LSM6DS0_REG_CTRL_REG8 0x22U
#define LSM6DS0_REG_OUT_X_L_XL 0x28U
#define DEG_TO_RAD 0.01745329252f
#define G_TO_M_S2 9.80665f

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef hlpuart1;
extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;

typedef enum
{
  ROBOT_CONTROLLER_PID = 0,
  ROBOT_CONTROLLER_LQR
} RobotController_t;

typedef enum
{
  ROBOT_DIAGNOSTIC_IDLE = 0,
  ROBOT_DIAGNOSTIC_LEFT_FORWARD,
  ROBOT_DIAGNOSTIC_LEFT_REVERSE,
  ROBOT_DIAGNOSTIC_RIGHT_FORWARD,
  ROBOT_DIAGNOSTIC_RIGHT_REVERSE
} RobotDiagnosticStage_t;

typedef enum
{
  ROBOT_DIAGNOSTIC_EVENT_NONE = 0,
  ROBOT_DIAGNOSTIC_EVENT_STARTED,
  ROBOT_DIAGNOSTIC_EVENT_FAULTED,
  ROBOT_DIAGNOSTIC_EVENT_REJECTED_ARMED,
  ROBOT_DIAGNOSTIC_EVENT_CANCELLED,
  ROBOT_DIAGNOSTIC_EVENT_REJECTED_TILT,
  ROBOT_DIAGNOSTIC_EVENT_COMPLETE
} RobotDiagnosticEvent_t;

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint32_t channel;
  GPIO_TypeDef *direction_port;
  uint16_t direction_pin;
  float motor_sign;
  int8_t direction;
  float estimated_speed_m_s;
  uint8_t output_running;
} RobotWheel_t;

static RobotWheel_t left_wheel = {
    .timer = &htim3, .channel = TIM_CHANNEL_1,
    .direction_port = GPIOA, .direction_pin = dir_left_Pin,
    .motor_sign = ROBOT_LEFT_MOTOR_SIGN};
static RobotWheel_t right_wheel = {
    .timer = &htim1, .channel = TIM_CHANNEL_1,
    .direction_port = dir_right_GPIO_Port, .direction_pin = dir_right_Pin,
    .motor_sign = ROBOT_RIGHT_MOTOR_SIGN};

static volatile uint32_t pending_control_ticks;
static volatile uint8_t application_fault;
static volatile uint8_t uart_tx_busy;
static volatile float requested_forward_speed;
static volatile float requested_turn_rate;
static volatile uint32_t last_command_tick;
static volatile uint8_t robot_armed;
static volatile RobotDiagnosticStage_t diagnostic_stage;
static volatile uint8_t diagnostic_event;
static volatile uint32_t control_tick_count;
static uint8_t uart_rx_byte;
static char uart_line[ROBOT_RX_BUFFER_SIZE];
static uint8_t uart_line_length;
static char uart_tx_buffer[128];
static RobotController_t active_controller = ROBOT_CONTROLLER_LQR;
static float gyro_bias_rad_s;
static float pitch_offset_rad;
static float pitch_rad;
static float pitch_integral;
static int16_t latest_gyro_raw[3];
static int16_t latest_accel_raw[3];
static float estimated_position_m;
static float estimated_velocity_m_s;
static float reference_position_m;
static float controller_velocity_m_s;
static uint32_t telemetry_ticks;
static uint32_t diagnostic_phase_ticks;
static uint32_t diagnostic_report_ticks;
static uint8_t fault_reported;
static const char *fault_reason = "init";

static void motors_disable(void);
static void send_diagnostic_event(void);

static uint16_t read_i16_le(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static HAL_StatusTypeDef imu_write_register(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(LSM6DS0_I2C_ADDRESS_7BIT << 1),
                           reg, I2C_MEMADD_SIZE_8BIT, &value, 1U,
                           ROBOT_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef imu_read_register(uint8_t reg, uint8_t *value)
{
  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(LSM6DS0_I2C_ADDRESS_7BIT << 1),
                          reg, I2C_MEMADD_SIZE_8BIT, value, 1U,
                          ROBOT_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef imu_read_vector(uint8_t reg, int16_t vector[3])
{
  uint8_t data[6];
  HAL_StatusTypeDef status =
      HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(LSM6DS0_I2C_ADDRESS_7BIT << 1),
                       reg, I2C_MEMADD_SIZE_8BIT, data, sizeof(data),
                       ROBOT_I2C_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    for (uint32_t axis = 0; axis < 3U; axis++)
    {
      vector[axis] = (int16_t)read_i16_le(&data[axis * 2U]);
    }
  }
  return status;
}

static HAL_StatusTypeDef imu_initialize_and_calibrate(void)
{
  uint8_t who_am_i = 0U;
  if (imu_read_register(LSM6DS0_REG_WHO_AM_I, &who_am_i) != HAL_OK ||
      who_am_i != LSM6DS0_WHO_AM_I_VALUE)
  {
    return HAL_ERROR;
  }

  /* 952 Hz, +/-500 dps gyroscope; enable all gyro axes. */
  if (imu_write_register(LSM6DS0_REG_CTRL_REG1_G, 0xD0U) != HAL_OK ||
      imu_write_register(LSM6DS0_REG_CTRL_REG4_G, 0x07U) != HAL_OK ||
      /* 952 Hz, +/-2 g accelerometer; BDU and register auto-increment. */
      imu_write_register(LSM6DS0_REG_CTRL_REG6_XL, 0xE0U) != HAL_OK ||
      imu_write_register(LSM6DS0_REG_CTRL_REG8, 0x44U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  float gyro_sum = 0.0f;
  float pitch_sum = 0.0f;
  for (uint32_t sample = 0U; sample < ROBOT_IMU_CALIBRATION_SAMPLES; sample++)
  {
    int16_t gyro[3];
    int16_t accel[3];
    if (imu_read_vector(LSM6DS0_REG_OUT_X_L_G, gyro) != HAL_OK ||
        imu_read_vector(LSM6DS0_REG_OUT_X_L_XL, accel) != HAL_OK)
    {
      return HAL_ERROR;
    }
    gyro_sum += (float)gyro[1] * 0.015258789f * DEG_TO_RAD;
    float ax = (float)accel[0] * 0.000061f * G_TO_M_S2;
    float az = (float)accel[2] * 0.000061f * G_TO_M_S2;
    pitch_sum += atan2f(ROBOT_ACCEL_PITCH_SIGN * ax, az);
    (void)HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(2U);
  }
  gyro_bias_rad_s = gyro_sum / (float)ROBOT_IMU_CALIBRATION_SAMPLES;
  pitch_offset_rad = pitch_sum / (float)ROBOT_IMU_CALIBRATION_SAMPLES;
  return HAL_OK;
}

static float meters_per_step(void)
{
  return (2.0f * 3.14159265359f * ROBOT_WHEEL_RADIUS_M *
          (ROBOT_STEP_ANGLE_DEG / 360.0f)) /
         (ROBOT_MICROSTEPS * ROBOT_GEAR_RATIO);
}

static float wheel_apply_speed(RobotWheel_t *wheel, float speed_m_s)
{
  float signed_speed = speed_m_s * wheel->motor_sign;
  float max_step_rate = fminf(ROBOT_MAX_STEP_RATE_HZ,
                              ROBOT_MAX_SPEED_M_S / meters_per_step());
  float step_rate = fabsf(signed_speed) / meters_per_step();
  if (step_rate > max_step_rate)
  {
    step_rate = max_step_rate;
    signed_speed = copysignf(step_rate * meters_per_step(), signed_speed);
  }
  if (step_rate < 1.0f)
  {
    __HAL_TIM_SET_COMPARE(wheel->timer, wheel->channel, 0U);
    wheel->estimated_speed_m_s = 0.0f;
    return 0.0f;
  }

  uint32_t period_counts = (uint32_t)(1000000.0f / step_rate + 0.5f);
  if (period_counts < (ROBOT_STEP_PULSE_US + 2U))
  {
    period_counts = ROBOT_STEP_PULSE_US + 2U;
  }
  if (period_counts > 65536U)
  {
    __HAL_TIM_SET_COMPARE(wheel->timer, wheel->channel, 0U);
    wheel->estimated_speed_m_s = 0.0f;
    return 0.0f;
  }

  int8_t direction = signed_speed >= 0.0f ? 1 : -1;
  if (direction != wheel->direction)
  {
    if (wheel->output_running)
    {
      (void)HAL_TIM_PWM_Stop(wheel->timer, wheel->channel);
      wheel->output_running = 0U;
    }
    __HAL_TIM_SET_COMPARE(wheel->timer, wheel->channel, 0U);
    HAL_GPIO_WritePin(wheel->direction_port, wheel->direction_pin,
                      direction > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    wheel->direction = direction;
    __HAL_TIM_SET_COUNTER(wheel->timer, 0U);
  }

  __HAL_TIM_SET_AUTORELOAD(wheel->timer, period_counts - 1U);
  __HAL_TIM_SET_COMPARE(wheel->timer, wheel->channel, ROBOT_STEP_PULSE_US);
  if (!wheel->output_running)
  {
    if (HAL_TIM_GenerateEvent(wheel->timer, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
    {
      application_fault = 1U;
      fault_reason = "timer";
      motors_disable();
      return 0.0f;
    }
    __HAL_TIM_SET_COUNTER(wheel->timer, 0U);
    if (HAL_TIM_PWM_Start(wheel->timer, wheel->channel) != HAL_OK)
    {
      application_fault = 1U;
      fault_reason = "pwm";
      motors_disable();
      return 0.0f;
    }
    wheel->output_running = 1U;
  }
  wheel->estimated_speed_m_s =
      (float)direction * (1000000.0f / (float)period_counts) * meters_per_step() *
      wheel->motor_sign;
  return wheel->estimated_speed_m_s;
}

static void motors_disable(void)
{
  robot_armed = 0U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
  if (right_wheel.output_running)
  {
    (void)HAL_TIM_PWM_Stop(right_wheel.timer, right_wheel.channel);
    right_wheel.output_running = 0U;
  }
  if (left_wheel.output_running)
  {
    (void)HAL_TIM_PWM_Stop(left_wheel.timer, left_wheel.channel);
    left_wheel.output_running = 0U;
  }
  HAL_GPIO_WritePin(motor_enable1_GPIO_Port, motor_enable1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor_enable2_GPIO_Port, motor_enable2_Pin, GPIO_PIN_SET);
}

static void process_command(const char *line)
{
  if (strcmp(line, "arm") == 0)
  {
    if (!application_fault && diagnostic_stage == ROBOT_DIAGNOSTIC_IDLE &&
        fabsf(pitch_rad) < 0.15f)
    {
      estimated_position_m = 0.0f;
      reference_position_m = 0.0f;
      pitch_integral = 0.0f;
      robot_armed = 1U;
      controller_velocity_m_s = 0.0f;
      HAL_GPIO_WritePin(motor_enable1_GPIO_Port, motor_enable1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(motor_enable2_GPIO_Port, motor_enable2_Pin, GPIO_PIN_RESET);
    }
    return;
  }
  if (strcmp(line, "disarm") == 0 || strcmp(line, "stop") == 0)
  {
    if (diagnostic_stage != ROBOT_DIAGNOSTIC_IDLE)
    {
      diagnostic_stage = ROBOT_DIAGNOSTIC_IDLE;
      diagnostic_event = ROBOT_DIAGNOSTIC_EVENT_CANCELLED;
    }
    motors_disable();
    requested_forward_speed = 0.0f;
    requested_turn_rate = 0.0f;
    return;
  }
  if (strcmp(line, "test") == 0)
  {
    if (application_fault)
    {
      diagnostic_event = ROBOT_DIAGNOSTIC_EVENT_FAULTED;
    }
    else if (robot_armed || diagnostic_stage != ROBOT_DIAGNOSTIC_IDLE)
    {
      diagnostic_event = ROBOT_DIAGNOSTIC_EVENT_REJECTED_ARMED;
    }
    else if (fabsf(pitch_rad) >= 0.15f)
    {
      diagnostic_event = ROBOT_DIAGNOSTIC_EVENT_REJECTED_TILT;
    }
    else
    {
      motors_disable();
      diagnostic_phase_ticks = 0U;
      diagnostic_report_ticks = 0U;
      requested_forward_speed = 0.0f;
      requested_turn_rate = 0.0f;
      diagnostic_stage = ROBOT_DIAGNOSTIC_LEFT_FORWARD;
      HAL_GPIO_WritePin(motor_enable1_GPIO_Port, motor_enable1_Pin,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(motor_enable2_GPIO_Port, motor_enable2_Pin,
                        GPIO_PIN_RESET);
      diagnostic_event = ROBOT_DIAGNOSTIC_EVENT_STARTED;
    }
    return;
  }
  if (strcmp(line, "pid") == 0)
  {
    active_controller = ROBOT_CONTROLLER_PID;
    pitch_integral = 0.0f;
    return;
  }
  if (strcmp(line, "lqr") == 0)
  {
    active_controller = ROBOT_CONTROLLER_LQR;
    return;
  }

  float value = 0.0f;
  char *end = NULL;
  if ((line[0] == 'v' || line[0] == 't') && line[1] == ' ')
  {
    const char *number_start = &line[2];
    value = strtof(number_start, &end);
    if (end == number_start)
    {
      return;
    }
    while (*end == ' ')
    {
      end++;
    }
    if (*end != '\0' || !isfinite(value))
    {
      return;
    }
    if (line[0] == 'v')
    {
      requested_forward_speed =
          fmaxf(-ROBOT_MAX_SPEED_M_S, fminf(ROBOT_MAX_SPEED_M_S, value));
    }
    else
    {
      requested_turn_rate = fmaxf(-4.0f, fminf(4.0f, value));
    }
    last_command_tick = control_tick_count;
  }
}

static void update_controller(float pitch_rate)
{
  const float dt = ROBOT_CONTROL_DT;
  reference_position_m += requested_forward_speed * dt;
  float position_error = estimated_position_m - reference_position_m;
  float velocity_error = estimated_velocity_m_s - requested_forward_speed;
  float acceleration;

  if (active_controller == ROBOT_CONTROLLER_LQR)
  {
    acceleration =
        -(ROBOT_LQR_K_POSITION * position_error +
          ROBOT_LQR_K_VELOCITY * velocity_error +
          ROBOT_LQR_K_PITCH * pitch_rad +
          ROBOT_LQR_K_PITCH_RATE * pitch_rate);
  }
  else
  {
    float next_integral = fmaxf(-0.5f, fminf(0.5f, pitch_integral + pitch_rad * dt));
    acceleration =
        ROBOT_PID_PITCH_KP * pitch_rad +
        ROBOT_PID_PITCH_KI * next_integral +
        ROBOT_PID_PITCH_KD * pitch_rate -
        ROBOT_PID_VELOCITY_KP * velocity_error -
        ROBOT_PID_POSITION_KP * position_error;
    if (fabsf(acceleration) < ROBOT_MAX_ACCEL_M_S2)
    {
      pitch_integral = next_integral;
    }
  }

  acceleration =
      fmaxf(-ROBOT_MAX_ACCEL_M_S2, fminf(ROBOT_MAX_ACCEL_M_S2, acceleration));
  controller_velocity_m_s =
      fmaxf(-ROBOT_MAX_SPEED_M_S,
            fminf(ROBOT_MAX_SPEED_M_S,
                  controller_velocity_m_s + acceleration * dt));
  float wheel_offset =
      requested_turn_rate * ROBOT_WHEEL_SEPARATION_M * 0.5f;
  if (robot_armed && !application_fault)
  {
    left_wheel.estimated_speed_m_s =
        wheel_apply_speed(&left_wheel, controller_velocity_m_s - wheel_offset);
    right_wheel.estimated_speed_m_s =
        wheel_apply_speed(&right_wheel, controller_velocity_m_s + wheel_offset);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    left_wheel.estimated_speed_m_s = 0.0f;
    right_wheel.estimated_speed_m_s = 0.0f;
    controller_velocity_m_s = 0.0f;
  }

  estimated_velocity_m_s =
      (left_wheel.estimated_speed_m_s + right_wheel.estimated_speed_m_s) * 0.5f;
  estimated_position_m += estimated_velocity_m_s * dt;
}

static HAL_StatusTypeDef update_imu(void)
{
  int16_t gyro[3];
  int16_t accel[3];
  if (imu_read_vector(LSM6DS0_REG_OUT_X_L_G, gyro) != HAL_OK ||
      imu_read_vector(LSM6DS0_REG_OUT_X_L_XL, accel) != HAL_OK)
  {
    return HAL_ERROR;
  }
  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    latest_gyro_raw[axis] = gyro[axis];
    latest_accel_raw[axis] = accel[axis];
  }
  float gyro_y = (float)gyro[1] * 0.015258789f * DEG_TO_RAD - gyro_bias_rad_s;
  float ax = (float)accel[0] * 0.000061f * G_TO_M_S2;
  float az = (float)accel[2] * 0.000061f * G_TO_M_S2;
  float accel_pitch = atan2f(ROBOT_ACCEL_PITCH_SIGN * ax, az) - pitch_offset_rad;
  float predicted = pitch_rad + gyro_y * ROBOT_CONTROL_DT;
  pitch_rad =
      ROBOT_PITCH_SIGN * (0.98f * predicted + 0.02f * accel_pitch);
  if (diagnostic_stage == ROBOT_DIAGNOSTIC_IDLE)
  {
    update_controller(ROBOT_PITCH_SIGN * gyro_y);
  }
  return HAL_OK;
}

static void report_fault(void)
{
  if (fault_reported || uart_tx_busy)
  {
    return;
  }
  int length = snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                        "FAULT,%s\r\n", fault_reason);
  if (length > 0 && (size_t)length < sizeof(uart_tx_buffer))
  {
    uart_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)uart_tx_buffer,
                              (uint16_t)length) == HAL_OK)
    {
      fault_reported = 1U;
    }
    else
    {
      uart_tx_busy = 0U;
    }
  }
}

static void send_telemetry(void)
{
  if (uart_tx_busy)
  {
    return;
  }
  int length = snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                        "T,%ld,%ld,%ld,%ld,%u,%u\r\n",
                        (long)(pitch_rad * 10000.0f),
                        (long)(estimated_position_m * 10000.0f),
                        (long)(estimated_velocity_m_s * 10000.0f),
                        (long)(controller_velocity_m_s * 10000.0f),
                        (unsigned)active_controller, (unsigned)robot_armed);
  if (length > 0 && (size_t)length < sizeof(uart_tx_buffer))
  {
    uart_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)uart_tx_buffer,
                              (uint16_t)length) != HAL_OK)
    {
      uart_tx_busy = 0U;
    }
  }
}

static void send_diagnostic_event(void)
{
  if (diagnostic_event == ROBOT_DIAGNOSTIC_EVENT_NONE || uart_tx_busy)
  {
    return;
  }

  const char *message;
  switch (diagnostic_event)
  {
  case ROBOT_DIAGNOSTIC_EVENT_STARTED:
    message = "TEST,START,step_hz=100,phase_ms=500\r\n";
    break;
  case ROBOT_DIAGNOSTIC_EVENT_FAULTED:
    message = "TEST,FAULTED\r\n";
    break;
  case ROBOT_DIAGNOSTIC_EVENT_REJECTED_ARMED:
    message = "TEST,REJECTED,disarm_first\r\n";
    break;
  case ROBOT_DIAGNOSTIC_EVENT_CANCELLED:
    message = "TEST,CANCELLED\r\n";
    break;
  case ROBOT_DIAGNOSTIC_EVENT_REJECTED_TILT:
    message = "TEST,REJECTED,hold_upright\r\n";
    break;
  case ROBOT_DIAGNOSTIC_EVENT_COMPLETE:
    message = "TEST,DONE\r\n";
    break;
  default:
    diagnostic_event = ROBOT_DIAGNOSTIC_EVENT_NONE;
    return;
  }

  size_t length = strlen(message);
  uart_tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)message,
                            (uint16_t)length) == HAL_OK)
  {
    diagnostic_event = ROBOT_DIAGNOSTIC_EVENT_NONE;
  }
  else
  {
    uart_tx_busy = 0U;
    application_fault = 1U;
    fault_reason = "diag_uart";
    diagnostic_stage = ROBOT_DIAGNOSTIC_IDLE;
    motors_disable();
  }
}

static void send_diagnostic_sample(void)
{
  if (uart_tx_busy)
  {
    return;
  }

  int length = snprintf(
      uart_tx_buffer, sizeof(uart_tx_buffer),
      "D,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
      (unsigned)diagnostic_stage,
      (long)(pitch_rad * 1000.0f),
      (long)((float)latest_gyro_raw[0] * 15.258789f),
      (long)((float)latest_gyro_raw[1] * 15.258789f),
      (long)((float)latest_gyro_raw[2] * 15.258789f),
      (long)((float)latest_accel_raw[0] * 0.061f),
      (long)((float)latest_accel_raw[1] * 0.061f),
      (long)((float)latest_accel_raw[2] * 0.061f));
  if (length <= 0 || (size_t)length >= sizeof(uart_tx_buffer))
  {
    application_fault = 1U;
    fault_reason = "diag_format";
    diagnostic_stage = ROBOT_DIAGNOSTIC_IDLE;
    motors_disable();
    return;
  }

  uart_tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)uart_tx_buffer,
                            (uint16_t)length) != HAL_OK)
  {
    uart_tx_busy = 0U;
    application_fault = 1U;
    fault_reason = "diag_uart";
    diagnostic_stage = ROBOT_DIAGNOSTIC_IDLE;
    motors_disable();
  }
}

static void run_motor_diagnostic_tick(void)
{
  if (diagnostic_stage == ROBOT_DIAGNOSTIC_IDLE || application_fault)
  {
    return;
  }

  RobotWheel_t *active_wheel = NULL;
  float speed = ROBOT_DIAGNOSTIC_STEP_RATE_HZ * meters_per_step();
  switch (diagnostic_stage)
  {
  case ROBOT_DIAGNOSTIC_LEFT_FORWARD:
    active_wheel = &left_wheel;
    break;
  case ROBOT_DIAGNOSTIC_LEFT_REVERSE:
    active_wheel = &left_wheel;
    speed = -speed;
    break;
  case ROBOT_DIAGNOSTIC_RIGHT_FORWARD:
    active_wheel = &right_wheel;
    break;
  case ROBOT_DIAGNOSTIC_RIGHT_REVERSE:
    active_wheel = &right_wheel;
    speed = -speed;
    break;
  default:
    application_fault = 1U;
    fault_reason = "diag_stage";
    diagnostic_stage = ROBOT_DIAGNOSTIC_IDLE;
    motors_disable();
    return;
  }

  float left_speed = active_wheel == &left_wheel ? speed : 0.0f;
  float right_speed = active_wheel == &right_wheel ? speed : 0.0f;
  (void)wheel_apply_speed(&left_wheel, left_speed);
  (void)wheel_apply_speed(&right_wheel, right_speed);
  if (application_fault)
  {
    diagnostic_stage = ROBOT_DIAGNOSTIC_IDLE;
    motors_disable();
    return;
  }

  diagnostic_phase_ticks++;
  diagnostic_report_ticks++;
  if (diagnostic_report_ticks >= ROBOT_DIAGNOSTIC_REPORT_TICKS)
  {
    diagnostic_report_ticks = 0U;
    send_diagnostic_sample();
  }
  if (diagnostic_phase_ticks >= ROBOT_DIAGNOSTIC_PHASE_TICKS)
  {
    diagnostic_phase_ticks = 0U;
    if (diagnostic_stage == ROBOT_DIAGNOSTIC_RIGHT_REVERSE)
    {
      diagnostic_stage = ROBOT_DIAGNOSTIC_IDLE;
      motors_disable();
      diagnostic_event = ROBOT_DIAGNOSTIC_EVENT_COMPLETE;
    }
    else
    {
      diagnostic_stage = (RobotDiagnosticStage_t)(diagnostic_stage + 1);
    }
  }
}

HAL_StatusTypeDef Robot_App_Init(void)
{
  motors_disable();
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) == HAL_OK)
  {
    right_wheel.output_running = 1U;
  }
  else
  {
    application_fault = 1U;
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) == HAL_OK)
  {
    left_wheel.output_running = 1U;
  }
  else
  {
    application_fault = 1U;
  }

  if (!application_fault && imu_initialize_and_calibrate() != HAL_OK)
  {
    application_fault = 1U;
  }
  if (HAL_UART_Receive_IT(&hlpuart1, &uart_rx_byte, 1U) != HAL_OK)
  {
    application_fault = 1U;
  }
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    application_fault = 1U;
  }
  if (application_fault)
  {
    motors_disable();
    (void)HAL_UART_Transmit(&hlpuart1, (uint8_t *)"FAULT,init\r\n", 12U, 50U);
    fault_reported = 1U;
  }
  return application_fault ? HAL_ERROR : HAL_OK;
}

void Robot_App_Run(void)
{
  send_diagnostic_event();
  uint32_t ticks;
  __disable_irq();
  ticks = pending_control_ticks;
  pending_control_ticks = 0U;
  __enable_irq();

  if (ticks == 0U)
  {
    __WFI();
    return;
  }
  if (ticks > 1U)
  {
    application_fault = 1U;
    fault_reason = "timing";
    motors_disable();
  }

  if (!application_fault && update_imu() != HAL_OK)
  {
    application_fault = 1U;
    fault_reason = "imu";
    motors_disable();
  }
  if ((uint32_t)(control_tick_count - last_command_tick) >=
      ROBOT_COMMAND_TIMEOUT_TICKS)
  {
    requested_forward_speed = 0.0f;
    requested_turn_rate = 0.0f;
  }
  if (fabsf(pitch_rad) >= ROBOT_FALL_ANGLE_RAD)
  {
    application_fault = 1U;
    fault_reason = "fall";
    motors_disable();
  }
  if (application_fault && diagnostic_stage != ROBOT_DIAGNOSTIC_IDLE)
  {
    diagnostic_stage = ROBOT_DIAGNOSTIC_IDLE;
    diagnostic_event = ROBOT_DIAGNOSTIC_EVENT_FAULTED;
  }
  if (application_fault)
  {
    motors_disable();
    report_fault();
  }

  if (!application_fault && diagnostic_stage != ROBOT_DIAGNOSTIC_IDLE)
  {
    run_motor_diagnostic_tick();
  }

  if (robot_armed && !application_fault)
  {
    telemetry_ticks++;
    if (telemetry_ticks >= 50U)
    {
      telemetry_ticks = 0U;
      send_telemetry();
    }
  }
  (void)HAL_IWDG_Refresh(&hiwdg);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
  if (timer->Instance == TIM6)
  {
    if (pending_control_ticks < UINT32_MAX)
    {
      pending_control_ticks++;
    }
    if (control_tick_count < UINT32_MAX)
    {
      control_tick_count++;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance != LPUART1)
  {
    return;
  }
  char byte = (char)uart_rx_byte;
  if (byte == '\n' || byte == '\r')
  {
    if (uart_line_length > 0U)
    {
      uart_line[uart_line_length] = '\0';
      process_command(uart_line);
      uart_line_length = 0U;
    }
  }
  else if (uart_line_length < (ROBOT_RX_BUFFER_SIZE - 1U))
  {
    uart_line[uart_line_length++] = byte;
  }
  else
  {
    uart_line_length = 0U;
  }
  (void)HAL_UART_Receive_IT(&hlpuart1, &uart_rx_byte, 1U);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance == LPUART1)
  {
    uart_tx_busy = 0U;
  }
}
