/* Author: Luca Obwegs */
/* Host test of the robot firmware: the real modules run against mocked HAL
   drivers and a linearised pendulum plant (same model as the simulation).
   Build and run from STM32Code (see README). */
#include <assert.h>
#include <stdio.h>

#include "main.h"

/* ARM interrupt/sleep instructions and the GPIO/RCC peripherals are mocked. */
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#undef __WFI
#define __WFI() ((void)0)
static GPIO_TypeDef gpio_a;
static GPIO_TypeDef gpio_b;
static GPIO_TypeDef gpio_c;
#undef GPIOA
#undef GPIOB
#undef GPIOC
#define GPIOA (&gpio_a)
#define GPIOB (&gpio_b)
#define GPIOC (&gpio_c)
#include "../BalancingRobot/Core/Src/comm.c"
#include "../BalancingRobot/Core/Src/controller.c"
#include "../BalancingRobot/Core/Src/encoder.c"
#include "../BalancingRobot/Core/Src/estimator.c"
#include "../BalancingRobot/Core/Src/fault.c"
#include "../BalancingRobot/Core/Src/imu.c"
#include "../BalancingRobot/Core/Src/robot_app.c"
#include "../BalancingRobot/Core/Src/stepper.c"
#include "../BalancingRobot/Core/Src/wheel_feedback.c"
#include "../BalancingRobot/Core/Src/wheel_kf.c"

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;
UART_HandleTypeDef hlpuart1;
IWDG_HandleTypeDef hiwdg;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

static TIM_TypeDef right_registers;
static TIM_TypeDef left_registers;
static I2C_TypeDef i2c3_registers;
static I2C_TypeDef i2c4_registers;
static uint8_t sensor_registers[256];
static HAL_StatusTypeDef encoder_result[2] = {HAL_OK, HAL_OK};
static uint16_t encoder_raw[2];
static uint8_t *uart_rx_target;
static char uart_log[1 << 16];
static size_t uart_log_length;

/* ---- HAL mocks ---------------------------------------------------------- */

uint32_t HAL_GetTick(void)
{
  return 0U;
}

void HAL_Delay(uint32_t delay)
{
  (void)delay;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  if (state == GPIO_PIN_SET)
    port->ODR |= pin;
  else
    port->ODR &= ~(uint32_t)pin;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
  return (port->ODR & pin) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
  assert(init->Mode == GPIO_MODE_OUTPUT_OD);
}

void HAL_NVIC_DisableIRQ(IRQn_Type irq)
{
}

HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *timer, uint32_t event)
{
  return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
  assert((timer == &htim1 || timer == &htim2) && channel == TIM_CHANNEL_1);
  timer->Instance->CR1 |= TIM_CR1_CEN;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
  timer->Instance->CR1 &= ~TIM_CR1_CEN;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer)
{
  assert(timer == &htim6);
  return HAL_OK;
}

HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *watchdog)
{
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
                                  uint16_t reg_size, uint8_t *data, uint16_t size,
                                  uint32_t timeout)
{
  assert(i2c == &hi2c1 && address == (0x6BU << 1) && reg + size <= 256U);
  memcpy(data, &sensor_registers[reg], size);
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
                                   uint16_t reg_size, uint8_t *data, uint16_t size,
                                   uint32_t timeout)
{
  assert(i2c == &hi2c1 && size == 1U);
  sensor_registers[reg] = data[0];
  return HAL_OK;
}

/* Encoder reads complete immediately through the real callbacks. */
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
                                     uint16_t reg_size, uint8_t *data, uint16_t size)
{
  uint8_t bus = (uint8_t)(i2c == &hi2c4);
  assert(address == (0x36U << 1) && reg == 0x0BU && size == 3U);
  if (encoder_result[bus] != HAL_OK)
  {
    HAL_I2C_ErrorCallback(i2c);
    return HAL_OK;
  }
  data[0] = 0x20U; /* magnet detected */
  data[1] = (uint8_t)(encoder_raw[bus] >> 8);
  data[2] = (uint8_t)encoder_raw[bus];
  HAL_I2C_MemRxCpltCallback(i2c);
  return HAL_OK;
}

uint32_t HAL_I2C_GetError(const I2C_HandleTypeDef *i2c)
{
  return HAL_I2C_ERROR_AF;
}

HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *i2c)
{
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *i2c)
{
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *i2c, uint32_t filter)
{
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *i2c, uint32_t filter)
{
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
  assert(uart == &hlpuart1 && size == 1U);
  uart_rx_target = data;
  return HAL_OK;
}

static void log_uart(const uint8_t *data, uint16_t size)
{
  assert(uart_log_length + size < sizeof(uart_log));
  memcpy(&uart_log[uart_log_length], data, size);
  uart_log_length += size;
  uart_log[uart_log_length] = '\0';
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, const uint8_t *data,
                                       uint16_t size)
{
  log_uart(data, size);
  HAL_UART_TxCpltCallback(uart);
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, const uint8_t *data,
                                   uint16_t size, uint32_t timeout)
{
  log_uart(data, size);
  return HAL_OK;
}

/* ---- Plant ---------------------------------------------------------------- */

typedef struct
{
  float theta;
  float theta_rate;
  float wheel_rad[2];
  float previous_speed;
} Plant;

static Plant plant;

static void set_accel_registers(float angle)
{
  /* Inverse of accel_correct_mg() in imu.c. */
  const float mg[3] = {ROBOT_ACCEL_PITCH_SIGN * 1000.0f * sinf(angle), 0.0f,
                       1000.0f * cosf(angle)};
  const float offsets[3] = {ROBOT_ACCEL_OFFSET_X_MG, ROBOT_ACCEL_OFFSET_Y_MG,
                            ROBOT_ACCEL_OFFSET_Z_MG};
  const float scales[3] = {ROBOT_ACCEL_SCALE_X, ROBOT_ACCEL_SCALE_Y, ROBOT_ACCEL_SCALE_Z};
  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    int16_t raw = (int16_t)lroundf((mg[axis] / scales[axis] + offsets[axis]) /
                                   LSM6DS0_ACCEL_MG_PER_LSB);
    sensor_registers[0x28U + axis * 2U] = (uint8_t)((uint16_t)raw & 255U);
    sensor_registers[0x29U + axis * 2U] = (uint8_t)((uint16_t)raw >> 8);
  }
}

static void set_gyro_register(float rate)
{
  int16_t raw = (int16_t)lroundf(ROBOT_PITCH_SIGN * ROBOT_GYRO_PITCH_SIGN * rate / DEG_TO_RAD /
                                 LSM6DS0_GYRO_DPS_PER_LSB);
  sensor_registers[0x1AU] = (uint8_t)((uint16_t)raw & 255U);
  sensor_registers[0x1BU] = (uint8_t)((uint16_t)raw >> 8);
}

/* Robot-forward wheel speed produced by the step timer and the DIR pin. */
static float wheel_speed(const TIM_TypeDef *timer, GPIO_TypeDef *dir_port, uint16_t dir_pin,
                         float motor_sign)
{
  if ((timer->CR1 & TIM_CR1_CEN) == 0U || timer->CCR1 == 0U)
    return 0.0f;
  float direction = HAL_GPIO_ReadPin(dir_port, dir_pin) == GPIO_PIN_SET ? 1.0f : -1.0f;
  return direction * 1000000.0f / (float)(timer->ARR + 1U) * meters_per_step() * motor_sign;
}

/* Raw AS5600 counts whose magnet-error correction gives the true counts. */
static uint16_t raw_counts(const float coefficients[4], float counts)
{
  float raw = counts;
  for (uint32_t i = 0U; i < 3U; i++)
  {
    uint16_t guess = (uint16_t)(((long)lroundf(raw) % 4096L + 4096L) % 4096L);
    raw = counts + ((float)guess - wheel_encoder_corrected_counts(coefficients, guess));
  }
  return (uint16_t)(((long)lroundf(raw) % 4096L + 4096L) % 4096L);
}

static void set_sensors(float wheel_accel)
{
  static const float coefficients[2][4] = {ROBOT_ENCODER_LEFT_ANGLE_ERR_COUNTS,
                                           ROBOT_ENCODER_RIGHT_ANGLE_ERR_COUNTS};
  static const float signs[2] = {ROBOT_ENCODER_LEFT_SIGN, ROBOT_ENCODER_RIGHT_SIGN};
  float theta_ddot =
      armed ? ROBOT_PLANT_ALPHA * plant.theta - ROBOT_PLANT_BETA * wheel_accel : 0.0f;
  float measured = plant.theta - (wheel_accel + ROBOT_IMU_HEIGHT_M * theta_ddot) / GRAVITY_M_S2;
  set_accel_registers(measured);
  set_gyro_register(plant.theta_rate);
  for (uint32_t w = 0U; w < 2U; w++)
    encoder_raw[w] = raw_counts(coefficients[w],
                                2000.0f + signs[w] * plant.wheel_rad[w] * 4096.0f /
                                              (2.0f * ROBOT_PI));
}

/* One 2 ms control tick: sensors from the plant, firmware step, plant update.
   The plant only moves while armed (otherwise the robot is held upright). */
static void tick(void)
{
  float left = wheel_speed(&left_registers, dir_left_GPIO_Port, dir_left_Pin,
                           ROBOT_LEFT_MOTOR_SIGN);
  float right = wheel_speed(&right_registers, dir_right_GPIO_Port, dir_right_Pin,
                            ROBOT_RIGHT_MOTOR_SIGN);
  float speed = 0.5f * (left + right);
  float accel = (speed - plant.previous_speed) / ROBOT_CONTROL_DT;
  plant.previous_speed = speed;
  set_sensors(accel);
  HAL_TIM_PeriodElapsedCallback(&htim6);
  Robot_App_Run();
  if (armed)
  {
    plant.theta_rate += (ROBOT_PLANT_ALPHA * plant.theta - ROBOT_PLANT_BETA * accel) *
                        ROBOT_CONTROL_DT;
    plant.theta += plant.theta_rate * ROBOT_CONTROL_DT;
    plant.wheel_rad[0] += left / ROBOT_WHEEL_RADIUS_M * ROBOT_CONTROL_DT;
    plant.wheel_rad[1] += right / ROBOT_WHEEL_RADIUS_M * ROBOT_CONTROL_DT;
  }
}

static void run(uint32_t ticks)
{
  for (uint32_t i = 0U; i < ticks; i++)
    tick();
}

static void send(const char *line)
{
  for (const char *c = line; *c != '\0'; c++)
  {
    *uart_rx_target = (uint8_t)*c;
    HAL_UART_RxCpltCallback(&hlpuart1);
  }
  *uart_rx_target = '\n';
  HAL_UART_RxCpltCallback(&hlpuart1);
}

/* Sends a command, runs a few ticks and checks the reply. */
static void expect_reply(const char *command, const char *reply_text)
{
  size_t start = uart_log_length;
  send(command);
  run(5U);
  if (strstr(&uart_log[start], reply_text) == NULL)
  {
    fprintf(stderr, "'%s': expected '%s', got '%s'\n", command, reply_text, &uart_log[start]);
    assert(0);
  }
}

static void reset_plant(void)
{
  memset(&plant, 0, sizeof(plant));
}

/* ---- Scenarios ------------------------------------------------------------ */

static void test_boot_and_commands(void)
{
  assert(Robot_App_Init() == HAL_OK && !fault_active());
  expect_reply("estimator", "ESTIMATOR,mode=offsetkf");
  expect_reply("wheelfb", "WHEELFB,mode=kf");
  expect_reply("hold", "HOLD,enabled=1");
  assert(controller_type() == CONTROLLER_LQR);
  puts("PASS: boot defaults lqr, hold on, estimator offsetkf, wheelfb kf");

  size_t start = uart_log_length;
  run(200U);
  assert(strstr(&uart_log[start], "T,") != NULL && strstr(&uart_log[start], ",1,0\r\n") != NULL);
  puts("PASS: telemetry while disarmed");

  expect_reply("estimator foo", "ESTIMATOR,REJECTED");
  expect_reply("wheelfb foo", "WHEELFB,REJECTED");
  expect_reply("hold foo", "HOLD,REJECTED");
  send("v abc");
  send("v 0.1x");
  run(2U);
  assert(requested_speed_m_s == 0.0f);
  send("v 5");
  send("t -9");
  run(2U);
  assert(requested_speed_m_s == ROBOT_MAX_SPEED_M_S &&
         requested_turn_rad_s == -ROBOT_MAX_TURN_RATE_RAD_S);
  run(ROBOT_COMMAND_TIMEOUT_TICKS);
  assert(requested_speed_m_s == 0.0f && requested_turn_rad_s == 0.0f);
  puts("PASS: command parsing, clamping and the 0.5 s motion timeout");

  /* Arming needs the robot upright. */
  plant.theta = 0.3f;
  run(300U);
  expect_reply("arm", "ARM,REJECTED,hold_upright");
  assert(!armed);
  reset_plant();
  run(300U);
  send("arm");
  run(2U);
  assert(armed);
  expect_reply("estimator kalman", "ESTIMATOR,REJECTED,stop_first");
  expect_reply("wheelfb steps", "WHEELFB,REJECTED,stop_first");
  expect_reply("hold off", "HOLD,REJECTED,stop_first");
  send("stop");
  run(2U);
  assert(!armed && (left_registers.CR1 & TIM_CR1_CEN) == 0U &&
         HAL_GPIO_ReadPin(motor_enable1_GPIO_Port, motor_enable1_Pin) == GPIO_PIN_SET);
  puts("PASS: arm checks, settings locked while armed, stop disables the drivers");
}

typedef struct
{
  float drive_m;
  float reference_m;
  float turn_m;
  float after_release_m;
  float max_pitch_deg;
  uint32_t telemetry_rows;
} DriveResult;

/* Keyboard teleop as robot_teleop.py drives it: v/t keepalive every 0.1 s while
   a key is held, a single "v 0"/"t 0" on release. */
static DriveResult teleop_drive(const char *controller_cmd, const char *estimator_cmd,
                                const char *wheelfb_cmd)
{
  enum { ARM = 300U, DRIVE = 1300U, TURN = 2800U, RELEASE = 4300U, END = 7300U };
  DriveResult result = {0};
  send(controller_cmd);
  send(estimator_cmd);
  send(wheelfb_cmd);
  reset_plant();
  float turn_start = 0.0f;
  size_t log_start = uart_log_length;
  for (uint32_t t = 0U; t < END; t++)
  {
    if (t == ARM)
      send("arm");
    if (t >= DRIVE && t < RELEASE && (t - DRIVE) % 50U == 0U)
    {
      send("v 0.2");
      if (t >= TURN)
        send("t 1.5");
    }
    if (t == TURN)
      turn_start = plant.wheel_rad[1] - plant.wheel_rad[0];
    if (t == RELEASE)
    {
      result.turn_m = (plant.wheel_rad[1] - plant.wheel_rad[0] - turn_start) *
                      ROBOT_WHEEL_RADIUS_M;
      send("v 0");
      send("t 0");
    }
    tick();
    if (fault_active() || (t > ARM + 2U && !armed))
    {
      fprintf(stderr, "%s %s %s: fault '%s' at tick %u, pitch %.2f deg\n", controller_cmd,
              estimator_cmd, wheelfb_cmd, fault_reason(), (unsigned)t,
              plant.theta / DEG_TO_RAD);
      assert(0);
    }
    if (t == RELEASE)
    {
      result.drive_m = wheelfb_position();
      result.reference_m = reference_position_m;
    }
    if (t > ARM + 1000U)
      result.max_pitch_deg = fmaxf(result.max_pitch_deg, fabsf(plant.theta) / DEG_TO_RAD);
  }
  result.after_release_m = fabsf(wheelfb_position() - result.drive_m);
  for (const char *p = &uart_log[log_start]; (p = strstr(p, "\nT,")) != NULL; p++)
    result.telemetry_rows++;
  send("stop");
  run(2U);
  assert(!armed);
  printf("  %-4s %-14s %-6s drive=%.3f m (ref %.3f) turn=%.3f m after_release=%.4f m "
         "max_pitch=%.2f deg\n",
         controller_cmd, estimator_cmd + 10, wheelfb_cmd + 8, result.drive_m,
         result.reference_m, result.turn_m, result.after_release_m, result.max_pitch_deg);
  return result;
}

static void test_teleop(void)
{
  const float expected_turn = 1.5f * ROBOT_WHEEL_SEPARATION_M * 3.0f;
  DriveResult r = teleop_drive("lqr", "estimator offsetkf", "wheelfb kf");
  /* 6 s at 0.2 m/s moves the reference 1.2 m and the robot follows it. */
  assert(fabsf(r.reference_m - 1.2f) < 0.02f && fabsf(r.drive_m - r.reference_m) < 0.05f);
  assert(fabsf(r.turn_m - expected_turn) < 0.1f * expected_turn);
  assert(r.after_release_m < 0.05f && r.max_pitch_deg < 3.0f);
  /* T telemetry at 10 Hz. */
  assert(r.telemetry_rows >= 7000U / ROBOT_TELEMETRY_TICKS - 2U);
  puts("PASS: teleop drive, turn and stop with the default configuration");

  const char *controllers[] = {"lqr", "pid"};
  const char *estimators[] = {"estimator complementary", "estimator kalman",
                              "estimator offsetkf"};
  const char *feedbacks[] = {"wheelfb kf", "wheelfb steps"};
  for (uint32_t c = 0U; c < 2U; c++)
  {
    for (uint32_t e = 0U; e < 3U; e++)
    {
      for (uint32_t f = 0U; f < 2U; f++)
      {
        r = teleop_drive(controllers[c], estimators[e], feedbacks[f]);
        assert(fabsf(r.turn_m - expected_turn) < 0.1f * expected_turn);
        assert(fabsf(r.drive_m - r.reference_m) < 0.05f && r.after_release_m < 0.05f);
        assert(r.max_pitch_deg < 5.0f);
      }
    }
  }
  puts("PASS: every controller/estimator/wheel feedback combination balances and drives");
  send("lqr");
  send("estimator offsetkf");
  send("wheelfb kf");
  run(2U);
}

static void test_encoder_fallback(void)
{
  reset_plant();
  send("arm");
  run(500U);
  size_t start = uart_log_length;
  encoder_result[1] = HAL_ERROR;
  run(500U);
  assert(armed && !fault_active());
  assert(strstr(&uart_log[start], "WHEELFB,FALLBACK,feedback=steps,reason=encoder_i2c4") != NULL);
  encoder_result[1] = HAL_OK;
  send("stop");
  run(2U);
  puts("PASS: encoder failure falls back to step feedback and keeps balancing");
}

static void test_fall(void)
{
  reset_plant();
  send("arm");
  run(100U);
  plant.theta = 0.6f;
  plant.theta_rate = 3.0f;
  run(200U);
  assert(fault_active() && !armed && strcmp(fault_reason(), "fall") == 0);
  assert(strstr(uart_log, "FAULT,fall\r\n") != NULL);
  assert(HAL_GPIO_ReadPin(motor_enable1_GPIO_Port, motor_enable1_Pin) == GPIO_PIN_SET);
  expect_reply("arm", "ARM,REJECTED,fault");
  puts("PASS: a fall latches FAULT,fall and disables the motors");
}

int main(void)
{
  setvbuf(stdout, NULL, _IONBF, 0);
  htim1.Instance = (TIM_TypeDef *)&right_registers;
  htim2.Instance = (TIM_TypeDef *)&left_registers;
  htim6.Instance = TIM6;
  hi2c3.Instance = &i2c3_registers;
  hi2c4.Instance = &i2c4_registers;
  hlpuart1.Instance = LPUART1;
  sensor_registers[0x0FU] = LSM6DS0_WHO_AM_I_VALUE;
  set_sensors(0.0f);

  test_boot_and_commands();
  test_teleop();
  test_encoder_fallback();
  test_fall();
  puts("ALL TESTS PASSED");
  return 0;
}
