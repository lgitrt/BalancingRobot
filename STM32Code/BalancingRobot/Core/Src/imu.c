/* Author: Luca Obwegs */
#include "imu.h"

#include "fault.h"
#include "robot_config.h"

#include <math.h>

#define LSM6DS0_REG_WHO_AM_I 0x0FU
#define LSM6DS0_REG_CTRL_REG1_G 0x10U
#define LSM6DS0_REG_OUT_X_L_G 0x18U
#define LSM6DS0_REG_CTRL_REG4_G 0x1EU
#define LSM6DS0_REG_CTRL_REG5_XL 0x1FU
#define LSM6DS0_REG_CTRL_REG6_XL 0x20U
#define LSM6DS0_REG_CTRL_REG8 0x22U
#define LSM6DS0_REG_OUT_X_L_XL 0x28U
#define LSM6DS0_GYRO_DPS_PER_LSB 0.0175f
#define LSM6DS0_ACCEL_MG_PER_LSB 0.061f

extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;

static uint8_t who_am_i;
static float gyro_bias_rad_s;
static float pitch_offset_rad;

static HAL_StatusTypeDef imu_write_register(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(LSM6DS0_I2C_ADDRESS_7BIT << 1), reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U, ROBOT_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef imu_read_register(uint8_t reg, uint8_t *value)
{
  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(LSM6DS0_I2C_ADDRESS_7BIT << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, value, 1U, ROBOT_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef imu_read_vector(uint8_t reg, int16_t vector[3])
{
  uint8_t data[6];
  HAL_StatusTypeDef status =
      HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(LSM6DS0_I2C_ADDRESS_7BIT << 1), reg,
                       I2C_MEMADD_SIZE_8BIT, data, sizeof(data), ROBOT_I2C_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    for (uint32_t axis = 0U; axis < 3U; axis++)
      vector[axis] = (int16_t)((uint16_t)data[axis * 2U] |
                               ((uint16_t)data[axis * 2U + 1U] << 8));
  }
  return status;
}

static HAL_StatusTypeDef imu_configure(void)
{
  /* 952 Hz, +/-500 dps, +/-2 g, XYZ enabled, BDU and auto-increment. */
  static const uint8_t settings[][2] = {
      {LSM6DS0_REG_CTRL_REG1_G, 0xC8U},
      {LSM6DS0_REG_CTRL_REG4_G, 0x38U},
      {LSM6DS0_REG_CTRL_REG5_XL, 0x38U},
      {LSM6DS0_REG_CTRL_REG6_XL, 0xC0U},
      {LSM6DS0_REG_CTRL_REG8, 0x44U}};
  for (uint32_t i = 0U; i < sizeof(settings) / sizeof(settings[0]); i++)
  {
    uint8_t readback;
    if (imu_write_register(settings[i][0], settings[i][1]) != HAL_OK ||
        imu_read_register(settings[i][0], &readback) != HAL_OK ||
        readback != settings[i][1])
      return HAL_ERROR;
  }
  return HAL_OK;
}

/* Six-face calibration of offset and scale per axis, in mg. */
static void accel_correct_mg(const int16_t raw[3], float corrected[3])
{
  corrected[0] = ((float)raw[0] * LSM6DS0_ACCEL_MG_PER_LSB - ROBOT_ACCEL_OFFSET_X_MG) *
                 ROBOT_ACCEL_SCALE_X;
  corrected[1] = ((float)raw[1] * LSM6DS0_ACCEL_MG_PER_LSB - ROBOT_ACCEL_OFFSET_Y_MG) *
                 ROBOT_ACCEL_SCALE_Y;
  corrected[2] = ((float)raw[2] * LSM6DS0_ACCEL_MG_PER_LSB - ROBOT_ACCEL_OFFSET_Z_MG) *
                 ROBOT_ACCEL_SCALE_Z;
}

static float gyro_y_rad_s(const int16_t gyro[3])
{
  return (float)gyro[1] * LSM6DS0_GYRO_DPS_PER_LSB * DEG_TO_RAD;
}

static float accel_angle_rad(const float corrected[3])
{
  return atan2f(ROBOT_ACCEL_PITCH_SIGN * corrected[0], corrected[2]);
}

HAL_StatusTypeDef imu_init(void)
{
  if (imu_read_register(LSM6DS0_REG_WHO_AM_I, &who_am_i) != HAL_OK)
  {
    fault_latch("imu_no_ack");
    return HAL_ERROR;
  }
  if (who_am_i != LSM6DS0_WHO_AM_I_VALUE)
  {
    fault_latch("imu_id");
    return HAL_ERROR;
  }
  if (imu_configure() != HAL_OK)
  {
    fault_latch("imu_config");
    return HAL_ERROR;
  }

  float gyro_sum = 0.0f;
  float pitch_sum = 0.0f;
  for (uint32_t sample = 0U; sample < ROBOT_IMU_CALIBRATION_SAMPLES; sample++)
  {
    int16_t gyro[3];
    int16_t accel[3];
    float corrected[3];
    if (imu_read_vector(LSM6DS0_REG_OUT_X_L_G, gyro) != HAL_OK ||
        imu_read_vector(LSM6DS0_REG_OUT_X_L_XL, accel) != HAL_OK)
    {
      fault_latch("imu_calibration");
      return HAL_ERROR;
    }
    accel_correct_mg(accel, corrected);
    gyro_sum += gyro_y_rad_s(gyro);
    pitch_sum += accel_angle_rad(corrected);
    (void)HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(2U);
  }
  gyro_bias_rad_s = gyro_sum / (float)ROBOT_IMU_CALIBRATION_SAMPLES;
  pitch_offset_rad = pitch_sum / (float)ROBOT_IMU_CALIBRATION_SAMPLES;
  return HAL_OK;
}

uint8_t imu_who_am_i(void)
{
  return who_am_i;
}

HAL_StatusTypeDef imu_read(ImuSample *sample)
{
  int16_t gyro[3];
  int16_t accel[3];
  float corrected[3];
  if (imu_read_vector(LSM6DS0_REG_OUT_X_L_G, gyro) != HAL_OK ||
      imu_read_vector(LSM6DS0_REG_OUT_X_L_XL, accel) != HAL_OK)
    return HAL_ERROR;
  accel_correct_mg(accel, corrected);
  sample->rate_rad_s =
      ROBOT_PITCH_SIGN * ROBOT_GYRO_PITCH_SIGN * (gyro_y_rad_s(gyro) - gyro_bias_rad_s);
  sample->accel_pitch_rad = ROBOT_PITCH_SIGN * (accel_angle_rad(corrected) - pitch_offset_rad);
  sample->accel_norm_mg = sqrtf(corrected[0] * corrected[0] + corrected[1] * corrected[1] +
                                corrected[2] * corrected[2]);
  return HAL_OK;
}
