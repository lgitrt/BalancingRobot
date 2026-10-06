/* Author: Luca Obwegs */
#ifndef IMU_H
#define IMU_H

#include "main.h"

/* LSM6DS0 on the X-NUCLEO-IKS01A1 (I2C1), already converted to the robot frame. */
typedef struct
{
  float rate_rad_s;      /* pitch rate minus the boot gyro bias */
  float accel_pitch_rad; /* accelerometer pitch minus the boot offset */
  float accel_norm_mg;   /* corrected acceleration magnitude */
} ImuSample;

/* Checks WHO_AM_I, configures 952 Hz / 500 dps / 2 g and averages
   ROBOT_IMU_CALIBRATION_SAMPLES samples (robot standing still) for the gyro
   bias and the pitch offset. Latches a fault on failure. */
HAL_StatusTypeDef imu_init(void);
uint8_t imu_who_am_i(void);
HAL_StatusTypeDef imu_read(ImuSample *sample);

#endif
