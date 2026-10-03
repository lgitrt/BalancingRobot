/* Author: Luca Obwegs */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "robot_config_generated.h"

#define ROBOT_CONTROL_DT (1.0f / ROBOT_CONTROL_HZ)

/* Verify these signs against the assembled robot and IMU orientation. */
#define ROBOT_PITCH_SIGN 1.0f
#define ROBOT_ACCEL_PITCH_SIGN 1.0f
#define ROBOT_LEFT_MOTOR_SIGN 1.0f
#define ROBOT_RIGHT_MOTOR_SIGN 1.0f

/* Verify the address against the board straps if the sensor is not detected. */
#define LSM6DS0_I2C_ADDRESS_7BIT 0x6AU
#define LSM6DS0_WHO_AM_I_VALUE 0x68U
#define ROBOT_IMU_CALIBRATION_SAMPLES 300U
#define ROBOT_I2C_TIMEOUT_MS 2U

/* Low-speed open-loop bench check: keep the wheels clear of the ground. */
#define ROBOT_DIAGNOSTIC_STEP_RATE_HZ 100.0f
#define ROBOT_DIAGNOSTIC_PHASE_TICKS 250U
#define ROBOT_DIAGNOSTIC_REPORT_TICKS 25U

#endif
