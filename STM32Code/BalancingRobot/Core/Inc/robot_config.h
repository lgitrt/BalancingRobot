/* Author: Luca Obwegs */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Hand-tuned firmware parameters. The mechanical model, PID and LQR gains are
   exported from the simulation into robot_config_generated.h. */
#include "robot_config_generated.h"

#define DEG_TO_RAD 0.01745329252f
#define GRAVITY_M_S2 9.80665f
#define ROBOT_PI 3.14159265359f

#define ROBOT_CONTROL_DT (1.0f / ROBOT_CONTROL_HZ)
/* First-order low-pass on the pitch rate fed to the controllers. */
#define ROBOT_CONTROL_RATE_FILTER_HZ 20.0f
/* Telemetry period in control ticks (50 ticks = 10 Hz). */
#define ROBOT_TELEMETRY_TICKS 50U
/* "v" and "t" fall back to zero when no command arrives for this many ticks. */
#define ROBOT_COMMAND_TIMEOUT_TICKS 250U
#define ROBOT_MAX_TURN_RATE_RAD_S 4.0f
/* Arming is only accepted close to upright. */
#define ROBOT_ARM_MAX_PITCH_RAD 0.15f

/* ---------------------------------------------------------------- estimator */
/* Pitch estimator at boot: 1 = offset Kalman filter (best on the hardware),
   0 = complementary filter (or the 2-state Kalman filter if the next define is 1). */
#ifndef ROBOT_ESTIMATOR_OFFSET_KF_DEFAULT
#define ROBOT_ESTIMATOR_OFFSET_KF_DEFAULT 1U
#endif
#define ROBOT_ESTIMATOR_KALMAN_DEFAULT 0U
/* The accelerometer angle reads theta - a/g while the wheels accelerate. Fed back
   uncorrected, this turns the slow velocity mode into a growing ~0.2-0.3 Hz
   oscillation. Add back the commanded acceleration (1 = full compensation). */
#define ROBOT_ACCEL_MOTION_COMPENSATION 1.0f
/* Height of the LSM6DS0 above the wheel axle (robot upright), used by the offset
   KF measurement model. */
#define ROBOT_IMU_HEIGHT_M 0.152f
/* Offset KF: 4 states [pitch, pitch rate, gyro bias, balance point] driven by the
   wheel acceleration. It always runs; in offsetkf mode the controller uses
   pitch - balance point and the KF rate, while the complementary filter keeps
   running for the fall and arming checks. Noise: rad/s^2/sqrt(s), deg/s/sqrt(s),
   deg/sqrt(s); measurement std per 2 ms sample. */
#define ROBOT_OFFSET_KF_TORQUE_NOISE 0.3f
#define ROBOT_OFFSET_KF_BIAS_WALK_DEG 0.2f
#define ROBOT_OFFSET_KF_OFFSET_WALK_DEG 0.15f
#define ROBOT_OFFSET_KF_GYRO_STD_DEG 0.5f
#define ROBOT_OFFSET_KF_ACCEL_STD_DEG 8.0f
/* Initial std at arming: balance point vs the trim, pitch, rate, gyro bias. */
#define ROBOT_OFFSET_KF_INIT_OFFSET_STD_DEG 0.3f
#define ROBOT_OFFSET_KF_INIT_PITCH_STD_DEG 1.0f
#define ROBOT_OFFSET_KF_INIT_RATE_STD_DEG 1.0f
#define ROBOT_OFFSET_KF_INIT_BIAS_STD_DEG 0.5f
#define ROBOT_OFFSET_KF_INNOVATION_SIGMA 5.0f
/* Slow position-integral backup for residual balance-point/model error. */
#define ROBOT_OFFSET_KF_POSITION_INTEGRAL_SCALE 0.25f
/* Complementary filter: fast 2 %/tick accel trust while disarmed (robot still),
   slower trust plus gyro bias tracking while balancing. */
#define ROBOT_COMPLEMENTARY_IDLE_GAIN 0.02f
#define ROBOT_COMPLEMENTARY_ARMED_KP_PER_S 2.5f
#define ROBOT_COMPLEMENTARY_ARMED_KI_PER_S2 0.5f
#define ROBOT_COMPLEMENTARY_MAX_BIAS_DEG_S 5.0f
/* 2-state Kalman filter [pitch, gyro bias]. Noise densities: degrees/sqrt(s),
   degrees/s/sqrt(s); measurement std in degrees. */
#define ROBOT_KALMAN_ANGLE_NOISE 0.5f
#define ROBOT_KALMAN_BIAS_NOISE 0.05f
#define ROBOT_KALMAN_ACCEL_STD_DEG 2.0f
#define ROBOT_KALMAN_MAX_NORM_ERROR_G 0.25f
#define ROBOT_KALMAN_MAX_INNOVATION_DEG 8.0f
#define ROBOT_KALMAN_INNOVATION_SIGMA 5.0f
#define ROBOT_KALMAN_MAX_CORRECTION_DEG 0.1f

/* ------------------------------------------------------ signs and balance */
#define ROBOT_PITCH_SIGN 1.0f
#define ROBOT_ACCEL_PITCH_SIGN 1.0f
/* atan2(ax, az) decreases for positive sensor Y rotation. */
#define ROBOT_GYRO_PITCH_SIGN -1.0f
#define ROBOT_LEFT_MOTOR_SIGN -1.0f
#define ROBOT_RIGHT_MOTOR_SIGN -1.0f
/* Static balance point used by the complementary/Kalman estimators and as the
   offset KF start value. The LQR position integral corrects the remainder. */
#define ROBOT_BALANCE_TRIM_DEG 0.0f

/* -------------------------------------------------------------------- IMU */
#define LSM6DS0_I2C_ADDRESS_7BIT 0x6BU
#define LSM6DS0_WHO_AM_I_VALUE 0x68U
#define ROBOT_IMU_CALIBRATION_SAMPLES 300U
#define ROBOT_I2C_TIMEOUT_MS 2U
/* Six-face accelerometer calibration: sensor X up, Y right, Z forward. */
#define ROBOT_ACCEL_OFFSET_X_MG (-33.425f)
#define ROBOT_ACCEL_OFFSET_Y_MG (-15.954f)
#define ROBOT_ACCEL_OFFSET_Z_MG (-19.995f)
#define ROBOT_ACCEL_SCALE_X 0.995957f
#define ROBOT_ACCEL_SCALE_Y 1.002499f
#define ROBOT_ACCEL_SCALE_Z 0.989083f

/* --------------------------------------------------------- wheel feedback */
/* Wheel feedback at boot ("wheelfb kf" / "wheelfb steps"). The encoders are
   always read while balancing; this selects what the controller uses. */
#define ROBOT_WHEELFB_KF_DEFAULT 1U
/* AS5600 sample period in control ticks (5 ticks = 100 Hz). */
#define ROBOT_ENCODER_SAMPLE_TICKS 5U
/* Consecutive failed reads per encoder (each followed by bus recovery) before
   the robot falls back to step feedback. */
#define ROBOT_ENCODER_MAX_CONSECUTIVE_ERRORS 10U
/* Wheel encoder Kalman filter, measured on the robot (rad, rad/s). */
#define ROBOT_ENCODER_LEFT_SIGN (-1.0f)  /* I2C3 counts decrease when driving forward */
#define ROBOT_ENCODER_RIGHT_SIGN 1.0f    /* I2C4 */
#define ROBOT_ENCODER_LEFT_ANGLE_ERR_COUNTS {5.159f, -10.068f, 5.680f, 7.831f}
#define ROBOT_ENCODER_RIGHT_ANGLE_ERR_COUNTS {8.344f, 2.635f, 3.096f, -9.656f}
#define ROBOT_ENCODER_KF_R_RAD2 4.783e-07f
#define ROBOT_ENCODER_KF_Q_RAD2_S3 1.228e-03f
/* Age of an AS5600 sample when the KF uses it: read started at tick k, used at
   tick k+1 (one 2 ms control tick) plus the measured 0.27 ms encoder lag. */
#define ROBOT_ENCODER_KF_DELAY_S 0.0023f
#define ROBOT_ENCODER_KF_GATE_SIGMA 6.0f
/* 2 full steps: above the ~0.85 step stepper load-angle slack at every move
   start, below the 4-step (0.126 rad) slip of a stalled stepper. */
#define ROBOT_ENCODER_KF_GATE_MIN_RAD 0.0628f
#define ROBOT_ENCODER_KF_MAX_REJECTS 2U
#define ROBOT_ENCODER_KF_INITIAL_SPEED_VAR 1.0f
/* Applied-vs-KF wheel speed difference that marks a stalled stepper. */
#define ROBOT_WHEELFB_STALL_SPEED_RAD_S 3.0f

#endif
