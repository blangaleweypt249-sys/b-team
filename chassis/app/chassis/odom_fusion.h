#ifndef ODOM_FUSION_H
#define ODOM_FUSION_H

#include "main.h"

/*
 * Encoder + IMU odometry fusion.
 *
 * Each tick:
 *  1. forward kinematics: 4 wheel speeds (eRPM from encoders) -> body vx, vy.
 *     The rotational part of each wheel speed is removed using the IMU yaw rate
 *     (Imu_GetGyroZ) BEFORE solving for translation, so wheel slip in the
 *     rotation term does not corrupt vx/vy.
 *  2. heading from the IMU (Imu_GetYaw) - NOT from the wheels (slip-prone).
 *  3. rotate body velocity by the IMU heading and integrate -> world x, y.
 *
 * Geometry is in macros (edit for your robot). Convention: body x=forward,
 * y=left, angles CCW. Wheel model (90-deg omni):
 *     v_i = vx*cos(b_i) + vy*sin(b_i) + w*L*sin(b_i - g_i)
 */

#define ODOM_PI  3.14159265358979f

/* ---- geometry (edit for your robot) ---- */
#define ODOM_WHEEL_RADIUS_M   0.050f
#define ODOM_WHEEL_BASE_M     0.150f
#define ODOM_POLE_PAIRS       21.0f

#define ODOM_LF_BETA_DEG    135.0f
#define ODOM_LF_GAMMA_DEG    45.0f
#define ODOM_RF_BETA_DEG     45.0f
#define ODOM_RF_GAMMA_DEG   315.0f
#define ODOM_LR_BETA_DEG    225.0f
#define ODOM_LR_GAMMA_DEG   135.0f
#define ODOM_RR_BETA_DEG    315.0f
#define ODOM_RR_GAMMA_DEG   225.0f

/* ---- fusion tuning (ZUPT / calibration / outlier) ---- */
#define ODOM_ZUPT_WHEEL_THRESH  0.05f   /* m/s: all wheels below this => "stationary" */
#define ODOM_ZUPT_BIAS_ALPHA    0.01f   /* ZUPT EMA: bias = (1-a)*bias + a*gyro */
#define ODOM_CALIB_TICKS        200U    /* startup gyro-bias samples (~2 s @ 100 Hz) */
#define ODOM_OUTLIER_THRESH     0.5f    /* m/s: wheel fit residual above this => outlier, excluded */

void  odom_init(void);          /* zero pose, t0 = now */
void  odom_tick(void);          /* one fusion step; call from main loop */

float odom_get_x_m(void);
float odom_get_y_m(void);
float odom_get_yaw_deg(void);
float odom_get_vx_mps(void);    /* body frame */
float odom_get_vy_mps(void);

float   odom_get_gyro_bias_deg_s(void); /* odom's residual gyro-bias estimate */
uint8_t odom_is_calibrated(void);       /* 1 after the startup bias calibration */

void  odom_print(void);         /* one line over USART1 */

#endif /* ODOM_FUSION_H */
