#include "odom_fusion.h"
#include "encoder.h"        /* encoder_get_erpm */
#include "imu.h"            /* Imu_GetYaw, Imu_GetGyroZ */
#include "chassis_motors.h" /* chassis_wheel_t, CHASSIS_WHEEL_COUNT */
#include "usart.h"          /* usart1_puts */
#include "print_util.h"

#include <math.h>

typedef struct { float beta_deg; float gamma_deg; } odom_wheel_geom_t;

static const odom_wheel_geom_t s_geom[CHASSIS_WHEEL_COUNT] = {
    { ODOM_LF_BETA_DEG, ODOM_LF_GAMMA_DEG },
    { ODOM_RF_BETA_DEG, ODOM_RF_GAMMA_DEG },
    { ODOM_LR_BETA_DEG, ODOM_LR_GAMMA_DEG },
    { ODOM_RR_BETA_DEG, ODOM_RR_GAMMA_DEG },
};

/* Precomputed per-wheel trig (filled in odom_init so the hot loop has no
 * cosf/sinf calls): cos(beta), sin(beta), sin(beta - gamma). */
static float s_cb[CHASSIS_WHEEL_COUNT];
static float s_sb[CHASSIS_WHEEL_COUNT];
static float s_sbg[CHASSIS_WHEEL_COUNT];

/* Pose / velocity state */
static float    s_x, s_y;        /* world position (m) */
static float    s_yaw;           /* heading (rad), from IMU */
static float    s_vx, s_vy;      /* body-frame velocity (m/s) */
static uint32_t s_last_tick;

/* Gyro bias (#1 ZUPT + #2 startup calibration) */
static float    s_gyro_bias;     /* residual bias (rad/s) */
static float    s_calib_sum;
static uint32_t s_calib_n;
static uint8_t  s_calibrated;

void odom_init(void)
{
    int i;
    s_x = 0.0f; s_y = 0.0f; s_yaw = 0.0f; s_vx = 0.0f; s_vy = 0.0f;
    s_gyro_bias = 0.0f; s_calib_sum = 0.0f; s_calib_n = 0U; s_calibrated = 0U;
    s_last_tick = HAL_GetTick();

    for (i = 0; i < CHASSIS_WHEEL_COUNT; i++)
    {
        float beta  = s_geom[i].beta_deg  * (ODOM_PI / 180.0f);
        float gamma = s_geom[i].gamma_deg * (ODOM_PI / 180.0f);
        s_cb[i]  = cosf(beta);
        s_sb[i]  = sinf(beta);
        s_sbg[i] = sinf(beta - gamma);
    }
}

/* motor eRPM -> wheel contact linear speed (m/s) */
static float erpm_to_mps(int32_t erpm)
{
    float mech_rpm = (float)erpm / ODOM_POLE_PAIRS;
    return mech_rpm / 60.0f * 2.0f * ODOM_PI * ODOM_WHEEL_RADIUS_M;
}

/* Least-squares solve of v_i' = vx*cos(b_i) + vy*sin(b_i) over the wheels
 * selected by use[]. Writes vx, vy. */
static void ls_solve(const float *vtrans, const int *use, float *vx, float *vy)
{
    float Sc2 = 0.0f, Ss2 = 0.0f, Scs = 0.0f, Scv = 0.0f, Ssv = 0.0f;
    float det;
    int i, n = 0;

    for (i = 0; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (!use[i]) { continue; }
        Sc2 += s_cb[i] * s_cb[i];
        Ss2 += s_sb[i] * s_sb[i];
        Scs += s_cb[i] * s_sb[i];
        Scv += s_cb[i] * vtrans[i];
        Ssv += s_sb[i] * vtrans[i];
        n++;
    }

    if (n < 2) { *vx = 0.0f; *vy = 0.0f; return; }
    det = Sc2 * Ss2 - Scs * Scs;
    if (fabsf(det) < 1e-6f) { *vx = 0.0f; *vy = 0.0f; return; }

    *vx = (Ss2 * Scv - Scs * Ssv) / det;
    *vy = (Sc2 * Ssv - Scs * Scv) / det;
}

/* Fit vx, vy with outlier rejection (#3): fit all 4, if the worst residual
 * exceeds ODOM_OUTLIER_THRESH, drop that wheel and refit with the other 3. */
static void odom_fit(const float *vtrans, float *vx, float *vy)
{
    int use[CHASSIS_WHEEL_COUNT] = { 1, 1, 1, 1 };
    int i, worst_i = -1;
    float worst = 0.0f;

    ls_solve(vtrans, use, vx, vy);
    for (i = 0; i < CHASSIS_WHEEL_COUNT; i++)
    {
        float r = fabsf(vtrans[i] - (*vx * s_cb[i] + *vy * s_sb[i]));
        if (r > worst) { worst = r; worst_i = i; }
    }
    if ((worst > ODOM_OUTLIER_THRESH) && (worst_i >= 0))
    {
        use[worst_i] = 0;
        ls_solve(vtrans, use, vx, vy);
    }
}

void odom_tick(void)
{
    uint32_t now = HAL_GetTick();
    float dt, gyro_rps, gyro_used, yaw, cy, sy, vwx, vwy, vx, vy;
    float v[CHASSIS_WHEEL_COUNT], vtrans[CHASSIS_WHEEL_COUNT];
    int i, stationary;

    if ((now - s_last_tick) < 10U)
    {
        return;   /* throttle to ~100 Hz; dt below is the real elapsed time */
    }
    dt = (float)(now - s_last_tick) / 1000.0f;   /* #4: actual dt, not a fixed step */
    s_last_tick = now;
    if (dt <= 0.0f || dt > 0.5f) { dt = 0.02f; }

    /* wheel speeds + stationary detection (for ZUPT) */
    stationary = 1;
    for (i = 0; i < CHASSIS_WHEEL_COUNT; i++)
    {
        v[i] = erpm_to_mps(encoder_get_erpm((chassis_wheel_t)i));
        if (fabsf(v[i]) >= ODOM_ZUPT_WHEEL_THRESH) { stationary = 0; }
    }

    /* gyro bias: startup calibration (#2), then runtime ZUPT (#1) */
    gyro_rps = Imu_GetGyroZ() * (ODOM_PI / 180.0f);
    if (!s_calibrated)
    {
        if (stationary) { s_calib_sum += gyro_rps; s_calib_n++; }
        if (s_calib_n >= ODOM_CALIB_TICKS)
        {
            s_gyro_bias = s_calib_sum / (float)s_calib_n;
            s_calibrated = 1U;
        }
        gyro_used = gyro_rps - s_gyro_bias;
    }
    else
    {
        if (stationary)
        {
            s_gyro_bias = (1.0f - ODOM_ZUPT_BIAS_ALPHA) * s_gyro_bias
                          + ODOM_ZUPT_BIAS_ALPHA * gyro_rps;
        }
        gyro_used = gyro_rps - s_gyro_bias;
    }

    /* remove the rotational part of each wheel speed using the corrected gyro */
    for (i = 0; i < CHASSIS_WHEEL_COUNT; i++)
    {
        vtrans[i] = v[i] - gyro_used * ODOM_WHEEL_BASE_M * s_sbg[i];
    }

    /* least-squares vx, vy with outlier rejection */
    odom_fit(vtrans, &vx, &vy);
    s_vx = vx;
    s_vy = vy;

    /* heading from IMU (not the wheels) -> rotate body vel to world, integrate */
    yaw = Imu_GetYaw() * (ODOM_PI / 180.0f);
    s_yaw = yaw;
    cy = cosf(yaw);
    sy = sinf(yaw);
    vwx = vx * cy - vy * sy;
    vwy = vx * sy + vy * cy;
    s_x += vwx * dt;
    s_y += vwy * dt;
}

float odom_get_x_m(void)            { return s_x; }
float odom_get_y_m(void)            { return s_y; }
float odom_get_yaw_deg(void)        { return s_yaw * (180.0f / ODOM_PI); }
float odom_get_vx_mps(void)         { return s_vx; }
float odom_get_vy_mps(void)         { return s_vy; }
float odom_get_gyro_bias_deg_s(void){ return s_gyro_bias * (180.0f / ODOM_PI); }
uint8_t odom_is_calibrated(void)    { return s_calibrated; }

void odom_print(void)
{
    usart1_puts("odom: x ");
    put_float(odom_get_x_m(), 3);
    usart1_puts(" y ");
    put_float(odom_get_y_m(), 3);
    usart1_puts(" m, yaw ");
    put_float(odom_get_yaw_deg(), 1);
    usart1_puts(" deg, body v (");
    put_float(odom_get_vx_mps(), 3);
    usart1_puts(", ");
    put_float(odom_get_vy_mps(), 3);
    usart1_puts(") m/s, gyro_bias ");
    put_float(odom_get_gyro_bias_deg_s(), 3);
    usart1_puts(" deg/s");
    usart1_puts(odom_is_calibrated() ? ", cal\r\n" : ", uncal\r\n");
}
