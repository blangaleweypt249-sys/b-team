#include "road.h"

#include "chassis_main.h"
#include "imu_main.h"

#include <math.h>
#include <stddef.h>

#define ROAD_WHEEL_RADIUS_M       0.155f
#define ROAD_RPM_TO_MPS           \
    (2.0f * 3.14159265358979323846f * ROAD_WHEEL_RADIUS_M / 60.0f)
#define ROAD_DEG_TO_RAD           0.01745329251994329577f
#define ROAD_RPM_DEADZONE         1.0f
#define ROAD_MAX_STEP_MS          100U

static road_data_t road_data;
static uint32_t road_last_ms;
static float road_yaw_zero_deg;
static bool road_started;

static float Road_Rpm(float rpm)
{
    return (fabsf(rpm) < ROAD_RPM_DEADZONE) ? 0.0f : rpm;
}

void Road_Init(void)
{
    road_data.x_m = 0.0f;
    road_data.y_m = 0.0f;
    road_data.distance_m = 0.0f;
    road_data.valid = false;
    road_last_ms = HAL_GetTick();
    road_yaw_zero_deg = 0.0f;
    road_started = false;
}

void Road_Run(void)
{
    vesc_motor_status_t wheel[CHASSIS_WHEEL_COUNT];
    imu_data_t imu;
    float lf;
    float rf;
    float lr;
    float rr;
    float vx;
    float vy;
    float yaw_deg;
    float yaw_rad;
    float dt;
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms;
    uint8_t i;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (!Chassis_GetStatus((chassis_wheel_t)i, &wheel[i]) ||
            !wheel[i].online)
        {
            road_data.valid = false;
            road_last_ms = now_ms;
            return;
        }
    }
    if (!ImuMain_GetData(&imu) || !imu.online || !imu.yaw_valid ||
        (imu.state != IMU_STATE_READY))
    {
        road_data.valid = false;
        road_last_ms = now_ms;
        return;
    }

    /* X 型全向轮：RF 和 LR 的电机正转方向与 LF、RR 相反。 */
    lf = Road_Rpm((float)wheel[CHASSIS_WHEEL_LF].actual_rpm);
    rf = Road_Rpm(-(float)wheel[CHASSIS_WHEEL_RF].actual_rpm);
    lr = Road_Rpm(-(float)wheel[CHASSIS_WHEEL_LR].actual_rpm);
    rr = Road_Rpm((float)wheel[CHASSIS_WHEEL_RR].actual_rpm);
    vx = (lf - rf - lr + rr) * (ROAD_RPM_TO_MPS * 0.25f);
    vy = (lf + rf + lr + rr) * (ROAD_RPM_TO_MPS * 0.25f);

    if (!road_started)
    {
        road_yaw_zero_deg = -imu.yaw_deg;
        road_last_ms = now_ms;
        road_started = true;
        road_data.valid = true;
        return;
    }

    elapsed_ms = now_ms - road_last_ms;
    road_last_ms = now_ms;
    road_data.valid = true;
    yaw_deg = -imu.yaw_deg - road_yaw_zero_deg;
    if (yaw_deg > 180.0f)
    {
        yaw_deg -= 360.0f;
    }
    else if (yaw_deg < -180.0f)
    {
        yaw_deg += 360.0f;
    }
    if ((elapsed_ms == 0U) || (elapsed_ms > ROAD_MAX_STEP_MS))
    {
        return;
    }

    yaw_rad = yaw_deg * ROAD_DEG_TO_RAD;
    dt = (float)elapsed_ms * 0.001f;
    /* 世界坐标：X 向左、Y 向后，航向逆时针为正。 */
    road_data.x_m += (-cosf(yaw_rad) * vx + sinf(yaw_rad) * vy) * dt;
    road_data.y_m += (-sinf(yaw_rad) * vx - cosf(yaw_rad) * vy) * dt;
    road_data.distance_m += sqrtf(vx * vx + vy * vy) * dt;
}

void Road_Reset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    road_data.x_m = 0.0f;
    road_data.y_m = 0.0f;
    road_data.distance_m = 0.0f;
    road_data.valid = false;
    road_started = false;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

bool Road_GetData(road_data_t *data)
{
    uint32_t primask;

    if (data == NULL)
    {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *data = road_data;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}
