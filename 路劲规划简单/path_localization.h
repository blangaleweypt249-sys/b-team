#ifndef PATH_LOCALIZATION_H
#define PATH_LOCALIZATION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 实车激光安装位置，以机器人中心为原点：前 +Y，左 -X。 */
#define PATH_LOCALIZATION_FRONT_SENSOR_OFFSET_M 0.2250f
#define PATH_LOCALIZATION_LEFT_SENSOR_OFFSET_M  0.1750f

typedef struct
{
    float map_x_m;
    float map_y_m;
    float front_wall_hit_x_m;
    float left_wall_hit_y_m;
} path_localization_result_t;

/**
 * 用前 DT35 到墙 B 南侧面、左 DT35 到西边界内侧面的距离反算机器人中心。
 * yaw_deg 使用 IMU 实测值；返回 false 表示光束方向或墙面交点不符合地图。
 */
bool PathLocalization_Calculate(float front_distance_m,
                                float left_distance_m,
                                float yaw_deg,
                                path_localization_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* PATH_LOCALIZATION_H */
