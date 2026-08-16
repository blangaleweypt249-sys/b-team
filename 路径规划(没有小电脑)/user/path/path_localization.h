#ifndef PATH_LOCALIZATION_H
#define PATH_LOCALIZATION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 实车激光安装位置，以机器人中心为原点：前 +Y，左 -X。 */
/* 前偏移为兼容既有接口保留；初始定位只使用左侧安装偏移。 */
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
 * 用左 DT35 到西边界内侧面的距离和 IMU 实测 yaw 反算中心 X；中心 Y
 * 恒为车长一半。front_distance_m 仅为接口兼容保留，允许无样本且不参与
 * 计算或校验；返回 false 表示左光方向或墙面交点不符合地图。
 */
bool PathLocalization_Calculate(float front_distance_m,
                                float left_distance_m,
                                float yaw_deg,
                                path_localization_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* PATH_LOCALIZATION_H */
