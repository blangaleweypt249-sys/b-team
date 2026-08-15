#include "path_localization.h"

#include "path_map.h"

#include <math.h>
#include <stddef.h>

#define PATH_LOCALIZATION_PI 3.14159265358979323846f
#define PATH_LOCALIZATION_WALL_EPSILON_M 0.0010f

static bool PathLocalization_IsFinite(float value)
{
    return !isnan(value) && !isinf(value);
}

bool PathLocalization_Calculate(float front_distance_m,
                                float left_distance_m,
                                float yaw_deg,
                                path_localization_result_t *result)
{
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float front_ray_origin_distance_m;
    float left_ray_origin_distance_m;

    if ((result == NULL) || !PathLocalization_IsFinite(front_distance_m) ||
        !PathLocalization_IsFinite(left_distance_m) ||
        !PathLocalization_IsFinite(yaw_deg) ||
        (front_distance_m <= 0.0f) || (left_distance_m <= 0.0f))
    {
        return false;
    }

    yaw_rad = yaw_deg * (PATH_LOCALIZATION_PI / 180.0f);
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    if (cos_yaw <= 0.0f)
    {
        return false;
    }

    front_ray_origin_distance_m =
        PATH_LOCALIZATION_FRONT_SENSOR_OFFSET_M + front_distance_m;
    left_ray_origin_distance_m =
        PATH_LOCALIZATION_LEFT_SENSOR_OFFSET_M + left_distance_m;

    /*
     * 车体系 +Y 前激光与 y=2.075 m 相交，-X 左激光与 x=0.049 m
     * 相交。两束光随实测 yaw 一起旋转，因此中心法向距离乘 cos(yaw)。
     */
    result->map_x_m = PATH_MAP_WEST_INNER_X_M +
                      left_ray_origin_distance_m * cos_yaw;
    result->map_y_m = PATH_MAP_WALL_B_SOUTH_Y_M -
                      front_ray_origin_distance_m * cos_yaw;
    result->front_wall_hit_x_m = result->map_x_m -
                                 front_ray_origin_distance_m * sin_yaw;
    result->left_wall_hit_y_m = result->map_y_m -
                                left_ray_origin_distance_m * sin_yaw;

    if (!PathLocalization_IsFinite(result->map_x_m) ||
        !PathLocalization_IsFinite(result->map_y_m) ||
        (result->map_x_m < 0.0f) ||
        (result->map_x_m > PATH_MAP_FIELD_WIDTH_M) ||
        (result->map_y_m < 0.0f) ||
        (result->map_y_m > PATH_MAP_FIELD_HEIGHT_M) ||
        (result->front_wall_hit_x_m <
         (PATH_MAP_WALL_B_X_MIN_M - PATH_LOCALIZATION_WALL_EPSILON_M)) ||
        (result->front_wall_hit_x_m >
         (PATH_MAP_WALL_B_X_MAX_M + PATH_LOCALIZATION_WALL_EPSILON_M)) ||
        (result->left_wall_hit_y_m <
         -PATH_LOCALIZATION_WALL_EPSILON_M) ||
        (result->left_wall_hit_y_m >
         (PATH_MAP_FIELD_HEIGHT_M + PATH_LOCALIZATION_WALL_EPSILON_M)))
    {
        return false;
    }

    return true;
}
