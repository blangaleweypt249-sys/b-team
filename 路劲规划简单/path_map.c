#include "path_map.h"

#include <math.h>
#include <stddef.h>

#define PATH_MAP_LATERAL_MARGIN_M      0.0500f
#define PATH_MAP_LONGITUDINAL_MARGIN_M 0.0200f
#define PATH_MAP_HALF_WIDTH_M          (0.5f * PATH_MAP_ROBOT_WIDTH_M)
#define PATH_MAP_HALF_LENGTH_M         (0.5f * PATH_MAP_ROBOT_LENGTH_M)
#define PATH_MAP_INFLATE_X_M           \
    (PATH_MAP_HALF_WIDTH_M + PATH_MAP_LATERAL_MARGIN_M)
#define PATH_MAP_INFLATE_Y_M           \
    (PATH_MAP_HALF_LENGTH_M + PATH_MAP_LONGITUDINAL_MARGIN_M)
#define PATH_MAP_DIRECTION_EPSILON     0.000001f

/*
 * 本表逐项来自 arena/01a00390-taisheng:user/path_planner/path_config.h。
 * 这里只保留地图数据，不包含该分支的样条、FSM、上位机定位或自动驾驶。
 */
static const path_map_wall_t path_map_walls[PATH_MAP_WALL_COUNT] =
{
    {0.000f, 0.000f, 3.000f, 0.049f},
    {0.000f, 5.951f, 3.000f, 6.000f},
    {0.000f, 0.000f, 0.049f, 6.000f},
    {2.951f, 0.000f, 3.000f, 6.000f},
    {1.050f, 1.070f, 3.000f, 1.120f},
    {PATH_MAP_WALL_B_X_MIN_M, PATH_MAP_WALL_B_SOUTH_Y_M,
     PATH_MAP_WALL_B_X_MAX_M, 2.125f},
    {1.050f, 3.075f, 3.000f, 3.125f}
};

/*
 * yaw 固定为 0 后不能采用参考分支的圆弧；按各墙端通道改成 Manhattan
 * 直线：上、右、上、左、上、右。每次按键只约束当前一段的轴。
 */
static const path_map_route_segment_t
path_map_route[PATH_MAP_ROUTE_SEGMENT_COUNT] =
{
    {PATH_MAP_AXIS_Y,  1, 1.650f},
    {PATH_MAP_AXIS_X,  1, 2.480f},
    {PATH_MAP_AXIS_Y,  1, 2.600f},
    {PATH_MAP_AXIS_X, -1, 0.360f},
    {PATH_MAP_AXIS_Y,  1, 3.700f},
    {PATH_MAP_AXIS_X,  1, 0.500f}
};

static float PathMap_MinFloat(float lhs, float rhs)
{
    return (lhs < rhs) ? lhs : rhs;
}

static float PathMap_MaxFloat(float lhs, float rhs)
{
    return (lhs > rhs) ? lhs : rhs;
}

static bool PathMap_PointInside(float x_m, float y_m,
                                const path_map_wall_t *box)
{
    return (x_m >= box->x_min_m) && (x_m <= box->x_max_m) &&
           (y_m >= box->y_min_m) && (y_m <= box->y_max_m);
}

static float PathMap_RayBoxDistance(float origin_x_m, float origin_y_m,
                                    float dir_x, float dir_y,
                                    const path_map_wall_t *box)
{
    float t_min = 0.0f;
    float t_max = PATH_MAP_CLEARANCE_NONE_M;
    float t1;
    float t2;
    float swap;

    /*
     * 若制动惯性或里程计漂移使中心落入膨胀墙，只允许朝最近边界退出；
     * 在边界上向墙内运动返回 0，不能穿墙，向外恢复则保持可用。
     */
    if (PathMap_PointInside(origin_x_m, origin_y_m, box))
    {
        float nearest_distance = origin_x_m - box->x_min_m;
        float outward_x = -1.0f;
        float outward_y = 0.0f;
        float edge_distance = box->x_max_m - origin_x_m;

        if (edge_distance < nearest_distance)
        {
            nearest_distance = edge_distance;
            outward_x = 1.0f;
            outward_y = 0.0f;
        }
        edge_distance = origin_y_m - box->y_min_m;
        if (edge_distance < nearest_distance)
        {
            nearest_distance = edge_distance;
            outward_x = 0.0f;
            outward_y = -1.0f;
        }
        edge_distance = box->y_max_m - origin_y_m;
        if (edge_distance < nearest_distance)
        {
            outward_x = 0.0f;
            outward_y = 1.0f;
        }
        return ((dir_x * outward_x + dir_y * outward_y) > 0.0f) ?
               PATH_MAP_CLEARANCE_NONE_M : 0.0f;
    }

    if (fabsf(dir_x) <= PATH_MAP_DIRECTION_EPSILON)
    {
        if ((origin_x_m < box->x_min_m) || (origin_x_m > box->x_max_m))
        {
            return PATH_MAP_CLEARANCE_NONE_M;
        }
    }
    else
    {
        t1 = (box->x_min_m - origin_x_m) / dir_x;
        t2 = (box->x_max_m - origin_x_m) / dir_x;
        if (t1 > t2)
        {
            swap = t1;
            t1 = t2;
            t2 = swap;
        }
        t_min = PathMap_MaxFloat(t_min, t1);
        t_max = PathMap_MinFloat(t_max, t2);
        if (t_min > t_max)
        {
            return PATH_MAP_CLEARANCE_NONE_M;
        }
    }

    if (fabsf(dir_y) <= PATH_MAP_DIRECTION_EPSILON)
    {
        if ((origin_y_m < box->y_min_m) || (origin_y_m > box->y_max_m))
        {
            return PATH_MAP_CLEARANCE_NONE_M;
        }
    }
    else
    {
        t1 = (box->y_min_m - origin_y_m) / dir_y;
        t2 = (box->y_max_m - origin_y_m) / dir_y;
        if (t1 > t2)
        {
            swap = t1;
            t1 = t2;
            t2 = swap;
        }
        t_min = PathMap_MaxFloat(t_min, t1);
        t_max = PathMap_MinFloat(t_max, t2);
        if (t_min > t_max)
        {
            return PATH_MAP_CLEARANCE_NONE_M;
        }
    }

    if (t_max < 0.0f)
    {
        return PATH_MAP_CLEARANCE_NONE_M;
    }
    return (t_min > 0.0f) ? t_min : 0.0f;
}

const path_map_wall_t *PathMap_GetWalls(uint8_t *count)
{
    if (count != NULL)
    {
        *count = PATH_MAP_WALL_COUNT;
    }
    return path_map_walls;
}

const path_map_route_segment_t *PathMap_GetRoute(uint8_t *count)
{
    if (count != NULL)
    {
        *count = PATH_MAP_ROUTE_SEGMENT_COUNT;
    }
    return path_map_route;
}

bool PathMap_SegmentReached(uint8_t segment_index,
                            float map_x_m,
                            float map_y_m)
{
    const path_map_route_segment_t *segment;
    float coordinate;

    if (segment_index >= PATH_MAP_ROUTE_SEGMENT_COUNT)
    {
        return false;
    }

    segment = &path_map_route[segment_index];
    coordinate = (segment->axis == PATH_MAP_AXIS_X) ? map_x_m : map_y_m;
    if (segment->direction > 0)
    {
        return coordinate >= segment->target_m;
    }
    return coordinate <= segment->target_m;
}

float PathMap_RayClearance(float map_x_m, float map_y_m,
                           float dir_x, float dir_y)
{
    path_map_wall_t inflated;
    float magnitude;
    float clearance = PATH_MAP_CLEARANCE_NONE_M;
    float candidate;
    uint8_t wall_index;

    magnitude = sqrtf(dir_x * dir_x + dir_y * dir_y);
    if (magnitude <= PATH_MAP_DIRECTION_EPSILON)
    {
        return PATH_MAP_CLEARANCE_NONE_M;
    }
    dir_x /= magnitude;
    dir_y /= magnitude;

    /*
     * 边界安全中心线按半车宽+5 cm、半车长+2 cm 计算；边界墙本身
     * 已包含 4.9 cm 厚度，因此这里不重复叠加墙厚。原始边界墙仍完整
     * 保留在 path_map_walls 中。
     */
    if (dir_x > PATH_MAP_DIRECTION_EPSILON)
    {
        candidate = ((PATH_MAP_FIELD_WIDTH_M -
                      PATH_MAP_BOUNDARY_MARGIN_X_M) - map_x_m) / dir_x;
        clearance = PathMap_MinFloat(clearance,
                                     (candidate > 0.0f) ? candidate : 0.0f);
    }
    else if (dir_x < -PATH_MAP_DIRECTION_EPSILON)
    {
        candidate = (PATH_MAP_BOUNDARY_MARGIN_X_M - map_x_m) / dir_x;
        clearance = PathMap_MinFloat(clearance,
                                     (candidate > 0.0f) ? candidate : 0.0f);
    }

    if (dir_y > PATH_MAP_DIRECTION_EPSILON)
    {
        candidate = ((PATH_MAP_FIELD_HEIGHT_M -
                      PATH_MAP_BOUNDARY_MARGIN_Y_M) - map_y_m) / dir_y;
        clearance = PathMap_MinFloat(clearance,
                                     (candidate > 0.0f) ? candidate : 0.0f);
    }
    else if (dir_y < -PATH_MAP_DIRECTION_EPSILON)
    {
        candidate = (PATH_MAP_BOUNDARY_MARGIN_Y_M - map_y_m) / dir_y;
        clearance = PathMap_MinFloat(clearance,
                                     (candidate > 0.0f) ? candidate : 0.0f);
    }

    /* 只对 3 段内墙做矩形膨胀；0..3 是已在上面处理的场地边界墙。 */
    for (wall_index = 4U; wall_index < PATH_MAP_WALL_COUNT; wall_index++)
    {
        inflated.x_min_m = path_map_walls[wall_index].x_min_m -
                           PATH_MAP_INFLATE_X_M;
        inflated.x_max_m = path_map_walls[wall_index].x_max_m +
                           PATH_MAP_INFLATE_X_M;
        inflated.y_min_m = path_map_walls[wall_index].y_min_m -
                           PATH_MAP_INFLATE_Y_M;
        inflated.y_max_m = path_map_walls[wall_index].y_max_m +
                           PATH_MAP_INFLATE_Y_M;
        candidate = PathMap_RayBoxDistance(map_x_m, map_y_m,
                                           dir_x, dir_y, &inflated);
        clearance = PathMap_MinFloat(clearance, candidate);
    }

    return clearance;
}
