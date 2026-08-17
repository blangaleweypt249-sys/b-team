#ifndef PATH_MAP_H
#define PATH_MAP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 地图坐标：+X 向右，+Y 向前；正常控制锁 yaw=0，初始定点用实测 yaw。 */
#define PATH_MAP_FIELD_WIDTH_M       3.0000f
#define PATH_MAP_FIELD_HEIGHT_M      6.0000f
#define PATH_MAP_WALL_THICKNESS_M       0.0490f
#define PATH_MAP_ROBOT_LENGTH_M         0.6170f
#define PATH_MAP_ROBOT_WIDTH_M          0.4400f
#define PATH_MAP_INITIAL_CENTER_Y_M     (0.5f * PATH_MAP_ROBOT_LENGTH_M)
#define PATH_MAP_BOUNDARY_MARGIN_X_M    0.2700f
#define PATH_MAP_BOUNDARY_MARGIN_Y_M    0.3285f
#define PATH_MAP_LOCK_YAW_DEG           0.0f
#define PATH_MAP_RETURN_YAW_DEG         180.0f
/*
 * 掉头对齐允差：机器人斜置时轴对齐包络显著变宽（20° 时半宽
 * 0.22 -> 0.31 m，吃光全部安全裕量并刮墙），因此收紧到 5°，
 * 对齐前平移保持闸死，由 yaw 闭环先完成掉头。
 */
#define PATH_MAP_RETURN_ALIGN_DEG       5.0f
#define PATH_MAP_WEST_INNER_X_M         0.0490f
#define PATH_MAP_EAST_INNER_X_M         2.9510f
#define PATH_MAP_SOUTH_INNER_Y_M        0.0490f
/*
 * 起始/回程贴靠面：初始中心 Y 硬编码为车长一半（车尾贴靠面），
 * 因此本模块的南侧贴靠基准面在地图 y = 0；回程末段前光校正与
 * 兜底判定都以该面为准（不能再叠加 0.049 墙厚，否则与初始 Y
 * 约定相差一个墙厚，段终点永远无法到达）。
 */
#define PATH_MAP_START_FACE_Y_M         0.0000f
/*
 * 回程末段允许贴靠回起点：南向净空线从常规的 0.3285 m 收窄到
 * 起点中心 Y 减 15 mm。地图限速在净空 ≤10 mm 处硬停，因此硬停线
 * 位于 0.3035 m，低于段终点 0.3085 m——倒车回程没有尾部激光，
 * 末段只能靠里程计蠕行越过段终点，硬停线必须让出这段距离；
 * 正向回程另有前光 10 cm 贴面兜底。若边距不足（例如 -5 mm），
 * 硬停线会卡在段终点上方 3 mm，倒车回程永远无法判定回到起点。
 */
#define PATH_MAP_FINAL_APPROACH_MARGIN_Y_M \
    (PATH_MAP_INITIAL_CENTER_Y_M - 0.0150f)
#define PATH_MAP_WALL_B_X_MIN_M         0.0000f
#define PATH_MAP_WALL_B_X_MAX_M         2.0000f
#define PATH_MAP_WALL_B_SOUTH_Y_M       2.0750f
#define PATH_MAP_WALL_COUNT             7U
#define PATH_MAP_ROUTE_SEGMENT_COUNT 6U
#define PATH_MAP_CLEARANCE_NONE_M    1000.0f

/*
 * 对抗镜像侧（场地左右互换）：
 * 机器人贴东墙起步，车头仍朝 +Y。镜像场地把 3 段内墙整体左右对调
 * （墙 1 贴西墙、墙 B 贴东墙、墙 C 贴西墙），路线也随之左右镜像。
 */
#define PATH_MAP_MIRRORED_START_X_M     2.6260f /* 3.000-0.049-0.175-0.150 */
#define PATH_MAP_MIRRORED_FIRST_WALL_Y_M PATH_MAP_WALL_B_SOUTH_Y_M

typedef enum
{
    PATH_MAP_AXIS_X = 0,
    PATH_MAP_AXIS_Y = 1
} path_map_axis_t;

typedef struct
{
    float x_min_m;
    float y_min_m;
    float x_max_m;
    float y_max_m;
} path_map_wall_t;

typedef struct
{
    path_map_axis_t axis;
    int8_t direction;
    float target_m;
} path_map_route_segment_t;

/** 返回由参考分支 path_config.h 提取的 4 面边界墙和 3 段内墙。 */
const path_map_wall_t *PathMap_GetWalls(uint8_t *count);

/** 返回将参考地图曲线路线改为 yaw=0 人工单轴通行的直线分段。 */
const path_map_route_segment_t *PathMap_GetRoute(uint8_t *count);

/**
 * 返回回程（掉头 180°、车头朝 -Y）直线分段：
 * 常规侧从去程终点 (0.5, 3.7) 回到起点 (0.374, 0.3085)；
 * 镜像侧从 (2.5, 3.7) 回到 (2.626, 0.3085)。
 * 与去程共用 PATH_MAP_ROUTE_SEGMENT_COUNT 段。
 */
const path_map_route_segment_t *PathMap_GetReturnRoute(uint8_t *count);

/**
 * 选择地图侧别：
 * - false：常规侧，机器人贴西墙起步，左 DT35 扫描西边界锚定 X；
 * - true ：对抗镜像侧，机器人贴东墙起步，左 DT35 无目标，
 *          由 path.c 在前光首次撞墙时自动选择并锚定。
 * 默认 false；选择后 GetWalls/GetRoute/SegmentReached/RayClearance
 * 全部使用对应侧的地图数据。
 */
void PathMap_SetMirrored(bool mirrored);

/** 查询当前是否使用对抗镜像地图。 */
bool PathMap_IsMirrored(void);

/**
 * 选择运行方向：false 去程（车头 +Y），true 回程（车头 -Y）。
 * 影响 GetReturnRoute 之外 SegmentReached 使用的路线表。
 */
void PathMap_SetReturnMode(bool returning);

/** 查询当前是否处于回程方向。 */
bool PathMap_IsReturnMode(void);

/**
 * 回程最后一段贴靠起点时置 true：南向边界净空线临时收窄到
 * PATH_MAP_FINAL_APPROACH_MARGIN_Y_M，允许机器人回到起点中心 Y。
 * 其余时间必须为 false，恢复常规 0.3285 m 南边界安全线。
 */
void PathMap_SetFinalApproach(bool enabled);

/** 查询回程末段贴靠模式是否开启。 */
bool PathMap_IsFinalApproach(void);


/** 判断融合里程计地图位姿是否已经越过当前直线段终点。 */
bool PathMap_SegmentReached(uint8_t segment_index,
                            float map_x_m,
                            float map_y_m);

/**
 * 计算机器人中心沿给定方向到膨胀后静态墙面的剩余直线距离。
 * dir_x/dir_y 可以不归一化；返回值是沿单位方向的米数。
 */
float PathMap_RayClearance(float map_x_m, float map_y_m,
                           float dir_x, float dir_y);

#ifdef __cplusplus
}
#endif

#endif /* PATH_MAP_H */
