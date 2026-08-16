#ifndef PATH_H
#define PATH_H

#include "path_map.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 遥控器六键（LoRa 本机控制帧 payload[4] 低 6 位；bit6/7 是肩键旋转，
 * 本层强制 z=0 不响应）。各键语义：
 *   按键 1：切换当前地图段的单轴模式（按下沿触发）
 *   按键 2：去程走完后进入回程（掉头 180° 返回起点）；回程走完后再按退出
 *   按键 3~6：预留
 */
#define PATH_REMOTE_BUTTON_1_BIT   (1U << 0U)
#define PATH_REMOTE_BUTTON_2_BIT   (1U << 1U)
#define PATH_REMOTE_BUTTON_3_BIT   (1U << 2U)
#define PATH_REMOTE_BUTTON_4_BIT   (1U << 3U)
#define PATH_REMOTE_BUTTON_5_BIT   (1U << 4U)
#define PATH_REMOTE_BUTTON_6_BIT   (1U << 5U)

/* 语义别名 */
#define PATH_REMOTE_MODE_BUTTON_BIT   PATH_REMOTE_BUTTON_1_BIT
#define PATH_REMOTE_RETURN_BUTTON_BIT PATH_REMOTE_BUTTON_2_BIT

typedef struct
{
    bool initialized;
    bool remote_online;
    bool odometry_valid;
    bool initial_position_valid;
    bool single_axis_enabled;
    bool route_complete;
    bool neutral_rearm_required;
    bool front_laser_online;
    bool left_laser_online;
    bool front_hard_blocked;
    bool left_hard_blocked;
    bool map_speed_limited;
    bool front_speed_limited;
    bool left_speed_limited;
    bool yaw_zero_lock_ready;
    bool map_mirrored;
    bool return_mode;
    bool return_yaw_aligned;
    bool return_complete;
    path_map_axis_t active_axis;
    uint8_t segment_index;
    uint8_t segment_count;
    /* 前初始定位字段为接口兼容保留且恒为 0；初始锚定只等待左光。 */
    uint8_t front_initial_sample_count;
    uint8_t left_initial_sample_count;
    uint16_t front_distance_cm;
    uint16_t left_distance_cm;
    uint32_t last_remote_ms;
    uint32_t manual_toggle_count;
    uint32_t automatic_cancel_count;
    uint32_t initial_position_reject_count;
    float map_x_m;
    float map_y_m;
    float initial_map_x_m;
    float initial_map_y_m;
    float initial_yaw_deg;
    /* 前距离和交点为接口兼容保留且恒为 0。 */
    float front_initial_distance_m;
    float left_initial_distance_m;
    float front_wall_hit_x_m;
    float left_wall_hit_y_m;
    float encoder_velocity_x_mps;
    float encoder_velocity_y_mps;
    float map_clearance_m;
    float front_required_distance_m;
    float left_required_distance_m;
    float front_allowed_speed_mps;
    float left_allowed_speed_mps;
    int16_t raw_vx;
    int16_t raw_vy;
    int16_t output_vx;
    int16_t output_vy;
    int16_t output_z;
} path_diagnostics_t;

/** 在任务启动前初始化人工路径辅助层。 */
void Path_Init(void);

/**
 * 由 LoRa 本机控制帧提交人工指令和六键状态。
 * 函数保存原始摇杆指令，并把参数替换成最近一次 1 ms 安全计算结果；
 * z 始终返回 0，因此两个肩键不能改变 yaw。
 */
void Path_SubmitRemoteCommand(int16_t *vx, int16_t *vy, int16_t *z,
                              uint8_t six_buttons, uint32_t now_ms);

/** LoRa 200 ms 超时时立即撤销待执行命令和单轴约束。 */
void Path_NotifyRemoteOffline(uint32_t now_ms);

/**
 * 无小电脑模式下，把其他来源的底盘速度替换为最近的安全人工输出，
 * 避免已有上位机协议旁路本安全层。
 */
void Path_ReplaceNonRemoteCommand(int16_t *vx, int16_t *vy, int16_t *z);

/** 在融合里程计更新后每 1 ms 调用。 */
void Path_Run1ms(uint32_t now_ms);

/** 获取人工分段、激光、里程计和限速状态。 */
bool Path_GetDiagnostics(path_diagnostics_t *diagnostics);

#ifdef __cplusplus
}
#endif

#endif /* PATH_H */
