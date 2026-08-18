/**
 * @file m3508.h
 * @brief 定义 M3508 电机配置、反馈、控制状态和接口。
 */

#ifndef M3508_H
#define M3508_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define M3508_CAN_BUS_COUNT  3U
#define M3508_MOTOR_COUNT    8U
#define M3508_DEFAULT_ACCEL_LIMIT_RAD_S2 52.35987756f

typedef enum
{
    M3508_MODE_STOP,
    M3508_MODE_CURRENT,
    M3508_MODE_VELOCITY,
    M3508_MODE_POSITION
} m3508_mode_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} m3508_pid_cfg_t;

typedef struct
{
    bool enabled;
    uint8_t strategy;
    uint8_t active_rule;
    float applied_kp;
    float applied_ki;
    float applied_kd;
} m3508_online_pid_state_t;

typedef struct
{
    float final_position_rad;
    float reference_position_rad;
    float final_velocity_rad_s;
    float reference_velocity_rad_s;
    float reference_acceleration_rad_s2;
    float current_command_a;
    float position_error_rad;
    float velocity_error_rad_s;
    float trajectory_progress;
    bool trajectory_active;
} m3508_motion_state_t;

typedef enum
{
    M3508_AUTOTUNE_IDLE,
    M3508_AUTOTUNE_RUNNING,
    M3508_AUTOTUNE_COMPLETE,
    M3508_AUTOTUNE_FAILED
} m3508_autotune_status_t;

typedef struct
{
    m3508_autotune_status_t status;
    float relay_current_a;
    float hysteresis_rad_s;
    float oscillation_amplitude_rad_s;
    float ultimate_period_s;
    float ultimate_gain;
    m3508_pid_cfg_t result;
} m3508_autotune_state_t;

typedef struct
{
    float current_limit_a;
    float position_vel_limit_rad_s;
    float acceleration_limit_rad_s2;
    uint32_t feedback_timeout_ms;
    uint32_t command_timeout_ms;
    m3508_pid_cfg_t speed_pid;
    m3508_pid_cfg_t position_pid;
} m3508_cfg_t;

typedef struct
{
    uint8_t can_bus;
    uint8_t motor_id;
    uint16_t rotor_encoder;
    int16_t rotor_speed_rpm;
    int16_t torque_current_raw;
    uint8_t temperature_c;
    int64_t total_encoder_counts;
    int64_t zero_encoder_counts;
    float output_pos_rad;
    float output_vel_rad_s;
    float torque_current_a;
    uint32_t updated_at_ms;
    uint32_t rx_frames;
} m3508_feedback_t;

typedef struct
{
    bool command_timed_out;
    bool feedback_timed_out;
    uint32_t command_timeout_count;
    uint32_t feedback_timeout_count;
} m3508_timeout_stats_t;

/* 功能：初始化全部 M3508 上下文和默认控制参数；用途：建立各 CAN 总线的反馈与双环控制状态；返回 true 表示配置被接受。 */
bool M3508_Init(const m3508_cfg_t *cfg);
/* 功能：设置指定 M3508 的停止、电流、速度或位置目标；用途：更新下一控制周期的命令；返回 true 表示模式和目标合法。 */
bool M3508_SetTarget(uint8_t can_bus,
                     uint8_t motor_id,
                     m3508_mode_t mode,
                     float target,
                     uint32_t tick_ms);
/* 功能：根据当前模式和反馈计算 M3508 原始电流命令；用途：驱动双环控制、轨迹和自动整定；返回 true 表示成功写出电流。 */
bool M3508_CalcCurrentRaw(uint8_t can_bus,
                          uint8_t motor_id,
                          uint32_t tick_ms,
                          int16_t *current_raw);
/* 功能：解析 M3508 DJI 反馈帧并展开位置；用途：更新闭环反馈、时间戳和通信统计；返回 true 表示帧地址有效。 */
bool M3508_OnFrame(uint8_t can_bus,
                    uint8_t motor_id,
                    const can_frame_t *frame,
                    uint32_t tick_ms);
/* 功能：读取指定 M3508 的最新反馈；用途：获取编码器、速度、电流和温度；返回 true 表示已收到有效帧。 */
bool M3508_GetFeedback(uint8_t can_bus,
                       uint8_t motor_id,
                       m3508_feedback_t *feedback);
/* 功能：读取带零点和多圈位置的 M3508 反馈快照；用途：供诊断与位置显示使用；返回 true 表示快照有效。 */
bool M3508_GetFeedbackSnapshot(uint8_t can_bus,
                               uint8_t motor_id,
                               m3508_feedback_t *feedback,
                               bool *zero_valid);
/* 功能：读取指定 M3508 的通信超时统计；用途：诊断丢帧、恢复次数和帧间隔；返回 true 表示统计已写出。 */
bool M3508_GetTimeoutStats(uint8_t can_bus,
                           uint8_t motor_id,
                           m3508_timeout_stats_t *stats);
/* 功能：设置指定 M3508 的速度环 PID；用途：调整速度响应并重新配置在线调参；返回 true 表示参数有效。 */
bool M3508_SetSpeedPid(uint8_t can_bus,
                       uint8_t motor_id,
                       const m3508_pid_cfg_t *cfg);
/* 功能：设置指定 M3508 的位置环 PID；用途：调整位置到速度的外环响应；返回 true 表示参数有效。 */
bool M3508_SetPositionPid(uint8_t can_bus,
                          uint8_t motor_id,
                          const m3508_pid_cfg_t *cfg);
/* 功能：设置指定 M3508 的软件电流限制；用途：约束所有闭环和直接控制输出；返回 true 表示限制合法。 */
bool M3508_SetCurrentLimit(uint8_t can_bus,
                           uint8_t motor_id,
                           float current_limit_a);
/* 功能：设置 M3508 位置模式的最大速度；用途：限制位置轨迹运动速度；返回 true 表示限制合法。 */
bool M3508_SetPositionVelocityLimit(uint8_t can_bus,
                                    uint8_t motor_id,
                                    float velocity_limit_rad_s);
/* 功能：设置 M3508 速度变化的加速度限制；用途：平滑速度和位置轨迹；返回 true 表示限制合法。 */
bool M3508_SetAccelerationLimit(uint8_t can_bus,
                                uint8_t motor_id,
                                float acceleration_limit_rad_s2);
/* 功能：启用或关闭指定 M3508 的速度环在线调参；用途：切换固定与自适应 PID；返回 true 表示设置成功。 */
bool M3508_SetOnlinePidEnabled(uint8_t can_bus,
                               uint8_t motor_id,
                               bool enabled);
/* 功能：把当前 M3508 多圈位置记录为软件零点；用途：建立输出轴相对位置基准；返回 true 表示已有反馈且置零成功。 */
bool M3508_ZeroPosition(uint8_t can_bus, uint8_t motor_id);
/* 功能：读取指定 M3508 的在线 PID 调参状态；用途：观察当前增益、误差和活动规则；返回 true 表示状态已写出。 */
bool M3508_GetOnlinePidState(uint8_t can_bus,
                             uint8_t motor_id,
                             m3508_online_pid_state_t *state);
/* 功能：读取 M3508 内部斜坡和位置轨迹状态；用途：诊断当前平滑给定及轨迹是否活动；返回 true 表示状态有效。 */
bool M3508_GetMotionState(uint8_t can_bus,
                          uint8_t motor_id,
                          m3508_motion_state_t *state);
/* 功能：读取指定 M3508 的速度环 PID 配置；用途：显示或保存当前控制参数；返回 true 表示配置已写出。 */
bool M3508_GetSpeedPid(uint8_t can_bus,
                       uint8_t motor_id,
                       m3508_pid_cfg_t *cfg);
/* 功能：读取指定 M3508 的位置环 PID 配置；用途：显示或保存当前控制参数；返回 true 表示配置已写出。 */
bool M3508_GetPositionPid(uint8_t can_bus,
                          uint8_t motor_id,
                          m3508_pid_cfg_t *cfg);
/* 功能：启动指定 M3508 的速度环自动整定；用途：通过继电反馈测试估算控制增益；返回 true 表示整定已进入运行态。 */
bool M3508_StartSpeedAutoTune(uint8_t can_bus,
                              uint8_t motor_id,
                              float relay_current_a,
                              float hysteresis_rad_s,
                              float safety_velocity_rad_s,
                              uint32_t tick_ms);
/* 功能：取消正在进行的 M3508 自动整定；用途：响应用户停止或安全条件变化；返回 true 表示地址有效并已取消。 */
bool M3508_CancelAutoTune(uint8_t can_bus, uint8_t motor_id);
/* 功能：读取指定 M3508 的自动整定状态和结果；用途：向调试界面报告进度、参数或失败原因；返回 true 表示状态已写出。 */
bool M3508_GetAutoTuneState(uint8_t can_bus,
                            uint8_t motor_id,
                            m3508_autotune_state_t *state);

#endif
