/**
 * @file m3508.h
 * @brief 定义 M3508 电机配置、反馈、控制状态和接口。
 */

#ifndef M3508_H
#define M3508_H /**< 防止 m3508.h 被重复包含。 */

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define M3508_CAN_BUS_COUNT  3U /**< M3508 驱动可同时管理的 CAN 总线数量。 */
#define M3508_MOTOR_COUNT    8U /**< M3508 驱动可同时管理的电机数量。 */
#define M3508_DEFAULT_ACCEL_LIMIT_RAD_S2 52.35987756f /**< 机械臂 M3508 输出轴默认使用的加速度上限，单位：弧度每二次方秒。 */

/** 表示 M3508 可选择的工作模式。 */
typedef enum
{
    M3508_MODE_STOP, /**< 停止输出。 */
    M3508_MODE_CURRENT, /**< 按目标电流控制。 */
    M3508_MODE_VELOCITY, /**< 按目标速度闭环控制。 */
    M3508_MODE_POSITION /**< 按目标位置闭环控制。 */
} m3508_mode_t;

/** 保存 M3508 初始化和控制所需的配置参数。 */
typedef struct
{
    float kp; /**< 比例增益。 */
    float ki; /**< 积分增益。 */
    float kd; /**< 微分增益。 */
    float integral_limit; /**< 积分累计值的绝对值上限。 */
    float output_limit; /**< 控制器输出绝对值上限。 */
} m3508_pid_cfg_t;

/** 保存 M3508 当前运行状态和中间计算数据。 */
typedef struct
{
    bool enabled; /**< M3508 速度环在线 PID 调参是否启用。 */
    uint8_t strategy; /**< 在线调参器当前采用的增益调整策略。 */
    uint8_t active_rule; /**< 本周期实际生效的专家调整规则。 */
    float applied_kp; /**< 本控制周期实际应用的比例增益。 */
    float applied_ki; /**< 本控制周期实际应用的积分增益。 */
    float applied_kd; /**< 本控制周期实际应用的微分增益。 */
} m3508_online_pid_state_t;

/** 保存 M3508 当前运行状态和中间计算数据。 */
typedef struct
{
    float final_position_rad; /**< M3508的目标位置，单位：弧度。 */
    float reference_position_rad; /**< M3508的轨迹参考位置，单位：弧度。 */
    float final_velocity_rad_s; /**< 当前轨迹结束时要求达到的目标速度，单位：弧度每秒。 */
    float reference_velocity_rad_s; /**< M3508的轨迹参考速度，单位：弧度每秒。 */
    float reference_acceleration_rad_s2; /**< M3508的轨迹加速度，单位：弧度每二次方秒。 */
    float current_command_a; /**< M3508的目标电流，单位：安培。 */
    float position_error_rad; /**< M3508的位置误差，单位：弧度。 */
    float velocity_error_rad_s; /**< M3508的速度误差，单位：弧度每秒。 */
    float trajectory_progress; /**< 当前轨迹已完成的归一化比例，范围为 0 至 1。 */
    bool trajectory_active; /**< 当前是否正在执行平滑位置轨迹。 */
} m3508_motion_state_t;

/** 表示 M3508 当前所处的运行状态。 */
typedef enum
{
    M3508_AUTOTUNE_IDLE, /**< 当前未执行该流程。 */
    M3508_AUTOTUNE_RUNNING, /**< 当前正在执行该流程。 */
    M3508_AUTOTUNE_COMPLETE, /**< 流程已经成功完成。 */
    M3508_AUTOTUNE_FAILED /**< 流程因条件不满足或超时而失败。 */
} m3508_autotune_status_t;

/** 保存 M3508 当前运行状态和中间计算数据。 */
typedef struct
{
    m3508_autotune_status_t status; /**< M3508 速度环自动整定状态。 */
    float relay_current_a; /**< 继电自动整定施加的测试电流幅值，单位：安培。 */
    float hysteresis_rad_s; /**< 继电自动整定切换电流方向的速度迟滞带，单位：弧度每秒。 */
    float oscillation_amplitude_rad_s; /**< 自动整定测得的稳态速度振荡幅值，单位：弧度每秒。 */
    float ultimate_period_s; /**< 自动整定测得的临界振荡周期，单位：秒。 */
    float ultimate_gain; /**< 根据继电测试计算得到的临界比例增益。 */
    m3508_pid_cfg_t result; /**< 自动整定完成后计算得到的速度环 PID 参数。 */
} m3508_autotune_state_t;

/** 保存 M3508 初始化和控制所需的配置参数。 */
typedef struct
{
    float current_limit_a; /**< M3508的电流上限，单位：安培。 */
    float position_vel_limit_rad_s; /**< 位置轨迹允许的最大输出轴速度，单位：弧度每秒。 */
    float acceleration_limit_rad_s2; /**< M3508 输出轴允许的最大加速度，单位：弧度每二次方秒。 */
    uint32_t feedback_timeout_ms; /**< M3508 反馈离线判定时间，单位：毫秒。 */
    uint32_t command_timeout_ms; /**< M3508 控制命令失效时间，单位：毫秒。 */
    m3508_pid_cfg_t speed_pid; /**< M3508 速度环 PID 参数。 */
    m3508_pid_cfg_t position_pid; /**< M3508 位置环 PID 参数。 */
} m3508_cfg_t;

/** 保存 M3508 最近一次有效反馈及其时间信息。 */
typedef struct
{
    uint8_t can_bus; /**< 该 M3508 反馈所属的 CAN 总线编号。 */
    uint8_t motor_id; /**< DJI 电机编号。 */
    uint16_t rotor_encoder; /**< 最近反馈的单圈转子编码器原始值。 */
    int16_t rotor_speed_rpm; /**< M3508的当前速度，单位：转每分。 */
    int16_t torque_current_raw; /**< DJI 反馈帧中的转矩电流原始值。 */
    uint8_t temperature_c; /**< M3508反馈的温度，单位：摄氏度。 */
    int64_t total_encoder_counts; /**< 展开单圈编码器后得到的累计转子计数。 */
    int64_t zero_encoder_counts; /**< 建立软件零点时记录的累计转子计数。 */
    float output_pos_rad; /**< M3508的当前位置，单位：弧度。 */
    float output_vel_rad_s; /**< M3508的当前速度，单位：弧度每秒。 */
    float torque_current_a; /**< 根据协议量程换算后的反馈电流，单位：安培。 */
    uint32_t updated_at_ms; /**< 最近一次收到有效反馈的系统毫秒时刻。 */
    uint32_t rx_frames; /**< 累计接收并接受的反馈帧数量。 */
} m3508_feedback_t;

/** 保存 M3508 通信和运行诊断数据。 */
typedef struct
{
    bool command_timed_out; /**< 电机命令是否已经超过允许的更新时间。 */
    bool feedback_timed_out; /**< 电机反馈是否已经超过允许的更新时间。 */
    uint32_t command_timeout_count; /**< 累计发生命令超时的次数。 */
    uint32_t feedback_timeout_count; /**< 累计发生反馈超时的次数。 */
} m3508_timeout_stats_t;

/* 功能：初始化全部 M3508 上下文和默认控制参数；用途：建立各 CAN 总线的反馈与双环控制状态；返回 true 表示配置被接受。 */
bool M3508_Init(const m3508_cfg_t *cfg /**< M3508 电流限幅、轨迹及超时配置 */);
/* 功能：设置指定 M3508 的停止、电流、速度或位置目标；用途：更新下一控制周期的命令；返回 true 表示模式和目标合法。 */
bool M3508_SetTarget(uint8_t can_bus /**< CAN 总线编号 */,
                     uint8_t motor_id /**< DJI 电机编号 */,
                     m3508_mode_t mode /**< M3508 目标值采用的控制模式 */,
                     float target /**< 按 M3508 控制模式解释的电流、速度或位置目标 */,
                     uint32_t tick_ms /**< 当前系统毫秒时刻 */);
/* 功能：根据当前模式和反馈计算 M3508 原始电流命令；用途：驱动双环控制、轨迹和自动整定；返回 true 表示成功写出电流。 */
bool M3508_CalcCurrentRaw(uint8_t can_bus /**< CAN 总线编号 */,
                          uint8_t motor_id /**< DJI 电机编号 */,
                          uint32_t tick_ms /**< 当前系统毫秒时刻 */,
                          int16_t *current_raw /**< DJI 协议中的电流命令原始值 */);
/* 功能：解析 M3508 DJI 反馈帧并展开位置；用途：更新闭环反馈、时间戳和通信统计；返回 true 表示帧地址有效。 */
bool M3508_OnFrame(uint8_t can_bus /**< CAN 总线编号 */,
                    uint8_t motor_id /**< DJI 电机编号 */,
                    const can_frame_t *frame /**< 待解析的 CAN 接收帧 */,
                    uint32_t tick_ms /**< 当前系统毫秒时刻 */);
/* 功能：读取指定 M3508 的最新反馈；用途：获取编码器、速度、电流和温度；返回 true 表示已收到有效帧。 */
bool M3508_GetFeedback(uint8_t can_bus /**< CAN 总线编号 */,
                       uint8_t motor_id /**< DJI 电机编号 */,
                       m3508_feedback_t *feedback /**< 用于写出最新 M3508 反馈的对象 */);
/* 功能：读取带零点和多圈位置的 M3508 反馈快照；用途：供诊断与位置显示使用；返回 true 表示快照有效。 */
bool M3508_GetFeedbackSnapshot(uint8_t can_bus /**< CAN 总线编号 */,
                               uint8_t motor_id /**< DJI 电机编号 */,
                               m3508_feedback_t *feedback /**< 用于写出最新 M3508 反馈的对象 */,
                               bool *zero_valid /**< 用于写出电机软件零点是否有效 */);
/* 功能：读取指定 M3508 的通信超时统计；用途：诊断丢帧、恢复次数和帧间隔；返回 true 表示统计已写出。 */
bool M3508_GetTimeoutStats(uint8_t can_bus /**< CAN 总线编号 */,
                           uint8_t motor_id /**< DJI 电机编号 */,
                           m3508_timeout_stats_t *stats /**< 用于写出 M3508 命令与反馈超时统计的对象 */);
/* 功能：设置指定 M3508 的速度环 PID；用途：调整速度响应并重新配置在线调参；返回 true 表示参数有效。 */
bool M3508_SetSpeedPid(uint8_t can_bus /**< CAN 总线编号 */,
                       uint8_t motor_id /**< DJI 电机编号 */,
                       const m3508_pid_cfg_t *cfg /**< 待设置或校验的 M3508 PID 配置 */);
/* 功能：设置指定 M3508 的位置环 PID；用途：调整位置到速度的外环响应；返回 true 表示参数有效。 */
bool M3508_SetPositionPid(uint8_t can_bus /**< CAN 总线编号 */,
                          uint8_t motor_id /**< DJI 电机编号 */,
                          const m3508_pid_cfg_t *cfg /**< 待设置或校验的 M3508 PID 配置 */);
/* 功能：设置指定 M3508 的软件电流限制；用途：约束所有闭环和直接控制输出；返回 true 表示限制合法。 */
bool M3508_SetCurrentLimit(uint8_t can_bus /**< CAN 总线编号 */,
                           uint8_t motor_id /**< DJI 电机编号 */,
                           float current_limit_a /**< 允许输出的最大电流，单位：安培 */);
/* 功能：设置 M3508 位置模式的最大速度；用途：限制位置轨迹运动速度；返回 true 表示限制合法。 */
bool M3508_SetPositionVelocityLimit(uint8_t can_bus /**< CAN 总线编号 */,
                                    uint8_t motor_id /**< DJI 电机编号 */,
                                    float velocity_limit_rad_s /**< 允许设置的速度上限，单位：弧度每秒 */);
/* 功能：设置 M3508 速度变化的加速度限制；用途：平滑速度和位置轨迹；返回 true 表示限制合法。 */
bool M3508_SetAccelerationLimit(uint8_t can_bus /**< CAN 总线编号 */,
                                uint8_t motor_id /**< DJI 电机编号 */,
                                float acceleration_limit_rad_s2 /**< 允许设置的加速度上限，单位：弧度每二次方秒 */);
/* 功能：启用或关闭指定 M3508 的速度环在线调参；用途：切换固定与自适应 PID；返回 true 表示设置成功。 */
bool M3508_SetOnlinePidEnabled(uint8_t can_bus /**< CAN 总线编号 */,
                               uint8_t motor_id /**< DJI 电机编号 */,
                               bool enabled /**< 是否启用 M3508 速度环在线 PID 调参 */);
/* 功能：把当前 M3508 多圈位置记录为软件零点；用途：建立输出轴相对位置基准；返回 true 表示已有反馈且置零成功。 */
bool M3508_ZeroPosition(uint8_t can_bus /**< CAN 总线编号 */, uint8_t motor_id /**< DJI 电机编号 */);
/* 功能：读取指定 M3508 的在线 PID 调参状态；用途：观察当前增益、误差和活动规则；返回 true 表示状态已写出。 */
bool M3508_GetOnlinePidState(uint8_t can_bus /**< CAN 总线编号 */,
                             uint8_t motor_id /**< DJI 电机编号 */,
                             m3508_online_pid_state_t *state /**< 用于写出 M3508 在线 PID 调参状态的对象 */);
/* 功能：读取 M3508 内部斜坡和位置轨迹状态；用途：诊断当前平滑给定及轨迹是否活动；返回 true 表示状态有效。 */
bool M3508_GetMotionState(uint8_t can_bus /**< CAN 总线编号 */,
                          uint8_t motor_id /**< DJI 电机编号 */,
                          m3508_motion_state_t *state /**< 用于写出 M3508 斜坡与位置轨迹状态的对象 */);
/* 功能：读取指定 M3508 的速度环 PID 配置；用途：显示或保存当前控制参数；返回 true 表示配置已写出。 */
bool M3508_GetSpeedPid(uint8_t can_bus /**< CAN 总线编号 */,
                       uint8_t motor_id /**< DJI 电机编号 */,
                       m3508_pid_cfg_t *cfg /**< 用于写出当前 M3508 PID 配置的对象 */);
/* 功能：读取指定 M3508 的位置环 PID 配置；用途：显示或保存当前控制参数；返回 true 表示配置已写出。 */
bool M3508_GetPositionPid(uint8_t can_bus /**< CAN 总线编号 */,
                          uint8_t motor_id /**< DJI 电机编号 */,
                          m3508_pid_cfg_t *cfg /**< 用于写出当前 M3508 PID 配置的对象 */);
/* 功能：启动指定 M3508 的速度环自动整定；用途：通过继电反馈测试估算控制增益；返回 true 表示整定已进入运行态。 */
bool M3508_StartSpeedAutoTune(uint8_t can_bus /**< CAN 总线编号 */,
                              uint8_t motor_id /**< DJI 电机编号 */,
                              float relay_current_a /**< 自动整定施加的继电测试电流，单位：安培 */,
                              float hysteresis_rad_s /**< 自动整定切换电流方向的速度迟滞带，单位：弧度每秒 */,
                              float safety_velocity_rad_s /**< 自动整定允许达到的安全速度上限，单位：弧度每秒 */,
                              uint32_t tick_ms /**< 当前系统毫秒时刻 */);
/* 功能：取消正在进行的 M3508 自动整定；用途：响应用户停止或安全条件变化；返回 true 表示地址有效并已取消。 */
bool M3508_CancelAutoTune(uint8_t can_bus /**< CAN 总线编号 */, uint8_t motor_id /**< DJI 电机编号 */);
/* 功能：读取指定 M3508 的自动整定状态和结果；用途：向调试界面报告进度、参数或失败原因；返回 true 表示状态已写出。 */
bool M3508_GetAutoTuneState(uint8_t can_bus /**< CAN 总线编号 */,
                            uint8_t motor_id /**< DJI 电机编号 */,
                            m3508_autotune_state_t *state /**< 用于写出 M3508 自动整定状态和结果的对象 */);

#endif
