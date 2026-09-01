/**
 * @file j4310_position_control.h
 * @brief 定义 J4310 位置控制器的数据结构和接口。
 */

#ifndef J4310_POSITION_CONTROL_H
/** 防止 j4310_position_control.h 被重复包含。 */
#define J4310_POSITION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/** 保存 J4310 运行过程中需要集中管理的数据。 */
typedef struct
{
    bool initialized; /**< 控制状态是否已经初始化。 */
    bool trajectory_active; /**< 当前是否正在执行平滑位置轨迹。 */
    bool feedback_seen; /**< 控制器是否至少收到过一帧 J4310 有效反馈。 */
    bool gravity_position_valid; /**< 重力模型使用的关节角是否已经由有效反馈初始化。 */
    bool gravity_learning_target_valid; /**< 重力辨识目标位置是否已经设置。 */
    bool gravity_learning_locked; /**< 重力模型参数是否已锁定并停止继续学习。 */
    uint32_t trajectory_start_ms; /**< 当前轨迹开始执行的系统毫秒时刻。 */
    uint32_t trajectory_duration_ms; /**< 当前轨迹计划执行的总时间，单位：毫秒。 */
    uint32_t last_feedback_ms; /**< 最近一次使用 J4310 有效反馈的系统毫秒时刻。 */
    uint32_t gravity_settle_feedback_count; /**< J4310 位置连续满足重力辨识条件的反馈帧数。 */
    uint32_t gravity_settle_required_count; /**< 开始更新重力模型前要求连续稳定的反馈帧数。 */
    float trajectory_start_position_rad; /**< J4310的轨迹起始位置，单位：弧度。 */
    float trajectory_target_position_rad; /**< J4310的目标位置，单位：弧度。 */
    float target_position_rad; /**< J4310的目标位置，单位：弧度。 */
    float target_velocity_rad_s; /**< J4310的目标速度，单位：弧度每秒。 */
    float max_velocity_rad_s; /**< J4310的最大允许速度，单位：弧度每秒。 */
    float max_acceleration_rad_s2; /**< J4310 平滑位置轨迹允许的最大加速度，单位：弧度每二次方秒。 */
    float gravity_model_limit_nm; /**< 重力模型允许输出的最大补偿转矩，单位：牛米。 */
    float gravity_learning_rate; /**< 每个有效样本更新重力模型参数的步长。 */
    float gravity_compensation_gain; /**< 重力模型估计值施加到控制命令的比例。 */
    float gravity_disable_half_width_rad; /**< 零位附近关闭重力补偿区域的半宽，单位：弧度。 */
    float gravity_settle_error_rad; /**< 允许重力辨识采样的位置误差，单位：弧度。 */
    float gravity_torque_rate_limit_nm_s; /**< J4310的转矩上限，单位：牛米。 */
    float gravity_learning_target_rad; /**< 当前重力辨识采样点的目标关节角，单位：弧度。 */
    float gravity_actual_position_rad; /**< J4310的实际位置，单位：弧度。 */
    float gravity_reference_position_rad; /**< J4310的轨迹参考位置，单位：弧度。 */
    float gravity_cos_nm; /**< 重力转矩模型中余弦基函数的系数，单位：牛米。 */
    float gravity_sin_nm; /**< 重力转矩模型中正弦基函数的系数，单位：牛米。 */
    float gravity_filtered_torque_nm; /**< 低通滤波后的 J4310 反馈转矩，单位：牛米。 */
    float gravity_torque_nm; /**< 本周期实际施加的 J4310 重力补偿转矩，单位：牛米。 */
} j4310_position_control_t;

/* 功能：校验参数并初始化 J4310 位置控制器；用途：建立轨迹和重力补偿运行状态；返回 true 表示初始化成功。 */
bool J4310PositionControl_Init(j4310_position_control_t *control /* 需要读取或更新的控制状态 */,
                               float max_velocity_rad_s /* 轨迹允许的最大速度，单位：弧度每秒 */,
                               float max_acceleration_rad_s2 /* 轨迹允许的最大加速度，单位：弧度每二次方秒 */,
                               float gravity_model_limit_nm /* 允许设置的转矩上限，单位：牛米 */,
                               float gravity_learning_rate /* 每个有效样本更新重力模型参数的步长 */,
                               float gravity_compensation_gain /* 重力模型估计值施加到控制命令的比例 */,
                               float gravity_disable_half_width_rad /* 零位附近关闭重力补偿区域的半宽，单位：弧度 */,
                               float gravity_settle_error_rad /* 允许重力辨识采样的位置误差，单位：弧度 */,
                               uint32_t gravity_settle_required_count /* 开始学习前位置连续稳定所需的反馈帧数 */,
                               float gravity_torque_rate_limit_nm_s /* 重力补偿转矩允许的最大变化率，单位：牛米每秒 */);
/* 功能：启动从当前位置到目标位置的五次轨迹；用途：生成平滑且受限的关节运动；返回 true 表示轨迹已建立。 */
bool J4310PositionControl_Start(j4310_position_control_t *control /* 需要读取或更新的控制状态 */,
                                uint32_t tick_ms /* 当前系统毫秒时刻 */,
                                float start_position_rad /* 轨迹起始关节角，单位：弧度 */,
                                float target_position_rad /* 轨迹目标关节角，单位：弧度 */);
/* 功能：取消运动轨迹并保持指定位置；用途：在启动或模式切换时建立静止目标；无返回值表示保持目标已更新。 */
void J4310PositionControl_Hold(j4310_position_control_t *control /* 需要读取或更新的控制状态 */,
                               float position_rad /* 目标或反馈位置，单位：弧度 */);
/* 功能：取消当前 J4310 位置轨迹；用途：停止轨迹推进并清除目标速度；无返回值表示轨迹已停用。 */
void J4310PositionControl_CancelTrajectory(
    j4310_position_control_t *control /* 需要读取或更新的控制状态 */);
/* 功能：按当前时刻采样 J4310 五次位置轨迹；用途：输出连续的位置和速度目标；无返回值表示采样结果已写入输出参数。 */
void J4310PositionControl_Sample(j4310_position_control_t *control /* 需要读取或更新的控制状态 */,
                                 uint32_t tick_ms /* 当前系统毫秒时刻 */,
                                 float *position_rad /* 目标或反馈位置，单位：弧度 */,
                                 float *velocity_rad_s /* 目标或反馈速度，单位：弧度每秒 */);
/* 功能：学习重力模型并合成最终 J4310 扭矩；用途：叠加请求扭矩、重力补偿和安全限幅；返回值表示最终扭矩命令。 */
float J4310PositionControl_ComposeTorque(
    j4310_position_control_t *control /* 需要读取或更新的控制状态 */,
    bool feedback_fresh /* 当前反馈是否仍在允许的超时时间内 */,
    uint32_t feedback_ms /* 当前反馈对应的系统毫秒时刻 */,
    float actual_position_rad /* J4310 当前实测关节角，单位：弧度 */,
    float actual_velocity_rad_s /* J4310 当前实测关节速度，单位：弧度每秒 */,
    float feedback_torque_nm /* J4310 当前反馈转矩，单位：牛米 */,
    float desired_position_rad /* J4310 当前期望关节角，单位：弧度 */,
    float desired_velocity_rad_s /* J4310 当前期望关节速度，单位：弧度每秒 */,
    float requested_torque_nm /* 位置控制器请求输出的转矩，单位：牛米 */,
    float torque_limit_nm /* 允许设置的转矩上限，单位：牛米 */);

#endif
