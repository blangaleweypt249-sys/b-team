/**
 * @file j4310_position_control.h
 * @brief 定义 J4310 位置控制器的数据结构和接口。
 */

#ifndef J4310_POSITION_CONTROL_H
#define J4310_POSITION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool initialized;
    bool trajectory_active;
    bool feedback_seen;
    bool gravity_position_valid;
    bool gravity_learning_target_valid;
    bool gravity_learning_locked;
    uint32_t trajectory_start_ms;
    uint32_t trajectory_duration_ms;
    uint32_t last_feedback_ms;
    uint32_t gravity_settle_feedback_count;
    uint32_t gravity_settle_required_count;
    float trajectory_start_position_rad;
    float trajectory_target_position_rad;
    float target_position_rad;
    float target_velocity_rad_s;
    float max_velocity_rad_s;
    float max_acceleration_rad_s2;
    float gravity_model_limit_nm;
    float gravity_learning_rate;
    float gravity_compensation_gain;
    float gravity_disable_half_width_rad;
    float gravity_settle_error_rad;
    float gravity_torque_rate_limit_nm_s;
    float gravity_learning_target_rad;
    float gravity_actual_position_rad;
    float gravity_reference_position_rad;
    float gravity_cos_nm;
    float gravity_sin_nm;
    float gravity_filtered_torque_nm;
    float gravity_torque_nm;
} j4310_position_control_t;

/* 功能：校验参数并初始化 J4310 位置控制器；用途：建立轨迹和重力补偿运行状态；返回 true 表示初始化成功。 */
bool J4310PositionControl_Init(j4310_position_control_t *control,
                               float max_velocity_rad_s,
                               float max_acceleration_rad_s2,
                               float gravity_model_limit_nm,
                               float gravity_learning_rate,
                               float gravity_compensation_gain,
                               float gravity_disable_half_width_rad,
                               float gravity_settle_error_rad,
                               uint32_t gravity_settle_required_count,
                               float gravity_torque_rate_limit_nm_s);
/* 功能：启动从当前位置到目标位置的五次轨迹；用途：生成平滑且受限的关节运动；返回 true 表示轨迹已建立。 */
bool J4310PositionControl_Start(j4310_position_control_t *control,
                                uint32_t tick_ms,
                                float start_position_rad,
                                float target_position_rad);
/* 功能：取消运动轨迹并保持指定位置；用途：在启动或模式切换时建立静止目标；无返回值表示保持目标已更新。 */
void J4310PositionControl_Hold(j4310_position_control_t *control,
                               float position_rad);
/* 功能：取消当前 J4310 位置轨迹；用途：停止轨迹推进并清除目标速度；无返回值表示轨迹已停用。 */
void J4310PositionControl_CancelTrajectory(
    j4310_position_control_t *control);
/* 功能：按当前时刻采样 J4310 五次位置轨迹；用途：输出连续的位置和速度目标；无返回值表示采样结果已写入输出参数。 */
void J4310PositionControl_Sample(j4310_position_control_t *control,
                                 uint32_t tick_ms,
                                 float *position_rad,
                                 float *velocity_rad_s);
/* 功能：学习重力模型并合成最终 J4310 扭矩；用途：叠加请求扭矩、重力补偿和安全限幅；返回值表示最终扭矩命令。 */
float J4310PositionControl_ComposeTorque(
    j4310_position_control_t *control,
    bool feedback_fresh,
    uint32_t feedback_ms,
    float actual_position_rad,
    float actual_velocity_rad_s,
    float feedback_torque_nm,
    float desired_position_rad,
    float desired_velocity_rad_s,
    float requested_torque_nm,
    float torque_limit_nm);

#endif
