/**
 * @file arm.c
 * @brief 实现机械臂命令下发、J4310 自动回位、位置轨迹和重力补偿。
 */

#include "arm.h"

#include <math.h>
#include <string.h>

#include "bsp_can.h"
#include "j4310.h"
#include "m3508.h"

/* 功能：检查数值是否有限且位于正负限制内；用途：校验机械臂目标；返回 true 表示输入安全有效。 */
static bool Arm_ValueWithin(float value /**< 待检查绝对值上限的机械臂控制量 */, float limit /**< 输入值允许达到的绝对值上限 */)
{
    return isfinite(value) && (value >= -limit) && (value <= limit);
}

/* 功能：检查位置是否位于机构的单向安全区间；用途：限制上位机和遥控目标；返回 true 表示目标可执行。 */
static bool Arm_PositionWithin(float value /**< 待检查上下限的机械臂位置目标 */, float minimum /**< 允许输出的下限 */, float maximum /**< 允许输出的上限 */)
{
    return isfinite(value) && (value >= minimum) && (value <= maximum);
}

/* 功能：检查数值是否为有限浮点数；用途：过滤机械臂命令中的异常值；返回 true 表示数值有效。 */
static bool Arm_ValueFinite(float value /**< 待检查有限性的机械臂控制量 */)
{
    return isfinite(value);
}

/* 功能：校验机械臂 PID 参数及上下限；用途：防止非法参数写入电机驱动；返回 true 表示配置可用。 */
static bool Arm_PidValid(const upper_pid_cfg_t *cfg /**< 待校验的机构 PID 配置 */)
{
    return (cfg != NULL) && isfinite(cfg->kp) && isfinite(cfg->ki) &&
           isfinite(cfg->kd) && isfinite(cfg->integral_limit) &&
           isfinite(cfg->output_limit) && (cfg->kp >= 0.0f) &&
           (cfg->ki >= 0.0f) && (cfg->kd >= 0.0f) &&
           (cfg->integral_limit >= 0.0f) && (cfg->output_limit > 0.0f);
}

/* 功能：比较两组机械臂 PID 配置是否完全相同；用途：避免重复下发未变化的参数；返回 true 表示一致。 */
static bool Arm_PidEqual(const upper_pid_cfg_t *left /**< 待比较的左侧机构 PID 配置 */,
                         const upper_pid_cfg_t *right /**< 待比较的右侧机构 PID 配置 */)
{
    return memcmp(left, right, sizeof(*left)) == 0;
}

/* 功能：校验机械臂目标并转换为各电机命令；用途：生成单周期可统一下发的输出快照；返回 true 表示目标合法且转换完成。 */
bool Arm_Calc(const arm_target_t *target /**< 本周期机械臂关节控制目标 */, arm_output_t *output /**< 用于写出机械臂电机命令的对象 */)
{
    uint32_t index;
    bool position_mode;

    if ((target == NULL) || (output == NULL))
    {
        return false;
    }
    (void)memset(output, 0, sizeof(*output));
    if (!target->enabled && !target->m3508_enabled)
    {
        uint32_t stop_index;

        output->j4310.mode = MOTOR_CMD_STOP;
        for (stop_index = 0U;
             stop_index < UPPER_ARM_M3508_COUNT;
             stop_index++)
        {
            output->m3508[stop_index].mode = MOTOR_CMD_STOP;
        }
        return true;
    }
    position_mode = target->position_mode;
    if (target->enabled &&
        (!Arm_ValueWithin(target->grip_pos_rad,
                          UPPER_J4310_POSITION_MAX_RAD) ||
         !Arm_ValueWithin(target->grip_vel_rad_s,
                          UPPER_J4310_VELOCITY_MAX_RAD_S) ||
         !isfinite(target->grip_kp) || (target->grip_kp < 0.0f) ||
         (target->grip_kp > UPPER_J4310_KP_MAX) ||
         !isfinite(target->grip_kd) || (target->grip_kd < 0.0f) ||
         (target->grip_kd > UPPER_J4310_KD_MAX)))
    {
        return false;
    }
    if (target->pid_update &&
        (!Arm_ValueFinite(target->grip_torque_limit_nm) ||
         (target->grip_torque_limit_nm <= 0.0f) ||
         (target->grip_torque_limit_nm > UPPER_J4310_TORQUE_MAP_MAX_NM) ||
         !Arm_ValueWithin(target->grip_torque_nm,
                          target->grip_torque_limit_nm) ||
         !Arm_PidValid(&target->m3508_speed_pid) ||
         !Arm_PidValid(&target->m3508_position_pid)))
    {
        return false;
    }
    for (index = 0U;
         target->m3508_enabled && (index < UPPER_ARM_M3508_COUNT);
         index++)
    {
        if (position_mode)
        {
            if (!Arm_PositionWithin(target->m3508_pos_rad[index],
                                    UPPER_M3508_POSITION_MIN_RAD,
                                    UPPER_M3508_POSITION_MAX_RAD))
            {
                return false;
            }
        }
        else if (!Arm_ValueWithin(target->m3508_vel_rad_s[index],
                                  UPPER_M3508_POSITION_VEL_LIMIT_RAD_S))
        {
            return false;
        }
    }

    output->enabled = target->enabled;
    output->m3508_enabled = target->m3508_enabled;
    output->j4310 = (motor_cmd_t)
    {
        .mode = target->enabled ? MOTOR_CMD_MIT : MOTOR_CMD_STOP,
        .pos_rad = target->grip_pos_rad,
        .vel_rad_s = target->grip_vel_rad_s,
        .kp = target->grip_kp,
        .kd = target->grip_kd,
        .torque_nm = target->grip_torque_nm
    };
    output->pid_update = target->pid_update;
    output->j4310_torque_limit_nm = target->grip_torque_limit_nm;
    output->m3508_speed_pid = target->m3508_speed_pid;
    output->m3508_position_pid = target->m3508_position_pid;
    for (index = 0U; index < UPPER_ARM_M3508_COUNT; index++)
    {
        output->m3508[index] = (motor_cmd_t)
        {
            .mode = !target->m3508_enabled ? MOTOR_CMD_STOP :
                    (position_mode ? MOTOR_CMD_POSITION :
                                     MOTOR_CMD_VELOCITY),
            .pos_rad = position_mode ? target->m3508_pos_rad[index] : 0.0f,
            .vel_rad_s = target->m3508_vel_rad_s[index]
        };
    }
    return true;
}

/* 功能：应用机械臂输出及可选 PID 更新；用途：把 J4310 和 M3508 命令提交给电机管理器；返回 true 表示全部设置成功。 */
bool Arm_Apply(motor_manager_t *manager /**< 需要操作的电机管理器 */, const arm_output_t *output /**< 待下发的机械臂电机命令 */)
{
    uint32_t index;
    bool success;
    static bool pid_applied;
    static float applied_torque_limit;
    static upper_pid_cfg_t applied_speed_pid;
    static upper_pid_cfg_t applied_position_pid;

    if ((manager == NULL) || (output == NULL))
    {
        return false;
    }

    if (output->pid_update &&
        (!pid_applied || (applied_torque_limit != output->j4310_torque_limit_nm) ||
         !Arm_PidEqual(&applied_speed_pid, &output->m3508_speed_pid) ||
         !Arm_PidEqual(&applied_position_pid, &output->m3508_position_pid)))
    {
        m3508_pid_cfg_t speed_pid;
        m3508_pid_cfg_t position_pid;

        speed_pid = (m3508_pid_cfg_t)
        {
            output->m3508_speed_pid.kp,
            output->m3508_speed_pid.ki,
            output->m3508_speed_pid.kd,
            output->m3508_speed_pid.integral_limit,
            output->m3508_speed_pid.output_limit
        };
        position_pid = (m3508_pid_cfg_t)
        {
            output->m3508_position_pid.kp,
            output->m3508_position_pid.ki,
            output->m3508_position_pid.kd,
            output->m3508_position_pid.integral_limit,
            output->m3508_position_pid.output_limit
        };
        success = J4310_SetTorqueLimit(NODE_ARM_J4310,
                                       output->j4310_torque_limit_nm);
        for (index = 0U; index < UPPER_ARM_M3508_COUNT; index++)
        {
            success = M3508_SetSpeedPid(CAN_BUS_ARM_M3508,
                                        (uint8_t)(NODE_ARM_M3508_1 + index),
                                        &speed_pid) && success;
            success = M3508_SetPositionPid(CAN_BUS_ARM_M3508,
                                           (uint8_t)(NODE_ARM_M3508_1 + index),
                                           &position_pid) && success;
        }
        if (success)
        {
            applied_torque_limit = output->j4310_torque_limit_nm;
            applied_speed_pid = output->m3508_speed_pid;
            applied_position_pid = output->m3508_position_pid;
            pid_applied = true;
        }
        else
        {
            return false;
        }
    }

    /* 在管理器执行单周期 CAN 下发前，先暂存机械臂的全部目标。
     * 即使 J4310 和两台 M3508 使用不同 CAN 总线，也要将它们作为同一组控制请求处理。 */
    success = MotorManager_SetCmd(manager,
                                  UPPER_MOTOR_ARM_J4310,
                                  &output->j4310);
    for (index = 0U; index < UPPER_ARM_M3508_COUNT; index++)
    {
        success = MotorManager_SetCmd(
                      manager,
                      (size_t)UPPER_MOTOR_ARM_M3508_1 + index,
                      &output->m3508[index]) && success;
    }
    if (!success)
    {
        return false;
    }
    success = MotorManager_SetEnabled(manager,
                                      UPPER_MOTOR_ARM_J4310,
                                      output->enabled);
    for (index = 0U; index < UPPER_ARM_M3508_COUNT; index++)
    {
        success = MotorManager_SetEnabled(
            manager,
            (size_t)UPPER_MOTOR_ARM_M3508_1 + index,
                      output->m3508_enabled) && success;
    }
    return success;
}

#define J4310_AUTO_RETURN_CONTROL_PERIOD_MS       10U /**< J4310 自动回位轨迹状态的更新周期，单位：毫秒。 */
#define J4310_AUTO_RETURN_MAX_VELOCITY_RAD_S       2.0f /**< 机械臂 J4310 关节轨迹规划使用的最大速度，单位：弧度每秒。 */
#define J4310_AUTO_RETURN_MAX_ACCEL_RAD_S2        10.0f /**< 机械臂 J4310 关节轨迹规划使用的最大加速度，单位：弧度每二次方秒。 */
#define J4310_AUTO_RETURN_SETTLE_POSITION_RAD      0.03f /**< 自动回位完成时允许的 J4310 位置误差，单位：弧度。 */
#define J4310_AUTO_RETURN_SETTLE_VELOCITY_RAD_S    0.20f /**< 自动回位完成时允许的 J4310 关节速度，单位：弧度每秒。 */
#define J4310_AUTO_RETURN_QUINTIC_VELOCITY_BOUND   1.875f /**< 五次平滑插值曲线的一阶导数最大系数，用于根据行程和速度计算轨迹时长。 */
#define J4310_AUTO_RETURN_QUINTIC_ACCEL_BOUND      5.7736f /**< 五次平滑插值曲线的二阶导数最大系数，用于根据行程和加速度计算轨迹时长。 */

/* 功能：按回零距离计算五次轨迹持续时间；用途：同时满足最大速度和加速度约束；返回值表示轨迹时长（毫秒）。 */
static uint32_t J4310AutoReturn_TrajectoryDurationMs(float distance_rad /**< 轨迹起点与目标之间的角距离，单位：弧度 */)
{
    float velocity_time_s;
    float acceleration_time_s;
    float duration_ms;
    uint32_t period_count;

    distance_rad = fabsf(distance_rad);
    if (distance_rad <= J4310_AUTO_RETURN_SETTLE_POSITION_RAD)
    {
        return 0U;
    }
    velocity_time_s = J4310_AUTO_RETURN_QUINTIC_VELOCITY_BOUND *
                      distance_rad /
                      J4310_AUTO_RETURN_MAX_VELOCITY_RAD_S;
    acceleration_time_s = sqrtf(
        J4310_AUTO_RETURN_QUINTIC_ACCEL_BOUND * distance_rad /
        J4310_AUTO_RETURN_MAX_ACCEL_RAD_S2);
    duration_ms = ((velocity_time_s > acceleration_time_s) ?
                   velocity_time_s : acceleration_time_s) * 1000.0f;
    period_count = (uint32_t)(duration_ms /
                              (float)J4310_AUTO_RETURN_CONTROL_PERIOD_MS);
    if ((float)(period_count * J4310_AUTO_RETURN_CONTROL_PERIOD_MS) <
        duration_ms)
    {
        period_count++;
    }
    if (period_count == 0U)
    {
        period_count = 1U;
    }
    return period_count * J4310_AUTO_RETURN_CONTROL_PERIOD_MS;
}

/* 功能：采样当前自动回零五次轨迹；用途：生成平滑的位置和速度目标；无返回值表示目标已写入控制器。 */
static void J4310AutoReturn_Sample(j4310_auto_return_t *control /**< J4310 自动回零流程控制器 */,
                                   uint32_t tick_ms /**< 当前系统毫秒时刻 */)
{
    uint32_t elapsed_ms;
    float normalized_time;
    float normalized_time_2;
    float normalized_time_3;
    float normalized_time_4;
    float normalized_time_5;
    float blend;
    float blend_rate;

    if (control->trajectory_duration_ms == 0U)
    {
        control->target_position_rad = 0.0f;
        control->target_velocity_rad_s = 0.0f;
        return;
    }
    elapsed_ms = tick_ms - control->trajectory_start_ms;
    if (elapsed_ms >= control->trajectory_duration_ms)
    {
        control->target_position_rad = 0.0f;
        control->target_velocity_rad_s = 0.0f;
        return;
    }

    normalized_time = (float)elapsed_ms /
                      (float)control->trajectory_duration_ms;
    normalized_time_2 = normalized_time * normalized_time;
    normalized_time_3 = normalized_time_2 * normalized_time;
    normalized_time_4 = normalized_time_3 * normalized_time;
    normalized_time_5 = normalized_time_4 * normalized_time;
    blend = 10.0f * normalized_time_3 -
            15.0f * normalized_time_4 +
            6.0f * normalized_time_5;
    blend_rate = (30.0f * normalized_time_2 -
                  60.0f * normalized_time_3 +
                  30.0f * normalized_time_4) *
                 (1000.0f / (float)control->trajectory_duration_ms);
    control->target_position_rad =
        control->trajectory_start_position_rad * (1.0f - blend);
    control->target_velocity_rad_s =
        -control->trajectory_start_position_rad * blend_rate;
}

/* 功能：从当前位置启动 J4310 自动回零轨迹；用途：电机重连后平滑返回零位；无返回值表示控制权和轨迹状态已更新。 */
static void J4310AutoReturn_Start(j4310_auto_return_t *control /**< J4310 自动回零流程控制器 */,
                                  uint32_t tick_ms /**< 当前系统毫秒时刻 */,
                                  float position_rad /**< 启动自动回零时的实测关节位置，单位：弧度 */,
                                  float velocity_rad_s /**< 启动自动回零时的实测关节速度，单位：弧度每秒 */)
{
    control->reconnect_armed = false;
    control->owns_control = true;
    control->trajectory_start_ms = tick_ms;
    control->trajectory_start_position_rad = position_rad;
    control->trajectory_duration_ms =
        J4310AutoReturn_TrajectoryDurationMs(position_rad);
    control->target_position_rad = position_rad;
    control->target_velocity_rad_s = 0.0f;
    if ((fabsf(position_rad) <= J4310_AUTO_RETURN_SETTLE_POSITION_RAD) &&
        (fabsf(velocity_rad_s) <=
         J4310_AUTO_RETURN_SETTLE_VELOCITY_RAD_S))
    {
        control->stage = J4310_AUTO_RETURN_HOLDING;
        control->target_position_rad = 0.0f;
    }
    else
    {
        control->stage = J4310_AUTO_RETURN_RUNNING;
    }
}

/* 功能：初始化 J4310 自动回零控制器；用途：设置使能状态并清空历史状态；无返回值表示控制器已复位。 */
void J4310AutoReturn_Init(j4310_auto_return_t *control /**< J4310 自动回零流程控制器 */,
                          bool enabled /**< 是否启用 J4310 自动回零流程 */)
{
    if (control == NULL)
    {
        return;
    }
    (void)memset(control, 0, sizeof(*control));
    control->enabled = enabled;
    control->stage = enabled ? J4310_AUTO_RETURN_ARMED :
                               J4310_AUTO_RETURN_DISABLED;
}

/* 功能：重新配置自动回零使能和反馈在线状态；用途：在系统启动或配置切换时重建状态机；无返回值表示配置已生效。 */
void J4310AutoReturn_Configure(j4310_auto_return_t *control /**< J4310 自动回零流程控制器 */,
                               bool enabled /**< 是否启用 J4310 自动回零流程 */,
                               bool feedback_fresh /**< 当前反馈是否仍在允许的超时时间内 */)
{
    if (control == NULL)
    {
        return;
    }
    control->enabled = enabled;
    control->online = feedback_fresh;
    control->reconnect_armed = enabled && control->seen_online &&
                               !feedback_fresh;
    control->owns_control = false;
    control->target_position_rad = 0.0f;
    control->target_velocity_rad_s = 0.0f;
    control->stage = enabled ? J4310_AUTO_RETURN_ARMED :
                               J4310_AUTO_RETURN_DISABLED;
}

/* 功能：取消正在进行或等待中的自动回零；用途：在人工控制或停机时释放控制权；无返回值表示回零目标已清除。 */
void J4310AutoReturn_Cancel(j4310_auto_return_t *control /**< J4310 自动回零流程控制器 */)
{
    if (control == NULL)
    {
        return;
    }
    control->reconnect_armed = false;
    control->owns_control = false;
    control->target_position_rad = 0.0f;
    control->target_velocity_rad_s = 0.0f;
    control->stage = control->enabled ? J4310_AUTO_RETURN_ARMED :
                                        J4310_AUTO_RETURN_DISABLED;
}

/* 功能：按反馈在线状态推进自动回零状态机；用途：检测掉线重连并生成回零目标；无返回值表示本周期状态已更新。 */
void J4310AutoReturn_Update(j4310_auto_return_t *control /**< J4310 自动回零流程控制器 */,
                            uint32_t tick_ms /**< 当前系统毫秒时刻 */,
                            bool feedback_fresh /**< 当前反馈是否仍在允许的超时时间内 */,
                            float position_rad /**< 当前实测 J4310 关节位置，单位：弧度 */,
                            float velocity_rad_s /**< 当前实测 J4310 关节速度，单位：弧度每秒 */,
                            bool control_allowed /**< 当前周期是否允许 J4310 自动回零输出控制命令 */)
{
    bool valid_feedback;
    bool was_online;

    if (control == NULL)
    {
        return;
    }
    valid_feedback = feedback_fresh && isfinite(position_rad) &&
                     isfinite(velocity_rad_s);
    if (!valid_feedback)
    {
        if (control->online || control->owns_control)
        {
            control->online = false;
            control->reconnect_armed = control->enabled &&
                                       control_allowed &&
                                       control->seen_online;
            control->owns_control = false;
        }
        control->stage = control->enabled ? J4310_AUTO_RETURN_ARMED :
                                            J4310_AUTO_RETURN_DISABLED;
        return;
    }

    was_online = control->online;
    control->online = true;
    if (!control->seen_online)
    {
        control->seen_online = true;
        control->reconnect_armed = false;
        control->owns_control = false;
        control->stage = control->enabled ? J4310_AUTO_RETURN_ARMED :
                                            J4310_AUTO_RETURN_DISABLED;
        return;
    }
    if (!control->enabled || !control_allowed)
    {
        control->reconnect_armed = false;
        control->owns_control = false;
        control->stage = control->enabled ? J4310_AUTO_RETURN_ARMED :
                                            J4310_AUTO_RETURN_DISABLED;
        return;
    }
    if (!was_online && control->reconnect_armed &&
        !control->owns_control)
    {
        J4310AutoReturn_Start(control,
                              tick_ms,
                              position_rad,
                              velocity_rad_s);
    }
    if (!control->owns_control)
    {
        return;
    }

    if ((fabsf(position_rad) <= J4310_AUTO_RETURN_SETTLE_POSITION_RAD) &&
        (fabsf(velocity_rad_s) <=
         J4310_AUTO_RETURN_SETTLE_VELOCITY_RAD_S))
    {
        control->stage = J4310_AUTO_RETURN_HOLDING;
        control->target_position_rad = 0.0f;
        control->target_velocity_rad_s = 0.0f;
        return;
    }
    if (control->stage == J4310_AUTO_RETURN_RUNNING)
    {
        J4310AutoReturn_Sample(control, tick_ms);
    }
}

/* 功能：判断自动回零控制器是否持有控制权；用途：决定上层是否采用回零目标；返回 true 表示自动回零正在生效。 */
bool J4310AutoReturn_IsActive(const j4310_auto_return_t *control /**< J4310 自动回零流程控制器 */)
{
    return (control != NULL) && control->owns_control;
}

#define J4310_POSITION_CONTROL_PERIOD_MS              1U /**< J4310 位置轨迹和重力补偿状态的更新周期，单位：毫秒。 */
#define J4310_POSITION_CONTROL_DISTANCE_EPSILON_RAD   0.0001f /**< J4310 位置轨迹将行程视为零时使用的角度容差，单位：弧度。 */
#define J4310_POSITION_CONTROL_QUINTIC_VELOCITY_BOUND 1.875f /**< 五次平滑插值曲线的一阶导数最大系数，用于根据行程和速度计算轨迹时长。 */
#define J4310_POSITION_CONTROL_QUINTIC_ACCEL_BOUND    5.7736f /**< 五次平滑插值曲线的二阶导数最大系数，用于根据行程和加速度计算轨迹时长。 */
#define J4310_GRAVITY_LEARN_ACTUAL_VELOCITY_RAD_S     0.05f /**< 允许采集 J4310 重力模型样本的最大实测速度，单位：弧度每秒。 */
#define J4310_GRAVITY_LEARN_DESIRED_VELOCITY_RAD_S    0.05f /**< 允许采集 J4310 重力模型样本的最大期望速度，单位：弧度每秒。 */
#define J4310_GRAVITY_LEARN_POSITION_ERROR_RAD         1.00f /**< 允许采集 J4310 重力模型样本的位置跟踪误差，单位：弧度。 */
#define J4310_GRAVITY_LEARN_REQUESTED_TORQUE_NM        0.05f /**< 允许采集 J4310 重力模型样本的最大位置控制转矩，单位：牛米。 */
#define J4310_GRAVITY_LEARN_RESIDUAL_DEADBAND_NM       0.05f /**< J4310 重力模型残差进入该死区后不再更新参数，单位：牛米。 */
#define J4310_GRAVITY_POSITION_DEADBAND_RAD             0.034906585f /**< J4310 重力补偿模型在零位附近不参与输出的角度死区，单位：弧度。 */
#define J4310_PI_RAD                                    3.14159265359f /**< 角度换算使用的圆周率数值。 */
#define J4310_TWO_PI_RAD                                6.28318530718f /**< 角度换算使用的二倍圆周率数值。 */

/* 功能：将数值限制在给定上下界内；用途：约束轨迹和扭矩控制量；返回值表示限幅后的数值。 */
static float J4310PositionControl_Clamp(float value /**< 待限制到指定区间的 J4310 控制量 */,
                                        float minimum /**< 允许输出的下限 */,
                                        float maximum /**< 允许输出的上限 */)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

/* 功能：按最大步长使当前值逼近目标值；用途：限制重力补偿扭矩的变化速度；返回值表示本周期更新值。 */
static float J4310PositionControl_Approach(float current /**< 本周期更新前的重力补偿转矩 */,
                                           float target /**< 当前斜坡的目标值 */,
                                           float maximum_step /**< 两帧编码器反馈仍可视为静止的最大计数变化量 */)
{
    return current + J4310PositionControl_Clamp(
        target - current, -maximum_step, maximum_step);
}

/* 功能：根据关节角和正余弦系数计算重力模型扭矩；用途：形成位置相关的前馈补偿；返回值表示估算扭矩。 */
static float J4310PositionControl_GravityModel(
    const j4310_position_control_t *control /**< J4310 位置轨迹控制器 */,
    float position_rad /**< 用于计算重力模型的关节位置，单位：弧度 */)
{
    return control->gravity_cos_nm * cosf(position_rad) +
           control->gravity_sin_nm * sinf(position_rad);
}

/* 功能：将角度归一化到负 π 至正 π；用途：统一重力补偿的周期角度；返回值表示归一化后的角度。 */
static float J4310PositionControl_WrapToPi(float position_rad /**< 待归一化的关节位置，单位：弧度 */)
{
    position_rad = fmodf(position_rad, J4310_TWO_PI_RAD);
    if (position_rad > J4310_PI_RAD)
    {
        position_rad -= J4310_TWO_PI_RAD;
    }
    else if (position_rad < -J4310_PI_RAD)
    {
        position_rad += J4310_TWO_PI_RAD;
    }
    return position_rad;
}

/* 功能：判断当前位置是否允许启用重力补偿；用途：在禁用角度窗口内抑制补偿；返回 true 表示允许补偿。 */
static bool J4310PositionControl_GravityEnabled(
    const j4310_position_control_t *control /**< J4310 位置轨迹控制器 */,
    float position_rad /**< 用于判断重力补偿区间的关节位置，单位：弧度 */)
{
    float wrapped_abs_rad = fabsf(
        J4310PositionControl_WrapToPi(position_rad));

    return (wrapped_abs_rad > control->gravity_disable_half_width_rad) &&
           ((J4310_PI_RAD - wrapped_abs_rad) >
            control->gravity_disable_half_width_rad);
}

/* 功能：按位移计算五次位置轨迹持续时间；用途：满足配置的速度和加速度限制；返回值表示轨迹时长（毫秒）。 */
static uint32_t J4310PositionControl_DurationMs(
    const j4310_position_control_t *control /**< J4310 位置轨迹控制器 */,
    float distance_rad /**< 轨迹起点与目标之间的角距离，单位：弧度 */)
{
    float velocity_time_s;
    float acceleration_time_s;
    float duration_ms;
    uint32_t period_count;

    distance_rad = fabsf(distance_rad);
    if (distance_rad <= J4310_POSITION_CONTROL_DISTANCE_EPSILON_RAD)
    {
        return 0U;
    }
    velocity_time_s = J4310_POSITION_CONTROL_QUINTIC_VELOCITY_BOUND *
                      distance_rad / control->max_velocity_rad_s;
    acceleration_time_s = sqrtf(
        J4310_POSITION_CONTROL_QUINTIC_ACCEL_BOUND * distance_rad /
        control->max_acceleration_rad_s2);
    duration_ms = ((velocity_time_s > acceleration_time_s) ?
                   velocity_time_s : acceleration_time_s) * 1000.0f;
    period_count = (uint32_t)(duration_ms /
                              (float)J4310_POSITION_CONTROL_PERIOD_MS);
    if ((float)(period_count * J4310_POSITION_CONTROL_PERIOD_MS) <
        duration_ms)
    {
        period_count++;
    }
    return (period_count == 0U) ? J4310_POSITION_CONTROL_PERIOD_MS :
                                  period_count *
                                  J4310_POSITION_CONTROL_PERIOD_MS;
}

/* 功能：校验参数并初始化 J4310 位置控制器；用途：建立轨迹和重力补偿运行状态；返回 true 表示初始化成功。 */
bool J4310PositionControl_Init(j4310_position_control_t *control /**< J4310 位置轨迹控制器 */,
                               float max_velocity_rad_s /**< 轨迹允许的最大速度，单位：弧度每秒 */,
                               float max_acceleration_rad_s2 /**< 轨迹允许的最大加速度，单位：弧度每二次方秒 */,
                               float gravity_model_limit_nm /**< 允许设置的转矩上限，单位：牛米 */,
                               float gravity_learning_rate /**< 每个有效样本更新重力模型参数的步长 */,
                               float gravity_compensation_gain /**< 重力模型估计值施加到控制命令的比例 */,
                               float gravity_disable_half_width_rad /**< 零位附近关闭重力补偿区域的半宽，单位：弧度 */,
                               float gravity_settle_error_rad /**< 允许重力辨识采样的位置误差，单位：弧度 */,
                               uint32_t gravity_settle_required_count /**< 开始学习前位置连续稳定所需的反馈帧数 */,
                               float gravity_torque_rate_limit_nm_s /**< 重力补偿转矩允许的最大变化率，单位：牛米每秒 */)
{
    if ((control == NULL) || !isfinite(max_velocity_rad_s) ||
        !isfinite(max_acceleration_rad_s2) ||
        !isfinite(gravity_model_limit_nm) ||
        !isfinite(gravity_learning_rate) ||
        !isfinite(gravity_compensation_gain) ||
        !isfinite(gravity_disable_half_width_rad) ||
        !isfinite(gravity_settle_error_rad) ||
        !isfinite(gravity_torque_rate_limit_nm_s) ||
        (max_velocity_rad_s <= 0.0f) ||
        (max_acceleration_rad_s2 <= 0.0f) ||
        (gravity_model_limit_nm <= 0.0f) ||
        (gravity_learning_rate <= 0.0f) ||
        (gravity_learning_rate > 1.0f) ||
        (gravity_compensation_gain <= 0.0f) ||
        (gravity_disable_half_width_rad <= 0.0f) ||
        (gravity_disable_half_width_rad >= (J4310_PI_RAD * 0.5f)) ||
        (gravity_settle_error_rad <= 0.0f) ||
        (gravity_settle_error_rad >= (J4310_PI_RAD * 0.5f)) ||
        (gravity_settle_required_count == 0U) ||
        (gravity_torque_rate_limit_nm_s <= 0.0f))
    {
        return false;
    }

    (void)memset(control, 0, sizeof(*control));
    control->max_velocity_rad_s = max_velocity_rad_s;
    control->max_acceleration_rad_s2 = max_acceleration_rad_s2;
    control->gravity_model_limit_nm = gravity_model_limit_nm;
    control->gravity_learning_rate = gravity_learning_rate;
    control->gravity_compensation_gain = gravity_compensation_gain;
    control->gravity_disable_half_width_rad =
        gravity_disable_half_width_rad;
    control->gravity_settle_error_rad = gravity_settle_error_rad;
    control->gravity_settle_required_count =
        gravity_settle_required_count;
    control->gravity_torque_rate_limit_nm_s =
        gravity_torque_rate_limit_nm_s;
    control->initialized = true;
    return true;
}

/* 功能：启动从当前位置到目标位置的五次轨迹；用途：生成平滑且受限的关节运动；返回 true 表示轨迹已建立。 */
bool J4310PositionControl_Start(j4310_position_control_t *control /**< J4310 位置轨迹控制器 */,
                                uint32_t tick_ms /**< 当前系统毫秒时刻 */,
                                float start_position_rad /**< 轨迹起始关节角，单位：弧度 */,
                                float target_position_rad /**< 轨迹目标关节角，单位：弧度 */)
{
    if ((control == NULL) || !control->initialized ||
        !isfinite(start_position_rad) || !isfinite(target_position_rad))
    {
        return false;
    }

    control->trajectory_start_ms = tick_ms;
    control->trajectory_start_position_rad = start_position_rad;
    control->trajectory_target_position_rad = target_position_rad;
    control->trajectory_duration_ms = J4310PositionControl_DurationMs(
        control, target_position_rad - start_position_rad);
    control->target_position_rad = (control->trajectory_duration_ms == 0U) ?
                                   target_position_rad : start_position_rad;
    control->target_velocity_rad_s = 0.0f;
    control->trajectory_active = control->trajectory_duration_ms != 0U;
    return true;
}

/* 功能：取消运动轨迹并保持指定位置；用途：在启动或模式切换时建立静止目标；无返回值表示保持目标已更新。 */
void J4310PositionControl_Hold(j4310_position_control_t *control /**< J4310 位置轨迹控制器 */,
                               float position_rad /**< 需要保持的关节目标位置，单位：弧度 */)
{
    if ((control == NULL) || !control->initialized ||
        !isfinite(position_rad))
    {
        return;
    }
    control->trajectory_active = false;
    control->trajectory_duration_ms = 0U;
    control->trajectory_start_position_rad = position_rad;
    control->trajectory_target_position_rad = position_rad;
    control->target_position_rad = position_rad;
    control->target_velocity_rad_s = 0.0f;
}

/* 功能：取消当前 J4310 位置轨迹；用途：停止轨迹推进并清除目标速度；无返回值表示轨迹已停用。 */
void J4310PositionControl_CancelTrajectory(
    j4310_position_control_t *control /**< J4310 位置轨迹控制器 */)
{
    if (control == NULL)
    {
        return;
    }
    control->trajectory_active = false;
    control->trajectory_duration_ms = 0U;
    control->target_velocity_rad_s = 0.0f;
}

/* 功能：按当前时刻采样 J4310 五次位置轨迹；用途：输出连续的位置和速度目标；无返回值表示采样结果已写入输出参数。 */
void J4310PositionControl_Sample(j4310_position_control_t *control /**< J4310 位置轨迹控制器 */,
                                 uint32_t tick_ms /**< 当前系统毫秒时刻 */,
                                 float *position_rad /**< 用于写出轨迹目标位置的地址，单位：弧度 */,
                                 float *velocity_rad_s /**< 用于写出轨迹目标速度的地址，单位：弧度每秒 */)
{
    uint32_t elapsed_ms;
    float normalized_time;
    float normalized_time_2;
    float normalized_time_3;
    float normalized_time_4;
    float normalized_time_5;
    float blend;
    float blend_rate;
    float distance_rad;

    if ((control == NULL) || !control->initialized)
    {
        return;
    }
    if (control->trajectory_active)
    {
        elapsed_ms = tick_ms - control->trajectory_start_ms;
        if (elapsed_ms >= control->trajectory_duration_ms)
        {
            control->trajectory_active = false;
            control->target_position_rad =
                control->trajectory_target_position_rad;
            control->target_velocity_rad_s = 0.0f;
        }
        else
        {
            normalized_time = (float)elapsed_ms /
                              (float)control->trajectory_duration_ms;
            normalized_time_2 = normalized_time * normalized_time;
            normalized_time_3 = normalized_time_2 * normalized_time;
            normalized_time_4 = normalized_time_3 * normalized_time;
            normalized_time_5 = normalized_time_4 * normalized_time;
            blend = 10.0f * normalized_time_3 -
                    15.0f * normalized_time_4 +
                    6.0f * normalized_time_5;
            blend_rate = (30.0f * normalized_time_2 -
                          60.0f * normalized_time_3 +
                          30.0f * normalized_time_4) *
                         (1000.0f /
                          (float)control->trajectory_duration_ms);
            distance_rad = control->trajectory_target_position_rad -
                           control->trajectory_start_position_rad;
            control->target_position_rad =
                control->trajectory_start_position_rad +
                distance_rad * blend;
            control->target_velocity_rad_s = distance_rad * blend_rate;
        }
    }
    if (position_rad != NULL)
    {
        *position_rad = control->target_position_rad;
    }
    if (velocity_rad_s != NULL)
    {
        *velocity_rad_s = control->target_velocity_rad_s;
    }
}

/* 功能：学习重力模型并合成最终 J4310 扭矩；用途：叠加请求扭矩、重力补偿和安全限幅；返回值表示最终扭矩命令。 */
float J4310PositionControl_ComposeTorque(
    j4310_position_control_t *control /**< J4310 位置轨迹控制器 */,
    bool feedback_fresh /**< 当前反馈是否仍在允许的超时时间内 */,
    uint32_t feedback_ms /**< 当前反馈对应的系统毫秒时刻 */,
    float actual_position_rad /**< J4310 当前实测关节角，单位：弧度 */,
    float actual_velocity_rad_s /**< J4310 当前实测关节速度，单位：弧度每秒 */,
    float feedback_torque_nm /**< J4310 当前反馈转矩，单位：牛米 */,
    float desired_position_rad /**< J4310 当前期望关节角，单位：弧度 */,
    float desired_velocity_rad_s /**< J4310 当前期望关节速度，单位：弧度每秒 */,
    float requested_torque_nm /**< 位置控制器请求输出的转矩，单位：牛米 */,
    float torque_limit_nm /**< 允许设置的转矩上限，单位：牛米 */)
{
    float cosine;
    float sine;
    float estimate_nm;
    float residual_nm;
    float model_limit_nm;
    float compensation_limit_nm;
    float gravity_position_rad;
    float gravity_enable_position_rad;
    float position_error_rad;
    float torque_step_nm;
    bool gravity_enabled;
    bool quasi_static;

    if ((control == NULL) || !control->initialized ||
        !isfinite(requested_torque_nm) || !isfinite(torque_limit_nm) ||
        (torque_limit_nm <= 0.0f))
    {
        return 0.0f;
    }

    model_limit_nm = (control->gravity_model_limit_nm < torque_limit_nm) ?
                     control->gravity_model_limit_nm : torque_limit_nm;
    if (isfinite(desired_position_rad) &&
        (!control->gravity_learning_target_valid ||
         (fabsf(desired_position_rad -
                control->gravity_learning_target_rad) >=
          (control->gravity_settle_error_rad * 0.5f))))
    {
        control->gravity_learning_target_valid = true;
        control->gravity_learning_target_rad = desired_position_rad;
        control->gravity_learning_locked = false;
        control->gravity_settle_feedback_count = 0U;
    }
    gravity_position_rad = desired_position_rad;
    gravity_enable_position_rad = desired_position_rad;
    if (feedback_fresh && isfinite(actual_position_rad))
    {
        gravity_enable_position_rad = actual_position_rad;
        control->gravity_actual_position_rad = actual_position_rad;
        if (!control->gravity_position_valid)
        {
            control->gravity_position_valid = true;
            control->gravity_reference_position_rad = actual_position_rad;
        }
        else if (fabsf(actual_position_rad -
                       control->gravity_reference_position_rad) >=
                 J4310_GRAVITY_POSITION_DEADBAND_RAD)
        {
            control->gravity_reference_position_rad = actual_position_rad;
        }
        gravity_position_rad = control->gravity_reference_position_rad;
    }
    else if (control->gravity_position_valid)
    {
        gravity_position_rad = control->gravity_reference_position_rad;
        gravity_enable_position_rad =
            control->gravity_actual_position_rad;
    }
    gravity_enabled = isfinite(gravity_enable_position_rad) &&
                      J4310PositionControl_GravityEnabled(
                          control, gravity_enable_position_rad);
    if (feedback_fresh && isfinite(actual_position_rad) &&
        isfinite(actual_velocity_rad_s) && isfinite(feedback_torque_nm) &&
        isfinite(desired_position_rad) &&
        isfinite(desired_velocity_rad_s) &&
        gravity_enabled &&
        (!control->feedback_seen ||
         (feedback_ms != control->last_feedback_ms)))
    {
        control->feedback_seen = true;
        control->last_feedback_ms = feedback_ms;
        cosine = cosf(actual_position_rad);
        sine = sinf(actual_position_rad);
        position_error_rad = fabsf(desired_position_rad -
                                   actual_position_rad);
        quasi_static =
            (fabsf(actual_velocity_rad_s) <=
             J4310_GRAVITY_LEARN_ACTUAL_VELOCITY_RAD_S) &&
            (fabsf(desired_velocity_rad_s) <=
             J4310_GRAVITY_LEARN_DESIRED_VELOCITY_RAD_S) &&
            (fabsf(requested_torque_nm) <=
             J4310_GRAVITY_LEARN_REQUESTED_TORQUE_NM) &&
            (fabsf(feedback_torque_nm) <= torque_limit_nm);
        if (quasi_static &&
            (position_error_rad <= control->gravity_settle_error_rad))
        {
            if (control->gravity_settle_feedback_count <
                control->gravity_settle_required_count)
            {
                control->gravity_settle_feedback_count++;
            }
        }
        else
        {
            control->gravity_settle_feedback_count = 0U;
        }
        estimate_nm = control->gravity_cos_nm * cosine +
                      control->gravity_sin_nm * sine;

    /* 在近似静态点，按电流换算的电机转矩就是关节重力负载。
     * 拟合 tau_g(q)=a*cos(q)+b*sin(q)，这是 J(q)^T*F_g 的单关节形式。 */
        if (!control->gravity_learning_locked && quasi_static &&
            (position_error_rad <=
             J4310_GRAVITY_LEARN_POSITION_ERROR_RAD))
        {
            residual_nm = feedback_torque_nm - estimate_nm;
            if (fabsf(residual_nm) >=
                J4310_GRAVITY_LEARN_RESIDUAL_DEADBAND_NM)
            {
                control->gravity_cos_nm = J4310PositionControl_Clamp(
                    control->gravity_cos_nm +
                    control->gravity_learning_rate * residual_nm * cosine,
                    -model_limit_nm,
                    model_limit_nm);
                control->gravity_sin_nm = J4310PositionControl_Clamp(
                    control->gravity_sin_nm +
                    control->gravity_learning_rate * residual_nm * sine,
                    -model_limit_nm,
                    model_limit_nm);
            }
        }
        if (control->gravity_settle_feedback_count >=
            control->gravity_settle_required_count)
        {
            control->gravity_learning_locked = true;
        }
    }

    if (!gravity_enabled)
    {
        estimate_nm = 0.0f;
        control->gravity_filtered_torque_nm = 0.0f;
    }
    else if (isfinite(gravity_position_rad))
    {
        estimate_nm = J4310PositionControl_GravityModel(
            control, gravity_position_rad) *
            control->gravity_compensation_gain;
    }
    else
    {
        estimate_nm = control->gravity_cos_nm *
                      control->gravity_compensation_gain;
    }
    if (gravity_enabled)
    {
        estimate_nm = J4310PositionControl_Clamp(
            estimate_nm, -model_limit_nm, model_limit_nm);
        torque_step_nm = control->gravity_torque_rate_limit_nm_s *
                         ((float)J4310_POSITION_CONTROL_PERIOD_MS /
                          1000.0f);
        control->gravity_filtered_torque_nm =
            J4310PositionControl_Clamp(
                J4310PositionControl_Approach(
                    control->gravity_filtered_torque_nm,
                    estimate_nm,
                    torque_step_nm),
                -model_limit_nm,
                model_limit_nm);
    }
    requested_torque_nm = J4310PositionControl_Clamp(
        requested_torque_nm, -torque_limit_nm, torque_limit_nm);
    compensation_limit_nm = torque_limit_nm - fabsf(requested_torque_nm);
    control->gravity_torque_nm = J4310PositionControl_Clamp(
        control->gravity_filtered_torque_nm,
        -compensation_limit_nm,
        compensation_limit_nm);
    return J4310PositionControl_Clamp(
        requested_torque_nm + control->gravity_torque_nm,
        -torque_limit_nm,
        torque_limit_nm);
}
