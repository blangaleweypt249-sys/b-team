/**
 * @file j4310_position_control.c
 * @brief 实现 J4310 位置轨迹、重力补偿学习和扭矩合成。
 */

#include "j4310_position_control.h"

#include <math.h>
#include <string.h>

#define J4310_POSITION_CONTROL_PERIOD_MS              1U
#define J4310_POSITION_CONTROL_DISTANCE_EPSILON_RAD   0.0001f
#define J4310_POSITION_CONTROL_QUINTIC_VELOCITY_BOUND 1.875f
#define J4310_POSITION_CONTROL_QUINTIC_ACCEL_BOUND    5.7736f
#define J4310_GRAVITY_LEARN_ACTUAL_VELOCITY_RAD_S     0.05f
#define J4310_GRAVITY_LEARN_DESIRED_VELOCITY_RAD_S    0.05f
#define J4310_GRAVITY_LEARN_POSITION_ERROR_RAD         1.00f
#define J4310_GRAVITY_LEARN_REQUESTED_TORQUE_NM        0.05f
#define J4310_GRAVITY_LEARN_RESIDUAL_DEADBAND_NM       0.05f
#define J4310_GRAVITY_POSITION_DEADBAND_RAD             0.034906585f
#define J4310_PI_RAD                                    3.14159265359f
#define J4310_TWO_PI_RAD                                6.28318530718f

/* 功能：将数值限制在给定上下界内；用途：约束轨迹和扭矩控制量；返回值表示限幅后的数值。 */
static float J4310PositionControl_Clamp(float value,
                                        float minimum,
                                        float maximum)
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
static float J4310PositionControl_Approach(float current,
                                           float target,
                                           float maximum_step)
{
    return current + J4310PositionControl_Clamp(
        target - current, -maximum_step, maximum_step);
}

/* 功能：根据关节角和正余弦系数计算重力模型扭矩；用途：形成位置相关的前馈补偿；返回值表示估算扭矩。 */
static float J4310PositionControl_GravityModel(
    const j4310_position_control_t *control,
    float position_rad)
{
    return control->gravity_cos_nm * cosf(position_rad) +
           control->gravity_sin_nm * sinf(position_rad);
}

/* 功能：将角度归一化到负 π 至正 π；用途：统一重力补偿的周期角度；返回值表示归一化后的角度。 */
static float J4310PositionControl_WrapToPi(float position_rad)
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
    const j4310_position_control_t *control,
    float position_rad)
{
    float wrapped_abs_rad = fabsf(
        J4310PositionControl_WrapToPi(position_rad));

    return (wrapped_abs_rad > control->gravity_disable_half_width_rad) &&
           ((J4310_PI_RAD - wrapped_abs_rad) >
            control->gravity_disable_half_width_rad);
}

/* 功能：按位移计算五次位置轨迹持续时间；用途：满足配置的速度和加速度限制；返回值表示轨迹时长（毫秒）。 */
static uint32_t J4310PositionControl_DurationMs(
    const j4310_position_control_t *control,
    float distance_rad)
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
bool J4310PositionControl_Init(j4310_position_control_t *control,
                               float max_velocity_rad_s,
                               float max_acceleration_rad_s2,
                               float gravity_model_limit_nm,
                               float gravity_learning_rate,
                               float gravity_compensation_gain,
                               float gravity_disable_half_width_rad,
                               float gravity_settle_error_rad,
                               uint32_t gravity_settle_required_count,
                               float gravity_torque_rate_limit_nm_s)
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
bool J4310PositionControl_Start(j4310_position_control_t *control,
                                uint32_t tick_ms,
                                float start_position_rad,
                                float target_position_rad)
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
void J4310PositionControl_Hold(j4310_position_control_t *control,
                               float position_rad)
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
    j4310_position_control_t *control)
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
void J4310PositionControl_Sample(j4310_position_control_t *control,
                                 uint32_t tick_ms,
                                 float *position_rad,
                                 float *velocity_rad_s)
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
    j4310_position_control_t *control,
    bool feedback_fresh,
    uint32_t feedback_ms,
    float actual_position_rad,
    float actual_velocity_rad_s,
    float feedback_torque_nm,
    float desired_position_rad,
    float desired_velocity_rad_s,
    float requested_torque_nm,
    float torque_limit_nm)
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
