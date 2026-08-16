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

static float J4310PositionControl_GravityModel(
    const j4310_position_control_t *control,
    float position_rad)
{
    return control->gravity_cos_nm * cosf(position_rad) +
           control->gravity_sin_nm * sinf(position_rad);
}

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

bool J4310PositionControl_Init(j4310_position_control_t *control,
                               float max_velocity_rad_s,
                               float max_acceleration_rad_s2,
                               float gravity_model_limit_nm,
                               float gravity_learning_rate)
{
    if ((control == NULL) || !isfinite(max_velocity_rad_s) ||
        !isfinite(max_acceleration_rad_s2) ||
        !isfinite(gravity_model_limit_nm) ||
        !isfinite(gravity_learning_rate) ||
        (max_velocity_rad_s <= 0.0f) ||
        (max_acceleration_rad_s2 <= 0.0f) ||
        (gravity_model_limit_nm <= 0.0f) ||
        (gravity_learning_rate <= 0.0f) ||
        (gravity_learning_rate > 1.0f))
    {
        return false;
    }

    (void)memset(control, 0, sizeof(*control));
    control->max_velocity_rad_s = max_velocity_rad_s;
    control->max_acceleration_rad_s2 = max_acceleration_rad_s2;
    control->gravity_model_limit_nm = gravity_model_limit_nm;
    control->gravity_learning_rate = gravity_learning_rate;
    control->initialized = true;
    return true;
}

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

    if ((control == NULL) || !control->initialized ||
        !isfinite(requested_torque_nm) || !isfinite(torque_limit_nm) ||
        (torque_limit_nm <= 0.0f))
    {
        return 0.0f;
    }

    model_limit_nm = (control->gravity_model_limit_nm < torque_limit_nm) ?
                     control->gravity_model_limit_nm : torque_limit_nm;
    gravity_position_rad = desired_position_rad;
    if (feedback_fresh && isfinite(actual_position_rad))
    {
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
    }
    if (feedback_fresh && isfinite(actual_position_rad) &&
        isfinite(actual_velocity_rad_s) && isfinite(feedback_torque_nm) &&
        isfinite(desired_position_rad) &&
        isfinite(desired_velocity_rad_s) &&
        (!control->feedback_seen ||
         (feedback_ms != control->last_feedback_ms)))
    {
        control->feedback_seen = true;
        control->last_feedback_ms = feedback_ms;
        cosine = cosf(actual_position_rad);
        sine = sinf(actual_position_rad);
        estimate_nm = control->gravity_cos_nm * cosine +
                      control->gravity_sin_nm * sine;

        /* At quasi-static points the current-derived motor torque is the
         * joint gravity load. Fit tau_g(q)=a*cos(q)+b*sin(q), which is the
         * one-joint form of J(q)^T*F_g. */
        if ((fabsf(actual_velocity_rad_s) <=
             J4310_GRAVITY_LEARN_ACTUAL_VELOCITY_RAD_S) &&
            (fabsf(desired_velocity_rad_s) <=
             J4310_GRAVITY_LEARN_DESIRED_VELOCITY_RAD_S) &&
            (fabsf(desired_position_rad - actual_position_rad) <=
             J4310_GRAVITY_LEARN_POSITION_ERROR_RAD) &&
            (fabsf(requested_torque_nm) <=
             J4310_GRAVITY_LEARN_REQUESTED_TORQUE_NM) &&
            (fabsf(feedback_torque_nm) <= torque_limit_nm))
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
    }

    if (isfinite(gravity_position_rad))
    {
        estimate_nm = J4310PositionControl_GravityModel(
            control, gravity_position_rad);
    }
    else
    {
        estimate_nm = control->gravity_cos_nm;
    }
    control->gravity_torque_nm = J4310PositionControl_Clamp(
        estimate_nm,
        -model_limit_nm,
        model_limit_nm);
    requested_torque_nm = J4310PositionControl_Clamp(
        requested_torque_nm, -torque_limit_nm, torque_limit_nm);
    compensation_limit_nm = torque_limit_nm - fabsf(requested_torque_nm);
    control->gravity_torque_nm = J4310PositionControl_Clamp(
        control->gravity_torque_nm,
        -compensation_limit_nm,
        compensation_limit_nm);
    return J4310PositionControl_Clamp(
        requested_torque_nm + control->gravity_torque_nm,
        -torque_limit_nm,
        torque_limit_nm);
}
