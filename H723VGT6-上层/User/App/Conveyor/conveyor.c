#include "conveyor.h"

#include <math.h>
#include <string.h>

#include "can_id.h"
#include "m2006.h"
#include "upper_config.h"

static bool Conveyor_PidValid(const upper_pid_cfg_t *cfg)
{
    return (cfg != NULL) && isfinite(cfg->kp) && isfinite(cfg->ki) &&
           isfinite(cfg->kd) && isfinite(cfg->integral_limit) &&
           isfinite(cfg->output_limit) && (cfg->kp >= 0.0f) &&
           (cfg->ki >= 0.0f) && (cfg->kd >= 0.0f) &&
           (cfg->integral_limit >= 0.0f) && (cfg->output_limit > 0.0f);
}

static bool Conveyor_PidEqual(const upper_pid_cfg_t *left,
                              const upper_pid_cfg_t *right)
{
    return memcmp(left, right, sizeof(*left)) == 0;
}

bool Conveyor_Calc(const conveyor_target_t *target,
                   conveyor_output_t *output)
{
    if ((target == NULL) || (output == NULL))
    {
        return false;
    }

    if (target->position_mode)
    {
        if (!isfinite(target->m2006_pos_rad))
        {
            return false;
        }
    }
    else if (!isfinite(target->m2006_vel_rad_s) ||
             (target->m2006_vel_rad_s <
              -UPPER_M2006_POSITION_VEL_LIMIT_RAD_S) ||
             (target->m2006_vel_rad_s >
              UPPER_M2006_POSITION_VEL_LIMIT_RAD_S))
    {
        return false;
    }
    if (target->pid_update &&
        (!Conveyor_PidValid(&target->m2006_speed_pid) ||
         !Conveyor_PidValid(&target->m2006_position_pid)))
    {
        return false;
    }

    output->enabled = target->enabled;
    output->pid_update = target->pid_update;
    output->m2006_speed_pid = target->m2006_speed_pid;
    output->m2006_position_pid = target->m2006_position_pid;
    output->m2006 = (motor_cmd_t)
    {
        .mode = target->position_mode ? MOTOR_CMD_POSITION : MOTOR_CMD_VELOCITY,
        .pos_rad = target->position_mode ? target->m2006_pos_rad : 0.0f,
        .vel_rad_s = target->m2006_vel_rad_s
    };
    return true;
}

bool Conveyor_Apply(motor_manager_t *manager,
                    const conveyor_output_t *output)
{
    bool success;
    static bool pid_applied;
    static upper_pid_cfg_t applied_speed_pid;
    static upper_pid_cfg_t applied_position_pid;

    if ((manager == NULL) || (output == NULL))
    {
        return false;
    }
    if (output->pid_update &&
        (!pid_applied ||
         !Conveyor_PidEqual(&applied_speed_pid, &output->m2006_speed_pid) ||
         !Conveyor_PidEqual(&applied_position_pid, &output->m2006_position_pid)))
    {
        m2006_pid_cfg_t speed_pid;
        m2006_pid_cfg_t position_pid;

        speed_pid = (m2006_pid_cfg_t)
        {
            output->m2006_speed_pid.kp,
            output->m2006_speed_pid.ki,
            output->m2006_speed_pid.kd,
            output->m2006_speed_pid.integral_limit,
            output->m2006_speed_pid.output_limit
        };
        position_pid = (m2006_pid_cfg_t)
        {
            output->m2006_position_pid.kp,
            output->m2006_position_pid.ki,
            output->m2006_position_pid.kd,
            output->m2006_position_pid.integral_limit,
            output->m2006_position_pid.output_limit
        };
        success = M2006_SetSpeedPid(CAN_BUS_AUX,
                                    NODE_CONVEYOR_M2006,
                                    &speed_pid);
        success = M2006_SetPositionPid(CAN_BUS_AUX,
                                       NODE_CONVEYOR_M2006,
                                       &position_pid) && success;
        if (!success)
        {
            return false;
        }
        applied_speed_pid = output->m2006_speed_pid;
        applied_position_pid = output->m2006_position_pid;
        pid_applied = true;
    }
    success = MotorManager_SetCmd(manager,
                                  UPPER_MOTOR_CONVEYOR_M2006,
                                  &output->m2006);
    success = MotorManager_SetEnabled(manager,
                                      UPPER_MOTOR_CONVEYOR_M2006,
                                      output->enabled) && success;
    return success;
}
