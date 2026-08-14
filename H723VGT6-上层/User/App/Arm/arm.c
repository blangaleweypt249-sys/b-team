#include "arm.h"

#include <math.h>
#include <string.h>

#include "can_id.h"
#include "j4310.h"
#include "m3508.h"
#include "upper_config.h"

/* 功能：检查数值是否有限且位于正负限制内；用途：校验机械臂目标；返回 true 表示输入安全有效。 */
static bool Arm_ValueWithin(float value, float limit)
{
    return isfinite(value) && (value >= -limit) && (value <= limit);
}

/* 功能：检查数值是否为有限浮点数；用途：过滤机械臂命令中的异常值；返回 true 表示数值有效。 */
static bool Arm_ValueFinite(float value)
{
    return isfinite(value);
}

/* 功能：校验机械臂 PID 参数及上下限；用途：防止非法参数写入电机驱动；返回 true 表示配置可用。 */
static bool Arm_PidValid(const upper_pid_cfg_t *cfg)
{
    return (cfg != NULL) && isfinite(cfg->kp) && isfinite(cfg->ki) &&
           isfinite(cfg->kd) && isfinite(cfg->integral_limit) &&
           isfinite(cfg->output_limit) && (cfg->kp >= 0.0f) &&
           (cfg->ki >= 0.0f) && (cfg->kd >= 0.0f) &&
           (cfg->integral_limit >= 0.0f) && (cfg->output_limit > 0.0f);
}

/* 功能：比较两组机械臂 PID 配置是否完全相同；用途：避免重复下发未变化的参数；返回 true 表示一致。 */
static bool Arm_PidEqual(const upper_pid_cfg_t *left,
                         const upper_pid_cfg_t *right)
{
    return memcmp(left, right, sizeof(*left)) == 0;
}

/* 功能：校验机械臂目标并转换为各电机命令；用途：生成单周期可统一下发的输出快照；返回 true 表示目标合法且转换完成。 */
bool Arm_Calc(const arm_target_t *target, arm_output_t *output)
{
    uint32_t index;
    bool position_mode;

    if ((target == NULL) || (output == NULL))
    {
        return false;
    }
    position_mode = target->position_mode;
    if (!Arm_ValueWithin(target->grip_pos_rad,
                         UPPER_J4310_POSITION_MAX_RAD) ||
        !Arm_ValueWithin(target->grip_vel_rad_s,
                         UPPER_J4310_VELOCITY_MAX_RAD_S) ||
        !isfinite(target->grip_kp) || (target->grip_kp < 0.0f) ||
        (target->grip_kp > UPPER_J4310_KP_MAX) ||
        !isfinite(target->grip_kd) || (target->grip_kd < 0.0f) ||
        (target->grip_kd > UPPER_J4310_KD_MAX))
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
    for (index = 0U; index < UPPER_ARM_M3508_COUNT; index++)
    {
        if (position_mode)
        {
            if (!Arm_ValueFinite(target->m3508_pos_rad[index]))
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
        .mode = MOTOR_CMD_MIT,
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
            .mode = position_mode ? MOTOR_CMD_POSITION : MOTOR_CMD_VELOCITY,
            .pos_rad = position_mode ? target->m3508_pos_rad[index] : 0.0f,
            .vel_rad_s = target->m3508_vel_rad_s[index]
        };
    }
    return true;
}

/* 功能：应用机械臂输出及可选 PID 更新；用途：把 J4310 和 M3508 命令提交给电机管理器；返回 true 表示全部设置成功。 */
bool Arm_Apply(motor_manager_t *manager, const arm_output_t *output)
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

    /* Stage every arm target before the manager's single-cycle CAN dispatch.
     * This keeps J4310 and both M3508 targets from being treated as separate
     * control requests even though their CAN frames use different buses. */
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
