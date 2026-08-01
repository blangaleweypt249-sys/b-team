#include "upper_motor_port.h"

#include <string.h>

#include "bsp_can.h"
#include "can_id.h"
#include "DJI/dji_group.h"
#include "DM J4310/j4310.h"
#include "LK MG5010E-i36/mg5010.h"
#include "M2006/m2006.h"
#include "M3508/m3508.h"
#include "upper_config.h"

#define UPPER_MOTOR_PORT_MAX_NODE_ID  255U
#define UPPER_CAN_BUS_COUNT            3U
#define UPPER_DJI_GROUP_COUNT         2U

static const motor_cfg_t *upper_motor_cfg_ref;
static size_t upper_motor_count;
static uint32_t upper_motor_tick_ms;
static bool upper_motor_initialized;
static bool mg5010_enabled[MG5010_MOTOR_ID_MAX + 1U];
static bool j4310_enabled[UPPER_MOTOR_PORT_MAX_NODE_ID + 1U];
static bool dji_group_dirty[UPPER_CAN_BUS_COUNT][UPPER_DJI_GROUP_COUNT];

static float UpperMotorPort_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float UpperMotorPort_Clamp(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

static bool UpperMotorPort_SendFrame(uint8_t can_bus,
                                     const can_frame_t *frame)
{
    return BspCan_Send(can_bus, frame);
}

static bool UpperMotorPort_MarkDjiGroup(uint8_t can_bus, uint8_t node_id)
{
    uint32_t group_index;

    if ((can_bus == 0U) || (can_bus > UPPER_CAN_BUS_COUNT) ||
        (node_id == 0U) || (node_id > 8U))
    {
        return false;
    }

    group_index = (uint32_t)(node_id - 1U) / DJI_GROUP_MOTOR_COUNT;
    dji_group_dirty[can_bus - 1U][group_index] = true;
    return true;
}

static bool UpperMotorPort_SendMg5010(const motor_cfg_t *cfg,
                                      const motor_cmd_t *cmd)
{
    can_frame_t command_frame;
    can_frame_t state_frame;
    bool built;

    if ((cfg->node_id < MG5010_MOTOR_ID_MIN) ||
        (cfg->node_id > MG5010_MOTOR_ID_MAX))
    {
        return false;
    }

    if (cmd->mode == MOTOR_CMD_STOP)
    {
        built = Mg5010_BuildStop(cfg->node_id, &state_frame);
        if (built && UpperMotorPort_SendFrame(cfg->can_bus, &state_frame))
        {
            mg5010_enabled[cfg->node_id] = false;
            return true;
        }
        return false;
    }

    switch (cmd->mode)
    {
    case MOTOR_CMD_CURRENT:
        built = Mg5010_BuildCurrent(
            cfg->node_id,
            UpperMotorPort_Clamp(cmd->current_a,
                                 -UPPER_MG5010_CURRENT_LIMIT_A,
                                 UPPER_MG5010_CURRENT_LIMIT_A),
            &command_frame);
        break;

    case MOTOR_CMD_VELOCITY:
        built = Mg5010_BuildVelocity(
            cfg->node_id,
            cmd->vel_rad_s,
            UPPER_MG5010_CURRENT_LIMIT_A,
            &command_frame);
        break;

    case MOTOR_CMD_POSITION:
        if (!Mg5010_PositionReady(cfg->node_id))
        {
            return Mg5010_BuildReadPosition(cfg->node_id, &state_frame) &&
                   UpperMotorPort_SendFrame(cfg->can_bus, &state_frame);
        }
        built = Mg5010_BuildPosition(cfg->node_id,
                                     cmd->pos_rad,
                                     UpperMotorPort_Abs(cmd->vel_rad_s),
                                     &command_frame);
        break;

    default:
        built = false;
        break;
    }

    if (!built)
    {
        return false;
    }
    if (!mg5010_enabled[cfg->node_id])
    {
        if (!Mg5010_BuildRun(cfg->node_id, &state_frame) ||
            !UpperMotorPort_SendFrame(cfg->can_bus, &state_frame))
        {
            return false;
        }
        mg5010_enabled[cfg->node_id] = true;
    }
    return UpperMotorPort_SendFrame(cfg->can_bus, &command_frame);
}

static bool UpperMotorPort_SendJ4310(const motor_cfg_t *cfg,
                                     const motor_cmd_t *cmd)
{
    can_frame_t command_frame;
    can_frame_t state_frame;

    if (cmd->mode == MOTOR_CMD_STOP)
    {
        if (J4310_BuildDisable(cfg->node_id, &state_frame) &&
            UpperMotorPort_SendFrame(cfg->can_bus, &state_frame))
        {
            j4310_enabled[cfg->node_id] = false;
            return true;
        }
        return false;
    }

    if (cmd->mode != MOTOR_CMD_MIT)
    {
        return false;
    }

    if (!J4310_BuildMit(cfg->node_id,
                        cmd->pos_rad,
                        cmd->vel_rad_s,
                        cmd->kp,
                        cmd->kd,
                        cmd->torque_nm,
                        &command_frame))
    {
        return false;
    }

    if (!j4310_enabled[cfg->node_id])
    {
        if (!J4310_BuildEnable(cfg->node_id, &state_frame) ||
            !UpperMotorPort_SendFrame(cfg->can_bus, &state_frame))
        {
            return false;
        }
        j4310_enabled[cfg->node_id] = true;
    }

    return UpperMotorPort_SendFrame(cfg->can_bus, &command_frame);
}

static bool UpperMotorPort_SendM3508(const motor_cfg_t *cfg,
                                     const motor_cmd_t *cmd)
{
    m3508_mode_t mode;
    float target;

    switch (cmd->mode)
    {
    case MOTOR_CMD_STOP:
        mode = M3508_MODE_STOP;
        target = 0.0f;
        break;

    case MOTOR_CMD_CURRENT:
        mode = M3508_MODE_CURRENT;
        target = cmd->current_a;
        break;

    case MOTOR_CMD_VELOCITY:
        mode = M3508_MODE_VELOCITY;
        target = cmd->vel_rad_s;
        break;

    case MOTOR_CMD_POSITION:
        mode = M3508_MODE_POSITION;
        target = cmd->pos_rad;
        break;

    default:
        return false;
    }

    if (!M3508_SetTarget(cfg->node_id, mode, target, upper_motor_tick_ms))
    {
        return false;
    }
    return UpperMotorPort_MarkDjiGroup(cfg->can_bus, cfg->node_id);
}

static bool UpperMotorPort_SendM2006(const motor_cfg_t *cfg,
                                     const motor_cmd_t *cmd)
{
    m2006_mode_t mode;
    float target;

    switch (cmd->mode)
    {
    case MOTOR_CMD_STOP:
        mode = M2006_MODE_STOP;
        target = 0.0f;
        break;

    case MOTOR_CMD_CURRENT:
        mode = M2006_MODE_CURRENT;
        target = cmd->current_a;
        break;

    case MOTOR_CMD_VELOCITY:
        mode = M2006_MODE_VELOCITY;
        target = cmd->vel_rad_s;
        break;

    case MOTOR_CMD_POSITION:
        mode = M2006_MODE_POSITION;
        target = cmd->pos_rad;
        break;

    default:
        return false;
    }

    if (!M2006_SetTarget(cfg->can_bus,
                         cfg->node_id,
                         mode,
                         target,
                         upper_motor_tick_ms))
    {
        return false;
    }
    return UpperMotorPort_MarkDjiGroup(cfg->can_bus, cfg->node_id);
}

bool UpperMotorPort_Init(const motor_cfg_t *cfg, size_t motor_count)
{
    j4310_limits_t j4310_limits;
    m2006_cfg_t m2006_driver_cfg;
    m3508_cfg_t m3508_driver_cfg;
    size_t index;

    upper_motor_initialized = false;
    upper_motor_cfg_ref = NULL;
    upper_motor_count = 0U;
    if ((cfg == NULL) || (motor_count == 0U))
    {
        return false;
    }

    (void)memset(mg5010_enabled, 0, sizeof(mg5010_enabled));
    (void)memset(j4310_enabled, 0, sizeof(j4310_enabled));
    (void)memset(dji_group_dirty, 0, sizeof(dji_group_dirty));
    Mg5010_Init();
    J4310_Init();

    j4310_limits = (j4310_limits_t)
    {
        UPPER_J4310_POSITION_MAX_RAD,
        UPPER_J4310_VELOCITY_MAX_RAD_S,
        UPPER_J4310_TORQUE_MAP_MAX_NM
    };
    for (index = 0U; index < motor_count; index++)
    {
        if ((cfg[index].model == MOTOR_MODEL_J4310) &&
            !J4310_AddMotor(cfg[index].node_id,
                            CAN_J4310_MASTER_ID,
                            cfg[index].node_id & 0x0FU,
                            &j4310_limits))
        {
            return false;
        }
    }

    m3508_driver_cfg = (m3508_cfg_t)
    {
        .current_limit_a = UPPER_M3508_CURRENT_LIMIT_A,
        .position_vel_limit_rad_s = UPPER_M3508_POSITION_VEL_LIMIT_RAD_S,
        .feedback_timeout_ms = UPPER_MOTOR_FEEDBACK_TIMEOUT_MS,
        .command_timeout_ms = UPPER_PC_TIMEOUT_MS,
        .speed_pid =
        {
            UPPER_M3508_SPEED_KP,
            UPPER_M3508_SPEED_KI,
            UPPER_M3508_SPEED_KD,
            UPPER_M3508_SPEED_I_LIMIT,
            UPPER_M3508_CURRENT_LIMIT_A
        },
        .position_pid =
        {
            UPPER_M3508_POSITION_KP,
            UPPER_M3508_POSITION_KI,
            UPPER_M3508_POSITION_KD,
            UPPER_M3508_POSITION_I_LIMIT,
            UPPER_M3508_POSITION_VEL_LIMIT_RAD_S
        }
    };
    if (!M3508_Init(&m3508_driver_cfg))
    {
        return false;
    }

    m2006_driver_cfg = (m2006_cfg_t)
    {
        .current_limit_a = UPPER_M2006_CURRENT_LIMIT_A,
        .position_vel_limit_rad_s = UPPER_M2006_POSITION_VEL_LIMIT_RAD_S,
        .feedback_timeout_ms = UPPER_MOTOR_FEEDBACK_TIMEOUT_MS,
        .command_timeout_ms = UPPER_PC_TIMEOUT_MS,
        .speed_pid =
        {
            UPPER_M2006_SPEED_KP,
            UPPER_M2006_SPEED_KI,
            UPPER_M2006_SPEED_KD,
            UPPER_M2006_SPEED_I_LIMIT,
            UPPER_M2006_CURRENT_LIMIT_A
        },
        .position_pid =
        {
            UPPER_M2006_POSITION_KP,
            UPPER_M2006_POSITION_KI,
            UPPER_M2006_POSITION_KD,
            UPPER_M2006_POSITION_I_LIMIT,
            UPPER_M2006_POSITION_VEL_LIMIT_RAD_S
        }
    };
    if (!M2006_Init(&m2006_driver_cfg))
    {
        return false;
    }

    upper_motor_cfg_ref = cfg;
    upper_motor_count = motor_count;
    upper_motor_tick_ms = 0U;
    upper_motor_initialized = true;
    return true;
}

void UpperMotorPort_BeginCycle(uint32_t tick_ms)
{
    upper_motor_tick_ms = tick_ms;
}

bool UpperMotorPort_Send(const motor_cfg_t *cfg,
                         const motor_cmd_t *cmd,
                         void *user_data)
{
    (void)user_data;
    if (!upper_motor_initialized || (cfg == NULL) || (cmd == NULL))
    {
        return false;
    }

    switch (cfg->model)
    {
    case MOTOR_MODEL_MG5010:
        return UpperMotorPort_SendMg5010(cfg, cmd);

    case MOTOR_MODEL_J4310:
        return UpperMotorPort_SendJ4310(cfg, cmd);

    case MOTOR_MODEL_M3508:
        return UpperMotorPort_SendM3508(cfg, cmd);

    case MOTOR_MODEL_M2006:
        return UpperMotorPort_SendM2006(cfg, cmd);

    default:
        return false;
    }
}

bool UpperMotorPort_Flush(void)
{
    uint32_t bus_index;
    uint32_t group_index;
    bool success;

    success = true;
    for (bus_index = 0U; bus_index < UPPER_CAN_BUS_COUNT; bus_index++)
    {
        for (group_index = 0U;
             group_index < UPPER_DJI_GROUP_COUNT;
             group_index++)
        {
            can_frame_t frame;
            int16_t current_raw[DJI_GROUP_MOTOR_COUNT] = {0};
            uint8_t can_bus;
            uint8_t start_motor_id;
            size_t motor_index;
            bool group_valid;

            if (!dji_group_dirty[bus_index][group_index])
            {
                continue;
            }

            can_bus = (uint8_t)(bus_index + 1U);
            start_motor_id =
                (uint8_t)(group_index * DJI_GROUP_MOTOR_COUNT + 1U);
            group_valid = true;
            for (motor_index = 0U;
                 motor_index < upper_motor_count;
                 motor_index++)
            {
                const motor_cfg_t *cfg;
                uint32_t slot;

                cfg = &upper_motor_cfg_ref[motor_index];
                if ((cfg->can_bus != can_bus) ||
                    (cfg->node_id < start_motor_id) ||
                    (cfg->node_id >=
                     (uint8_t)(start_motor_id + DJI_GROUP_MOTOR_COUNT)))
                {
                    continue;
                }

                slot = (uint32_t)(cfg->node_id - start_motor_id);
                if (cfg->model == MOTOR_MODEL_M3508)
                {
                    if (!M3508_CalcCurrentRaw(cfg->node_id,
                                              upper_motor_tick_ms,
                                              &current_raw[slot]))
                    {
                        group_valid = false;
                    }
                }
                else if (cfg->model == MOTOR_MODEL_M2006)
                {
                    if (!M2006_CalcCurrentRaw(can_bus,
                                              cfg->node_id,
                                              upper_motor_tick_ms,
                                              &current_raw[slot]))
                    {
                        group_valid = false;
                    }
                }
            }

            if (!group_valid ||
                !DjiGroup_BuildFrame(start_motor_id,
                                     current_raw,
                                     &frame) ||
                !UpperMotorPort_SendFrame(can_bus, &frame))
            {
                success = false;
                continue;
            }
            dji_group_dirty[bus_index][group_index] = false;
        }
    }
    return success;
}

void UpperMotorPort_OnFrame(uint8_t can_bus,
                            const can_frame_t *frame,
                            uint32_t tick_ms)
{
    size_t index;

    if (!upper_motor_initialized || (frame == NULL))
    {
        return;
    }

    for (index = 0U; index < upper_motor_count; index++)
    {
        const motor_cfg_t *cfg;

        cfg = &upper_motor_cfg_ref[index];
        if (cfg->can_bus != can_bus)
        {
            continue;
        }

        switch (cfg->model)
        {
        case MOTOR_MODEL_MG5010:
            if (Mg5010_OnFrame(cfg->node_id, frame, tick_ms))
            {
                return;
            }
            break;

        case MOTOR_MODEL_J4310:
            if (J4310_OnFrame(frame, tick_ms))
            {
                return;
            }
            break;

        case MOTOR_MODEL_M3508:
            if (M3508_OnFrame(cfg->node_id, frame, tick_ms))
            {
                return;
            }
            break;

        case MOTOR_MODEL_M2006:
            if (M2006_OnFrame(can_bus, cfg->node_id, frame, tick_ms))
            {
                return;
            }
            break;

        default:
            break;
        }
    }
}
