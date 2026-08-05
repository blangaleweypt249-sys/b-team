#include "upper_motor_port.h"

#include <string.h>

#include "bsp_can.h"
#include "can_id.h"
#include "DJI/dji_group.h"
#include "DM J4310/j4310.h"
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
static bool j4310_enabled[UPPER_MOTOR_PORT_MAX_NODE_ID + 1U];
static uint32_t j4310_enabled_at_ms[UPPER_MOTOR_PORT_MAX_NODE_ID + 1U];
static bool dji_group_dirty[UPPER_CAN_BUS_COUNT][UPPER_DJI_GROUP_COUNT];
static bool upper_motor_active[MOTOR_MANAGER_MAX_COUNT];
static uint32_t upper_motor_active_since_ms[MOTOR_MANAGER_MAX_COUNT];

static bool UpperMotorPort_IsDjiModel(motor_model_t model)
{
    return (model == MOTOR_MODEL_M3508) ||
           (model == MOTOR_MODEL_M2006);
}

static bool UpperMotorPort_CheckCfg(const motor_cfg_t *cfg,
                                    size_t motor_count)
{
    size_t index;

    if ((cfg == NULL) || (motor_count == 0U) ||
        (motor_count > MOTOR_MANAGER_MAX_COUNT))
    {
        return false;
    }

    for (index = 0U; index < motor_count; index++)
    {
        size_t previous;

        if ((cfg[index].can_bus == 0U) ||
            (cfg[index].can_bus > UPPER_CAN_BUS_COUNT) ||
            (cfg[index].period_ms != UPPER_CONTROL_PERIOD_MS) ||
            (cfg[index].phase_ms != 0U) || !cfg[index].protocol_ready)
        {
            return false;
        }

        if (UpperMotorPort_IsDjiModel(cfg[index].model))
        {
            if ((cfg[index].node_id == 0U) || (cfg[index].node_id > 8U))
            {
                return false;
            }
        }
        else if (cfg[index].model == MOTOR_MODEL_J4310)
        {
            if ((cfg[index].node_id == 0U) ||
                (cfg[index].node_id > 0x0FU))
            {
                return false;
            }
        }
        else
        {
            return false;
        }

        for (previous = 0U; previous < index; previous++)
        {
            if ((cfg[previous].can_bus == cfg[index].can_bus) &&
                (cfg[previous].node_id == cfg[index].node_id) &&
                ((cfg[previous].model == cfg[index].model) ||
                 (UpperMotorPort_IsDjiModel(cfg[previous].model) &&
                  UpperMotorPort_IsDjiModel(cfg[index].model))))
            {
                return false;
            }
        }
    }

    return true;
}

static bool UpperMotorPort_SendFrame(uint8_t can_bus,
                                     const can_frame_t *frame)
{
    return BspCan_Send(can_bus, frame);
}

static size_t UpperMotorPort_FindCfg(const motor_cfg_t *cfg)
{
    size_t index;

    if ((cfg == NULL) || (upper_motor_cfg_ref == NULL))
    {
        return upper_motor_count;
    }
    for (index = 0U; index < upper_motor_count; index++)
    {
        if (&upper_motor_cfg_ref[index] == cfg)
        {
            return index;
        }
    }
    return upper_motor_count;
}

static bool UpperMotorPort_FeedbackFresh(uint32_t updated_at_ms,
                                         uint32_t tick_ms)
{
    return (tick_ms - updated_at_ms) <= UPPER_MOTOR_FEEDBACK_TIMEOUT_MS;
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

static bool UpperMotorPort_FlushDjiGroup(uint8_t can_bus,
                                         uint32_t group_index)
{
    can_frame_t frame;
    int16_t current_raw[DJI_GROUP_MOTOR_COUNT] = {0};
    uint8_t start_motor_id;
    size_t motor_index;
    bool group_valid;

    if ((can_bus == 0U) || (can_bus > UPPER_CAN_BUS_COUNT) ||
        (group_index >= UPPER_DJI_GROUP_COUNT))
    {
        return false;
    }
    if (!dji_group_dirty[can_bus - 1U][group_index])
    {
        return true;
    }

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
            if (!M3508_CalcCurrentRaw(can_bus,
                                      cfg->node_id,
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
        !DjiGroup_BuildFrame(start_motor_id, current_raw, &frame) ||
        !UpperMotorPort_SendFrame(can_bus, &frame))
    {
        return false;
    }
    dji_group_dirty[can_bus - 1U][group_index] = false;
    return true;
}

static bool UpperMotorPort_SendJ4310(const motor_cfg_t *cfg,
                                     const motor_cmd_t *cmd)
{
    can_frame_t command_frame;
    can_frame_t state_frame;
    j4310_feedback_t feedback;
    uint32_t group_index;
    bool feedback_fresh;

    for (group_index = 0U;
         group_index < UPPER_DJI_GROUP_COUNT;
         group_index++)
    {
        if (!UpperMotorPort_FlushDjiGroup(cfg->can_bus, group_index))
        {
            return false;
        }
    }

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

    feedback_fresh = J4310_GetFeedback(cfg->node_id, &feedback) &&
                     UpperMotorPort_FeedbackFresh(feedback.updated_at_ms,
                                                  upper_motor_tick_ms);
    if (feedback_fresh && (feedback.fault != 0U))
    {
        if (!j4310_enabled[cfg->node_id])
        {
            return true;
        }
        if (J4310_BuildDisable(cfg->node_id, &state_frame) &&
            UpperMotorPort_SendFrame(cfg->can_bus, &state_frame))
        {
            j4310_enabled[cfg->node_id] = false;
            return true;
        }
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
        j4310_enabled_at_ms[cfg->node_id] = upper_motor_tick_ms;
        return true;
    }

    if (!feedback_fresh &&
        ((upper_motor_tick_ms - j4310_enabled_at_ms[cfg->node_id]) >
         UPPER_MOTOR_FEEDBACK_TIMEOUT_MS))
    {
        if (J4310_BuildDisable(cfg->node_id, &state_frame) &&
            UpperMotorPort_SendFrame(cfg->can_bus, &state_frame))
        {
            j4310_enabled[cfg->node_id] = false;
            return true;
        }
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

    if (!M3508_SetTarget(cfg->can_bus,
                         cfg->node_id,
                         mode,
                         target,
                         upper_motor_tick_ms))
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
    if (!UpperMotorPort_CheckCfg(cfg, motor_count))
    {
        return false;
    }

    (void)memset(j4310_enabled, 0, sizeof(j4310_enabled));
    (void)memset(j4310_enabled_at_ms, 0, sizeof(j4310_enabled_at_ms));
    (void)memset(dji_group_dirty, 0, sizeof(dji_group_dirty));
    (void)memset(upper_motor_active, 0, sizeof(upper_motor_active));
    (void)memset(upper_motor_active_since_ms,
                 0,
                 sizeof(upper_motor_active_since_ms));
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
            (!J4310_AddMotor(cfg[index].node_id,
                             CAN_J4310_MASTER_ID,
                             cfg[index].node_id & 0x0FU,
                             &j4310_limits) ||
             !J4310_SetOnlineMitEnabled(cfg[index].node_id, true)))
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
    size_t motor_index;
    bool active;

    (void)user_data;
    if (!upper_motor_initialized || (cfg == NULL) || (cmd == NULL))
    {
        return false;
    }

    motor_index = UpperMotorPort_FindCfg(cfg);
    if (motor_index >= upper_motor_count)
    {
        return false;
    }
    active = cmd->mode != MOTOR_CMD_STOP;
    if (active && !upper_motor_active[motor_index])
    {
        upper_motor_active_since_ms[motor_index] = upper_motor_tick_ms;
    }
    upper_motor_active[motor_index] = active;

    switch (cfg->model)
    {
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
            if (!UpperMotorPort_FlushDjiGroup(
                    (uint8_t)(bus_index + 1U), group_index))
            {
                success = false;
            }
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
        case MOTOR_MODEL_J4310:
            if (J4310_OnFrame(frame, tick_ms))
            {
                return;
            }
            break;

        case MOTOR_MODEL_M3508:
            if (M3508_OnFrame(can_bus,
                              cfg->node_id,
                              frame,
                              tick_ms))
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

bool UpperMotorPort_GetHealth(uint32_t tick_ms,
                              upper_motor_health_t *health)
{
    size_t index;

    if (!upper_motor_initialized || (health == NULL))
    {
        return false;
    }

    (void)memset(health, 0, sizeof(*health));
    for (index = 0U; index < upper_motor_count; index++)
    {
        const motor_cfg_t *cfg;
        uint32_t mask;
        bool feedback_valid;
        uint32_t updated_at_ms;
        bool faulted;

        if (!upper_motor_active[index])
        {
            continue;
        }
        cfg = &upper_motor_cfg_ref[index];
        mask = 1UL << index;
        health->active_mask |= mask;
        if (!cfg->protocol_ready)
        {
            health->protocol_block_mask |= mask;
            continue;
        }

        feedback_valid = false;
        updated_at_ms = 0U;
        faulted = false;
        switch (cfg->model)
        {
        case MOTOR_MODEL_J4310:
        {
            j4310_feedback_t feedback;

            feedback_valid = J4310_GetFeedback(cfg->node_id, &feedback);
            updated_at_ms = feedback.updated_at_ms;
            faulted = feedback.fault != 0U;
            break;
        }

        case MOTOR_MODEL_M3508:
        {
            m3508_feedback_t feedback;

            feedback_valid = M3508_GetFeedback(cfg->can_bus,
                                               cfg->node_id,
                                               &feedback);
            updated_at_ms = feedback.updated_at_ms;
            break;
        }

        case MOTOR_MODEL_M2006:
        {
            m2006_feedback_t feedback;

            feedback_valid = M2006_GetFeedback(cfg->can_bus,
                                               cfg->node_id,
                                               &feedback);
            updated_at_ms = feedback.updated_at_ms;
            break;
        }

        default:
            health->protocol_block_mask |= mask;
            continue;
        }

        if (faulted)
        {
            health->fault_mask |= mask;
        }
        if ((!feedback_valid &&
             ((tick_ms - upper_motor_active_since_ms[index]) >
              UPPER_MOTOR_FEEDBACK_TIMEOUT_MS)) ||
            (feedback_valid &&
             !UpperMotorPort_FeedbackFresh(updated_at_ms, tick_ms)))
        {
            health->offline_mask |= mask;
        }
    }
    return true;
}
