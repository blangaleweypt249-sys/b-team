#include "up_main.h"

#include "fdcan_task.h"

#define ARRAY_SIZE(array) ((uint8_t)(sizeof(array) / sizeof((array)[0])))

#define RS_HOST_ID          0xFDU
#define DM_MOTOR_L_RX_ID    0x15U
#define DM_MOTOR_F_RX_ID    0x17U

static const rs_app_motor_config_t rs_motor_config[] = {
    {
        .id = RS_MOTOR_L_ID,
        .period_ms = RS_APP_CONTROL_PERIOD_MS,
        .command = {
            .mode = RS_CSP,
            .data.csp = {
                .angle_deg = 0.0f,
                .max_speed_rad_s = 9.5f
            }
        }
    },
    {
        .id = RS_MOTOR_F_ID,
        .period_ms = RS_APP_CONTROL_PERIOD_MS,
        .command = {
            .mode = RS_CSP,
            .data.csp = {
                .angle_deg = 0.0f,
                .max_speed_rad_s = 9.5f
            }
        }
    }
};

static const dm_app_motor_config_t dm_motor_config[] = {
    {
        .tx_id = DM_MOTOR_L_ID,
        .master_id = DM_MOTOR_L_RX_ID,
        .feedback_id = DM_MOTOR_L_ID,
        .command = {
            .angle_deg = 0.0f,
            .speed_rad_s = 0.2f,
            .torque_nm = 0.0f,
            .kp = 70.0f,
            .kd = 1.25f
        }
    },
    {
        .tx_id = DM_MOTOR_F_ID,
        .master_id = DM_MOTOR_F_RX_ID,
        .feedback_id = DM_MOTOR_F_ID,
        .command = {
            .angle_deg = 0.0f,
            .speed_rad_s = 0.2f,
            .torque_nm = 0.0f,
            .kp = 70.0f,
            .kd = 1.25f
        }
    }
};

static const m2006_config_t m2006_motor_config[] = {
    {
        .id = M2006_MOTOR_L_ID,
        .target_position_deg = 3600.0f,
        .pid = {
            .kp = 45.0f,
            .ki = 0.0f,
            .kd = 5.0f
        }
    },
    {
        .id = M2006_MOTOR_F_ID,
        .target_position_deg = 3600.0f,
        .pid = {
            .kp = 45.0f,
            .ki = 0.0f,
            .kd = 5.0f
        }
    }
};

static const uint16_t dm_2006_rx_ids[] = {
    DM_MOTOR_L_RX_ID,
    DM_MOTOR_F_RX_ID,
    C610_FEEDBACK_ID(M2006_MOTOR_L_ID),
    C610_FEEDBACK_ID(M2006_MOTOR_F_ID)
};

static const fdcan_task_config_t fdcan_config = {
    .rs_host_id = RS_HOST_ID,
    .dm_2006_rx_ids = dm_2006_rx_ids,
    .dm_2006_rx_id_count = ARRAY_SIZE(dm_2006_rx_ids)
};

static rs_app_t rs_app;
static dm_app_t dm_app;
static c610_bus_t c610_bus;
static m2006_motor_t m2006_motors[ARRAY_SIZE(m2006_motor_config)];
static bool rs_initialization_requested[ARRAY_SIZE(rs_motor_config)];
static bool dm_initialization_requested[ARRAY_SIZE(dm_motor_config)];
static bool app_ready;
static bool auto_initialize_enabled;
static bool motors_initialized;

static HAL_StatusTypeDef initialize_rs_motor(uint8_t index, bool restart)
{
    const rs_app_motor_config_t *config = &rs_motor_config[index];
    HAL_StatusTypeDef status;

    if (RS_AppSetCommand(&rs_app, config->id, &config->command) != HAL_OK)
    {
        return HAL_ERROR;
    }
    status = restart ? RS_AppRestart(&rs_app, config->id)
                     : RS_AppSetEnabled(&rs_app, config->id, true);
    if (status != HAL_OK)
    {
        return HAL_ERROR;
    }

    rs_initialization_requested[index] = true;
    return HAL_OK;
}

static HAL_StatusTypeDef initialize_dm_motor(uint8_t index, bool restart)
{
    const dm_app_motor_config_t *config = &dm_motor_config[index];
    dm_result_t result;

    if (DM_AppSetMitCommand(&dm_app, config->tx_id, &config->command) != DM_OK)
    {
        return HAL_ERROR;
    }
    result = restart ? DM_AppRestart(&dm_app, config->tx_id)
                     : DM_AppSetEnabled(&dm_app, config->tx_id, true);
    if (result != DM_OK)
    {
        return HAL_ERROR;
    }

    dm_initialization_requested[index] = true;
    return HAL_OK;
}

static void initialize_offline_motors(uint32_t now_ms)
{
    uint8_t i;

    if (!auto_initialize_enabled)
    {
        return;
    }

    for (i = 0U; i < ARRAY_SIZE(rs_motor_config); i++)
    {
        rs_app_status_t status;

        if (!RS_AppGetStatus(&rs_app, rs_motor_config[i].id, now_ms, &status))
        {
            continue;
        }
        if (status.online)
        {
            rs_initialization_requested[i] = false;
        }
        else if (!rs_initialization_requested[i] &&
                 (!status.has_feedback || (status.feedback.fault == 0U)))
        {
            (void)initialize_rs_motor(i, true);
        }
    }

    for (i = 0U; i < ARRAY_SIZE(dm_motor_config); i++)
    {
        dm_app_status_t status;

        if (!DM_AppGetStatus(&dm_app, dm_motor_config[i].tx_id, now_ms,
                             &status))
        {
            continue;
        }
        if (status.online)
        {
            dm_initialization_requested[i] = false;
        }
        else if (!dm_initialization_requested[i] &&
                 (!status.has_feedback ||
                  (status.feedback.fault == DM_FAULT_NONE)))
        {
            (void)initialize_dm_motor(i, true);
        }
    }
}

HAL_StatusTypeDef Up_MainInit(void)
{
    if (app_ready)
    {
        return HAL_OK;
    }

    if (FDCAN_TaskInit(&fdcan_config) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (RS_AppInit(&rs_app, FDCAN_TaskGetRsBus(), rs_motor_config,
                   ARRAY_SIZE(rs_motor_config)) != HAL_OK)
    {
        FDCAN_TaskStop();
        return HAL_ERROR;
    }
    if (DM_AppInit(&dm_app, FDCAN_TaskGetDm2006Bus(), dm_motor_config,
                   ARRAY_SIZE(dm_motor_config)) != DM_OK)
    {
        FDCAN_TaskStop();
        return HAL_ERROR;
    }
    if (C610_Init(&c610_bus, FDCAN_TaskGetDm2006Bus(), m2006_motors,
                  m2006_motor_config,
                  ARRAY_SIZE(m2006_motor_config)) != HAL_OK)
    {
        FDCAN_TaskStop();
        return HAL_ERROR;
    }
    app_ready = true;
    if (Up_MainInitializeMotors() != HAL_OK)
    {
        app_ready = false;
        FDCAN_TaskStop();
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Up_MainInitializeMotors(void)
{
    bool restart = motors_initialized;
    uint8_t i;

    if (!app_ready)
    {
        return HAL_ERROR;
    }

    auto_initialize_enabled = true;
    for (i = 0U; i < ARRAY_SIZE(rs_motor_config); i++)
    {
        if (initialize_rs_motor(i, restart) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    for (i = 0U; i < ARRAY_SIZE(dm_motor_config); i++)
    {
        if (initialize_dm_motor(i, restart) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    for (i = 0U; i < ARRAY_SIZE(m2006_motor_config); i++)
    {
        C610_SetPosition(&c610_bus, m2006_motor_config[i].id,
                         m2006_motor_config[i].target_position_deg);
    }
    motors_initialized = true;
    return HAL_OK;
}

void Up_MainRun1ms(void)
{
    uint32_t now_ms;

    if (!app_ready)
    {
        return;
    }

    now_ms = HAL_GetTick();
    FDCAN_TaskRun1ms();
    initialize_offline_motors(now_ms);
    RS_AppUpdate(&rs_app, now_ms);
    DM_AppUpdate(&dm_app, now_ms);
    C610_Update(&c610_bus, now_ms);
}

void Up_MainStopAll(void)
{
    if (!app_ready)
    {
        return;
    }

    auto_initialize_enabled = false;
    RS_AppDisableAll(&rs_app);
    DM_AppDisableAll(&dm_app);
    C610_DisableAll(&c610_bus);
}

bool Up_MainGetRsStatus(uint8_t id, rs_app_status_t *status)
{
    return app_ready && RS_AppGetStatus(&rs_app, id, HAL_GetTick(), status);
}

bool Up_MainGetDmStatus(uint16_t id, dm_app_status_t *status)
{
    return app_ready && DM_AppGetStatus(&dm_app, id, HAL_GetTick(), status);
}

bool Up_MainGetM2006Status(uint8_t id, m2006_status_t *status)
{
    return app_ready && C610_GetStatus(&c610_bus, id, status);
}
