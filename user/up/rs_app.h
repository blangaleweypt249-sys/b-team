#ifndef RS_APP_H
#define RS_APP_H

#include "rs00.h"

#include <stdbool.h>


#define RS_APP_MAX_MOTORS          8U
#define RS_APP_CONTROL_PERIOD_MS   1U
#define RS_APP_FEEDBACK_TIMEOUT_MS 50U

typedef struct
{
    float angle_deg;
    float speed_rad_s;
    float torque_nm;
    float kp;
    float kd;
} rs_motion_command_t;

typedef struct
{
    float iq;
} rs_iq_command_t;

typedef struct
{
    float speed_rad_s;
    float max_iq;
} rs_speed_command_t;

typedef struct
{
    float angle_deg;
    float max_speed_rad_s;
} rs_csp_command_t;

typedef struct
{
    float angle_deg;
    float max_speed_rad_s;
    float acceleration_rad_s2;
} rs_pp_command_t;

typedef union
{
    rs_motion_command_t motion;
    rs_iq_command_t iq;
    rs_speed_command_t speed;
    rs_csp_command_t csp;
    rs_pp_command_t pp;
} rs_command_data_t;

typedef struct
{
    rs_mode_t mode;
    rs_command_data_t data;
} rs_command_t;

typedef struct
{
    uint8_t id;
    uint32_t period_ms;
    rs_command_t command;
} rs_app_motor_config_t;

typedef struct
{
    rs_motor_t motor;
    rs_command_t command;
    HAL_StatusTypeDef last_result;
    uint32_t period_ms;
    uint32_t next_control_ms;
    bool enable_requested;
} rs_app_motor_t;

typedef struct
{
    float angle_deg;
    float speed_rad_s;
    float torque_nm;
    float temperature_c;
    uint32_t fault;
    uint32_t warning;
    uint32_t valid;
    uint32_t sequence;
    uint8_t state;
} rs_app_feedback_t;

typedef struct
{
    rs_bus_t *bus;
    rs_app_motor_t motors[RS_APP_MAX_MOTORS];
    uint8_t motor_count;
    bool ready;
} rs_app_t;

typedef struct
{
    rs_app_feedback_t feedback;
    HAL_StatusTypeDef last_result;
    bool has_feedback;
    bool online;
    bool enable_requested;
    bool active;
} rs_app_status_t;

HAL_StatusTypeDef RS_AppInit(rs_app_t *app, rs_bus_t *bus,
                             const rs_app_motor_config_t *config,
                             uint8_t count);
void RS_AppUpdate(rs_app_t *app, uint32_t now_ms);
HAL_StatusTypeDef RS_AppSetCommand(rs_app_t *app, uint8_t id,
                                   const rs_command_t *command);
HAL_StatusTypeDef RS_AppSetEnabled(rs_app_t *app, uint8_t id, bool enabled);
HAL_StatusTypeDef RS_AppRestart(rs_app_t *app, uint8_t id);
HAL_StatusTypeDef RS_AppSetMechanicalZero(rs_app_t *app, uint8_t id);
HAL_StatusTypeDef RS_AppClearFault(rs_app_t *app, uint8_t id);
void RS_AppDisableAll(rs_app_t *app);
bool RS_AppGetStatus(const rs_app_t *app, uint8_t id, uint32_t now_ms,
                     rs_app_status_t *status);


#endif
