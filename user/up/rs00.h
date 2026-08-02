#ifndef RS00_H
#define RS00_H

#include "rs_bus.h"

typedef enum
{
    RS_MOTION = 0,
    RS_PP = 1,
    RS_SPD = 2,
    RS_IQ = 3,
    RS_CSP = 5
} rs_mode_t;

typedef enum
{
    RS_RESET = 0,
    RS_CALI = 1,
    RS_RUN = 2
} rs_state_t;

typedef enum
{
    RS_FDB_POSITION = 1U << 0,
    RS_FDB_SPEED = 1U << 1,
    RS_FDB_TORQUE = 1U << 2,
    RS_FDB_TEMP = 1U << 3,
    RS_FDB_STATE = 1U << 4,
    RS_FDB_FAULT = 1U << 5
} rs_feedback_valid_t;

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
    uint32_t fault;
    uint32_t warning;
    uint32_t valid;
    uint32_t sequence;
    uint8_t state;
} rs_feedback_t;

typedef struct
{
    rs_bus_t *bus;
    rs_feedback_t feedback;
    uint32_t last_rx_ms;
    float pp_speed_rad_s;
    float pp_acceleration_rad_s2;
    float limit_speed_rad_s;
    float limit_iq;
    uint32_t transition_due_ms;
    uint8_t id;
    uint8_t mode;
    uint8_t active;
    uint8_t pp_configured;
    uint8_t limit_valid;
    uint8_t start_step;
    uint8_t requested_mode;
} rs_motor_t;

HAL_StatusTypeDef RS_MotorInit(rs_motor_t *motor, rs_bus_t *bus, uint8_t id);

HAL_StatusTypeDef RS_MotorSetMechanicalZero(rs_motor_t *motor);

HAL_StatusTypeDef RS_MotorStart(rs_motor_t *motor, rs_mode_t mode,
                                uint32_t now_ms);
HAL_StatusTypeDef RS_MotorStop(rs_motor_t *motor);
HAL_StatusTypeDef RS_MotorClearFault(rs_motor_t *motor);

HAL_StatusTypeDef RS_MotorSetMotion(rs_motor_t *motor, float position_rad,
                                    float velocity_rad_s, float torque_nm,
                                    float kp, float kd);
HAL_StatusTypeDef RS_MotorSetIq(rs_motor_t *motor, float iq);
HAL_StatusTypeDef RS_MotorSetSpeed(rs_motor_t *motor, float velocity_rad_s,
                                   float max_iq);
HAL_StatusTypeDef RS_MotorSetCsp(rs_motor_t *motor, float position_rad,
                                 float max_velocity_rad_s);
HAL_StatusTypeDef RS_MotorSetPp(rs_motor_t *motor, float position_rad,
                                float max_velocity_rad_s,
                                float acceleration_rad_s2);
HAL_StatusTypeDef RS_MotorHaltPp(rs_motor_t *motor);

HAL_StatusTypeDef RS_MotorGetFeedback(const rs_motor_t *motor,
                                      rs_feedback_t *feedback);
void RS_MotorParse(rs_motor_t *motor, uint32_t id, const uint8_t data[8],
                   uint32_t tick_ms);


#endif
