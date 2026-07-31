#include "upper_config.h"

#include "can_id.h"

const motor_cfg_t upper_motor_cfg[UPPER_MOTOR_COUNT] =
{
    [UPPER_MOTOR_ARM_MG5010] =
    {
        "arm_mg5010", MOTOR_MODEL_MG5010,
        CAN_BUS_ARM, NODE_ARM_MG5010, UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_ARM_J4310] =
    {
        "arm_j4310", MOTOR_MODEL_J4310,
        CAN_BUS_ARM, NODE_ARM_J4310, UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_MOVE_M3508_L] =
    {
        "move_m3508_l", MOTOR_MODEL_M3508,
        CAN_BUS_MOVE, NODE_MOVE_M3508_L, UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_MOVE_M3508_R] =
    {
        "move_m3508_r", MOTOR_MODEL_M3508,
        CAN_BUS_MOVE, NODE_MOVE_M3508_R, UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_MOVE_M2006_L] =
    {
        "move_m2006_l", MOTOR_MODEL_M2006,
        CAN_BUS_MOVE, NODE_MOVE_M2006_L, UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_MOVE_M2006_R] =
    {
        "move_m2006_r", MOTOR_MODEL_M2006,
        CAN_BUS_MOVE, NODE_MOVE_M2006_R, UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_ORE_M2006] =
    {
        "ore_m2006", MOTOR_MODEL_M2006,
        CAN_BUS_AUX, NODE_ORE_M2006, UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_GATE_M2006] =
    {
        "gate_m2006", MOTOR_MODEL_M2006,
        CAN_BUS_AUX, NODE_GATE_M2006, UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_CONVEYOR_L] =
    {
        "conveyor_m4216_l", MOTOR_MODEL_DJM4216,
        CAN_BUS_AUX, NODE_CONVEYOR_M4216_L,
        UPPER_CONTROL_PERIOD_MS, 0U, false
    },
    [UPPER_MOTOR_CONVEYOR_R] =
    {
        "conveyor_m4216_r", MOTOR_MODEL_DJM4216,
        CAN_BUS_AUX, NODE_CONVEYOR_M4216_R,
        UPPER_CONTROL_PERIOD_MS, 0U, false
    }
};
