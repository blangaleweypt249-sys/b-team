#include "upper_config.h"

#include "can_id.h"

const motor_cfg_t upper_motor_cfg[UPPER_MOTOR_COUNT] =
{
    [UPPER_MOTOR_ARM_M3508_1] =
    {
        "arm_m3508_1", MOTOR_MODEL_M3508,
        CAN_BUS_ARM_M3508, NODE_ARM_M3508_1,
        UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_ARM_M3508_2] =
    {
        "arm_m3508_2", MOTOR_MODEL_M3508,
        CAN_BUS_ARM_M3508, NODE_ARM_M3508_2,
        UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_ARM_J4310] =
    {
        "arm_j4310", MOTOR_MODEL_J4310,
        CAN_BUS_ARM_J4310, NODE_ARM_J4310,
        UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_CONVEYOR_M2006] =
    {
        "conveyor_m2006", MOTOR_MODEL_M2006,
        CAN_BUS_AUX, NODE_CONVEYOR_M2006, UPPER_CONTROL_PERIOD_MS, 0U, true
    },
    [UPPER_MOTOR_GRIPPER_M2006] =
    {
        "gripper_m2006", MOTOR_MODEL_M2006,
        CAN_BUS_AUX, NODE_GRIPPER_M2006, UPPER_CONTROL_PERIOD_MS, 0U, true
    }
};
