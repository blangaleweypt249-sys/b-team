/**
 * @file robot_state.h
 * @brief 定义上层机器人的运行状态枚举。
 */

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

typedef enum
{
    ROBOT_INIT,
    ROBOT_READY,
    ROBOT_RUN,
    ROBOT_STOP,
    ROBOT_ERROR
} robot_state_t;

#endif
