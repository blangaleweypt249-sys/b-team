/**
 * @file robot_state.h
 * @brief 定义上层机器人的运行状态枚举。
 */

#ifndef ROBOT_STATE_H
/** 防止 robot_state.h 被重复包含。 */
#define ROBOT_STATE_H

/** 表示 模块 当前所处的运行状态。 */
typedef enum
{
    ROBOT_INIT, /**< 正在初始化上层对象和电机驱动。 */
    ROBOT_READY, /**< 初始化完成，等待启动命令。 */
    ROBOT_RUN, /**< 机器人正在执行周期控制。 */
    ROBOT_STOP, /**< 停止输出。 */
    ROBOT_ERROR /**< 检测到错误，禁止继续输出。 */
} robot_state_t;

#endif
