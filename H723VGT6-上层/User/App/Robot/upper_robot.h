/**
 * @file upper_robot.h
 * @brief 定义上层机器人对象、目标数据和控制接口。
 */

#ifndef UPPER_ROBOT_H
#define UPPER_ROBOT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "arm.h"
#include "gate.h"
#include "gripper.h"
#include "robot_state.h"

typedef struct
{
    bool position_mode;
    arm_target_t arm;
    gate_target_t gate;
    gripper_target_t gripper;
} upper_target_t;

typedef struct
{
    robot_state_t state;
    upper_target_t target;
    motor_manager_t motor_manager;
    uint32_t control_count;
} upper_robot_t;

/* 功能：初始化上层机器人对象和电机管理器；用途：让 DJI 电机以上电位置零点进入位控；返回 true 表示机器人进入运行态。 */
bool UpperRobot_Init(upper_robot_t *robot,
                     const motor_cfg_t *motor_cfg,
                     size_t motor_count,
                     motor_send_t send,
                     void *user_data);
/* 功能：保存一份新的整机控制目标；用途：供后续 1 ms 控制周期统一读取；无返回值表示覆盖目标快照。 */
void UpperRobot_SetTarget(upper_robot_t *robot,
                          const upper_target_t *target);
/* 功能：将就绪或停止状态切换为运行；用途：允许周期控制开始下发电机命令；无返回值表示仅执行合法状态转换。 */
void UpperRobot_Start(upper_robot_t *robot);
/* 功能：停止全部电机并进入停止态；用途：执行正常停机；无返回值表示状态和电机均被更新。 */
void UpperRobot_Stop(upper_robot_t *robot);
/* 功能：紧急停止全部电机并进入停止态；用途：立即撤销输出并等待新的显式控制目标。 */
void UpperRobot_EStop(upper_robot_t *robot);
/* 功能：把错误态恢复为就绪态；用途：故障排除后重新允许启动；无返回值表示仅在当前为错误态时生效。 */
void UpperRobot_ClearError(upper_robot_t *robot);
/* 功能：执行整机 1 ms 控制周期；用途：独立计算并提交各机构命令；单个机构异常时仅关闭对应电机。 */
void UpperRobot_Control1ms(upper_robot_t *robot, uint32_t tick_ms);

#endif
