/**
 * @file vofa_bridge.h
 * @brief 声明非生产 VOFA 电机调试桥的初始化、收发和周期控制接口。
 */

#ifndef VOFA_BRIDGE_H
#define VOFA_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_frame.h"

/* 功能：初始化全部 VOFA 会话和接收状态；用途：建立调试桥默认参数；无返回值表示所有会话被复位。 */
void VofaBridge_Init(void);
/* 功能：接收字节流并按行解析 VOFA 命令；用途：从 UART 数据中提取文本控制消息；返回 true 表示至少消费了 VOFA 数据。 */
bool VofaBridge_Receive(const uint8_t *data,
                        size_t size,
                        uint32_t tick_ms);
/* 功能：执行 VOFA 桥的 1 ms 周期任务；用途：处理会话超时、目标应用、分组发送和遥测；无返回值表示完成一次调度。 */
void VofaBridge_Control1ms(uint32_t tick_ms);
/* 功能：把 DJI CAN 反馈路由到对应 VOFA 会话；用途：更新调试电机反馈和在线整定输入；无返回值表示匹配帧已记录。 */
void VofaBridge_OnCanFrame(uint8_t can_bus,
                           const can_frame_t *frame,
                           uint32_t tick_ms);
/* 功能：查询是否存在活动 VOFA 会话；用途：判断调试桥是否正在占用电机控制；返回 true 表示至少一个会话已启动。 */
bool VofaBridge_IsActive(void);

#endif
