/**
 * @file bsp_can.h
 * @brief 声明板级 CAN 帧发送接口。
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

/* 逻辑总线与节点拓扑；具体设备驱动负责换算协议 CAN ID。 */
#define CAN_BUS_ARM_J4310      1U
#define CAN_BUS_ARM_M3508      2U
#define CAN_BUS_AUX            3U

#define NODE_ARM_J4310         0x06U
/* 必须与达妙配置工具中保存的 J4310 主机 ID 一致。 */
#define CAN_J4310_MASTER_ID    0x016U

#define NODE_ARM_M3508_1       1U
#define NODE_ARM_M3508_2       2U

#define NODE_GATE_M2006        1U
#define NODE_GRIPPER_M2006     2U

#define CAN_FILTER_ARM_J4310_ID       CAN_J4310_MASTER_ID
#define CAN_FILTER_DJI_NODE_1_ID      0x201U
#define CAN_FILTER_DJI_NODE_2_ID      0x202U
#define CAN_FILTER_AUX_GATE_ID        (0x200U + NODE_GATE_M2006)
#define CAN_FILTER_AUX_GRIPPER_ID     (0x200U + NODE_GRIPPER_M2006)

/* 功能：通过指定 FDCAN 总线发送经典 CAN 帧；用途：为上层电机协议提供统一发送口；返回 true 表示帧成功进入发送队列。 */
bool BspCan_Send(uint8_t can_bus, const can_frame_t *frame);

#endif
