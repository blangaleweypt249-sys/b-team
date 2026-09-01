/**
 * @file bsp_can.h
 * @brief 声明板级 CAN 帧发送接口。
 */

#ifndef BSP_CAN_H
/** 防止 bsp_can.h 被重复包含。 */
#define BSP_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

/* 逻辑总线与节点拓扑；具体设备驱动负责换算协议 CAN ID。 */
/** 机械臂 J4310 所连接的 CAN 总线编号。 */
#define CAN_BUS_ARM_J4310      1U
/** 两台机械臂 M3508 所连接的 CAN 总线编号。 */
#define CAN_BUS_ARM_M3508      2U
/** 挡板和夹爪 M2006 所连接的 CAN 总线编号。 */
#define CAN_BUS_AUX            3U

/** 机械臂 J4310 在达妙协议中的节点编号。 */
#define NODE_ARM_J4310         0x06U
/* 必须与达妙配置工具中保存的 J4310 主机 ID 一致。 */
/** J4310 反馈帧发送给主控时使用的 CAN 标识符。 */
#define CAN_J4310_MASTER_ID    0x016U

/** 第一台机械臂 M3508 的 DJI 电机编号。 */
#define NODE_ARM_M3508_1       1U
/** 第二台机械臂 M3508 的 DJI 电机编号。 */
#define NODE_ARM_M3508_2       2U

/** 挡板 M2006 的 DJI 电机编号。 */
#define NODE_GATE_M2006        1U
/** 夹爪 M2006 的 DJI 电机编号。 */
#define NODE_GRIPPER_M2006     2U

/** 机械臂 J4310 反馈帧的硬件滤波标识符。 */
#define CAN_FILTER_ARM_J4310_ID       CAN_J4310_MASTER_ID
/** DJI 一号电机反馈帧的硬件滤波标识符。 */
#define CAN_FILTER_DJI_NODE_1_ID      0x201U
/** DJI 二号电机反馈帧的硬件滤波标识符。 */
#define CAN_FILTER_DJI_NODE_2_ID      0x202U
/** 挡板 M2006 反馈帧的硬件滤波标识符。 */
#define CAN_FILTER_AUX_GATE_ID        (0x200U + NODE_GATE_M2006)
/** 夹爪 M2006 反馈帧的硬件滤波标识符。 */
#define CAN_FILTER_AUX_GRIPPER_ID     (0x200U + NODE_GRIPPER_M2006)

/* 功能：通过指定 FDCAN 总线发送经典 CAN 帧；用途：为上层电机协议提供统一发送口；返回 true 表示帧成功进入发送队列。 */
bool BspCan_Send(uint8_t can_bus /* CAN 总线编号 */, const can_frame_t *frame /* 需要解析或发送的 CAN 或协议帧 */);

#endif
