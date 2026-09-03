/**
 * @file dji_group.h
 * @brief 声明 DJI 电机分组电流帧构造接口。
 */

#ifndef DJI_GROUP_H
#define DJI_GROUP_H /**< 防止 dji_group.h 被重复包含。 */

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define DJI_GROUP_MOTOR_COUNT  4U /**< 一个 DJI 电流控制组帧可携带的电机命令数量。 */

/* 功能：把四路电流命令打包为 DJI 分组 CAN 帧；用途：控制 1-4 或 5-8 号电机；返回 true 表示构帧成功。 */
bool DjiGroup_BuildFrame(uint8_t start_motor_id /**< DJI 组帧覆盖的第一个电机编号 */,
                         const int16_t current_raw[DJI_GROUP_MOTOR_COUNT] /**< DJI 协议中的电流命令原始值 */,
                         can_frame_t *frame /**< 用于写出待发送 CAN 帧的对象 */);

#endif
