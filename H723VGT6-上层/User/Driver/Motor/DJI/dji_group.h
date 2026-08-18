/**
 * @file dji_group.h
 * @brief 声明 DJI 电机分组电流帧构造接口。
 */

#ifndef DJI_GROUP_H
#define DJI_GROUP_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define DJI_GROUP_MOTOR_COUNT  4U

/* 功能：把四路电流命令打包为 DJI 分组 CAN 帧；用途：控制 1-4 或 5-8 号电机；返回 true 表示构帧成功。 */
bool DjiGroup_BuildFrame(uint8_t start_motor_id,
                         const int16_t current_raw[DJI_GROUP_MOTOR_COUNT],
                         can_frame_t *frame);

#endif
