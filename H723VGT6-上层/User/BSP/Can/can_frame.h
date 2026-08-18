/**
 * @file can_frame.h
 * @brief 定义板级 CAN 接口统一使用的 CAN 帧数据结构。
 */

#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t id;
    bool extended;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

#endif
