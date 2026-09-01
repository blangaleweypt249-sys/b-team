/**
 * @file can_frame.h
 * @brief 定义板级 CAN 接口统一使用的 CAN 帧数据结构。
 */

#ifndef CAN_FRAME_H
/** 防止 can_frame.h 被重复包含。 */
#define CAN_FRAME_H

#include <stdbool.h>
#include <stdint.h>

/** 保存一帧待发送或已接收的 CAN 数据。 */
typedef struct
{
    uint32_t id; /**< CAN 帧的标准或扩展标识符。 */
    bool extended; /**< 该帧是否使用 29 位扩展 CAN 标识符。 */
    uint8_t dlc; /**< CAN 帧携带的有效数据字节数。 */
    uint8_t data[8]; /**< CAN 帧的有效数据字节。 */
} can_frame_t;

#endif
