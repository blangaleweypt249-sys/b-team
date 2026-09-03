/**
 * @file fdcan_dlc.h
 * @brief 提供经典 CAN 数据长度与 HAL DLC 编码的换算函数。
 */

#ifndef FDCAN_DLC_H
#define FDCAN_DLC_H /**< 防止 fdcan_dlc.h 被重复包含。 */

#include <stdint.h>

/* STM32H7 HAL 提供解码后的 DLC 编码；对于经典 CAN，编码 0..8 等于载荷字节数。 */
/* 功能：把 HAL FDCAN DLC 编码转换为经典 CAN 字节数；用途：校验并解析接收帧长度；返回值表示 0 至 8 字节的有效长度。 */
static inline uint8_t FdcanDlc_DecodeClassic(uint32_t hal_dlc /**< 待解码的HAL FDCAN数据长度字段 */)
{
    return (uint8_t)((hal_dlc > 8U) ? 8U : hal_dlc);
}

/* 功能：把经典 CAN 字节数转换为 HAL FDCAN DLC 编码；用途：构造发送帧头；返回值表示 HAL 使用的 DLC 字段。 */
static inline uint32_t FdcanDlc_EncodeClassic(uint8_t byte_length /**< 待编码为HAL DLC字段的经典CAN数据长度 */)
{
    return (uint32_t)byte_length;
}

#endif
