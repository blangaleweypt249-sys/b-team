#ifndef FDCAN_DLC_H
#define FDCAN_DLC_H

#include <stdint.h>

/* STM32H7 HAL exposes the decoded DLC code. For classic CAN, codes 0..8
   equal the payload byte length. */
static inline uint8_t FdcanDlc_DecodeClassic(uint32_t hal_dlc)
{
    return (uint8_t)((hal_dlc > 8U) ? 8U : hal_dlc);
}

static inline uint32_t FdcanDlc_EncodeClassic(uint8_t byte_length)
{
    return (uint32_t)byte_length;
}

#endif
