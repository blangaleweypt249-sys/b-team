#include "bsp_can.h"

#include "fdcan.h"
#include "fdcan_dlc.h"

/* 功能：把逻辑 CAN 总线号映射为 HAL 句柄；用途：统一选择 FDCAN 外设；返回 NULL 表示总线号无效。 */
static FDCAN_HandleTypeDef *BspCan_GetHandle(uint8_t can_bus)
{
    switch (can_bus)
    {
    case 1U:
        return &hfdcan1;

    case 2U:
        return &hfdcan2;

    case 3U:
        return &hfdcan3;

    default:
        return NULL;
    }
}

/* 功能：通过指定 FDCAN 总线发送经典 CAN 帧；用途：为上层电机协议提供统一发送口；返回 true 表示帧成功进入发送队列。 */
bool BspCan_Send(uint8_t can_bus, const can_frame_t *frame)
{
    FDCAN_HandleTypeDef *hfdcan;
    FDCAN_TxHeaderTypeDef header;

    if ((frame == NULL) || (frame->dlc > 8U))
    {
        return false;
    }

    hfdcan = BspCan_GetHandle(can_bus);
    if (hfdcan == NULL)
    {
        return false;
    }

    header.Identifier = frame->id;
    header.IdType = frame->extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FdcanDlc_EncodeClassic(frame->dlc);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan,
                                          &header,
                                          frame->data) == HAL_OK;
}
