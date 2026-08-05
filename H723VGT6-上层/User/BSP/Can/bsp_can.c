#include "bsp_can.h"

#include "fdcan.h"

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
    header.DataLength = ((uint32_t)frame->dlc << 16U);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan,
                                          &header,
                                          frame->data) == HAL_OK;
}
