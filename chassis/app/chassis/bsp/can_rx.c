#include "can_rx.h"
#include "can.h"            /* hcan1 */
#include "chassis_motors.h"  /* chassis_mark_seen */
#include "encoder.h"        /* encoder_on_rx */

/* VESC CAN packet ids used to interpret received frames (VESC 6.x enum). */
#define VESC_CAN_PACKET_STATUS   9U    /* VESC -> host status broadcast */
#define VESC_CAN_PACKET_PONG    18U    /* reply to CAN_PACKET_PING; data[0] = ponging id */

void can_rx_init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef filter = {0};

  /* Accept-all filter into FIFO0 (id mask 0 matches everything). */
  filter.FilterBank           = 0U;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh         = 0U;
  filter.FilterIdLow          = 0U;
  filter.FilterMaskIdHigh     = 0U;
  filter.FilterMaskIdLow      = 0U;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation     = ENABLE;
  filter.SlaveStartFilterBank = 14U;   /* F405: 14 filter banks for CAN1 */
  (void)HAL_CAN_ConfigFilter(hcan, &filter);

  /* Enable the FIFO0 message-pending interrupt. */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  (void)HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* CAN1 RX FIFO0 interrupt -> HAL dispatcher. */
void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

/*
 * Called by the HAL once a frame is pending in FIFO0.
 * Every received extended frame is FROM a VESC, so the source controller id is
 * carried in the low byte of ExtId (for status/short frames) or, for a PONG,
 * in data[0]. Feed whichever to the chassis presence tracker.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx = {0};
  uint8_t data[8];

  if (hcan->Instance == CAN1)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) == HAL_OK)
    {
      if (rx.IDE == CAN_ID_EXT)
      {
        uint8_t packet = (uint8_t)(rx.ExtId >> 8);
        uint8_t cid    = (uint8_t)(rx.ExtId & 0x00FFU);

        if ((packet == VESC_CAN_PACKET_PONG) && (rx.DLC >= 1U))
        {
          /* PONG is addressed to the pinger; the ponging id is in data[0]. */
          chassis_mark_seen(data[0]);
        }
        else
        {
          /* All other VESC->host frames carry the source id in the low byte. */
          chassis_mark_seen(cid);
        }

        /* Decode status frames (rpm / tachometer) for encoder + odometry. */
        encoder_on_rx(cid, packet, data, rx.DLC);
      }
    }
  }
}
