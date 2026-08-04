#ifndef CAN_RX_H
#define CAN_RX_H

#include "main.h"

/*
 * CAN1 reception (RX) layer.
 *
 * The project originally had CAN TX only. This adds reception so the chassis
 * presence check can see VESC status/pong frames. It is purely ADDITIVE:
 *  - configures an accept-all filter on FIFO0,
 *  - enables the FIFO0 message-pending interrupt (CAN1_RX0),
 *  - feeds every received extended frame to chassis_mark_seen() (which only
 *    cares about registered ids).
 *
 * Call can_rx_init(&hcan1) once after the CAN peripheral has been started.
 * TX (vesc_can_*) and the CAN configuration itself are not modified.
 */
void can_rx_init(CAN_HandleTypeDef *hcan);

#endif /* CAN_RX_H */
