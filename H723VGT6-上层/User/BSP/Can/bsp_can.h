#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

bool BspCan_Send(uint8_t can_bus, const can_frame_t *frame);

#endif
