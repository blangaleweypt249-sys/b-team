#ifndef DJI_GROUP_H
#define DJI_GROUP_H

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define DJI_GROUP_MOTOR_COUNT  4U

bool DjiGroup_BuildFrame(uint8_t start_motor_id,
                         const int16_t current_raw[DJI_GROUP_MOTOR_COUNT],
                         can_frame_t *frame);

#endif
