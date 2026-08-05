#include "dji_group.h"

#include <string.h>

#define DJI_GROUP_ID_1_TO_4  0x200U
#define DJI_GROUP_ID_5_TO_8  0x1FFU

static void DjiGroup_WriteI16Be(uint8_t *data, int16_t value)
{
    uint16_t raw;

    raw = (uint16_t)value;
    data[0] = (uint8_t)(raw >> 8U);
    data[1] = (uint8_t)raw;
}

bool DjiGroup_BuildFrame(uint8_t start_motor_id,
                         const int16_t current_raw[DJI_GROUP_MOTOR_COUNT],
                         can_frame_t *frame)
{
    uint32_t index;

    if ((current_raw == NULL) || (frame == NULL) ||
        ((start_motor_id != 1U) && (start_motor_id != 5U)))
    {
        return false;
    }

    (void)memset(frame, 0, sizeof(*frame));
    frame->id = (start_motor_id == 1U) ? DJI_GROUP_ID_1_TO_4 :
                                        DJI_GROUP_ID_5_TO_8;
    frame->dlc = 8U;
    for (index = 0U; index < DJI_GROUP_MOTOR_COUNT; index++)
    {
        DjiGroup_WriteI16Be(&frame->data[index * 2U], current_raw[index]);
    }
    return true;
}
