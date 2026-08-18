/**
 * @file dji_group.c
 * @brief 实现 DJI 电机分组电流命令帧的编码。
 */

#include "dji_group.h"

#include <string.h>

#define DJI_GROUP_ID_1_TO_4  0x200U
#define DJI_GROUP_ID_5_TO_8  0x1FFU

/* 功能：把 16 位有符号数写成大端字节；用途：编码 DJI 电机电流字段；无返回值表示结果写入 data。 */
static void DjiGroup_WriteI16Be(uint8_t *data, int16_t value)
{
    uint16_t raw;

    raw = (uint16_t)value;
    data[0] = (uint8_t)(raw >> 8U);
    data[1] = (uint8_t)raw;
}

/* 功能：把四路电流命令打包为 DJI 分组 CAN 帧；用途：控制 1-4 或 5-8 号电机；返回 true 表示构帧成功。 */
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
