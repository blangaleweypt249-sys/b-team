/**
 * @file upper_remote_link.c
 * @brief 实现遥控器定长帧接收、重同步、状态更新和诊断统计。
 */

#include "upper_remote_link.h"

#include <string.h>

#define REMOTE_HEADER_0            0xA5U
#define REMOTE_HEADER_1            0x5AU
#define REMOTE_FRAME_SIZE          10U
#define REMOTE_KEY_INDEX           8U
#define REMOTE_SWITCH_INDEX        9U
#define UPPER_REMOTE_WATCHDOGS_ENABLED 0U

/* 功能：从遥控接收缓存头部丢弃指定字节；用途：消费已处理数据或恢复帧同步；无返回值表示缓存已前移。 */
static void UpperRemoteLink_Discard(upper_remote_link_t *link, size_t size)
{
    if (size >= link->buffered_size)
    {
        link->buffered_size = 0U;
        return;
    }

    link->buffered_size -= size;
    (void)memmove(link->frame_buffer,
                  &link->frame_buffer[size],
                  link->buffered_size);
}

/* 功能：解码并接收一帧合法遥控数据；用途：更新按键、摇杆、序列号和接收时刻；无返回值表示最新控制状态已保存。 */
static void UpperRemoteLink_AcceptFrame(upper_remote_link_t *link,
                                        const uint8_t *frame,
                                        uint32_t tick_ms)
{
    link->control_version++;
    link->control.key_bits = frame[REMOTE_KEY_INDEX] &
                             UPPER_REMOTE_KEY_MASK;
    link->control.switch_bits = frame[REMOTE_SWITCH_INDEX] &
                                UPPER_REMOTE_SWITCH_MASK;
    link->control.sequence = 0U;
    link->control.updated_at_ms = tick_ms;
    link->control.online = true;
    link->has_valid_control = true;
    link->control_version++;
    link->diagnostics.valid_frame_count++;
}

/* 功能：扫描遥控接收缓存并提取完整帧；用途：处理分包、粘包、噪声和帧头重同步；无返回值表示当前可解析数据已消费。 */
static void UpperRemoteLink_Process(upper_remote_link_t *link,
                                    uint32_t tick_ms)
{
    size_t header_offset;

    for (;;)
    {
        if (link->buffered_size < 2U)
        {
            return;
        }

        header_offset = 0U;
        while ((header_offset + 1U) < link->buffered_size)
        {
            if ((link->frame_buffer[header_offset] == REMOTE_HEADER_0) &&
                (link->frame_buffer[header_offset + 1U] == REMOTE_HEADER_1))
            {
                break;
            }
            header_offset++;
        }
        if ((header_offset + 1U) >= link->buffered_size)
        {
            if (link->frame_buffer[link->buffered_size - 1U] == REMOTE_HEADER_0)
            {
                link->frame_buffer[0] = REMOTE_HEADER_0;
                link->buffered_size = 1U;
            }
            else
            {
                link->buffered_size = 0U;
            }
            return;
        }
        if (header_offset > 0U)
        {
            UpperRemoteLink_Discard(link, header_offset);
        }
        if (link->buffered_size < REMOTE_FRAME_SIZE)
        {
            return;
        }

        UpperRemoteLink_AcceptFrame(link, link->frame_buffer, tick_ms);
        UpperRemoteLink_Discard(link, REMOTE_FRAME_SIZE);
    }
}

/* 功能：初始化遥控链路对象和诊断状态；用途：建立可接收定长遥控帧的初始状态；无返回值表示对象已复位。 */
void UpperRemoteLink_Init(upper_remote_link_t *link)
{
    if (link != NULL)
    {
        (void)memset(link, 0, sizeof(*link));
    }
}

/* 功能：向遥控链路压入新收到的字节流；用途：缓存数据并触发定长帧解析；返回值表示本次接收的有效帧数。 */
void UpperRemoteLink_Push(upper_remote_link_t *link,
                          const uint8_t *data,
                          size_t size,
                          uint32_t tick_ms)
{
    size_t index;

    if ((link == NULL) || (data == NULL) || (size == 0U))
    {
        return;
    }

    for (index = 0U; index < size; index++)
    {
        if (link->buffered_size >= sizeof(link->frame_buffer))
        {
            link->diagnostics.buffer_overflow_count++;
            UpperRemoteLink_Discard(link, 1U);
        }
        link->frame_buffer[link->buffered_size] = data[index];
        link->buffered_size++;
        UpperRemoteLink_Process(link, tick_ms);
    }
}

/* 功能：读取当前遥控输入并判断是否超时；用途：向应用层提供按键、摇杆和在线状态；返回 true 表示输出参数有效。 */
bool UpperRemoteLink_GetControl(const upper_remote_link_t *link,
                                uint32_t tick_ms,
                                upper_remote_control_t *control)
{
    bool online;
    uint32_t version_before;
    uint32_t version_after;

    if (control == NULL)
    {
        return false;
    }
    (void)memset(control, 0, sizeof(*control));
    if ((link == NULL) || !link->has_valid_control)
    {
        return false;
    }

    for (;;)
    {
        version_before = link->control_version;
        if ((version_before & 1U) != 0U)
        {
            continue;
        }
        control->key_bits = link->control.key_bits;
        control->switch_bits = link->control.switch_bits;
        control->sequence = link->control.sequence;
        control->updated_at_ms = link->control.updated_at_ms;
        control->online = link->control.online;
        version_after = link->control_version;
        if ((version_before == version_after) &&
            ((version_after & 1U) == 0U))
        {
            break;
        }
    }

    if (UPPER_REMOTE_WATCHDOGS_ENABLED != 0U)
    {
        online = (uint32_t)(tick_ms - control->updated_at_ms) <
                 UPPER_REMOTE_TIMEOUT_MS;
    }
    else
    {
        (void)tick_ms;
        online = control->online;
    }
    control->online = online;
    if (!online)
    {
        control->key_bits = 0U;
        control->switch_bits = 0U;
    }
    return online;
}

/* 功能：读取遥控链路诊断统计；用途：观察收帧、丢帧、重同步和缓存状态；无返回值表示统计已复制到输出。 */
void UpperRemoteLink_GetDiagnostics(const upper_remote_link_t *link,
                                    upper_remote_diagnostics_t *diagnostics)
{
    if (diagnostics == NULL)
    {
        return;
    }
    if (link == NULL)
    {
        (void)memset(diagnostics, 0, sizeof(*diagnostics));
        return;
    }
    *diagnostics = link->diagnostics;
}
