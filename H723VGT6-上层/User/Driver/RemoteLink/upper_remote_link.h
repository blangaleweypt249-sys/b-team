/**
 * @file upper_remote_link.h
 * @brief 定义遥控器链路的数据格式、运行状态和访问接口。
 */

#ifndef UPPER_REMOTE_LINK_H
#define UPPER_REMOTE_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UPPER_REMOTE_TIMEOUT_MS       200U
#define UPPER_REMOTE_PRIMARY_KEY_MASK    0x06U
#define UPPER_REMOTE_PRIMARY_SWITCH_MASK 0x03U
#define UPPER_REMOTE_KEY_MASK         0x3FU
#define UPPER_REMOTE_SWITCH_MASK      0x37U
#define UPPER_REMOTE_FRAME_BUFFER_SIZE 32U

#define UPPER_REMOTE_PRIMARY_KEY_PC0     (1U << 2U)
#define UPPER_REMOTE_PRIMARY_KEY_PC1     (1U << 1U)
#define UPPER_REMOTE_PRIMARY_SWITCH_PE0 (1U << 0U)
#define UPPER_REMOTE_PRIMARY_SWITCH_PD6 (1U << 1U)
#define UPPER_REMOTE_SWITCH_PD6          (1U << 4U)

typedef struct
{
    uint8_t primary_key_bits;
    uint8_t primary_switch;
    uint8_t key_bits;
    uint8_t switch_bits;
    uint8_t sequence;
    uint32_t updated_at_ms;
    bool online;
} upper_remote_control_t;

typedef struct
{
    uint32_t valid_frame_count;
    uint32_t ignored_frame_count;
    uint32_t crc_error_count;
    uint32_t format_error_count;
    uint32_t buffer_overflow_count;
    uint32_t lost_frame_count;
    uint32_t duplicate_frame_count;
} upper_remote_diagnostics_t;

typedef struct
{
    uint8_t frame_buffer[UPPER_REMOTE_FRAME_BUFFER_SIZE];
    size_t buffered_size;
    volatile upper_remote_control_t control;
    volatile uint32_t control_version;
    upper_remote_diagnostics_t diagnostics;
    volatile bool has_valid_control;
    bool has_sequence;
    uint8_t last_sequence;
} upper_remote_link_t;

/* 功能：初始化遥控链路对象和诊断状态；用途：建立可接收定长遥控帧的初始状态；无返回值表示对象已复位。 */
void UpperRemoteLink_Init(upper_remote_link_t *link);
/* 功能：向遥控链路压入新收到的字节流；用途：缓存数据并触发定长帧解析；返回值表示本次接收的有效帧数。 */
void UpperRemoteLink_Push(upper_remote_link_t *link,
                          const uint8_t *data,
                          size_t size,
                          uint32_t tick_ms);
/* 功能：读取当前遥控输入并判断是否超时；用途：向应用层提供按键、摇杆和在线状态；返回 true 表示输出参数有效。 */
bool UpperRemoteLink_GetControl(const upper_remote_link_t *link,
                                uint32_t tick_ms,
                                upper_remote_control_t *control);
/* 功能：读取遥控链路诊断统计；用途：观察收帧、丢帧、重同步和缓存状态；无返回值表示统计已复制到输出。 */
void UpperRemoteLink_GetDiagnostics(const upper_remote_link_t *link,
                                    upper_remote_diagnostics_t *diagnostics);

#endif
