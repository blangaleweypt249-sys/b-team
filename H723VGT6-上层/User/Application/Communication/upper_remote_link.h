#ifndef UPPER_REMOTE_LINK_H
#define UPPER_REMOTE_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UPPER_REMOTE_TIMEOUT_MS       200U
#define UPPER_REMOTE_KEY_MASK         0x3FU
#define UPPER_REMOTE_SWITCH_MASK      0x37U
#define UPPER_REMOTE_FRAME_BUFFER_SIZE 32U

typedef struct
{
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

void UpperRemoteLink_Init(upper_remote_link_t *link);
void UpperRemoteLink_Push(upper_remote_link_t *link,
                          const uint8_t *data,
                          size_t size,
                          uint32_t tick_ms);
bool UpperRemoteLink_GetControl(const upper_remote_link_t *link,
                                uint32_t tick_ms,
                                upper_remote_control_t *control);
void UpperRemoteLink_GetDiagnostics(const upper_remote_link_t *link,
                                    upper_remote_diagnostics_t *diagnostics);

#endif
