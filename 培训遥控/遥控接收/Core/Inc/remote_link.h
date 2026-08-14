#ifndef REMOTE_LINK_H
#define REMOTE_LINK_H

#include "main.h"

#define REMOTE_LINK_FRAME_OVERHEAD 7U
#define REMOTE_LINK_MAX_PAYLOAD_SIZE 6U
#define REMOTE_LINK_MAX_FRAME_SIZE (REMOTE_LINK_FRAME_OVERHEAD + REMOTE_LINK_MAX_PAYLOAD_SIZE)
#define REMOTE_LINK_TIMEOUT_MS 200U
#define REMOTE_LINK_SWITCH_COUNT 6U
#define REMOTE_LINK_KEY_COUNT 12U
#define REMOTE_LINK_LORA_CONFIG_SIZE 6U

typedef enum
{
    REMOTE_LINK_LORA_CONFIG_NOT_STARTED = 0U,
    REMOTE_LINK_LORA_CONFIGURING,
    REMOTE_LINK_LORA_CONFIG_READY,
    REMOTE_LINK_LORA_CONFIG_AUX_TIMEOUT,
    REMOTE_LINK_LORA_CONFIG_UART_ERROR,
    REMOTE_LINK_LORA_CONFIG_READ_ERROR,
    REMOTE_LINK_LORA_CONFIG_WRITE_ERROR,
    REMOTE_LINK_LORA_CONFIG_VERIFY_ERROR,
    REMOTE_LINK_LORA_CONFIG_NORMAL_MODE_TIMEOUT
} remote_link_lora_config_status_t;

typedef struct
{
    uint16_t left_shoulder;
    uint16_t right_shoulder;
    uint8_t left_x;
    uint8_t left_y;
    uint8_t right_x;
    uint8_t right_y;
    uint8_t switch_state[REMOTE_LINK_SWITCH_COUNT];
    uint8_t key_state[REMOTE_LINK_KEY_COUNT];
    uint8_t adc_error;
    uint8_t sequence;
} remote_link_data_t;

extern volatile remote_link_data_t remote_link_data;
extern volatile uint32_t remote_link_valid_frames;
extern volatile uint32_t remote_link_crc_errors;
extern volatile uint32_t remote_link_lost_frames;
extern volatile uint32_t remote_link_uart_errors;
extern volatile uint32_t remote_link_rx_bytes;
extern volatile uint32_t remote_link_rx_callbacks;
extern volatile uint32_t remote_link_rx_arm_errors;
extern volatile uint32_t remote_link_forward_errors;
extern volatile uint32_t remote_link_lora_config_attempts;
extern volatile remote_link_lora_config_status_t remote_link_lora_config_status;
extern volatile uint8_t remote_link_lora_config_readback[REMOTE_LINK_LORA_CONFIG_SIZE];

void RemoteLink_Init(void);
void RemoteLink_Process(void);
void RemoteLink_ForwardRawFrame(void);
uint8_t RemoteLink_LoRaReady(void);
uint8_t RemoteLink_IsConnected(void);
uint8_t RemoteLink_GetSnapshot(remote_link_data_t *snapshot);

#endif
