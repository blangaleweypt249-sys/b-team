#ifndef UPPER_PC_LINK_H
#define UPPER_PC_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pc_protocol.h"
#include "upper_motor_port.h"
#include "upper_robot.h"

#define UPPER_PC_CMD_PAYLOAD_SIZE 34U
#define UPPER_PC_POSITION_CMD_PAYLOAD_SIZE 34U
#define UPPER_PC_POSITION_TORQUE_CMD_PAYLOAD_SIZE 42U
#define UPPER_PC_EXTENDED_POSITION_CMD_PAYLOAD_SIZE 122U
#define UPPER_PC_HANDSHAKE_PAYLOAD_SIZE 4U
#define UPPER_PC_MOTOR_ACTION_PAYLOAD_SIZE 3U
#define UPPER_PC_MOTOR_CONFIG_ACTION_PAYLOAD_SIZE 4U
#define UPPER_PC_FLASH_INFO_PAYLOAD_SIZE 20U
#define UPPER_PC_DJI_DIAGNOSTIC_COUNT 4U
#define UPPER_PC_FDCAN_COUNT 3U

#define UPPER_PC_ACTION_J4310_SAVE_ZERO 1U
#define UPPER_PC_ACTION_J4310_AUTO_RETURN 2U

typedef struct
{
    bool storage_ready;
    bool enabled;
    bool active;
    uint8_t stage;
} upper_j4310_auto_return_status_t;

typedef void (*upper_pc_cmd_handler_t)(const upper_target_t *target,
                                       void *user_data);
typedef void (*upper_pc_estop_handler_t)(void *user_data);
typedef void (*upper_pc_motor_action_handler_t)(uint8_t action,
                                                uint8_t can_bus,
                                                uint8_t node_id,
                                                uint8_t value,
                                                void *user_data);

typedef struct
{
    pc_parser_t parser;
    upper_pc_cmd_handler_t cmd_handler;
    upper_pc_estop_handler_t estop_handler;
    upper_pc_motor_action_handler_t motor_action_handler;
    void *user_data;
    volatile uint32_t last_rx_tick_ms;
    volatile bool remote_active;
    volatile bool session_active;
    volatile bool remote_timeout_pending;
    volatile bool handshake_pending;
    volatile uint16_t handshake_sequence;
    volatile uint32_t handshake_received_tick_ms;
    volatile uint32_t handshake_received_count;
    volatile bool flash_info_pending;
    volatile uint16_t flash_info_sequence;
    uint32_t current_rx_tick_ms;
    uint16_t last_rx_sequence;
    uint16_t tx_sequence;
    uint32_t command_error_count;
} upper_pc_link_t;

void UpperPcLink_Init(upper_pc_link_t *link,
                      upper_pc_cmd_handler_t cmd_handler,
                      upper_pc_estop_handler_t estop_handler,
                      upper_pc_motor_action_handler_t motor_action_handler,
                      void *user_data);
void UpperPcLink_Push(upper_pc_link_t *link,
                      const uint8_t *data,
                      size_t size,
                      uint32_t tick_ms);
bool UpperPcLink_IsTimedOut(upper_pc_link_t *link, uint32_t tick_ms);
bool UpperPcLink_IsSessionActive(upper_pc_link_t *link, uint32_t tick_ms);
bool UpperPcLink_HasHandshakePending(const upper_pc_link_t *link);
bool UpperPcLink_IsHandshakeAckDue(const upper_pc_link_t *link,
                                   uint32_t tick_ms,
                                   uint32_t guard_ms);
uint16_t UpperPcLink_GetHandshakeSequence(const upper_pc_link_t *link);
size_t UpperPcLink_BuildHandshakeAck(const upper_pc_link_t *link,
                                     uint8_t *output,
                                     size_t output_size);
void UpperPcLink_MarkHandshakeAckSent(upper_pc_link_t *link,
                                      uint16_t sequence);
bool UpperPcLink_HasFlashInfoPending(const upper_pc_link_t *link);
uint16_t UpperPcLink_GetFlashInfoSequence(const upper_pc_link_t *link);
size_t UpperPcLink_BuildFlashInfo(const upper_pc_link_t *link,
                                  uint8_t init_status,
                                  bool initialized,
                                  uint32_t jedec_id,
                                  uint32_t capacity_kb,
                                  uint32_t sector_count,
                                  uint16_t page_size_byte,
                                  uint32_t sector_size_byte,
                                  uint8_t *output,
                                  size_t output_size);
void UpperPcLink_MarkFlashInfoSent(upper_pc_link_t *link,
                                   uint16_t sequence);
size_t UpperPcLink_BuildState(upper_pc_link_t *link,
                              const upper_robot_t *robot,
                              uint32_t tick_ms,
                              bool j4310_position_valid,
                              float j4310_position_rad,
                              uint32_t j4310_bus_rx_frames,
                              const upper_j4310_rx_diagnostic_t *j4310_rx,
                              const upper_j4310_tx_diagnostic_t *j4310_tx,
                              const upper_j4310_auto_return_status_t *
                                  j4310_auto_return,
                              uint8_t *output,
                              size_t output_size);
size_t UpperPcLink_BuildDjiTelemetry(
                              upper_pc_link_t *link,
                              const upper_dji_diagnostic_t *diagnostic,
                              const uint32_t fdcan_rx_count[UPPER_PC_FDCAN_COUNT],
                              uint8_t *output,
                              size_t output_size);
size_t UpperPcLink_BuildMotorActionResult(upper_pc_link_t *link,
                                          uint8_t action,
                                          uint8_t can_bus,
                                          uint8_t node_id,
                                          uint8_t status,
                                          uint32_t tick_ms,
                                          uint8_t *output,
                                          size_t output_size);
size_t UpperPcLink_BuildMotorFault(upper_pc_link_t *link,
                                   uint8_t model,
                                   uint8_t can_bus,
                                   uint8_t node_id,
                                   uint8_t error_code,
                                   uint32_t tick_ms,
                                   uint8_t *output,
                                   size_t output_size);

#endif
