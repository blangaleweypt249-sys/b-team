#ifndef UPPER_PC_LINK_H
#define UPPER_PC_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pc_protocol.h"
#include "upper_robot.h"

#define UPPER_PC_CMD_PAYLOAD_SIZE 34U

typedef void (*upper_pc_cmd_handler_t)(const upper_target_t *target,
                                       void *user_data);
typedef void (*upper_pc_estop_handler_t)(void *user_data);

typedef struct
{
    pc_parser_t parser;
    upper_pc_cmd_handler_t cmd_handler;
    upper_pc_estop_handler_t estop_handler;
    void *user_data;
    volatile uint32_t last_rx_tick_ms;
    volatile bool remote_active;
    uint32_t current_rx_tick_ms;
    uint16_t last_rx_sequence;
    uint16_t tx_sequence;
    uint32_t command_error_count;
} upper_pc_link_t;

void UpperPcLink_Init(upper_pc_link_t *link,
                      upper_pc_cmd_handler_t cmd_handler,
                      upper_pc_estop_handler_t estop_handler,
                      void *user_data);
void UpperPcLink_Push(upper_pc_link_t *link,
                      const uint8_t *data,
                      size_t size,
                      uint32_t tick_ms);
bool UpperPcLink_IsTimedOut(const upper_pc_link_t *link, uint32_t tick_ms);
size_t UpperPcLink_BuildState(upper_pc_link_t *link,
                              const upper_robot_t *robot,
                              uint32_t tick_ms,
                              uint8_t *output,
                              size_t output_size);

#endif
