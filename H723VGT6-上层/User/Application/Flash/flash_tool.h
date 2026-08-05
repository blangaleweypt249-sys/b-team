#ifndef FLASH_TOOL_H
#define FLASH_TOOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Initialize the W25Qxx port and make UART4 the Flash text console.
 */
void FlashTool_Init(void);

/**
 * @brief Send the startup status through UART4 without waiting for a command.
 * @param comm_ready true when the communication runtime initialized normally.
 */
void FlashTool_SendReady(bool comm_ready);

/**
 * @brief Route UART4 data to the Flash text protocol when it is active.
 * @retval true The data belongs to the Flash text protocol.
 * @retval false The data should continue to the normal PC protocol.
 */
bool FlashTool_Receive(const uint8_t *data, size_t size);

/**
 * @brief Return whether UART4 is currently owned by the Flash text tool.
 */
bool FlashTool_IsActive(void);

#endif
