/**
 * @file pc_protocol.h
 * @brief 定义 PC 链路协议帧、解析器和编解码接口。
 */

#ifndef PC_PROTOCOL_H
#define PC_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PC_PROTOCOL_VERSION       3U
#define PC_PROTOCOL_MAX_PAYLOAD   128U
#define PC_PROTOCOL_FRAME_OVERHEAD 10U

typedef enum
{
    PC_MSG_HEARTBEAT = 0x01,
    PC_MSG_ESTOP = 0x02,
    PC_MSG_HANDSHAKE = 0x03,
    PC_MSG_UPPER_CMD = 0x10,
    PC_MSG_LOWER_CMD = 0x11,
    /* 位置目标支持旧版 34 字节布局和扩展版 122 字节布局。 */
    PC_MSG_UPPER_POSITION_CMD = 0x12,
    PC_MSG_MOTOR_ACTION = 0x13,
    PC_MSG_FLASH_INFO_REQUEST = 0x14,
    PC_MSG_AUX_CONTROL = 0x15,
    PC_MSG_ROBOT_STATE = 0x20,
    PC_MSG_MOTOR_ACTION_RESULT = 0x21,
    PC_MSG_DJI_TELEMETRY = 0x22,
    PC_MSG_FLASH_INFO = 0x23,
    PC_MSG_ACK = 0x7E,
    PC_MSG_FAULT = 0x7F
} pc_msg_type_t;

typedef struct
{
    uint8_t type;
    uint16_t sequence;
    uint16_t payload_len;
    uint8_t payload[PC_PROTOCOL_MAX_PAYLOAD];
} pc_frame_t;

typedef void (*pc_frame_handler_t)(const pc_frame_t *frame, void *user_data);

typedef struct
{
    uint8_t buffer[PC_PROTOCOL_MAX_PAYLOAD + PC_PROTOCOL_FRAME_OVERHEAD];
    uint16_t received;
    uint16_t expected;
    uint32_t valid_count;
    uint32_t crc_error_count;
    uint32_t length_error_count;
} pc_parser_t;

/* 功能：清零并初始化流式协议解析器；用途：开始接收新会话；无返回值表示解析状态被复位。 */
void PcProtocol_Init(pc_parser_t *parser);
/* 功能：编码一帧完整的上位机协议数据；用途：生成同步头、帧头、载荷和 CRC；返回 0 表示参数或缓冲区无效。 */
size_t PcProtocol_Encode(uint8_t type,
                         uint16_t sequence,
                         const uint8_t *payload,
                         uint16_t payload_len,
                         uint8_t *output,
                         size_t output_size);
/* 功能：逐字节推进上位机协议解析；用途：从任意长度的数据块中识别完整帧；有效帧通过 handler 回调交付。 */
void PcProtocol_Push(pc_parser_t *parser,
                     const uint8_t *data,
                     size_t size,
                     pc_frame_handler_t handler,
                     void *user_data);
/* 功能：计算 Modbus 多项式形式的 CRC16；用途：校验上位机帧完整性；返回值表示校验码。 */
uint16_t PcProtocol_Crc16(const uint8_t *data, size_t size);

#ifdef __cplusplus
}
#endif

#endif
