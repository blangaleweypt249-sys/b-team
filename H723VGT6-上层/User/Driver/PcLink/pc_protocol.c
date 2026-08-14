#include "pc_protocol.h"

#include <string.h>

#define PC_SYNC_0       0xA5U
#define PC_SYNC_1       0x5AU
#define PC_HEADER_SIZE  8U
#define PC_CRC_SIZE     2U

/* 功能：从字节流读取小端 16 位整数；用途：解析上位机协议字段；返回值表示解码结果。 */
static uint16_t PcProtocol_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/* 功能：将 16 位整数写成小端字节；用途：编码上位机协议字段；无返回值表示结果写入 data。 */
static void PcProtocol_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

/* 功能：计算 Modbus 多项式形式的 CRC16；用途：校验上位机帧完整性；返回值表示校验码。 */
uint16_t PcProtocol_Crc16(const uint8_t *data, size_t size)
{
    uint16_t crc;
    size_t index;

    crc = 0xFFFFU;
    for (index = 0U; index < size; index++)
    {
        uint8_t bit;

        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 1U) != 0U ?
                  (uint16_t)((crc >> 1U) ^ 0xA001U) :
                  (uint16_t)(crc >> 1U);
        }
    }

    return crc;
}

/* 功能：清零并初始化流式协议解析器；用途：开始接收新会话；无返回值表示解析状态被复位。 */
void PcProtocol_Init(pc_parser_t *parser)
{
    if (parser != NULL)
    {
        (void)memset(parser, 0, sizeof(*parser));
    }
}

/* 功能：编码一帧完整的上位机协议数据；用途：生成同步头、帧头、载荷和 CRC；返回 0 表示参数或缓冲区无效。 */
size_t PcProtocol_Encode(uint8_t type,
                         uint16_t sequence,
                         const uint8_t *payload,
                         uint16_t payload_len,
                         uint8_t *output,
                         size_t output_size)
{
    size_t frame_size;
    uint16_t crc;

    frame_size = (size_t)payload_len + PC_PROTOCOL_FRAME_OVERHEAD;
    if ((output == NULL) || (payload_len > PC_PROTOCOL_MAX_PAYLOAD) ||
        ((payload_len > 0U) && (payload == NULL)) ||
        (output_size < frame_size))
    {
        return 0U;
    }

    output[0] = PC_SYNC_0;
    output[1] = PC_SYNC_1;
    output[2] = PC_PROTOCOL_VERSION;
    output[3] = type;
    PcProtocol_WriteU16(&output[4], sequence);
    PcProtocol_WriteU16(&output[6], payload_len);
    if (payload_len > 0U)
    {
        (void)memcpy(&output[PC_HEADER_SIZE], payload, payload_len);
    }

    crc = PcProtocol_Crc16(&output[2], 6U + payload_len);
    PcProtocol_WriteU16(&output[PC_HEADER_SIZE + payload_len], crc);
    return frame_size;
}

/* 功能：复位协议解析进度并尝试保留新的同步首字节；用途：错误后快速重新同步；无返回值表示状态已更新。 */
static void PcProtocol_Reset(pc_parser_t *parser, uint8_t last_byte)
{
    parser->received = 0U;
    parser->expected = 0U;
    if (last_byte == PC_SYNC_0)
    {
        parser->buffer[0] = last_byte;
        parser->received = 1U;
    }
}

/* 功能：校验并提交一帧已收齐的数据；用途：检查 CRC 后调用业务回调；失败时更新错误计数并重新同步。 */
static void PcProtocol_Deliver(pc_parser_t *parser,
                               pc_frame_handler_t handler,
                               void *user_data)
{
    pc_frame_t frame;
    uint16_t payload_len;
    uint16_t received_crc;
    uint16_t calculated_crc;

    payload_len = PcProtocol_ReadU16(&parser->buffer[6]);
    received_crc = PcProtocol_ReadU16(&parser->buffer[PC_HEADER_SIZE +
                                                       payload_len]);
    calculated_crc = PcProtocol_Crc16(&parser->buffer[2], 6U + payload_len);
    if (received_crc != calculated_crc)
    {
        parser->crc_error_count++;
        PcProtocol_Reset(parser, parser->buffer[parser->received - 1U]);
        return;
    }

    frame.type = parser->buffer[3];
    frame.sequence = PcProtocol_ReadU16(&parser->buffer[4]);
    frame.payload_len = payload_len;
    if (payload_len > 0U)
    {
        (void)memcpy(frame.payload, &parser->buffer[PC_HEADER_SIZE], payload_len);
    }

    parser->valid_count++;
    parser->received = 0U;
    parser->expected = 0U;
    if (handler != NULL)
    {
        handler(&frame, user_data);
    }
}

/* 功能：逐字节推进上位机协议解析；用途：从任意长度的数据块中识别完整帧；有效帧通过 handler 回调交付。 */
void PcProtocol_Push(pc_parser_t *parser,
                     const uint8_t *data,
                     size_t size,
                     pc_frame_handler_t handler,
                     void *user_data)
{
    size_t index;

    if ((parser == NULL) || (data == NULL))
    {
        return;
    }

    for (index = 0U; index < size; index++)
    {
        uint8_t byte;

        byte = data[index];
        if ((parser->received == 0U) && (byte != PC_SYNC_0))
        {
            continue;
        }

        if ((parser->received == 1U) && (byte != PC_SYNC_1))
        {
            PcProtocol_Reset(parser, byte);
            continue;
        }

        parser->buffer[parser->received++] = byte;
        if (parser->received == PC_HEADER_SIZE)
        {
            uint16_t payload_len;

            payload_len = PcProtocol_ReadU16(&parser->buffer[6]);
            if ((parser->buffer[2] != PC_PROTOCOL_VERSION) ||
                (payload_len > PC_PROTOCOL_MAX_PAYLOAD))
            {
                parser->length_error_count++;
                PcProtocol_Reset(parser, byte);
                continue;
            }
            parser->expected = (uint16_t)(PC_PROTOCOL_FRAME_OVERHEAD +
                                          payload_len);
        }

        if ((parser->expected > 0U) &&
            (parser->received == parser->expected))
        {
            PcProtocol_Deliver(parser, handler, user_data);
        }
    }
}
