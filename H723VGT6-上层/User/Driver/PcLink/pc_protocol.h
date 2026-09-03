/**
 * @file pc_protocol.h
 * @brief 定义 PC 链路协议帧、解析器和编解码接口。
 */

#ifndef PC_PROTOCOL_H
#define PC_PROTOCOL_H /**< 防止 pc_protocol.h 被重复包含。 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PC_PROTOCOL_VERSION       3U /**< 当前上位机通信帧格式的版本号。 */
#define PC_PROTOCOL_MAX_PAYLOAD   128U /**< 单个上位机通信帧允许携带的最大载荷字节数。 */
#define PC_PROTOCOL_FRAME_OVERHEAD 10U /**< 上位机通信帧中同步头、帧头和校验字段占用的字节数。 */

/** 表示上位机通信帧承载的消息种类。 */
typedef enum
{
    PC_MSG_HEARTBEAT = 0x01, /**< 上位机维持通信在线状态的心跳消息。 */
    PC_MSG_ESTOP = 0x02, /**< 上位机请求机器人立即停止输出的急停消息。 */
    PC_MSG_HANDSHAKE = 0x03, /**< 建立上位机控制会话的握手消息。 */
    PC_MSG_UPPER_CMD = 0x10, /**< 发送给上层控制板的基础控制命令。 */
    PC_MSG_LOWER_CMD = 0x11, /**< 发送给底盘控制板的基础控制命令。 */
    /* 位置目标支持旧版 34 字节布局和扩展版 122 字节布局。 */
    PC_MSG_UPPER_POSITION_CMD = 0x12, /**< 发送机构位置、速度和 PID 目标的上层命令。 */
    PC_MSG_MOTOR_ACTION = 0x13, /**< 请求电机使能、置零或自动回位的动作命令。 */
    PC_MSG_FLASH_INFO_REQUEST = 0x14, /**< 请求读取外部 Flash 器件信息。 */
    PC_MSG_AUX_CONTROL = 0x15, /**< 更新辅助控制板输出位的命令。 */
    PC_MSG_ROBOT_STATE = 0x20, /**< 上层控制板主动上报的机器人状态。 */
    PC_MSG_MOTOR_ACTION_RESULT = 0x21, /**< 电机动作执行结果的响应消息。 */
    PC_MSG_DJI_TELEMETRY = 0x22, /**< DJI 电机反馈和零点诊断遥测。 */
    PC_MSG_FLASH_INFO = 0x23, /**< 外部 Flash 初始化状态和容量信息。 */
    PC_MSG_ACK = 0x7E, /**< 对握手或控制请求的确认消息。 */
    PC_MSG_FAULT = 0x7F /**< 电机或控制流程检测到的故障消息。 */
} pc_msg_type_t;

/** 保存一帧解析完成或等待编码的上位机消息。 */
typedef struct
{
    uint8_t type; /**< 该帧载荷所表达的上位机消息种类。 */
    uint16_t sequence; /**< 当前协议帧携带的消息序号。 */
    uint16_t payload_len; /**< 该帧实际携带的载荷字节数。 */
    uint8_t payload[PC_PROTOCOL_MAX_PAYLOAD]; /**< 该帧携带的业务数据。 */
} pc_frame_t;

typedef void (*pc_frame_handler_t)(const pc_frame_t *frame /**< 待解析的上位机协议帧 */, void *user_data /**< 调用回调函数时传递的用户上下文 */);

/** 保存 上位机链路 运行过程中需要集中管理的数据。 */
typedef struct
{
    uint8_t buffer[PC_PROTOCOL_MAX_PAYLOAD + PC_PROTOCOL_FRAME_OVERHEAD]; /**< 用于暂存尚未完成解析的数据字节。 */
    uint16_t received; /**< 当前已接收的数据字节数。 */
    uint16_t expected; /**< 当前帧解析完成所需的总字节数。 */
    uint32_t valid_count; /**< 协议解析器累计交付的有效帧数量。 */
    uint32_t crc_error_count; /**< 累计 CRC 校验失败的数据帧数量。 */
    uint32_t length_error_count; /**< 累计因载荷长度超限而拒绝的数据帧数量。 */
} pc_parser_t;

/* 功能：清零并初始化流式协议解析器；用途：开始接收新会话；无返回值表示解析状态被复位。 */
void PcProtocol_Init(pc_parser_t *parser /**< 需要操作的协议解析器 */);
/* 功能：编码一帧完整的上位机协议数据；用途：生成同步头、帧头、载荷和 CRC；返回 0 表示参数或缓冲区无效。 */
size_t PcProtocol_Encode(uint8_t type /**< 待编码的上位机协议消息类型 */,
                         uint16_t sequence /**< 用于匹配请求和响应的消息序号 */,
                         const uint8_t *payload /**< 待封装进上位机帧的载荷首地址 */,
                         uint16_t payload_len /**< 协议帧实际携带的载荷字节数 */,
                         uint8_t *output /**< 用于写出编码后协议帧的缓冲区 */,
                         size_t output_size /**< 输出缓冲区可用的字节数 */);
/* 功能：逐字节推进上位机协议解析；用途：从任意长度的数据块中识别完整帧；有效帧通过 handler 回调交付。 */
void PcProtocol_Push(pc_parser_t *parser /**< 需要操作的协议解析器 */,
                     const uint8_t *data /**< 待送入上位机协议解析器的原始字节流 */,
                     size_t size /**< 本次送入协议解析器的字节数 */,
                     pc_frame_handler_t handler /**< 收到有效数据后调用的处理函数 */,
                     void *user_data /**< 调用回调函数时传递的用户上下文 */);
/* 功能：计算 Modbus 多项式形式的 CRC16；用途：校验上位机帧完整性；返回值表示校验码。 */
uint16_t PcProtocol_Crc16(const uint8_t *data /**< 待计算CRC16的上位机协议数据 */, size_t size /**< 参与CRC16计算的数据字节数 */);

#ifdef __cplusplus
}
#endif

#endif
