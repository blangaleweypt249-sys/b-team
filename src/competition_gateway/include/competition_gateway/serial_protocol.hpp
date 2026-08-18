#ifndef COMPETITION_GATEWAY__SERIAL_PROTOCOL_HPP_
#define COMPETITION_GATEWAY__SERIAL_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace competition_gateway
{

constexpr uint8_t k_tx_header_0 = 0xAA;
constexpr uint8_t k_tx_header_1 = 0x55;
constexpr uint8_t k_rx_header_0 = 0x55;
constexpr uint8_t k_rx_header_1 = 0xAA;
constexpr uint8_t k_frame_tail_0 = 0x0D;
constexpr uint8_t k_frame_tail_1 = 0x0A;
constexpr uint8_t k_tx_perception_type = 0x10;
constexpr uint8_t k_tx_position_type = 0x11;
constexpr uint8_t k_rx_status_type = 0x20;

// 帧大小: header(2) + type(1) + seq(1) + flags(1) + timestamp(4) + payload + crc16(2) + tail(2)
constexpr std::size_t k_tx_frame_size = 37;          // 感知帧: 6 floats = 24 bytes payload
constexpr std::size_t k_tx_position_frame_size = 29;  // 位置帧: 4 floats = 16 bytes payload
constexpr std::size_t k_rx_status_frame_size = 9;     // 状态帧: state(1) + error(1) payload, 无 timestamp

// 感知帧有效位
enum perception_flag_t : uint8_t
{
  PERCEPTION_BLOCK_VALID = 1U << 0,  // 当前跟踪块有效
  PERCEPTION_BALL_VALID = 1U << 2,
};

// 位置帧有效位
enum position_flag_t : uint8_t
{
  POSITION_FIELD_VALID = 1U << 0,
};

// 感知帧数据: 当前跟踪块 + 球位置, 坐标单位为米
struct perception_data_t
{
  float block_x_m = 0.0F;
  float block_y_m = 0.0F;
  float block_z_m = 0.0F;
  float ball_x_m = 0.0F;
  float ball_y_m = 0.0F;
  float ball_z_m = 0.0F;
  uint8_t flags = 0U;
  uint32_t timestamp_ms = 0U;  // 数据生成时间戳（毫秒）
};

// 位置帧数据: 机器人赛场坐标 + 偏航角
struct position_data_t
{
  float field_x_m = 0.0F;
  float field_y_m = 0.0F;
  float field_z_m = 0.0F;
  float field_yaw = 0.0F;
  uint8_t flags = 0U;
  uint32_t timestamp_ms = 0U;  // 数据生成时间戳（毫秒）
};

struct controller_status_t
{
  uint8_t state = 0U;
  uint8_t error = 0U;
};

/**
 * @brief 计算 CRC16-CCITT (多项式 0x1021, 初始值 0xFFFF)。
 */
uint16_t SerialProtocol_CRC16(const uint8_t * data, std::size_t size);

/**
 * @brief 将当前跟踪块和球位置编码为感知帧 (37 字节)。
 */
std::array<uint8_t, k_tx_frame_size> SerialProtocol_EncodePerception(
  const perception_data_t & data,
  uint8_t sequence);

/**
 * @brief 将机器人位置和偏航角编码为位置帧 (29 字节)。
 */
std::array<uint8_t, k_tx_position_frame_size> SerialProtocol_EncodePosition(
  const position_data_t & data,
  uint8_t sequence);

/**
 * @brief 校验并解析下位机发送的状态帧。
 */
bool SerialProtocol_DecodeStatus(
  const std::array<uint8_t, k_rx_status_frame_size> & frame,
  controller_status_t * status);

}  // namespace competition_gateway

#endif  // COMPETITION_GATEWAY__SERIAL_PROTOCOL_HPP_
