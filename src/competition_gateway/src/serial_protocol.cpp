#include "competition_gateway/serial_protocol.hpp"

#include <cstring>

namespace competition_gateway
{

namespace
{

constexpr std::size_t k_type_offset = 2;
constexpr std::size_t k_sequence_offset = 3;
constexpr std::size_t k_flags_offset = 4;
constexpr std::size_t k_timestamp_offset = 5;   // 4 bytes, uint32_t
constexpr std::size_t k_payload_offset = 9;     // 数据紧跟 timestamp 之后

// 感知帧 (49 字节) 偏移
constexpr std::size_t k_perception_crc_offset = 45;   // 2 bytes CRC16
constexpr std::size_t k_perception_tail_0_offset = 47;
constexpr std::size_t k_perception_tail_1_offset = 48;

// 位置帧 (29 字节) 偏移
constexpr std::size_t k_position_crc_offset = 25;     // 2 bytes CRC16
constexpr std::size_t k_position_tail_0_offset = 27;
constexpr std::size_t k_position_tail_1_offset = 28;

// 状态帧 (9 字节) 偏移
constexpr std::size_t k_status_state_offset = 3;
constexpr std::size_t k_status_error_offset = 4;
constexpr std::size_t k_status_crc_offset = 5;        // 2 bytes CRC16
constexpr std::size_t k_status_tail_0_offset = 7;
constexpr std::size_t k_status_tail_1_offset = 8;

void SerialProtocol_WriteFloat(
  uint8_t * data,
  std::size_t offset,
  float value)
{
  std::memcpy(data + offset, &value, sizeof(value));
}

void SerialProtocol_WriteUInt32(
  uint8_t * data,
  std::size_t offset,
  uint32_t value)
{
  std::memcpy(data + offset, &value, sizeof(value));
}

void SerialProtocol_WriteUInt16(
  uint8_t * data,
  std::size_t offset,
  uint16_t value)
{
  std::memcpy(data + offset, &value, sizeof(value));
}

}  // namespace

uint16_t SerialProtocol_CRC16(const uint8_t * data, std::size_t size)
{
  uint16_t crc = 0xFFFFU;
  for (std::size_t index = 0; index < size; ++index)
  {
    crc ^= static_cast<uint16_t>(data[index]) << 8;
    for (int bit = 0; bit < 8; ++bit)
    {
      if (crc & 0x8000U)
      {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

std::array<uint8_t, k_tx_frame_size> SerialProtocol_EncodePerception(
  const perception_data_t & data,
  uint8_t sequence)
{
  std::array<uint8_t, k_tx_frame_size> frame{};
  frame[0] = k_tx_header_0;
  frame[1] = k_tx_header_1;
  frame[k_type_offset] = k_tx_perception_type;
  frame[k_sequence_offset] = sequence;
  frame[k_flags_offset] = data.flags;
  SerialProtocol_WriteUInt32(frame.data(), k_timestamp_offset, data.timestamp_ms);

  // 块在球前面
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset, data.red_x_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 4, data.red_y_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 8, data.red_z_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 12, data.blue_x_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 16, data.blue_y_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 20, data.blue_z_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 24, data.ball_x_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 28, data.ball_y_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 32, data.ball_z_m);

  // CRC16 覆盖 type 到最后一个数据字节
  const uint16_t crc = SerialProtocol_CRC16(
    frame.data() + k_type_offset,
    k_perception_crc_offset - k_type_offset);
  SerialProtocol_WriteUInt16(frame.data(), k_perception_crc_offset, crc);
  frame[k_perception_tail_0_offset] = k_frame_tail_0;
  frame[k_perception_tail_1_offset] = k_frame_tail_1;
  return frame;
}

std::array<uint8_t, k_tx_position_frame_size> SerialProtocol_EncodePosition(
  const position_data_t & data,
  uint8_t sequence)
{
  std::array<uint8_t, k_tx_position_frame_size> frame{};
  frame[0] = k_tx_header_0;
  frame[1] = k_tx_header_1;
  frame[k_type_offset] = k_tx_position_type;
  frame[k_sequence_offset] = sequence;
  frame[k_flags_offset] = data.flags;
  SerialProtocol_WriteUInt32(frame.data(), k_timestamp_offset, data.timestamp_ms);

  SerialProtocol_WriteFloat(frame.data(), k_payload_offset, data.field_x_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 4, data.field_y_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 8, data.field_z_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 12, data.field_yaw);

  const uint16_t crc = SerialProtocol_CRC16(
    frame.data() + k_type_offset,
    k_position_crc_offset - k_type_offset);
  SerialProtocol_WriteUInt16(frame.data(), k_position_crc_offset, crc);
  frame[k_position_tail_0_offset] = k_frame_tail_0;
  frame[k_position_tail_1_offset] = k_frame_tail_1;
  return frame;
}

bool SerialProtocol_DecodeStatus(
  const std::array<uint8_t, k_rx_status_frame_size> & frame,
  controller_status_t * status)
{
  if (status == nullptr || frame[0] != k_rx_header_0 || frame[1] != k_rx_header_1 ||
    frame[k_type_offset] != k_rx_status_type || frame[k_status_tail_0_offset] != k_frame_tail_0 ||
    frame[k_status_tail_1_offset] != k_frame_tail_1)
  {
    return false;
  }

  // CRC16 覆盖 type 到 error
  uint16_t expected_crc = 0U;
  std::memcpy(&expected_crc, frame.data() + k_status_crc_offset, sizeof(expected_crc));
  const uint16_t actual_crc = SerialProtocol_CRC16(
    frame.data() + k_type_offset,
    k_status_crc_offset - k_type_offset);
  if (expected_crc != actual_crc)
  {
    return false;
  }

  status->state = frame[k_status_state_offset];
  status->error = frame[k_status_error_offset];
  return true;
}

}  // namespace competition_gateway
