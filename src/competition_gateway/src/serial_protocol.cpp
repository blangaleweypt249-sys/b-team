#include "competition_gateway/serial_protocol.hpp"

#include <cstring>

namespace competition_gateway
{

namespace
{

constexpr std::size_t k_type_offset = 2;
constexpr std::size_t k_sequence_offset = 3;
constexpr std::size_t k_flags_offset = 4;
constexpr std::size_t k_payload_offset = 5;

// 感知帧 (44 字节) 偏移
constexpr std::size_t k_perception_checksum_offset = 41;
constexpr std::size_t k_perception_tail_0_offset = 42;
constexpr std::size_t k_perception_tail_1_offset = 43;

// 位置帧 (24 字节) 偏移
constexpr std::size_t k_position_checksum_offset = 21;
constexpr std::size_t k_position_tail_0_offset = 22;
constexpr std::size_t k_position_tail_1_offset = 23;

void SerialProtocol_WriteFloat(
  uint8_t * data,
  std::size_t offset,
  float value)
{
  std::memcpy(data + offset, &value, sizeof(value));
}

}  // namespace

uint8_t SerialProtocol_Checksum(const uint8_t * data, std::size_t size)
{
  uint8_t checksum = 0U;
  for (std::size_t index = 0; index < size; ++index)
  {
    checksum = static_cast<uint8_t>(checksum + data[index]);
  }
  return checksum;
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

  frame[k_perception_checksum_offset] = SerialProtocol_Checksum(
    frame.data() + k_type_offset,
    k_perception_checksum_offset - k_type_offset);
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

  SerialProtocol_WriteFloat(frame.data(), k_payload_offset, data.field_x_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 4, data.field_y_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 8, data.field_z_m);
  SerialProtocol_WriteFloat(frame.data(), k_payload_offset + 12, data.field_yaw);

  frame[k_position_checksum_offset] = SerialProtocol_Checksum(
    frame.data() + k_type_offset,
    k_position_checksum_offset - k_type_offset);
  frame[k_position_tail_0_offset] = k_frame_tail_0;
  frame[k_position_tail_1_offset] = k_frame_tail_1;
  return frame;
}

bool SerialProtocol_DecodeStatus(
  const std::array<uint8_t, k_rx_status_frame_size> & frame,
  controller_status_t * status)
{
  constexpr std::size_t k_status_offset = 3;
  constexpr std::size_t k_error_offset = 4;
  constexpr std::size_t k_status_checksum_offset = 5;
  constexpr std::size_t k_status_tail_0_offset = 6;
  constexpr std::size_t k_status_tail_1_offset = 7;

  if (status == nullptr || frame[0] != k_rx_header_0 || frame[1] != k_rx_header_1 ||
    frame[k_type_offset] != k_rx_status_type || frame[k_status_tail_0_offset] != k_frame_tail_0 ||
    frame[k_status_tail_1_offset] != k_frame_tail_1)
  {
    return false;
  }
  if (frame[k_status_checksum_offset] != SerialProtocol_Checksum(
      frame.data() + k_type_offset,
      k_status_checksum_offset - k_type_offset))
  {
    return false;
  }

  status->state = frame[k_status_offset];
  status->error = frame[k_error_offset];
  return true;
}

}  // namespace competition_gateway
