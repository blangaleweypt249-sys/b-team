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
constexpr std::size_t k_checksum_offset = 29;
constexpr std::size_t k_tail_0_offset = 30;
constexpr std::size_t k_tail_1_offset = 31;

void SerialProtocol_WriteFloat(
  std::array<uint8_t, k_tx_frame_size> * frame,
  std::size_t offset,
  float value)
{
  // 使用 memcpy 避免通过指针类型转换访问 float 产生未定义行为。
  std::memcpy(frame->data() + offset, &value, sizeof(value));
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

  SerialProtocol_WriteFloat(&frame, k_payload_offset, data.field_x_m);
  SerialProtocol_WriteFloat(&frame, k_payload_offset + 4, data.field_y_m);
  SerialProtocol_WriteFloat(&frame, k_payload_offset + 8, data.field_z_m);
  SerialProtocol_WriteFloat(&frame, k_payload_offset + 12, data.target_x_m);
  SerialProtocol_WriteFloat(&frame, k_payload_offset + 16, data.target_y_m);
  SerialProtocol_WriteFloat(&frame, k_payload_offset + 20, data.target_z_m);

  // 校验范围不含帧头、校验位和帧尾，便于下位机使用同一算法验证。
  frame[k_checksum_offset] = SerialProtocol_Checksum(
    frame.data() + k_type_offset,
    k_checksum_offset - k_type_offset);
  frame[k_tail_0_offset] = k_frame_tail_0;
  frame[k_tail_1_offset] = k_frame_tail_1;
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