#include "competition_gateway/serial_protocol.hpp"

#include <gtest/gtest.h>

namespace competition_gateway
{

TEST(SerialProtocolTest, CRC16DetectsSingleByteChange)
{
  std::array<uint8_t, 4> data = {0x01, 0x02, 0x03, 0x04};
  const auto crc_original = SerialProtocol_CRC16(data.data(), data.size());

  data[2] = 0x05;
  const auto crc_modified = SerialProtocol_CRC16(data.data(), data.size());

  EXPECT_NE(crc_original, crc_modified);
}

TEST(SerialProtocolTest, CRC16DetectsByteSwap)
{
  std::array<uint8_t, 4> a = {0x01, 0x02, 0x03, 0x04};
  std::array<uint8_t, 4> b = {0x02, 0x01, 0x03, 0x04};

  EXPECT_NE(SerialProtocol_CRC16(a.data(), a.size()), SerialProtocol_CRC16(b.data(), b.size()));
}

TEST(SerialProtocolTest, EncodePerceptionCreatesValidFrame)
{
  perception_data_t data{};
  data.red_x_m = 1.5F;
  data.red_y_m = 2.0F;
  data.red_z_m = 0.3F;
  data.blue_x_m = -1.0F;
  data.blue_y_m = 0.5F;
  data.blue_z_m = 0.2F;
  data.ball_x_m = 3.0F;
  data.ball_y_m = -2.0F;
  data.ball_z_m = 0.1F;
  data.flags = PERCEPTION_RED_VALID | PERCEPTION_BLUE_VALID | PERCEPTION_BALL_VALID;
  data.timestamp_ms = 123456U;

  const auto frame = SerialProtocol_EncodePerception(data, 7U);

  EXPECT_EQ(frame[0], k_tx_header_0);
  EXPECT_EQ(frame[1], k_tx_header_1);
  EXPECT_EQ(frame[2], k_tx_perception_type);
  EXPECT_EQ(frame[3], 7U);
  EXPECT_EQ(frame[4], data.flags);
  EXPECT_EQ(frame.size(), k_tx_frame_size);
  EXPECT_EQ(frame[k_tx_frame_size - 2], k_frame_tail_0);
  EXPECT_EQ(frame[k_tx_frame_size - 1], k_frame_tail_1);

  // CRC16 校验
  uint16_t expected_crc = SerialProtocol_CRC16(frame.data() + 2, 43);
  uint16_t actual_crc = 0U;
  std::memcpy(&actual_crc, frame.data() + 45, sizeof(actual_crc));
  EXPECT_EQ(expected_crc, actual_crc);
}

TEST(SerialProtocolTest, EncodePositionCreatesValidFrame)
{
  position_data_t data{};
  data.field_x_m = 5.0F;
  data.field_y_m = 3.0F;
  data.field_z_m = 0.0F;
  data.field_yaw = 1.57F;
  data.flags = POSITION_FIELD_VALID;
  data.timestamp_ms = 999U;

  const auto frame = SerialProtocol_EncodePosition(data, 3U);

  EXPECT_EQ(frame[0], k_tx_header_0);
  EXPECT_EQ(frame[1], k_tx_header_1);
  EXPECT_EQ(frame[2], k_tx_position_type);
  EXPECT_EQ(frame[3], 3U);
  EXPECT_EQ(frame[4], data.flags);
  EXPECT_EQ(frame.size(), k_tx_position_frame_size);
  EXPECT_EQ(frame[k_tx_position_frame_size - 2], k_frame_tail_0);
  EXPECT_EQ(frame[k_tx_position_frame_size - 1], k_frame_tail_1);

  // CRC16 校验
  uint16_t expected_crc = SerialProtocol_CRC16(frame.data() + 2, 23);
  uint16_t actual_crc = 0U;
  std::memcpy(&actual_crc, frame.data() + 25, sizeof(actual_crc));
  EXPECT_EQ(expected_crc, actual_crc);
}

TEST(SerialProtocolTest, DecodeStatusRejectsBadCRC)
{
  std::array<uint8_t, k_rx_status_frame_size> frame{
    k_rx_header_0, k_rx_header_1, k_rx_status_type, 2U, 3U, 0U, 0U, k_frame_tail_0, k_frame_tail_1};
  controller_status_t status{};

  EXPECT_FALSE(SerialProtocol_DecodeStatus(frame, &status));

  const uint16_t crc = SerialProtocol_CRC16(frame.data() + 2, 3);
  std::memcpy(frame.data() + 5, &crc, sizeof(crc));
  EXPECT_TRUE(SerialProtocol_DecodeStatus(frame, &status));
  EXPECT_EQ(status.state, 2U);
  EXPECT_EQ(status.error, 3U);
}

}  // namespace competition_gateway
