#include "competition_gateway/serial_protocol.hpp"

#include <gtest/gtest.h>

namespace competition_gateway
{

TEST(SerialProtocolTest, EncodePerceptionCreatesValidFrame)
{
  perception_data_t data{};
  data.field_x_m = 1.25F;
  data.target_z_m = 0.75F;
  data.flags = PERCEPTION_FIELD_POSE_VALID | PERCEPTION_TARGET_VALID;

  const auto frame = SerialProtocol_EncodePerception(data, 7U);

  EXPECT_EQ(frame[0], k_tx_header_0);
  EXPECT_EQ(frame[1], k_tx_header_1);
  EXPECT_EQ(frame[2], k_tx_perception_type);
  EXPECT_EQ(frame[3], 7U);
  EXPECT_EQ(frame[4], data.flags);
  EXPECT_EQ(frame[30], k_frame_tail_0);
  EXPECT_EQ(frame[31], k_frame_tail_1);
  EXPECT_EQ(frame[29], SerialProtocol_Checksum(frame.data() + 2, 27));
}

TEST(SerialProtocolTest, DecodeStatusRejectsBadChecksum)
{
  std::array<uint8_t, k_rx_status_frame_size> frame{
    k_rx_header_0, k_rx_header_1, k_rx_status_type, 2U, 3U, 0U, k_frame_tail_0, k_frame_tail_1};
  controller_status_t status{};

  EXPECT_FALSE(SerialProtocol_DecodeStatus(frame, &status));

  frame[5] = SerialProtocol_Checksum(frame.data() + 2, 3);
  EXPECT_TRUE(SerialProtocol_DecodeStatus(frame, &status));
  EXPECT_EQ(status.state, 2U);
  EXPECT_EQ(status.error, 3U);
}

}  // namespace competition_gateway