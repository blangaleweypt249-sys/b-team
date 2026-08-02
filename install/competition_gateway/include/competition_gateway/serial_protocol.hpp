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
constexpr uint8_t k_rx_status_type = 0x20;
constexpr std::size_t k_tx_frame_size = 32;
constexpr std::size_t k_rx_status_frame_size = 8;

// 每一位独立表示相应载荷未超时；无效数据的 float 值不能被下位机使用。
enum perception_flag_t : uint8_t
{
  PERCEPTION_FIELD_POSE_VALID = 1U << 0,
  PERCEPTION_TARGET_VALID = 1U << 1,
};

// 坐标单位均为米：field 为比赛场地坐标，target 为视觉节点发布的相机坐标。
struct perception_data_t
{
  float field_x_m = 0.0F;
  float field_y_m = 0.0F;
  float field_z_m = 0.0F;
  float target_x_m = 0.0F;
  float target_y_m = 0.0F;
  float target_z_m = 0.0F;
  uint8_t flags = 0U;
};

struct controller_status_t
{
  uint8_t state = 0U;
  uint8_t error = 0U;
};

/**
 * @brief 计算帧中指定数据段的 8 位累加校验和。
 * @param data 待校验数据的首地址。
 * @param size 待校验数据长度。
 * @return 8 位累加校验和。
 */
uint8_t SerialProtocol_Checksum(const uint8_t * data, std::size_t size);

/**
 * @brief 将定位与视觉数据编码为固定长度的下行感知帧。
 * @param data 最新的定位和目标数据。
 * @param sequence 发送序号，供下位机识别重复帧和丢帧。
 * @return 完整的 32 字节串口帧。
 */
std::array<uint8_t, k_tx_frame_size> SerialProtocol_EncodePerception(
  const perception_data_t & data,
  uint8_t sequence);

/**
 * @brief 校验并解析下位机发送的状态帧。
 * @param frame 固定长度状态帧。
 * @param status 解析成功后写入的下位机状态。
 * @return 帧头、类型、校验和和帧尾均合法时返回 true。
 */
bool SerialProtocol_DecodeStatus(
  const std::array<uint8_t, k_rx_status_frame_size> & frame,
  controller_status_t * status);

}  // namespace competition_gateway

#endif  // COMPETITION_GATEWAY__SERIAL_PROTOCOL_HPP_