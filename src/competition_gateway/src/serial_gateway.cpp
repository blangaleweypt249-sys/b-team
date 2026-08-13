#include "competition_gateway/serial_protocol.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace competition_gateway
{

namespace
{

constexpr int k_serial_read_buffer_size = 128;
constexpr int k_serial_data_bits = CS8;

speed_t SerialGateway_GetBaudrate(int baudrate)
{
  switch (baudrate)
  {
    case 9600:
      return B9600;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return B115200;
  }
}

}  // namespace

// 感知帧缓存: 红蓝块 + 球位置
struct perception_cache_t
{
  perception_data_t data;
  rclcpp::Time red_time;
  rclcpp::Time blue_time;
  rclcpp::Time ball_time;
  bool red_received = false;
  bool blue_received = false;
  bool ball_received = false;
};

// 位置帧缓存: 机器人位置 + 旋转
struct position_cache_t
{
  position_data_t data;
  rclcpp::Time field_pose_time;
  bool field_pose_received = false;
};

class serial_gateway_t : public rclcpp::Node
{
public:
  serial_gateway_t()
  : Node("serial_gateway"), serial_fd_(-1), perception_sequence_(0U), position_sequence_(0U)
  {
    this->declare_parameter("serial_device", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("send_period_ms", 20);
    this->declare_parameter("data_timeout_ms", 200);
    this->declare_parameter("controller_timeout_ms", 500);
    this->declare_parameter("field_pose_topic", "/competition/field_pose");
    this->declare_parameter("ball_position_topic", "/perception/ball_position");
    this->declare_parameter("block_red_position_topic", "/perception/block_red_position");
    this->declare_parameter("block_blue_position_topic", "/perception/block_blue_position");
    this->declare_parameter("controller_connected_topic", "/competition/serial/controller_connected");
    this->declare_parameter("controller_state_topic", "/competition/serial/controller_state");
    this->declare_parameter("controller_error_topic", "/competition/serial/controller_error");

    field_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      this->get_parameter("field_pose_topic").as_string(), 10,
      std::bind(&serial_gateway_t::SerialGateway_FieldPoseCallback, this, std::placeholders::_1));
    ball_position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      this->get_parameter("ball_position_topic").as_string(), 10,
      std::bind(&serial_gateway_t::SerialGateway_BallPositionCallback, this, std::placeholders::_1));
    block_red_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      this->get_parameter("block_red_position_topic").as_string(), 10,
      std::bind(&serial_gateway_t::SerialGateway_BlockRedCallback, this, std::placeholders::_1));
    block_blue_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      this->get_parameter("block_blue_position_topic").as_string(), 10,
      std::bind(&serial_gateway_t::SerialGateway_BlockBlueCallback, this, std::placeholders::_1));

    controller_connected_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      this->get_parameter("controller_connected_topic").as_string(), 10);
    controller_state_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      this->get_parameter("controller_state_topic").as_string(), 10);
    controller_error_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      this->get_parameter("controller_error_topic").as_string(), 10);

    SerialGateway_Open();
    last_controller_time_ = this->now();
    const auto send_period_ms = this->get_parameter("send_period_ms").as_int();
    serial_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(send_period_ms),
      std::bind(&serial_gateway_t::SerialGateway_Update, this));
  }

  ~serial_gateway_t() override
  {
    SerialGateway_Close();
  }

private:
  void SerialGateway_Open()
  {
    if (serial_fd_ >= 0)
    {
      return;
    }

    const auto serial_device = this->get_parameter("serial_device").as_string();
    serial_fd_ = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0)
    {
      RCLCPP_WARN(this->get_logger(), "无法打开串口 %s：%s", serial_device.c_str(), std::strerror(errno));
      return;
    }

    termios serial_cfg{};
    if (tcgetattr(serial_fd_, &serial_cfg) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "无法读取串口配置：%s", std::strerror(errno));
      SerialGateway_Close();
      return;
    }

    cfmakeraw(&serial_cfg);
    const auto baudrate = this->get_parameter("baudrate").as_int();
    const auto baudrate_cfg = SerialGateway_GetBaudrate(baudrate);
    cfsetispeed(&serial_cfg, baudrate_cfg);
    cfsetospeed(&serial_cfg, baudrate_cfg);
    serial_cfg.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD | k_serial_data_bits);
    serial_cfg.c_cflag &= static_cast<tcflag_t>(~PARENB);
    serial_cfg.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    serial_cfg.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    serial_cfg.c_cflag |= k_serial_data_bits;
    serial_cfg.c_cc[VMIN] = 0;
    serial_cfg.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &serial_cfg) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "无法设置串口配置：%s", std::strerror(errno));
      SerialGateway_Close();
      return;
    }
    tcflush(serial_fd_, TCIOFLUSH);
    RCLCPP_INFO(this->get_logger(), "串口网关已打开 %s，波特率 %d。", serial_device.c_str(), baudrate);
  }

  void SerialGateway_Close()
  {
    if (serial_fd_ >= 0)
    {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void SerialGateway_FieldPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(position_mutex_);
    position_cache_.data.field_x_m = static_cast<float>(message->pose.position.x);
    position_cache_.data.field_y_m = static_cast<float>(message->pose.position.y);
    position_cache_.data.field_z_m = static_cast<float>(message->pose.position.z);
    // 从四元数提取绕 Z 轴的偏航角（弧度），场地坐标系下的机器人朝向。
    const float w = static_cast<float>(message->pose.orientation.w);
    const float x = static_cast<float>(message->pose.orientation.x);
    const float y = static_cast<float>(message->pose.orientation.y);
    const float z = static_cast<float>(message->pose.orientation.z);
    position_cache_.data.field_yaw = std::atan2(2.0F * (w * z + x * y), 1.0F - 2.0F * (y * y + z * z));
    position_cache_.field_pose_time = this->now();
    position_cache_.field_pose_received = true;
  }

  void SerialGateway_BallPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(perception_mutex_);
    perception_cache_.data.ball_x_m = static_cast<float>(message->point.x);
    perception_cache_.data.ball_y_m = static_cast<float>(message->point.y);
    perception_cache_.data.ball_z_m = static_cast<float>(message->point.z);
    perception_cache_.ball_time = this->now();
    perception_cache_.ball_received = true;
  }

  void SerialGateway_BlockRedCallback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(perception_mutex_);
    perception_cache_.data.red_x_m = static_cast<float>(message->point.x);
    perception_cache_.data.red_y_m = static_cast<float>(message->point.y);
    perception_cache_.data.red_z_m = static_cast<float>(message->point.z);
    perception_cache_.red_time = this->now();
    perception_cache_.red_received = true;
  }

  void SerialGateway_BlockBlueCallback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(perception_mutex_);
    perception_cache_.data.blue_x_m = static_cast<float>(message->point.x);
    perception_cache_.data.blue_y_m = static_cast<float>(message->point.y);
    perception_cache_.data.blue_z_m = static_cast<float>(message->point.z);
    perception_cache_.blue_time = this->now();
    perception_cache_.blue_received = true;
  }

  void SerialGateway_Update()
  {
    if (serial_fd_ < 0)
    {
      SerialGateway_Open();
      SerialGateway_PublishControllerConnected(false);
      return;
    }

    SerialGateway_Read();
    SerialGateway_SendPosition();
    SerialGateway_SendPerception();
    SerialGateway_PublishConnectionTimeout();
  }

  void SerialGateway_SendPerception()
  {
    perception_data_t perception_data{};
    {
      std::lock_guard<std::mutex> lock(perception_mutex_);
      perception_data = perception_cache_.data;
      const auto now = this->now();
      const auto timeout_ms = this->get_parameter("data_timeout_ms").as_int();
      const auto timeout = rclcpp::Duration::from_nanoseconds(timeout_ms * 1000000LL);
      if (perception_cache_.red_received && (now - perception_cache_.red_time) < timeout)
      {
        perception_data.flags |= PERCEPTION_RED_VALID;
      }
      if (perception_cache_.blue_received && (now - perception_cache_.blue_time) < timeout)
      {
        perception_data.flags |= PERCEPTION_BLUE_VALID;
      }
      if (perception_cache_.ball_received && (now - perception_cache_.ball_time) < timeout)
      {
        perception_data.flags |= PERCEPTION_BALL_VALID;
      }
    }

    const auto frame = SerialProtocol_EncodePerception(perception_data, perception_sequence_++);
    const auto write_size = write(serial_fd_, frame.data(), frame.size());
    if (write_size != static_cast<ssize_t>(frame.size()))
    {
      RCLCPP_ERROR(this->get_logger(), "感知帧串口发送失败：%s", std::strerror(errno));
      SerialGateway_Close();
    }
  }

  void SerialGateway_SendPosition()
  {
    position_data_t position_data{};
    {
      std::lock_guard<std::mutex> lock(position_mutex_);
      position_data = position_cache_.data;
      const auto now = this->now();
      const auto timeout_ms = this->get_parameter("data_timeout_ms").as_int();
      const auto timeout = rclcpp::Duration::from_nanoseconds(timeout_ms * 1000000LL);
      if (position_cache_.field_pose_received && (now - position_cache_.field_pose_time) < timeout)
      {
        position_data.flags |= POSITION_FIELD_VALID;
      }
    }

    const auto frame = SerialProtocol_EncodePosition(position_data, position_sequence_++);
    const auto write_size = write(serial_fd_, frame.data(), frame.size());
    if (write_size != static_cast<ssize_t>(frame.size()))
    {
      RCLCPP_ERROR(this->get_logger(), "位置帧串口发送失败：%s", std::strerror(errno));
      SerialGateway_Close();
    }
  }

  void SerialGateway_Read()
  {
    std::array<uint8_t, k_serial_read_buffer_size> read_buffer{};
    const auto read_size = read(serial_fd_, read_buffer.data(), read_buffer.size());
    if (read_size <= 0)
    {
      return;
    }
    receive_buffer_.insert(receive_buffer_.end(), read_buffer.begin(), read_buffer.begin() + read_size);

    while (receive_buffer_.size() >= k_rx_status_frame_size)
    {
      if (receive_buffer_[0] != k_rx_header_0 || receive_buffer_[1] != k_rx_header_1)
      {
        receive_buffer_.erase(receive_buffer_.begin());
        continue;
      }

      std::array<uint8_t, k_rx_status_frame_size> frame{};
      std::copy_n(receive_buffer_.begin(), k_rx_status_frame_size, frame.begin());
      controller_status_t status{};
      if (!SerialProtocol_DecodeStatus(frame, &status))
      {
        receive_buffer_.erase(receive_buffer_.begin());
        continue;
      }

      receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + k_rx_status_frame_size);
      last_controller_time_ = this->now();
      controller_state_pub_->publish(std_msgs::msg::UInt8().set__data(status.state));
      controller_error_pub_->publish(std_msgs::msg::UInt8().set__data(status.error));
      SerialGateway_PublishControllerConnected(true);
    }
  }

  void SerialGateway_PublishConnectionTimeout()
  {
    const auto timeout_ms = this->get_parameter("controller_timeout_ms").as_int();
    const auto timeout = rclcpp::Duration::from_nanoseconds(timeout_ms * 1000000LL);
    if ((this->now() - last_controller_time_) >= timeout)
    {
      SerialGateway_PublishControllerConnected(false);
    }
  }

  void SerialGateway_PublishControllerConnected(bool connected)
  {
    controller_connected_pub_->publish(std_msgs::msg::Bool().set__data(connected));
  }

  int serial_fd_;
  uint8_t perception_sequence_;
  uint8_t position_sequence_;
  perception_cache_t perception_cache_;
  std::mutex perception_mutex_;
  position_cache_t position_cache_;
  std::mutex position_mutex_;
  std::vector<uint8_t> receive_buffer_;
  rclcpp::Time last_controller_time_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr field_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ball_position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr block_red_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr block_blue_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr controller_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr controller_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr controller_error_pub_;
  rclcpp::TimerBase::SharedPtr serial_timer_;
};

}  // namespace competition_gateway

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<competition_gateway::serial_gateway_t>());
  rclcpp::shutdown();
  return 0;
}
