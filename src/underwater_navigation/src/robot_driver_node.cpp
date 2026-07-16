#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

namespace
{
constexpr uint16_t kMagic = 0x55AA;
constexpr uint8_t kVersion = 1;
constexpr size_t kHeaderSize = 16;
constexpr size_t kChecksumSize = 2;
constexpr uint8_t kSensorImu = 1;
constexpr uint8_t kSensorMag = 2;
constexpr double kGaussToTesla = 1.0e-4;
constexpr double kMinAccelNorm = 0.5;       // m/s^2, catches missing/stuck data.
constexpr double kMaxAccelNorm = 40.0;      // m/s^2, intentionally broad sanity limit.
constexpr double kMinMagNormTesla = 5.0e-6;
constexpr double kMaxMagNormTesla = 200.0e-6;

uint16_t read_u16_le(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

uint64_t read_u64_le(const uint8_t * data)
{
  uint64_t value = 0;
  for (size_t i = 0; i < 8; ++i) {
    value |= static_cast<uint64_t>(data[i]) << (8 * i);
  }
  return value;
}

float read_float_le(const uint8_t * data)
{
  float value = 0.0F;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

uint16_t crc16_ccitt_false(const uint8_t * data, size_t length)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

builtin_interfaces::msg::Time stamp_from_timestamp_us(uint64_t timestamp_us)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(timestamp_us / 1000000ULL);
  stamp.nanosec = static_cast<uint32_t>((timestamp_us % 1000000ULL) * 1000ULL);
  return stamp;
}

bool all_finite(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

double vector_norm(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}
}  // namespace

class RobotDriverNode : public rclcpp::Node
{
public:
  RobotDriverNode()
  : Node("robot_driver_node")
  {
    udp_port_ = this->declare_parameter<int>("udp_port", 14550);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 50);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/mag", 50);

    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&RobotDriverNode::print_stats, this));

    receive_thread_ = std::thread(&RobotDriverNode::receive_loop, this);
    RCLCPP_INFO(this->get_logger(), "robot_driver_node listening on UDP port %d", udp_port_);
  }

  ~RobotDriverNode() override
  {
    running_.store(false);
    if (socket_fd_ >= 0) {
      shutdown(socket_fd_, SHUT_RDWR);
      close(socket_fd_);
      socket_fd_ = -1;
    }
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
  }

private:
  struct RawFrame
  {
    uint8_t sensor_type = 0;
    uint16_t payload_len = 0;
    uint16_t seq = 0;
    uint64_t timestamp_us = 0;
    const uint8_t * payload = nullptr;
  };

  struct QualityStats
  {
    uint64_t last_timestamp_us = 0;
    uint64_t dt_samples = 0;
    uint64_t dt_min_us = std::numeric_limits<uint64_t>::max();
    uint64_t dt_max_us = 0;
    double dt_sum_us = 0.0;
    uint64_t norm_samples = 0;
    double norm_min = std::numeric_limits<double>::infinity();
    double norm_max = 0.0;
    double norm_sum = 0.0;
    uint64_t warn_count = 0;
  };

  struct StatsSnapshot
  {
    uint64_t dt_samples = 0;
    uint64_t dt_min_us = 0;
    uint64_t dt_max_us = 0;
    double dt_mean_us = 0.0;
    uint64_t norm_samples = 0;
    double norm_min = 0.0;
    double norm_max = 0.0;
    double norm_mean = 0.0;
    uint64_t warn_count = 0;
  };

  void receive_loop()
  {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "failed to create UDP socket: %s", strerror(errno));
      return;
    }

    int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    timeval receive_timeout {};
    receive_timeout.tv_usec = 200000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout));

    sockaddr_in local_address {};
    local_address.sin_family = AF_INET;
    local_address.sin_addr.s_addr = htonl(INADDR_ANY);
    local_address.sin_port = htons(static_cast<uint16_t>(udp_port_));

    if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&local_address), sizeof(local_address)) < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "failed to bind UDP port %d: %s", udp_port_, strerror(errno));
      return;
    }

    std::array<uint8_t, 4096> buffer {};
    while (running_.load() && rclcpp::ok()) {
      const ssize_t received = recvfrom(
        socket_fd_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
      if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
          continue;
        }
        if (running_.load()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "UDP receive failed: %s", strerror(errno));
        }
        continue;
      }

      RawFrame frame;
      if (!parse_frame(buffer.data(), static_cast<size_t>(received), frame)) {
        continue;
      }

      if (frame.sensor_type == kSensorImu) {
        publish_imu(frame);
      } else if (frame.sensor_type == kSensorMag) {
        publish_mag(frame);
      }
    }
  }

  bool parse_frame(const uint8_t * data, size_t length, RawFrame & frame)
  {
    if (length < kHeaderSize + kChecksumSize) {
      invalid_frames_.fetch_add(1);
      return false;
    }

    const uint16_t magic = read_u16_le(&data[0]);
    const uint8_t version = data[2];
    const uint8_t sensor_type = data[3];
    const uint16_t payload_len = read_u16_le(&data[4]);
    const uint16_t seq = read_u16_le(&data[6]);
    const uint64_t timestamp_us = read_u64_le(&data[8]);
    const size_t frame_len = kHeaderSize + payload_len + kChecksumSize;

    if (magic != kMagic || version != kVersion || length < frame_len) {
      invalid_frames_.fetch_add(1);
      return false;
    }

    const uint16_t received_crc = read_u16_le(&data[kHeaderSize + payload_len]);
    const uint16_t expected_crc = crc16_ccitt_false(data, frame_len - kChecksumSize);
    if (received_crc != expected_crc) {
      checksum_errors_.fetch_add(1);
      return false;
    }

    if ((sensor_type == kSensorImu && payload_len != 24) ||
      (sensor_type == kSensorMag && payload_len != 12))
    {
      invalid_frames_.fetch_add(1);
      return false;
    }

    if (timestamp_us == 0) {
      invalid_frames_.fetch_add(1);
      return false;
    }

    frame.sensor_type = sensor_type;
    frame.payload_len = payload_len;
    frame.seq = seq;
    frame.timestamp_us = timestamp_us;
    frame.payload = &data[kHeaderSize];
    return true;
  }

  void publish_imu(const RawFrame & frame)
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp_from_timestamp_us(frame.timestamp_us);
    msg.header.frame_id = "imu_link";

    msg.linear_acceleration.x = read_float_le(&frame.payload[0]);
    msg.linear_acceleration.y = read_float_le(&frame.payload[4]);
    msg.linear_acceleration.z = read_float_le(&frame.payload[8]);
    msg.angular_velocity.x = read_float_le(&frame.payload[12]);
    msg.angular_velocity.y = read_float_le(&frame.payload[16]);
    msg.angular_velocity.z = read_float_le(&frame.payload[20]);
    msg.orientation_covariance[0] = -1.0;

    update_imu_quality(frame, msg);

    imu_pub_->publish(msg);
    imu_count_.fetch_add(1);
  }

  void publish_mag(const RawFrame & frame)
  {
    sensor_msgs::msg::MagneticField msg;
    msg.header.stamp = stamp_from_timestamp_us(frame.timestamp_us);
    msg.header.frame_id = "imu_link";

    msg.magnetic_field.x = read_float_le(&frame.payload[0]) * kGaussToTesla;
    msg.magnetic_field.y = read_float_le(&frame.payload[4]) * kGaussToTesla;
    msg.magnetic_field.z = read_float_le(&frame.payload[8]) * kGaussToTesla;

    update_mag_quality(frame, msg);

    mag_pub_->publish(msg);
    mag_count_.fetch_add(1);
  }

  void update_imu_quality(const RawFrame & frame, const sensor_msgs::msg::Imu & msg)
  {
    const double acc_norm = vector_norm(
      msg.linear_acceleration.x,
      msg.linear_acceleration.y,
      msg.linear_acceleration.z);
    const bool finite = all_finite(
      msg.linear_acceleration.x,
      msg.linear_acceleration.y,
      msg.linear_acceleration.z) &&
      all_finite(
      msg.angular_velocity.x,
      msg.angular_velocity.y,
      msg.angular_velocity.z);
    const bool norm_ok =
      std::isfinite(acc_norm) && acc_norm >= kMinAccelNorm && acc_norm <= kMaxAccelNorm;

    bool timestamp_bad = false;
    bool dt_bad = false;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      update_common_quality(imu_stats_, frame.timestamp_us, acc_norm, timestamp_bad, dt_bad);
      if (!finite || !norm_ok || timestamp_bad || dt_bad) {
        imu_stats_.warn_count++;
      }
    }

    if (!finite) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "IMU data contains NaN/Inf");
    }
    if (timestamp_bad || dt_bad) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "IMU timestamp not strictly increasing: seq=%u timestamp_us=%llu",
        static_cast<unsigned int>(frame.seq),
        static_cast<unsigned long long>(frame.timestamp_us));
    }
    if (!norm_ok) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "IMU |acc| abnormal: %.3f m/s^2", acc_norm);
    }
  }

  void update_mag_quality(const RawFrame & frame, const sensor_msgs::msg::MagneticField & msg)
  {
    const double mag_norm = vector_norm(
      msg.magnetic_field.x,
      msg.magnetic_field.y,
      msg.magnetic_field.z);
    const bool finite = all_finite(
      msg.magnetic_field.x,
      msg.magnetic_field.y,
      msg.magnetic_field.z);
    const bool norm_ok = std::isfinite(mag_norm) &&
      mag_norm >= kMinMagNormTesla && mag_norm <= kMaxMagNormTesla;

    bool timestamp_bad = false;
    bool dt_bad = false;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      update_common_quality(mag_stats_, frame.timestamp_us, mag_norm, timestamp_bad, dt_bad);
      if (!finite || !norm_ok || timestamp_bad || dt_bad) {
        mag_stats_.warn_count++;
      }
    }

    if (!finite) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "MAG data contains NaN/Inf");
    }
    if (timestamp_bad || dt_bad) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "MAG timestamp not strictly increasing: seq=%u timestamp_us=%llu",
        static_cast<unsigned int>(frame.seq),
        static_cast<unsigned long long>(frame.timestamp_us));
    }
    if (!norm_ok) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "MAG |mag| abnormal: %.2f uT", mag_norm * 1.0e6);
    }
  }

  void update_common_quality(
    QualityStats & stats,
    uint64_t timestamp_us,
    double norm,
    bool & timestamp_bad,
    bool & dt_bad)
  {
    if (stats.last_timestamp_us != 0) {
      if (timestamp_us <= stats.last_timestamp_us) {
        timestamp_bad = true;
        dt_bad = true;
      } else {
        const uint64_t dt_us = timestamp_us - stats.last_timestamp_us;
        if (dt_us == 0) {
          dt_bad = true;
        } else {
          stats.dt_samples++;
          stats.dt_min_us = std::min(stats.dt_min_us, dt_us);
          stats.dt_max_us = std::max(stats.dt_max_us, dt_us);
          stats.dt_sum_us += static_cast<double>(dt_us);
        }
      }
    }

    stats.last_timestamp_us = timestamp_us;
    if (std::isfinite(norm)) {
      stats.norm_samples++;
      stats.norm_min = std::min(stats.norm_min, norm);
      stats.norm_max = std::max(stats.norm_max, norm);
      stats.norm_sum += norm;
    }
  }

  void print_stats()
  {
    const uint64_t imu_now = imu_count_.load();
    const uint64_t mag_now = mag_count_.load();
    const uint64_t invalid_now = invalid_frames_.load();
    const uint64_t checksum_now = checksum_errors_.load();

    const double imu_hz = static_cast<double>(imu_now - last_imu_count_) / 5.0;
    const double mag_hz = static_cast<double>(mag_now - last_mag_count_) / 5.0;

    StatsSnapshot imu_stats;
    StatsSnapshot mag_stats;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      imu_stats = take_snapshot_and_reset(imu_stats_);
      mag_stats = take_snapshot_and_reset(mag_stats_);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "IMU quality: %.1f Hz dt_ms min=%.3f max=%.3f mean=%.3f |acc| min=%.3f max=%.3f mean=%.3f warn=%lu",
      imu_hz,
      imu_stats.dt_min_us / 1000.0,
      imu_stats.dt_max_us / 1000.0,
      imu_stats.dt_mean_us / 1000.0,
      imu_stats.norm_min,
      imu_stats.norm_max,
      imu_stats.norm_mean,
      static_cast<unsigned long>(imu_stats.warn_count));

    RCLCPP_INFO(
      this->get_logger(),
      "MAG quality: %.1f Hz dt_ms min=%.3f max=%.3f mean=%.3f |mag|_uT min=%.2f max=%.2f mean=%.2f warn=%lu",
      mag_hz,
      mag_stats.dt_min_us / 1000.0,
      mag_stats.dt_max_us / 1000.0,
      mag_stats.dt_mean_us / 1000.0,
      mag_stats.norm_min * 1.0e6,
      mag_stats.norm_max * 1.0e6,
      mag_stats.norm_mean * 1.0e6,
      static_cast<unsigned long>(mag_stats.warn_count));

    RCLCPP_INFO(
      this->get_logger(),
      "raw sensor frame errors: invalid=%lu checksum=%lu",
      static_cast<unsigned long>(invalid_now),
      static_cast<unsigned long>(checksum_now));

    last_imu_count_ = imu_now;
    last_mag_count_ = mag_now;
  }

  StatsSnapshot take_snapshot_and_reset(QualityStats & stats)
  {
    StatsSnapshot snapshot;
    snapshot.dt_samples = stats.dt_samples;
    snapshot.dt_min_us = stats.dt_samples > 0 ? stats.dt_min_us : 0;
    snapshot.dt_max_us = stats.dt_samples > 0 ? stats.dt_max_us : 0;
    snapshot.dt_mean_us = stats.dt_samples > 0 ?
      stats.dt_sum_us / static_cast<double>(stats.dt_samples) : 0.0;
    snapshot.norm_samples = stats.norm_samples;
    snapshot.norm_min = stats.norm_samples > 0 ? stats.norm_min : 0.0;
    snapshot.norm_max = stats.norm_samples > 0 ? stats.norm_max : 0.0;
    snapshot.norm_mean = stats.norm_samples > 0 ?
      stats.norm_sum / static_cast<double>(stats.norm_samples) : 0.0;
    snapshot.warn_count = stats.warn_count;

    const uint64_t last_timestamp_us = stats.last_timestamp_us;
    stats = QualityStats {};
    stats.last_timestamp_us = last_timestamp_us;
    return snapshot;
  }

  int udp_port_ = 9000;
  int socket_fd_ = -1;
  std::atomic<bool> running_ {true};
  std::thread receive_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::atomic<uint64_t> imu_count_ {0};
  std::atomic<uint64_t> mag_count_ {0};
  std::atomic<uint64_t> invalid_frames_ {0};
  std::atomic<uint64_t> checksum_errors_ {0};
  uint64_t last_imu_count_ = 0;
  uint64_t last_mag_count_ = 0;
  std::mutex stats_mutex_;
  QualityStats imu_stats_;
  QualityStats mag_stats_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDriverNode>());
  rclcpp::shutdown();
  return 0;
}
