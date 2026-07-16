#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <string>

#include "underwater_navigation/simulation/TrajectoryModel.hpp"

class SimulationSensorNode : public rclcpp::Node
{
public:
    SimulationSensorNode()
    : Node("simulation_sensor_node"),
      sim_time_ns_(0),
      tick_count_(0),
      rng_(std::random_device{}())
    {
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
        depth_topic_ = declare_parameter<std::string>("depth_topic", "/depth");
        mag_topic_ = declare_parameter<std::string>("mag_topic", "/mag");
        publish_clock_ = declare_parameter<bool>("publish_clock", true);
        gravity_sign_ = declare_parameter<double>("gravity_sign", 1.0);
        trajectory_parameters_.start_x = declare_parameter<double>("start_x", -14.0);
        trajectory_parameters_.start_y = declare_parameter<double>("start_y", 0.0);
        trajectory_parameters_.start_z = declare_parameter<double>("start_z", 0.0);
        trajectory_parameters_.forward_speed = declare_parameter<double>("forward_speed", 0.2);
        trajectory_parameters_.lateral_amp = declare_parameter<double>("lateral_amp", 0.3);
        trajectory_parameters_.lateral_freq = declare_parameter<double>("lateral_freq", 0.2);
        trajectory_parameters_.depth_amp = declare_parameter<double>("depth_amp", 0.2);
        trajectory_parameters_.depth_freq = declare_parameter<double>("depth_freq", 0.15);
        trajectory_parameters_.roll_amp = declare_parameter<double>("roll_amp", 0.05);
        trajectory_parameters_.roll_freq = declare_parameter<double>("roll_freq", 0.25);
        trajectory_parameters_.pitch_amp = declare_parameter<double>("pitch_amp", 0.05);
        trajectory_parameters_.pitch_freq = declare_parameter<double>("pitch_freq", 0.2);
        trajectory_parameters_.yaw_amp = declare_parameter<double>("yaw_amp", 0.2);
        trajectory_parameters_.yaw_freq = declare_parameter<double>("yaw_freq", 0.1);
        trajectory_model_ = std::make_unique<underwater_navigation::simulation::TrajectoryModel>(
            trajectory_parameters_);
        imu_offset_body_.x = declare_parameter<double>("imu_offset_x", 0.0);
        imu_offset_body_.y = declare_parameter<double>("imu_offset_y", 0.0);
        imu_offset_body_.z = declare_parameter<double>("imu_offset_z", 0.08);
        imu_acc_noise_std_ = declare_parameter<double>("imu_acc_noise_std", 0.02);
        imu_gyro_noise_std_ = declare_parameter<double>("imu_gyro_noise_std", 0.001);
        depth_noise_std_ = declare_parameter<double>("depth_noise_std", 0.02);
        mag_noise_std_ = declare_parameter<double>("mag_noise_std", 0.01);

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            imu_topic_,
            rclcpp::QoS(200));
        depth_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(
            depth_topic_,
            rclcpp::QoS(20));
        mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
            mag_topic_,
            rclcpp::QoS(50));

        if (publish_clock_)
        {
            clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(
                "/clock",
                rclcpp::ClockQoS());
        }

        tick_timer_ = create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&SimulationSensorNode::onTick, this));

        RCLCPP_INFO(
            get_logger(),
            "Simulation sensors started imu=%s depth=%s mag=%s gravity=[0, 0, -9.81] imu_rate=200Hz publish_clock=%s",
            imu_topic_.c_str(),
            depth_topic_.c_str(),
            mag_topic_.c_str(),
            publish_clock_ ? "true" : "false");

        if (std::abs(gravity_sign_ - 1.0) > 1e-9)
        {
            RCLCPP_WARN(
                get_logger(),
                "gravity_sign is ignored; simulation IMU always uses g_world=[0, 0, -9.81]");
        }
    }

private:
    builtin_interfaces::msg::Time currentStamp() const
    {
        builtin_interfaces::msg::Time stamp;
        stamp.sec = static_cast<int32_t>(sim_time_ns_ / 1000000000LL);
        stamp.nanosec = static_cast<uint32_t>(sim_time_ns_ % 1000000000LL);
        return stamp;
    }

    void onTick()
    {
        sim_time_ns_ += 5000000LL;
        ++tick_count_;

        const auto stamp = currentStamp();
        const double time_s = static_cast<double>(sim_time_ns_) * 1e-9;
        const auto truth = trajectory_model_->evaluate(time_s);

        if (publish_clock_ && clock_pub_)
        {
            rosgraph_msgs::msg::Clock clock_msg;
            clock_msg.clock = stamp;
            clock_pub_->publish(clock_msg);
        }

        publishImu(stamp, truth);

        if (tick_count_ % 10 == 0)
        {
            publishDepth(stamp, truth);
        }

        if (tick_count_ % 4 == 0)
        {
            publishMag(stamp, truth);
        }

    }

    void publishImu(
        const builtin_interfaces::msg::Time& stamp,
        const underwater_navigation::simulation::TrajectoryState& truth)
    {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "imu_link";

        msg.orientation_covariance[0] = -1.0;

        const underwater_navigation::simulation::Vector3 gravity_world{
            0.0,
            0.0,
            -9.81};
        const auto imu_acceleration_world =
            underwater_navigation::simulation::TrajectoryModel::accelerationAtBodyOffset(
                truth,
                imu_offset_body_);
        const underwater_navigation::simulation::Vector3 specific_force_world{
            imu_acceleration_world.x - gravity_world.x,
            imu_acceleration_world.y - gravity_world.y,
            imu_acceleration_world.z - gravity_world.z};
        const auto acc_body =
            underwater_navigation::simulation::TrajectoryModel::worldToBody(
                truth.rotation_world_body,
                specific_force_world);

        msg.angular_velocity.x =
            truth.body_angular_velocity.x + sampleNoise(imu_gyro_noise_std_);
        msg.angular_velocity.y =
            truth.body_angular_velocity.y + sampleNoise(imu_gyro_noise_std_);
        msg.angular_velocity.z =
            truth.body_angular_velocity.z + sampleNoise(imu_gyro_noise_std_);

        msg.linear_acceleration.x = acc_body.x + sampleNoise(imu_acc_noise_std_);
        msg.linear_acceleration.y = acc_body.y + sampleNoise(imu_acc_noise_std_);
        msg.linear_acceleration.z = acc_body.z + sampleNoise(imu_acc_noise_std_);

        imu_pub_->publish(msg);

        if (tick_count_ % 200 == 0)
        {
            RCLCPP_INFO(
                get_logger(),
                "[IMU_TRUTH] acc_body=(%.3f, %.3f, %.3f) gyro_body=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
                acc_body.x,
                acc_body.y,
                acc_body.z,
                truth.body_angular_velocity.x,
                truth.body_angular_velocity.y,
                truth.body_angular_velocity.z,
                truth.roll,
                truth.pitch,
                truth.yaw);
        }
    }

    void publishDepth(
        const builtin_interfaces::msg::Time& stamp,
        const underwater_navigation::simulation::TrajectoryState& truth)
    {
        sensor_msgs::msg::FluidPressure msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "depth_link";

        // Temporary project convention: fluid_pressure carries positive depth in meters.
        msg.fluid_pressure = -truth.position_world.z + sampleNoise(depth_noise_std_);

        depth_pub_->publish(msg);
    }

    void publishMag(
        const builtin_interfaces::msg::Time& stamp,
        const underwater_navigation::simulation::TrajectoryState& truth)
    {
        sensor_msgs::msg::MagneticField msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "mag_link";

        msg.magnetic_field.x = std::cos(truth.yaw) + sampleNoise(mag_noise_std_);
        msg.magnetic_field.y = -std::sin(truth.yaw) + sampleNoise(mag_noise_std_);
        msg.magnetic_field.z = sampleNoise(mag_noise_std_);

        mag_pub_->publish(msg);
    }

    double sampleNoise(double stddev)
    {
        if (stddev <= 0.0)
        {
            return 0.0;
        }
        std::normal_distribution<double> distribution(0.0, stddev);
        return distribution(rng_);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::TimerBase::SharedPtr tick_timer_;

    std::string imu_topic_;
    std::string depth_topic_;
    std::string mag_topic_;
    bool publish_clock_;
    double gravity_sign_;
    double imu_acc_noise_std_;
    double imu_gyro_noise_std_;
    double depth_noise_std_;
    double mag_noise_std_;
    int64_t sim_time_ns_;
    uint64_t tick_count_;
    underwater_navigation::simulation::TrajectoryParameters trajectory_parameters_;
    std::unique_ptr<underwater_navigation::simulation::TrajectoryModel> trajectory_model_;
    underwater_navigation::simulation::Vector3 imu_offset_body_;
    std::mt19937 rng_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulationSensorNode>());
    rclcpp::shutdown();
    return 0;
}
