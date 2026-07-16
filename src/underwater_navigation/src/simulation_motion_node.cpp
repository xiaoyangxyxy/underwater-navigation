#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "underwater_navigation/simulation/TrajectoryModel.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <string>

class SimulationMotionNode : public rclcpp::Node
{
public:
    SimulationMotionNode()
    : Node("simulation_motion_node"),
      sim_time_ns_(0),
      next_dvl_publish_ns_(0),
      has_ground_truth_origin_(false),
      rng_(std::random_device{}()),
      warned_missing_service_(false)
    {
        robot_model_name_ = declare_parameter<std::string>("robot_model_name", "pipe_robot");
        set_entity_state_service_ =
            declare_parameter<std::string>("set_entity_state_service", "/set_entity_state");
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
        dvl_topic_ = declare_parameter<std::string>("dvl_topic", "/dvl");
        dvl_noise_std_ = declare_parameter<double>("dvl_noise_std", 0.02);
        dvl_rate_hz_ = declare_parameter<double>("dvl_rate_hz", 10.0);
        dvl_period_ns_ = rateToPeriodNs(dvl_rate_hz_);
        publish_clock_ = declare_parameter<bool>("publish_clock", true);
        use_relative_ground_truth_ =
            declare_parameter<bool>("use_relative_ground_truth", true);

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/ground_truth/pose",
            rclcpp::QoS(20));
        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/ground_truth/path",
            rclcpp::QoS(10).transient_local());
        dvl_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            dvl_topic_,
            rclcpp::QoS(20));

        if (publish_clock_)
        {
            clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(
                "/clock",
                rclcpp::ClockQoS());
        }

        set_state_client_ =
            create_client<gazebo_msgs::srv::SetEntityState>(set_entity_state_service_);

        path_msg_.header.frame_id = "map";

        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SimulationMotionNode::onTick, this));

        RCLCPP_INFO(
            get_logger(),
            "Simulation motion started model=%s service=%s start=[%.2f, %.2f, %.2f] vx=%.3f m/s y/z_amp=[%.3f, %.3f] rpy_amp=[%.3f, %.3f, %.3f] dvl=%s dvl_rate_hz=%.2f publish_clock=%s relative_gt=%s",
            robot_model_name_.c_str(),
            set_entity_state_service_.c_str(),
            trajectory_parameters_.start_x,
            trajectory_parameters_.start_y,
            trajectory_parameters_.start_z,
            trajectory_parameters_.forward_speed,
            trajectory_parameters_.lateral_amp,
            trajectory_parameters_.depth_amp,
            trajectory_parameters_.roll_amp,
            trajectory_parameters_.pitch_amp,
            trajectory_parameters_.yaw_amp,
            dvl_topic_.c_str(),
            dvl_rate_hz_,
            publish_clock_ ? "true" : "false",
            use_relative_ground_truth_ ? "true" : "false");
    }

private:
    builtin_interfaces::msg::Time currentStamp() const
    {
        builtin_interfaces::msg::Time stamp;
        stamp.sec = static_cast<int32_t>(sim_time_ns_ / 1000000000LL);
        stamp.nanosec = static_cast<uint32_t>(sim_time_ns_ % 1000000000LL);
        return stamp;
    }

    static void setRpy(
        geometry_msgs::msg::Pose& pose,
        double roll,
        double pitch,
        double yaw)
    {
        const double half_roll = 0.5 * roll;
        const double half_pitch = 0.5 * pitch;
        const double half_yaw = 0.5 * yaw;
        const double cr = std::cos(half_roll);
        const double sr = std::sin(half_roll);
        const double cp = std::cos(half_pitch);
        const double sp = std::sin(half_pitch);
        const double cy = std::cos(half_yaw);
        const double sy = std::sin(half_yaw);

        pose.orientation.w = cr * cp * cy + sr * sp * sy;
        pose.orientation.x = sr * cp * cy - cr * sp * sy;
        pose.orientation.y = cr * sp * cy + sr * cp * sy;
        pose.orientation.z = cr * cp * sy - sr * sp * cy;
    }

    void onTick()
    {
        sim_time_ns_ += 20000000LL;
        const double t = static_cast<double>(sim_time_ns_) * 1e-9;
        const auto stamp = currentStamp();

        if (publish_clock_ && clock_pub_)
        {
            rosgraph_msgs::msg::Clock clock_msg;
            clock_msg.clock = stamp;
            clock_pub_->publish(clock_msg);
        }

        const auto truth = trajectory_model_->evaluate(t);

        geometry_msgs::msg::PoseStamped absolute_pose_msg;
        absolute_pose_msg.header.stamp = stamp;
        absolute_pose_msg.header.frame_id = "map";
        absolute_pose_msg.pose.position.x = truth.position_world.x;
        absolute_pose_msg.pose.position.y = truth.position_world.y;
        absolute_pose_msg.pose.position.z = truth.position_world.z;
        setRpy(
            absolute_pose_msg.pose,
            truth.roll,
            truth.pitch,
            truth.yaw);

        if (!has_ground_truth_origin_)
        {
            ground_truth_origin_ = absolute_pose_msg.pose.position;
            has_ground_truth_origin_ = true;
        }

        geometry_msgs::msg::PoseStamped pose_msg = absolute_pose_msg;
        pose_msg.header.frame_id = "map";
        if (use_relative_ground_truth_)
        {
            pose_msg.pose.position.x =
                absolute_pose_msg.pose.position.x - ground_truth_origin_.x;
            pose_msg.pose.position.y =
                absolute_pose_msg.pose.position.y - ground_truth_origin_.y;
            pose_msg.pose.position.z =
                absolute_pose_msg.pose.position.z - ground_truth_origin_.z;
        }

        pose_pub_->publish(pose_msg);

        path_msg_.header.stamp = stamp;
        path_msg_.poses.push_back(pose_msg);
        path_pub_->publish(path_msg_);

        if (sim_time_ns_ >= next_dvl_publish_ns_)
        {
            publishDvl(stamp, truth);
            next_dvl_publish_ns_ = sim_time_ns_ + dvl_period_ns_;
        }

        updateGazeboPose(absolute_pose_msg, truth);
    }

    void publishDvl(
        const builtin_interfaces::msg::Time& stamp,
        const underwater_navigation::simulation::TrajectoryState& truth)
    {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "base_link";

        const auto velocity_body =
            underwater_navigation::simulation::TrajectoryModel::worldToBody(
                truth.rotation_world_body,
                truth.velocity_world);

        msg.twist.linear.x = velocity_body.x + sampleNoise(dvl_noise_std_);
        msg.twist.linear.y = velocity_body.y + sampleNoise(dvl_noise_std_);
        msg.twist.linear.z = velocity_body.z + sampleNoise(dvl_noise_std_);

        dvl_pub_->publish(msg);
    }

    void updateGazeboPose(
        const geometry_msgs::msg::PoseStamped& pose_msg,
        const underwater_navigation::simulation::TrajectoryState& truth)
    {
        if (!set_state_client_->service_is_ready())
        {
            if (!warned_missing_service_)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Gazebo SetEntityState service %s is not available yet. Ground truth is still published.",
                    set_entity_state_service_.c_str());
                warned_missing_service_ = true;
            }
            return;
        }

        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        request->state.name = robot_model_name_;
        request->state.pose = pose_msg.pose;
        request->state.twist.linear.x = truth.velocity_world.x;
        request->state.twist.linear.y = truth.velocity_world.y;
        request->state.twist.linear.z = truth.velocity_world.z;
        const auto angular_velocity_world =
            underwater_navigation::simulation::TrajectoryModel::bodyToWorld(
                truth.rotation_world_body,
                truth.body_angular_velocity);
        request->state.twist.angular.x = angular_velocity_world.x;
        request->state.twist.angular.y = angular_velocity_world.y;
        request->state.twist.angular.z = angular_velocity_world.z;
        request->state.reference_frame = "world";

        set_state_client_->async_send_request(request);
    }

    static int64_t rateToPeriodNs(double rate_hz)
    {
        if (rate_hz <= 0.0)
        {
            return 100000000LL;
        }
        return static_cast<int64_t>(std::llround(1000000000.0 / rate_hz));
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

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_state_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path path_msg_;
    std::string robot_model_name_;
    std::string set_entity_state_service_;
    std::string dvl_topic_;
    underwater_navigation::simulation::TrajectoryParameters trajectory_parameters_;
    std::unique_ptr<underwater_navigation::simulation::TrajectoryModel> trajectory_model_;
    double dvl_noise_std_;
    double dvl_rate_hz_;
    bool publish_clock_;
    bool use_relative_ground_truth_;
    int64_t sim_time_ns_;
    int64_t dvl_period_ns_;
    int64_t next_dvl_publish_ns_;
    geometry_msgs::msg::Point ground_truth_origin_;
    bool has_ground_truth_origin_;
    std::mt19937 rng_;
    bool warned_missing_service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulationMotionNode>());
    rclcpp::shutdown();
    return 0;
}
