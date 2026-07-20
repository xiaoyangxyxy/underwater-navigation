#include "underwater_navigation/imu_subscriber.hpp"
#include "underwater_navigation/GraphManager.hpp"

#include <cmath>

ImuSubscriber::ImuSubscriber()
: Node("imu_subscriber"),
  last_processed_imu_stamp_(0, 0, RCL_ROS_TIME),
  imu_message_count_(0)
{
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu");
    depth_topic_ = this->declare_parameter<std::string>("depth_topic", "/depth");
    dvl_topic_ = this->declare_parameter<std::string>("dvl_topic", "/dvl");
    mag_topic_ = this->declare_parameter<std::string>("mag_topic", "/mag");
    sonar_up_topic_ = this->declare_parameter<std::string>("sonar_up_topic", "/sonar/up");
    sonar_down_topic_ = this->declare_parameter<std::string>("sonar_down_topic", "/sonar/down");
    sonar_left_topic_ = this->declare_parameter<std::string>("sonar_left_topic", "/sonar/left");
    sonar_right_topic_ = this->declare_parameter<std::string>("sonar_right_topic", "/sonar/right");
    sonar_up_offset_body_ = gtsam::Vector3(
        this->declare_parameter<double>("sonar_up_offset_x", 0.0),
        this->declare_parameter<double>("sonar_up_offset_y", 0.0),
        this->declare_parameter<double>("sonar_up_offset_z", 0.0));
    sonar_down_offset_body_ = gtsam::Vector3(
        this->declare_parameter<double>("sonar_down_offset_x", 0.0),
        this->declare_parameter<double>("sonar_down_offset_y", 0.0),
        this->declare_parameter<double>("sonar_down_offset_z", 0.0));
    sonar_left_offset_body_ = gtsam::Vector3(
        this->declare_parameter<double>("sonar_left_offset_x", 0.0),
        this->declare_parameter<double>("sonar_left_offset_y", 0.0),
        this->declare_parameter<double>("sonar_left_offset_z", 0.0));
    sonar_right_offset_body_ = gtsam::Vector3(
        this->declare_parameter<double>("sonar_right_offset_x", 0.0),
        this->declare_parameter<double>("sonar_right_offset_y", 0.0),
        this->declare_parameter<double>("sonar_right_offset_z", 0.0));
    imu_window_size_ = this->declare_parameter<int>("imu_window_size", 50);
    if (imu_window_size_ <= 0)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Invalid imu_window_size=%d, fallback to 50",
            imu_window_size_);
        imu_window_size_ = 50;
    }

    graph_manager_ = std::make_shared<GraphManager>(this);

    graph_manager_->addPriorPose();
    graph_manager_->addPriorVelocity();
    graph_manager_->addPriorBias();

    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        100,
        std::bind(&ImuSubscriber::callback, this, std::placeholders::_1)
    );

    depth_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
        depth_topic_,
        100,
        std::bind(&ImuSubscriber::depthCallback, this, std::placeholders::_1)
    );

    dvl_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        dvl_topic_,
        100,
        std::bind(&ImuSubscriber::dvlCallback, this, std::placeholders::_1)
    );

    mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        mag_topic_,
        100,
        std::bind(&ImuSubscriber::magCallback, this, std::placeholders::_1)
    );

    sonar_up_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        sonar_up_topic_,
        100,
        std::bind(&ImuSubscriber::sonarUpCallback, this, std::placeholders::_1)
    );

    sonar_down_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        sonar_down_topic_,
        100,
        std::bind(&ImuSubscriber::sonarDownCallback, this, std::placeholders::_1)
    );

    sonar_left_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        sonar_left_topic_,
        100,
        std::bind(&ImuSubscriber::sonarLeftCallback, this, std::placeholders::_1)
    );

    sonar_right_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        sonar_right_topic_,
        100,
        std::bind(&ImuSubscriber::sonarRightCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "IMU Subscriber Started: %s", imu_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Depth Subscriber Started: %s", depth_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "DVL Subscriber Started: %s", dvl_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "MAG Subscriber Started: %s", mag_topic_.c_str());
    RCLCPP_INFO(
        this->get_logger(),
        "Sonar Subscribers Started: up=%s down=%s left=%s right=%s",
        sonar_up_topic_.c_str(),
        sonar_down_topic_.c_str(),
        sonar_left_topic_.c_str(),
        sonar_right_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "IMU window size: %d", imu_window_size_);
}

void ImuSubscriber::callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Drop IMU message with zero header stamp");
        return;
    }

    gtsam::Vector3 acc(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );

    gtsam::Vector3 gyro(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );

    imu_buffer_.push_back(GraphManager::TimedImuMeasurement{
        stamp,
        acc,
        gyro
    });

    if (++imu_message_count_ % static_cast<size_t>(imu_window_size_) == 0)
    {
        processBuffers();
    }
}

void ImuSubscriber::depthCallback(
    const sensor_msgs::msg::FluidPressure::SharedPtr msg)
{
    const rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Drop depth message with zero header stamp");
        return;
    }

    const double depth = msg->fluid_pressure;
    depth_buffer_.push_back(GraphManager::TimedDepthMeasurement{
        stamp,
        depth
    });
}

void ImuSubscriber::dvlCallback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    const rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Drop DVL message with zero header stamp");
        return;
    }

    dvl_buffer_.push_back(GraphManager::TimedDvlMeasurement{
        stamp,
        gtsam::Vector3(
            msg->twist.linear.x,
            msg->twist.linear.y,
            msg->twist.linear.z)
    });
}

void ImuSubscriber::magCallback(
    const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
    const rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Drop MAG message with zero header stamp");
        return;
    }

    mag_buffer_.push_back(GraphManager::TimedMagMeasurement{
        stamp,
        gtsam::Vector3(
            msg->magnetic_field.x,
            msg->magnetic_field.y,
            msg->magnetic_field.z)
    });
}

void ImuSubscriber::sonarUpCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    sonarCallback(
        msg,
        "up",
        sonar_up_offset_body_,
        gtsam::Vector3(0.0, 0.0, 1.0));
}

void ImuSubscriber::sonarDownCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    sonarCallback(
        msg,
        "down",
        sonar_down_offset_body_,
        gtsam::Vector3(0.0, 0.0, -1.0));
}

void ImuSubscriber::sonarLeftCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    sonarCallback(
        msg,
        "left",
        sonar_left_offset_body_,
        gtsam::Vector3(0.0, 1.0, 0.0));
}

void ImuSubscriber::sonarRightCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    sonarCallback(
        msg,
        "right",
        sonar_right_offset_body_,
        gtsam::Vector3(0.0, -1.0, 0.0));
}

void ImuSubscriber::sonarCallback(
    const sensor_msgs::msg::Range::SharedPtr msg,
    const std::string& name,
    const gtsam::Vector3& offset_body,
    const gtsam::Vector3& direction_body)
{
    const rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Drop sonar %s message with zero header stamp", name.c_str());
        return;
    }

    const double range = msg->range;
    if (!std::isfinite(range) ||
        range < static_cast<double>(msg->min_range) ||
        range > static_cast<double>(msg->max_range))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Drop invalid sonar %s range=%.3f min=%.3f max=%.3f",
            name.c_str(),
            range,
            static_cast<double>(msg->min_range),
            static_cast<double>(msg->max_range));
        return;
    }

    sonar_buffer_.push_back(GraphManager::TimedSonarMeasurement{
        stamp,
        range,
        offset_body,
        direction_body,
        name,
        true
    });
}

void ImuSubscriber::processBuffers()
{
    if (graph_manager_->processBuffers(
            imu_buffer_,
            depth_buffer_,
            dvl_buffer_,
            mag_buffer_,
            sonar_buffer_,
            last_processed_imu_stamp_))
    {
        graph_manager_->optimize();
        graph_manager_->publishPath(last_processed_imu_stamp_);
    }
}
