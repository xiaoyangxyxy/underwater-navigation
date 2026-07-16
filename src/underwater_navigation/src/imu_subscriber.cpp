#include "underwater_navigation/imu_subscriber.hpp"
#include "underwater_navigation/GraphManager.hpp"

ImuSubscriber::ImuSubscriber()
: Node("imu_subscriber"),
  last_processed_imu_stamp_(0, 0, RCL_ROS_TIME),
  imu_message_count_(0)
{
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu");
    depth_topic_ = this->declare_parameter<std::string>("depth_topic", "/depth");
    dvl_topic_ = this->declare_parameter<std::string>("dvl_topic", "/dvl");
    mag_topic_ = this->declare_parameter<std::string>("mag_topic", "/mag");
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

    RCLCPP_INFO(this->get_logger(), "IMU Subscriber Started: %s", imu_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Depth Subscriber Started: %s", depth_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "DVL Subscriber Started: %s", dvl_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "MAG topic reserved: %s", mag_topic_.c_str());
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

void ImuSubscriber::processBuffers()
{
    if (graph_manager_->processBuffers(
            imu_buffer_,
            depth_buffer_,
            dvl_buffer_,
            last_processed_imu_stamp_))
    {
        graph_manager_->optimize();
        graph_manager_->publishPath(last_processed_imu_stamp_);
    }
}
