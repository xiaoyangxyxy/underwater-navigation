#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <optional>
#include <stdexcept>
#include <string>

namespace
{
constexpr char kDefaultOutputPath[] =
    "/home/xiaoyang/water_robot/auv_ws/slam_logs/slam_debug_6dof.csv";

struct DebugRecord
{
    double time;
    double gt_x;
    double gt_y;
    double gt_z;
    double slam_x;
    double slam_y;
    double slam_z;
    double err_x;
    double err_y;
    double err_z;
    int dvl_matched;
    double dvl_time_diff;
    double dvl_residual_x;
    double dvl_residual_y;
    double dvl_residual_z;
};

std::optional<double> readDouble(
    const std::string& message,
    const std::string& field)
{
    const std::string prefix = field + " ";
    size_t position = message.find(prefix);
    while (position != std::string::npos)
    {
        if (position == 0 || message[position - 1] == ' ')
        {
            const char* value_start = message.c_str() + position + prefix.size();
            char* value_end = nullptr;
            errno = 0;
            const double value = std::strtod(value_start, &value_end);
            if (value_end != value_start && errno != ERANGE)
            {
                return value;
            }
            return std::nullopt;
        }
        position = message.find(prefix, position + prefix.size());
    }
    return std::nullopt;
}

std::optional<int> readInteger(
    const std::string& message,
    const std::string& field)
{
    const auto value = readDouble(message, field);
    if (!value || (*value != 0.0 && *value != 1.0))
    {
        return std::nullopt;
    }
    return static_cast<int>(*value);
}

std::optional<DebugRecord> parseDebugRecord(const std::string& message)
{
    const auto time = readDouble(message, "time");
    const auto gt_x = readDouble(message, "gt_x");
    const auto gt_y = readDouble(message, "gt_y");
    const auto gt_z = readDouble(message, "gt_z");
    const auto slam_x = readDouble(message, "slam_x");
    const auto slam_y = readDouble(message, "slam_y");
    const auto slam_z = readDouble(message, "slam_z");
    const auto err_x = readDouble(message, "err_x");
    const auto err_y = readDouble(message, "err_y");
    const auto err_z = readDouble(message, "err_z");
    const auto dvl_matched = readInteger(message, "dvl_matched");
    const auto dvl_time_diff = readDouble(message, "dvl_time_diff");
    const auto dvl_residual_x = readDouble(message, "dvl_residual_x");
    const auto dvl_residual_y = readDouble(message, "dvl_residual_y");
    const auto dvl_residual_z = readDouble(message, "dvl_residual_z");

    if (!time || !gt_x || !gt_y || !gt_z || !slam_x || !slam_y || !slam_z ||
        !err_x || !err_y || !err_z || !dvl_matched || !dvl_time_diff ||
        !dvl_residual_x || !dvl_residual_y || !dvl_residual_z)
    {
        return std::nullopt;
    }

    return DebugRecord{
        *time,
        *gt_x,
        *gt_y,
        *gt_z,
        *slam_x,
        *slam_y,
        *slam_z,
        *err_x,
        *err_y,
        *err_z,
        *dvl_matched,
        *dvl_time_diff,
        *dvl_residual_x,
        *dvl_residual_y,
        *dvl_residual_z};
}
}  // namespace

class SlamDebugRecorderNode : public rclcpp::Node
{
public:
    SlamDebugRecorderNode()
    : Node("slam_debug_recorder_node")
    {
        const std::string debug_topic =
            declare_parameter<std::string>("slam_debug_topic", "/slam_debug");
        output_path_ = declare_parameter<std::string>(
            "output_csv_path",
            kDefaultOutputPath);

        openOutputFile();

        debug_sub_ = create_subscription<std_msgs::msg::String>(
            debug_topic,
            rclcpp::QoS(100),
            std::bind(
                &SlamDebugRecorderNode::debugCallback,
                this,
                std::placeholders::_1));
    }

private:
    void openOutputFile()
    {
        const std::filesystem::path output_path(output_path_);
        std::error_code error;
        std::filesystem::create_directories(output_path.parent_path(), error);
        if (error)
        {
            RCLCPP_FATAL(
                get_logger(),
                "Failed to create CSV directory %s: %s",
                output_path.parent_path().c_str(),
                error.message().c_str());
            throw std::runtime_error("Cannot create slam debug output directory");
        }

        csv_.open(output_path, std::ios::out | std::ios::trunc);
        if (!csv_.is_open())
        {
            RCLCPP_FATAL(
                get_logger(),
                "Failed to open CSV output %s",
                output_path_.c_str());
            throw std::runtime_error("Cannot open slam debug CSV file");
        }

        csv_ << "time,gt_x,gt_y,gt_z,slam_x,slam_y,slam_z,"
             << "err_x,err_y,err_z,position_error_norm,dvl_matched,"
             << "dvl_time_diff,dvl_residual_x,dvl_residual_y,dvl_residual_z\n";
        csv_.flush();

        RCLCPP_INFO(
            get_logger(),
            "Recording /slam_debug to %s",
            output_path_.c_str());
    }

    void debugCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        const auto record = parseDebugRecord(msg->data);
        if (!record)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Drop malformed /slam_debug message; required fields are missing");
            return;
        }

        const double position_error_norm = std::sqrt(
            record->err_x * record->err_x +
            record->err_y * record->err_y +
            record->err_z * record->err_z);

        csv_ << std::fixed << std::setprecision(9)
             << record->time << ','
             << record->gt_x << ',' << record->gt_y << ',' << record->gt_z << ','
             << record->slam_x << ',' << record->slam_y << ',' << record->slam_z << ','
             << record->err_x << ',' << record->err_y << ',' << record->err_z << ','
             << position_error_norm << ','
             << record->dvl_matched << ','
             << record->dvl_time_diff << ','
             << record->dvl_residual_x << ','
             << record->dvl_residual_y << ','
             << record->dvl_residual_z << '\n';
        csv_.flush();
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_sub_;
    std::ofstream csv_;
    std::string output_path_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamDebugRecorderNode>());
    rclcpp::shutdown();
    return 0;
}
