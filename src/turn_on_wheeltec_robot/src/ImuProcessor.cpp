#include <cmath>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace
{
bool isNearZero(const double value, const double threshold)
{
    return std::abs(value) < threshold;
}

double normalizeAngle(const double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

class ImuProcessor : public rclcpp::Node
{
public:
    ImuProcessor()
    : Node("imu_processor")
    {
        input_topic_ = declare_parameter<std::string>("input_topic", "/imu/data_raw");
        output_topic_ = declare_parameter<std::string>("output_topic", "/imu/data_bias_compensated");
        compatibility_topic_ = declare_parameter<std::string>("compatibility_topic", "/imu/data_filtered");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
        linear_velocity_threshold_ = declare_parameter<double>("linear_velocity_threshold", 0.03);
        angular_velocity_threshold_ = declare_parameter<double>("angular_velocity_threshold", 0.03);
        gyro_z_static_threshold_ = declare_parameter<double>("gyro_z_static_threshold", 0.03);
        bias_alpha_ = declare_parameter<double>("bias_alpha", 0.01);
        startup_bias_samples_ = declare_parameter<int>("startup_bias_samples", 200);
        enable_runtime_bias_update_ = declare_parameter<bool>("enable_runtime_bias_update", true);
        use_odom_for_static_detection_ = declare_parameter<bool>("use_odom_for_static_detection", true);

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ImuProcessor::imuCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&ImuProcessor::odomCallback, this, std::placeholders::_1));

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, rclcpp::SensorDataQoS());
        compatibility_pub_ = create_publisher<sensor_msgs::msg::Imu>(compatibility_topic_, rclcpp::SensorDataQoS());
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        linear_vel_x_ = msg->twist.twist.linear.x;
        linear_vel_y_ = msg->twist.twist.linear.y;
        angular_vel_z_odom_ = msg->twist.twist.angular.z;
        odom_valid_ = true;
    }

    bool isRobotStatic(const sensor_msgs::msg::Imu & msg) const
    {
        const bool imu_static = isNearZero(msg.angular_velocity.z, gyro_z_static_threshold_);
        if (!use_odom_for_static_detection_) {
            return imu_static;
        }

        if (!odom_valid_) {
            return false;
        }

        return imu_static &&
                     isNearZero(linear_vel_x_, linear_velocity_threshold_) &&
                     isNearZero(linear_vel_y_, linear_velocity_threshold_) &&
                     isNearZero(angular_vel_z_odom_, angular_velocity_threshold_);
    }

    void initializeYawFromRawOrientation(const sensor_msgs::msg::Imu & msg)
    {
        tf2::Quaternion raw_q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
        if (raw_q.length2() < 1e-12) {
            corrected_yaw_ = 0.0;
            return;
        }

        raw_q.normalize();
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(raw_q).getRPY(roll, pitch, yaw);
        corrected_yaw_ = yaw;
    }

    void updateBiasEstimate(const sensor_msgs::msg::Imu & msg, const bool is_static)
    {
        if (!is_static) {
            return;
        }

        if (startup_static_samples_ < startup_bias_samples_) {
            startup_static_sum_ += msg.angular_velocity.z;
            startup_static_samples_++;
            gyro_z_bias_ = startup_static_sum_ / static_cast<double>(startup_static_samples_);
            if (startup_static_samples_ == startup_bias_samples_) {
                RCLCPP_INFO(
                    get_logger(),
                    "Initialized gyro z bias to %.8f rad/s after %d stationary samples",
                    gyro_z_bias_, startup_static_samples_);
            }
            return;
        }

        if (enable_runtime_bias_update_) {
            gyro_z_bias_ = (1.0 - bias_alpha_) * gyro_z_bias_ + bias_alpha_ * msg.angular_velocity.z;
        }
    }

    void updateCorrectedYaw(const sensor_msgs::msg::Imu & msg, const bool is_static)
    {
        if (!yaw_initialized_) {
            initializeYawFromRawOrientation(msg);
            yaw_initialized_ = true;
            last_imu_stamp_ = msg.header.stamp;
            return;
        }

        const double dt = (rclcpp::Time(msg.header.stamp) - rclcpp::Time(last_imu_stamp_)).seconds();
        last_imu_stamp_ = msg.header.stamp;

        if (dt <= 0.0 || dt > 0.5) {
            return;
        }

        const double corrected_gyro_z = msg.angular_velocity.z - gyro_z_bias_;
        if (is_static && startup_static_samples_ >= startup_bias_samples_) {
            corrected_yaw_ = normalizeAngle(corrected_yaw_);
            return;
        }

        corrected_yaw_ = normalizeAngle(corrected_yaw_ + corrected_gyro_z * dt);
    }

    sensor_msgs::msg::Imu buildCompensatedMessage(const sensor_msgs::msg::Imu & msg) const
    {
        sensor_msgs::msg::Imu out = msg;

        out.angular_velocity.z = msg.angular_velocity.z - gyro_z_bias_;

        // This topic is the planar attitude input for robot_localization. Keeping
        // raw roll/pitch here lets imu0_relative form Q0^-1 * Qt before
        // two_d_mode zeros those axes, leaking roll/pitch motion into yaw.
        // /imu/data_raw remains unchanged for consumers that need full attitude.
        tf2::Quaternion corrected_q;
        corrected_q.setRPY(0.0, 0.0, corrected_yaw_);
        corrected_q.normalize();

        out.orientation.x = corrected_q.x();
        out.orientation.y = corrected_q.y();
        out.orientation.z = corrected_q.z();
        out.orientation.w = corrected_q.w();
        return out;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const bool is_static = isRobotStatic(*msg);
        updateBiasEstimate(*msg, is_static);
        updateCorrectedYaw(*msg, is_static);

        if (!yaw_initialized_) {
            return;
        }

        const sensor_msgs::msg::Imu compensated = buildCompensatedMessage(*msg);
        imu_pub_->publish(compensated);
        compatibility_pub_->publish(compensated);
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string compatibility_topic_;
    std::string odom_topic_;

    double linear_velocity_threshold_ = 0.03;
    double angular_velocity_threshold_ = 0.03;
    double gyro_z_static_threshold_ = 0.03;
    double bias_alpha_ = 0.01;
    int startup_bias_samples_ = 200;
    bool enable_runtime_bias_update_ = true;
    bool use_odom_for_static_detection_ = true;

    double linear_vel_x_ = 0.0;
    double linear_vel_y_ = 0.0;
    double angular_vel_z_odom_ = 0.0;
    bool odom_valid_ = false;

    double gyro_z_bias_ = 0.0;
    double startup_static_sum_ = 0.0;
    int startup_static_samples_ = 0;

    bool yaw_initialized_ = false;
    double corrected_yaw_ = 0.0;
    builtin_interfaces::msg::Time last_imu_stamp_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr compatibility_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
