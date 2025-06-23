#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <deque>
#include <cmath>
#include <eigen3/Eigen/Geometry>

using std::placeholders::_1;

class ImuFilterNode : public rclcpp::Node {
public:
    ImuFilterNode()
    : Node("imu_filter_node") {
        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ImuFilterNode::imu_callback, this, _1));
        pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    std::deque<sensor_msgs::msg::Imu> buffer_;
    const size_t window_size_ = 10;

    // Separate thresholds
    const double accel_threshold_x_ = 2.5;  // m/s²
    const double accel_threshold_y_ = 2.5;
    const double gyro_threshold_z_  = 2.05; // rad/s
    const double orientation_threshold_ = 2.05; // in quaternion distance

    sensor_msgs::msg::Imu::SharedPtr last_valid_orientation_ = nullptr;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        sensor_msgs::msg::Imu filtered = *msg;

        // Set covariances
        filtered.orientation_covariance = {0.0025, 0.0, 0.0,
                                            0.0, 0.0025, 0.0,
                                            0.0, 0.0, 0.0025};
        filtered.angular_velocity_covariance = {0.02, 0.0, 0.0,
                                                 0.0, 0.02, 0.0,
                                                 0.0, 0.0, 0.02};
        filtered.linear_acceleration_covariance = {0.4, 0.0, 0.0,
                                                    0.0, 0.4, 0.0,
                                                    0.0, 0.0, 0.4};

        // Push to buffer
        buffer_.push_back(*msg);
        if (buffer_.size() > window_size_)
            buffer_.pop_front();

        if (buffer_.size() == window_size_ && is_stationary()) {
            // IMU is stationary
            // Force Z acceleration = 9.81 (gravity), X/Y = 0
            filtered.linear_acceleration.x = 0.0;
            filtered.linear_acceleration.y = 0.0;
            filtered.linear_acceleration.z = 9.81;

            // Zero gyroscope except z
            filtered.angular_velocity.x = 0.0;
            filtered.angular_velocity.y = 0.0;
            // Keep z gyro as is (or set to 0 if you want)

            // Fix orientation
            if (last_valid_orientation_) {
                filtered.orientation = last_valid_orientation_->orientation;
            }
        } else {
            // Moving → selective updates
            if (std::abs(msg->linear_acceleration.x) > accel_threshold_x_)
                filtered.linear_acceleration.x = msg->linear_acceleration.x;
            else
                filtered.linear_acceleration.x = 0.0;

            if (std::abs(msg->linear_acceleration.y) > accel_threshold_y_)
                filtered.linear_acceleration.y = msg->linear_acceleration.y;
            else
                filtered.linear_acceleration.y = 0.0;

            // Always force z acceleration = 9.81
            filtered.linear_acceleration.z = 9.81;

            // Gyroscope filtering
            filtered.angular_velocity.x = 0.0;
            filtered.angular_velocity.y = 0.0;
            if (std::abs(msg->angular_velocity.z) > gyro_threshold_z_)
                filtered.angular_velocity.z = msg->angular_velocity.z;
            else
                filtered.angular_velocity.z = 0.0;

            // Orientation update
            if (!last_valid_orientation_ || orientation_changed(msg)) {
                last_valid_orientation_ = std::make_shared<sensor_msgs::msg::Imu>(*msg);
                filtered.orientation = msg->orientation;
            } else if (last_valid_orientation_) {
                filtered.orientation = last_valid_orientation_->orientation;
            }
        }

        pub_->publish(filtered);
    }

    bool is_stationary() {
        double mean_ax = 0.0, mean_ay = 0.0;
        for (const auto& msg : buffer_) {
            mean_ax += msg.linear_acceleration.x;
            mean_ay += msg.linear_acceleration.y;
        }
        mean_ax /= window_size_;
        mean_ay /= window_size_;

        for (const auto& msg : buffer_) {
            if (std::abs(msg.linear_acceleration.x - mean_ax) > accel_threshold_x_ ||
                std::abs(msg.linear_acceleration.y - mean_ay) > accel_threshold_y_)
                return false;
        }
        return true;
    }

    bool orientation_changed(const sensor_msgs::msg::Imu::SharedPtr& new_msg) {
        Eigen::Quaterniond q_last(
            last_valid_orientation_->orientation.w,
            last_valid_orientation_->orientation.x,
            last_valid_orientation_->orientation.y,
            last_valid_orientation_->orientation.z
        );
        Eigen::Quaterniond q_new(
            new_msg->orientation.w,
            new_msg->orientation.x,
            new_msg->orientation.y,
            new_msg->orientation.z
        );

        double angular_distance = q_last.angularDistance(q_new);
        return angular_distance > orientation_threshold_;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuFilterNode>());
    rclcpp::shutdown();
    return 0;
}
