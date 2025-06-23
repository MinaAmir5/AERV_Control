// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/imu.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "sensor_msgs/msg/nav_sat_fix.hpp"
// #include <deque>
// #include <cmath>
// #include <eigen3/Eigen/Geometry>

// using std::placeholders::_1;

// class ImuFilterNode : public rclcpp::Node {
// public:
//     ImuFilterNode()
//     : Node("imu_filter_node"), vx_(0.0), vy_(0.0), x_(0.0), y_(0.0) {
//         sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
//             "/imu", 10, std::bind(&ImuFilterNode::imu_callback, this, _1));
//         pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", 10);
//         sub_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>(
//             "/fix", 10, std::bind(&ImuFilterNode::gps_callback, this, _1));
//         pub_gps = this->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);
//         odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/imu", 10);
//     }

// private:
//     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
//     rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
//     rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps;
//     rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps;
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

//     rclcpp::Time last_time_;
//     double vx_, vy_; // Velocity estimates
//     double x_, y_;   // Position estimates

//     const double accel_threshold_x_ = 0.03;
//     const double accel_threshold_y_ = 0.03;
//     const double gyro_threshold_z_ = 0.05;
//     const double orientation_threshold_ = 0.001;

//     sensor_msgs::msg::Imu::SharedPtr last_valid_orientation_ = nullptr;

//     void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
//         rclcpp::Time current_time = msg->header.stamp;

//         sensor_msgs::msg::Imu filtered = *msg;
//         filtered.header.frame_id = "imu_link";
//         filtered.orientation_covariance = {0.0025, 0.0, 0.0,
//                                            0.0, 0.0025, 0.0,
//                                            0.0, 0.0, 0.0025};
//         filtered.angular_velocity_covariance = {0.02, 0.0, 0.0,
//                                                 0.0, 0.02, 0.0,
//                                                 0.0, 0.0, 0.02};
//         filtered.linear_acceleration_covariance = {0.4, 0.0, 0.0,
//                                                    0.0, 0.4, 0.0,
//                                                    0.0, 0.0, 0.4};

//         // Acceleration filtering
//         double ax = (std::abs(msg->linear_acceleration.x) > accel_threshold_x_) ? msg->linear_acceleration.x : 0.0;
//         double ay = (std::abs(msg->linear_acceleration.y) > accel_threshold_y_) ? msg->linear_acceleration.y : 0.0;
//         filtered.linear_acceleration.x = ax;
//         filtered.linear_acceleration.y = ay;
//         filtered.linear_acceleration.z = 9.81;

//         // Gyroscope
//         filtered.angular_velocity.x = 0.0;
//         filtered.angular_velocity.y = 0.0;
//         filtered.angular_velocity.z = (std::abs(msg->angular_velocity.z) > gyro_threshold_z_)
//                                       ? msg->angular_velocity.z : 0.0;

//         // Orientation
//         if (!last_valid_orientation_ || orientation_changed(msg)) {
//             last_valid_orientation_ = std::make_shared<sensor_msgs::msg::Imu>(*msg);
//             filtered.orientation = msg->orientation;
//         } else {
//             filtered.orientation = last_valid_orientation_->orientation;
//         }

//         pub_->publish(filtered);

//         // ==== Integration for Odometry ====
//         if (!last_time_.nanoseconds()) {
//             last_time_ = current_time;
//             return;
//         }

//         double dt = (current_time - last_time_).seconds();
//         last_time_ = current_time;
//         if (ax == 0 || ay == 0){
//             vx_ = 0;
//             vy_ = 0;
//         }
//         else
//         {
//             vx_ += ax * dt;
//             vy_ += ay * dt;
//         }
//         x_ += vx_ * dt;
//         y_ += vy_ * dt;

//         nav_msgs::msg::Odometry odom;
//         odom.header.stamp = current_time;
//         odom.header.frame_id = "odom";
//         odom.child_frame_id = "base_footprint";

//         odom.pose.pose.position.x = x_;
//         odom.pose.pose.position.y = y_;
//         odom.pose.pose.position.z = 0.0;
//         odom.pose.pose.orientation = filtered.orientation;

//         odom.twist.twist.linear.x = vx_;
//         odom.twist.twist.linear.y = vy_;
//         odom.twist.twist.angular.z = filtered.angular_velocity.z;

//         odom_pub_->publish(odom);
//     }

//     void gps_callback(volatile sensor_msgs::msg::NavSatFix::SharedPtr msg) {};

//     bool orientation_changed(const sensor_msgs::msg::Imu::SharedPtr& new_msg) {
//         Eigen::Quaterniond q_last(
//             last_valid_orientation_->orientation.w,
//             last_valid_orientation_->orientation.x,
//             last_valid_orientation_->orientation.y,
//             last_valid_orientation_->orientation.z
//         );
//         Eigen::Quaterniond q_new(
//             new_msg->orientation.w,
//             new_msg->orientation.x,
//             new_msg->orientation.y,
//             new_msg->orientation.z
//         );
//         return q_last.angularDistance(q_new) > orientation_threshold_;
//     }
// };


// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/imu.hpp"
// #include "sensor_msgs/msg/nav_sat_fix.hpp"
// #include <deque>
// #include <cmath>
// #include <eigen3/Eigen/Geometry>

// using std::placeholders::_1;

// class ImuFilterNode : public rclcpp::Node {
// public:
//     ImuFilterNode()
//     : Node("imu_filter_node") {
//         sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
//             "/imu", 10, std::bind(&ImuFilterNode::imu_callback, this, _1));
//         sub_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>(
//             "/fix", 10, std::bind(&ImuFilterNode::gps_callback, this, _1));
//         pub_gps = this->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);
//         pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", 10);
//     }

// private:
//     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
//     rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps;
//     rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps;
//     rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

//     // Separate thresholds
//     const double accel_threshold_x_ = 0.5;  // m/s²
//     const double accel_threshold_y_ = 0.5;
//     const double gyro_threshold_z_  = 0.05; // rad/s
//     const double orientation_threshold_ = 0.05; // in quaternion distance

//     sensor_msgs::msg::Imu::SharedPtr last_valid_orientation_ = nullptr;

//     void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
//         sensor_msgs::msg::Imu filtered = *msg;

//         // Set covariances
//         filtered.header.frame_id="imu_link";
//         filtered.orientation_covariance = {0.0025, 0.0, 0.0,
//                                             0.0, 0.0025, 0.0,
//                                             0.0, 0.0, 0.0025};
//         filtered.angular_velocity_covariance = {0.02, 0.0, 0.0,
//                                                  0.0, 0.02, 0.0,
//                                                  0.0, 0.0, 0.02};
//         filtered.linear_acceleration_covariance = {0.4, 0.0, 0.0,
//                                                     0.0, 0.4, 0.0,
//                                                     0.0, 0.0, 0.4};

//         // Acceleration
//         filtered.linear_acceleration.x = (std::abs(msg->linear_acceleration.x) > accel_threshold_x_)
//                                          ? msg->linear_acceleration.x : 0.0;
//         filtered.linear_acceleration.y = (std::abs(msg->linear_acceleration.y) > accel_threshold_y_)
//                                          ? msg->linear_acceleration.y : 0.0;
//         filtered.linear_acceleration.z = 9.81; // always force z-gravity

//         // Gyroscope
//         filtered.angular_velocity.x = 0.0;
//         filtered.angular_velocity.y = 0.0;
//         filtered.angular_velocity.z = (std::abs(msg->angular_velocity.z) > gyro_threshold_z_)
//                                        ? msg->angular_velocity.z : 0.0;

//         // Orientation
//         if (!last_valid_orientation_ || orientation_changed(msg)) {
//             last_valid_orientation_ = std::make_shared<sensor_msgs::msg::Imu>(*msg);
//             filtered.orientation = msg->orientation;
//             // if (filtered.angular_velocity.z == 0.0 && filtered.linear_acceleration.x == 0.0 && filtered.linear_acceleration.y == 0.0) {
//             //     filtered.orientation.x = 0.0;
//             //     filtered.orientation.y = 0.0;
//             //     filtered.orientation.z = 0.0;
//             //     filtered.orientation.w = 1.0;
//             // } else {
//             //     filtered.orientation = msg->orientation;
//             //     last_valid_orientation_ = std::make_shared<sensor_msgs::msg::Imu>(*msg);
//             // }
//         } else if (last_valid_orientation_) {
//             filtered.orientation = last_valid_orientation_->orientation;
//         }

//         pub_->publish(filtered);
//     }

//     void gps_callback(volatile sensor_msgs::msg::NavSatFix::SharedPtr msg) {};

//     bool orientation_changed(const sensor_msgs::msg::Imu::SharedPtr& new_msg) {
//         Eigen::Quaterniond q_last(
//             last_valid_orientation_->orientation.w,
//             last_valid_orientation_->orientation.x,
//             last_valid_orientation_->orientation.y,
//             last_valid_orientation_->orientation.z
//         );
//         Eigen::Quaterniond q_new(
//             new_msg->orientation.w,
//             new_msg->orientation.x,
//             new_msg->orientation.y,
//             new_msg->orientation.z
//         );

//         double angular_distance = q_last.angularDistance(q_new);
//         return angular_distance > orientation_threshold_;
//     }
// };

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <eigen3/Eigen/Geometry>

using std::placeholders::_1;

class ImuFilterNode : public rclcpp::Node {
public:
    ImuFilterNode()
    : Node("imu_filter_node"), x_(0.0), y_(0.0), theta_(0.0) {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ImuFilterNode::imu_callback, this, _1));
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ImuFilterNode::cmd_callback, this, _1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/cmd", 10);

        last_time_ = this->now();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    sensor_msgs::msg::Imu::SharedPtr last_valid_orientation_;
    rclcpp::Time last_time_;
    double x_, y_, theta_; // Pose

    geometry_msgs::msg::Twist last_cmd_vel_;

    const double accel_threshold_x_ = 0.0;
    const double accel_threshold_y_ = 0.0;
    const double gyro_threshold_z_  = 0.00;
    const double orientation_threshold_ = 0.08;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        sensor_msgs::msg::Imu filtered = *msg;
        filtered.header.frame_id = "imu_link";

        filtered.orientation_covariance = {0.0025, 0.0, 0.0,
                                           0.0, 0.0025, 0.0,
                                           0.0, 0.0, 0.0025};
        filtered.angular_velocity_covariance = {0.02, 0.0, 0.0,
                                                0.0, 0.02, 0.0,
                                                0.0, 0.0, 0.02};
        filtered.linear_acceleration_covariance = {0.4, 0.0, 0.0,
                                                   0.0, 0.4, 0.0,
                                                   0.0, 0.0, 0.4};

        filtered.linear_acceleration.x = (std::abs(msg->linear_acceleration.x) > accel_threshold_x_)
                                         ? msg->linear_acceleration.x : 0.0;
        filtered.linear_acceleration.y = (std::abs(msg->linear_acceleration.y) > accel_threshold_y_)
                                         ? msg->linear_acceleration.y : 0.0;
        filtered.linear_acceleration.z = 9.81;

        filtered.angular_velocity.x = 0.0;
        filtered.angular_velocity.y = 0.0;
        filtered.angular_velocity.z = (std::abs(msg->angular_velocity.z) > gyro_threshold_z_)
                                       ? msg->angular_velocity.z : 0.0;

        if (!last_valid_orientation_ || orientation_changed(msg)) {
            last_valid_orientation_ = std::make_shared<sensor_msgs::msg::Imu>(*msg);
            filtered.orientation = msg->orientation;
        } else {
            filtered.orientation = last_valid_orientation_->orientation;
        }

        imu_pub_->publish(filtered);
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        double v = msg->linear.x;
        double w = msg->angular.z;

        // Integrate pose
        theta_ += w * dt;
        x_ += v * std::cos(theta_) * dt;
        y_ += v * std::sin(theta_) * dt;

        // Publish synthetic odometry
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation = create_quaternion_from_yaw(theta_);

        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = w;

        odom_pub_->publish(odom);
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
        return q_last.angularDistance(q_new) > orientation_threshold_;
    }

    geometry_msgs::msg::Quaternion create_quaternion_from_yaw(double yaw) {
        geometry_msgs::msg::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw * 0.5);
        q.w = std::cos(yaw * 0.5);
        return q;
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuFilterNode>());
    rclcpp::shutdown();
    return 0;
}
