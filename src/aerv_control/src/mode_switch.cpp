#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mqtt/async_client.h"
#include <nlohmann/json.hpp>
#include <thread>
#include <mutex>

using json = nlohmann::json;

const std::string MQTT_SERVER_ADDRESS = "tcp://hawk.rmq.cloudamqp.com:1883";
const std::string MQTT_CLIENT_ID = "ros2_mqtt_pc2";
const std::string MQTT_USERNAME = "kxndtpqt:kxndtpqt";
const std::string MQTT_PASSWORD = "xIzj0y2xst_uMj2EI5c0t-a32m5vhBIX";

class SwitchNode : public rclcpp::Node
{
public:
    SwitchNode()
        : Node("mode_switch_node"),
          mqtt_client_(MQTT_SERVER_ADDRESS, MQTT_CLIENT_ID),
          emerg_flag_(0), prev_emerg_flag_(-1)
    {
        // Connect to MQTT broker
        mqtt::connect_options connOpts;
        connOpts.set_user_name(MQTT_USERNAME);
        connOpts.set_password(MQTT_PASSWORD);

        try {
            mqtt_client_.connect(connOpts)->wait();
            RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker.");
        } catch (const mqtt::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "MQTT Connection failed: %s", e.what());
        }

        mqtt_client_.start_consuming();
        mqtt_client_.subscribe("/emerg", 1)->wait();

        mqtt_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                auto msg = mqtt_client_.consume_message();
                if (!msg) continue;

                try {
                    auto topic = msg->get_topic();
                    auto payload = msg->to_string();

                    if (topic == "/emerg") {
                        auto j = json::parse(payload);
                        if (j.is_number_integer()) {
                            std::lock_guard<std::mutex> lock(flag_mutex_);
                            emerg_flag_ = j.get<int>();
                        }
                    }
                } catch (const std::exception &e) {
                    RCLCPP_WARN(this->get_logger(), "MQTT parse error: %s", e.what());
                }
            }
        });

        teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_tele", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                teleop_msg_ = *msg;
            });

        auton_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_smoothed", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                auton_msg_ = *msg;
            });

        final_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);  // NEW

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SwitchNode::publish_velocity, this));
    }

    ~SwitchNode()
    {
        mqtt_client_.stop_consuming();
        mqtt_client_.disconnect()->wait();
        if (mqtt_thread_.joinable())
            mqtt_thread_.join();
    }

private:
    mqtt::async_client mqtt_client_;
    std::thread mqtt_thread_;
    std::mutex flag_mutex_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auton_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr final_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist teleop_msg_;
    geometry_msgs::msg::Twist auton_msg_;
    int emerg_flag_;
    int prev_emerg_flag_;

    void publish_velocity()
    {
        geometry_msgs::msg::Twist cmd_to_publish;
        {
            std::lock_guard<std::mutex> lock(flag_mutex_);
            cmd_to_publish = (emerg_flag_ == 1) ? auton_msg_ : teleop_msg_;

            if (emerg_flag_ == 1 && prev_emerg_flag_ != 1) {
            // Publish the goal_pose ONCE when switching to emerg = 1
            geometry_msgs::msg::PoseStamped goal_msg;
            goal_msg.header.stamp = this->now();
            goal_msg.header.frame_id = "map";  // or "odom", depending on your use case
            goal_msg.pose.position.x = 15.5;
            goal_msg.pose.position.y = -5.5;
            goal_msg.pose.position.z = 0.0;
            goal_msg.pose.orientation.w = 1.0;
            goal_msg.pose.orientation.x = 0.0;
            goal_msg.pose.orientation.y = 0.0;
            goal_msg.pose.orientation.z = 1.53;

            goal_pose_pub_->publish(goal_msg);
            RCLCPP_INFO(this->get_logger(), "Published goal_pose on emergency mode.");
            }

            prev_emerg_flag_ = emerg_flag_;  // update tracker
        }
        final_pub_->publish(cmd_to_publish);
    }
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwitchNode>());
    rclcpp::shutdown();
    return 0;
}
