#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "mqtt/async_client.h"
#include <nlohmann/json.hpp>
#include <thread>

using json = nlohmann::json;

const std::string MQTT_SERVER_ADDRESS = "tcp://hawk.rmq.cloudamqp.com:1883";
const std::string MQTT_CLIENT_ID = "ros2_mqtt_pc";
const std::string MQTT_USERNAME = "kxndtpqt:kxndtpqt";
const std::string MQTT_PASSWORD = "xIzj0y2xst_uMj2EI5c0t-a32m5vhBIX";
const double imu_publish_interval_sec_ = 1.0;  // 5 Hz

const double cmd_publish_interval_sec_ = 1.0;  // 5 Hz

#define mqtt_pi 1
#define mqtt_pc 2

#define mqtt_host mqtt_pc // Change this line only. options: mqtt_pi, mqtt_pc

#if mqtt_host == mqtt_pi

class MQTTBridgeNode : public rclcpp::Node {
public:
  MQTTBridgeNode()
  : Node("mqtt_bridge_node"),
    mqtt_client_(MQTT_SERVER_ADDRESS, MQTT_CLIENT_ID)
  {
    mqtt::connect_options connOpts;
    connOpts.set_user_name(MQTT_USERNAME);
    connOpts.set_password(MQTT_PASSWORD);
    connOpts.set_clean_session(true);
    connOpts.set_automatic_reconnect(true);

    try {
      mqtt_client_.connect(connOpts)->wait();
      RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker.");
    } catch (const mqtt::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "MQTT Connection failed: %s", e.what());
    }


    sub_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>("/fix", 10,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            json j = {
                {"latitude", msg->latitude},
                {"longitude", msg->longitude},
                {"altitude", msg->altitude}
            };
            publish_mqtt_atmostonce("/fix", j.dump());
        });

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>("/imu", 10,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            rclcpp::Clock clock(RCL_SYSTEM_TIME);
            auto now = clock.now();
            if ((now - last_imu_time_).seconds() < imu_publish_interval_sec_) return;
            last_imu_time_ = now;
            json j = {
                {"orientation", {
                    {"x", msg->orientation.x},
                    {"y", msg->orientation.y},
                    {"z", msg->orientation.z},
                    {"w", msg->orientation.w}
                }}
            };
            publish_mqtt_atmostonce("/imu", j.dump());
        });

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            json j = {
                {"angle_min", msg->angle_min},
                {"angle_max", msg->angle_max},
                {"ranges", msg->ranges}
            };
            publish_mqtt_atmostonce("/scan", j.dump());
        });

    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            json j = {
                {"linear", {
                    {"x", msg->linear.x},
                    {"y", msg->linear.y},
                    {"z", msg->linear.z}
                }},
                {"angular", {
                    {"x", msg->angular.x},
                    {"y", msg->angular.y},
                    {"z", msg->angular.z}
                }}
            };
            publish_mqtt_atleastonce("/cmd_vel", j.dump());
        });

        pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_goal_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        mqtt_client_.start_consuming();
        mqtt_client_.subscribe("/cmd_vel", 1)->wait();
        mqtt_client_.subscribe("/goal_pose", 1)->wait();

        mqtt_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                auto msg = mqtt_client_.consume_message();
                if (!msg) continue;

                if (msg->get_topic() == "/cmd_vel") {
                    auto j = json::parse(msg->to_string());
                    geometry_msgs::msg::Twist twist;
                    twist.linear.x = j["linear"]["x"];
                    twist.linear.y = j["linear"]["y"];
                    twist.linear.z = j["linear"]["z"];
                    twist.angular.x = j["angular"]["x"];
                    twist.angular.y = j["angular"]["y"];
                    twist.angular.z = j["angular"]["z"];
                    pub_cmd_vel_->publish(twist);
                }
                else if (msg->get_topic() == "/goal_pose") {
                    auto j = json::parse(msg->to_string());
                    geometry_msgs::msg::PoseStamped pose;
                    pose.pose.position.x = j["position"]["x"];
                    pose.pose.position.y = j["position"]["y"];
                    pose.pose.position.z = j["position"]["z"];
                    pose.pose.orientation.x = j["orientation"]["x"];
                    pose.pose.orientation.y = j["orientation"]["y"];
                    pose.pose.orientation.z = j["orientation"]["z"];
                    pose.pose.orientation.w = j["orientation"]["w"];
                    pub_goal_pose_->publish(pose);
                }
            }
        });
  }

  ~MQTTBridgeNode() {
    mqtt_client_.stop_consuming();
    mqtt_client_.disconnect()->wait();
    if (mqtt_thread_.joinable()) mqtt_thread_.join();
  }

private:
  mqtt::async_client mqtt_client_;
  std::thread mqtt_thread_;
  rclcpp::Time last_imu_time_{0, 0, RCL_SYSTEM_TIME};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;

  void publish_mqtt_atleastonce(const std::string &topic, const std::string &payload) {
    auto msg = mqtt::make_message(topic, payload);
    msg->set_qos(1);
    mqtt_client_.publish(msg);
  }
  void publish_mqtt_atmostonce(const std::string &topic, const std::string &payload) {
    auto msg = mqtt::make_message(topic, payload);
    msg->set_qos(0);
    mqtt_client_.publish(msg);
  }
};



#elif mqtt_host == mqtt_pc

class MQTTBridgeNode : public rclcpp::Node {
    public:
      MQTTBridgeNode()
      : Node("mqtt_bridge_node"),
        mqtt_client_(MQTT_SERVER_ADDRESS, MQTT_CLIENT_ID)
      {
        mqtt::connect_options connOpts;
        connOpts.set_user_name(MQTT_USERNAME);
        connOpts.set_password(MQTT_PASSWORD);
    
        try {
          mqtt_client_.connect(connOpts)->wait();
          RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker.");
        } catch (const mqtt::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "MQTT Connection failed: %s", e.what());
        }

        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                rclcpp::Clock clock(RCL_SYSTEM_TIME);
                auto now = clock.now();
                if ((now - last_cmd_time_).seconds() < cmd_publish_interval_sec_) return;
                last_cmd_time_ = now;
                json j = {
                    {"linear", {{"x", msg->linear.x}, {"y", msg->linear.y}, {"z", msg->linear.z}}},
                    {"angular", {{"x", msg->angular.x}, {"y", msg->angular.y}, {"z", msg->angular.z}}}
                };
                publish_mqtt("/cmd_vel", j.dump());
            });

        sub_goal_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                json j = {
                    {"position", {
                        {"x", msg->pose.position.x},
                        {"y", msg->pose.position.y},
                        {"z", msg->pose.position.z}
                    }},
                    {"orientation", {
                        {"x", msg->pose.orientation.x},
                        {"y", msg->pose.orientation.y},
                        {"z", msg->pose.orientation.z},
                        {"w", msg->pose.orientation.w}
                    }}
                };
                publish_mqtt("/goal_pose", j.dump());
            });

        // MQTT ➝ ROS
        pub_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);
        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 4);
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 4);

        mqtt_client_.start_consuming();
        mqtt_client_.subscribe("/fix", 1)->wait();
        mqtt_client_.subscribe("/imu", 1)->wait();
        mqtt_client_.subscribe("/scan", 1)->wait();
        mqtt_client_.subscribe("/cmd_vel", 1)->wait();

        mqtt_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                auto msg = mqtt_client_.consume_message();
                if (!msg) continue;

                auto topic = msg->get_topic();
                auto payload = msg->to_string();
                auto j = json::parse(payload);

                if (topic == "/fix") {
                    sensor_msgs::msg::NavSatFix fix;
                    fix.latitude = j["latitude"];
                    fix.longitude = j["longitude"];
                    fix.altitude = j["altitude"];
                    pub_fix_->publish(fix);
                }
                else if (topic == "/imu") {
                    sensor_msgs::msg::Imu imu;
                    imu.orientation.x = j["orientation"]["x"];
                    imu.orientation.y = j["orientation"]["y"];
                    imu.orientation.z = j["orientation"]["z"];
                    imu.orientation.w = j["orientation"]["w"];
                    pub_imu_->publish(imu);
                }
                else if (topic == "/scan") {
                    sensor_msgs::msg::LaserScan scan;
                    scan.angle_min = j["angle_min"];
                    scan.angle_max = j["angle_max"];
                    for (auto r : j["ranges"])
                        scan.ranges.push_back(r);
                    pub_scan_->publish(scan);
                }
                else if (topic == "/cmd_vel") {
                    geometry_msgs::msg::Twist twist;
                    twist.linear.x = j["linear"]["x"];
                    twist.linear.y = j["linear"]["y"];
                    twist.linear.z = j["linear"]["z"];
                    twist.angular.x = j["angular"]["x"];
                    twist.angular.y = j["angular"]["y"];
                    twist.angular.z = j["angular"]["z"];
                    pub_cmd_vel_->publish(twist);
                }
            }
        });
    }

    ~MQTTBridgeNode() {
        mqtt_client_.stop_consuming();
        mqtt_client_.disconnect()->wait();
        if (mqtt_thread_.joinable()) mqtt_thread_.join();
    }
    private:
    mqtt::async_client mqtt_client_;
    std::thread mqtt_thread_;
    rclcpp::Time last_cmd_time_{0, 0, RCL_SYSTEM_TIME};

    // ROS ➝ MQTT
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;

    // MQTT ➝ ROS
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_fix_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

    void publish_mqtt(const std::string& topic, const std::string& payload) {
        mqtt::message_ptr pubmsg = mqtt::make_message(topic, payload);
        pubmsg->set_qos(1);
        mqtt_client_.publish(pubmsg);
    }
};

#endif

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MQTTBridgeNode>());
    rclcpp::shutdown();
    return 0;
  }