#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "jsoncpp/json/json.h"
#include <mqtt/async_client.h>

class MQTTAppNode : public rclcpp::Node {
public:
    MQTTAppNode()
    : Node("mqtt_app_node"),
      mqtt_client_("tcp://hawk.rmq.cloudamqp.com:1883", "ros2_mqtt_cpp"),
      mqtt_options_()
    {
        // Set credentials
        mqtt_options_.set_user_name("kxndtpqt:kxndtpqt");
        mqtt_options_.set_password("xIzj0y2xst_uMj2EI5c0t-a32m5vhBIX");

        // Connect to MQTT broker
        try {
            mqtt_client_.connect(mqtt_options_)->wait();
            RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker.");
        } catch (const mqtt::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "MQTT connect failed: %s", e.what());
        }

        // ROS 2 subscriptions
        fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10,
            std::bind(&MQTTAppNode::fixCallback, this, std::placeholders::_1)
        );

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", 10,
            std::bind(&MQTTAppNode::mapCallback, this, std::placeholders::_1)
        );
    }

private:
    mqtt::async_client mqtt_client_;
    mqtt::connect_options mqtt_options_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        Json::Value json;
        json["latitude"] = msg->latitude;
        json["longitude"] = msg->longitude;

        Json::StreamWriterBuilder writer;
        std::string payload = Json::writeString(writer, json);

        mqtt_client_.publish("/location_updates", payload.c_str(), payload.size(), 0, false);
        RCLCPP_INFO(this->get_logger(), "Published GPS to /location_updates");
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        Json::Value json;
        json["width"] = msg->info.width;
        json["height"] = msg->info.height;
        json["resolution"] = msg->info.resolution;

        Json::Value origin;
        origin["x"] = msg->info.origin.position.x;
        origin["y"] = msg->info.origin.position.y;
        origin["z"] = msg->info.origin.position.z;
        json["origin"] = origin;

        Json::Value data(Json::arrayValue);
        for (size_t i = 0; i < msg->data.size(); ++i) {
            data.append(msg->data[i]);
        }
        json["data"] = data;

        Json::StreamWriterBuilder writer;
        std::string payload = Json::writeString(writer, json);

        mqtt_client_.publish("/map", payload.c_str(), payload.size(), 0, false);
        RCLCPP_INFO(this->get_logger(), "Published map to /map");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MQTTAppNode>());
    rclcpp::shutdown();
    return 0;
}
