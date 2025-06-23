// File: laser_to_range_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <algorithm>
#include <string>
#include <vector>

class LaserToRangeNode : public rclcpp::Node {
public:
  LaserToRangeNode() : Node("laser_to_range_node") {
    for (int i = 1; i <= 6; ++i) {
      std::string laser_topic = "/range" + std::to_string(i) + "/scan";
      std::string range_topic = "/range" + std::to_string(i);

      auto sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_topic, 10,
        [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          this->laser_callback(msg, i);
        }
      );

      auto pub = this->create_publisher<sensor_msgs::msg::Range>(range_topic, 10);

      laser_subs_.push_back(sub);
      range_pubs_.push_back(pub);

      RCLCPP_INFO(this->get_logger(), "Subscribed to %s, publishing to %s",
                  laser_topic.c_str(), range_topic.c_str());
    }
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg, int index) {
    // Extract the middle value of the LaserScan
    float range_value = msg->ranges[msg->ranges.size() / 2];

    sensor_msgs::msg::Range range_msg;
    range_msg.header = msg->header;
    range_msg.header.frame_id = "ultrasonic_sensor" + std::to_string(index) + "_link";
    range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    range_msg.field_of_view = msg->angle_max - msg->angle_min;
    range_msg.min_range = msg->range_min;
    range_msg.max_range = msg->range_max;
    range_msg.range = std::clamp(range_value, msg->range_min, msg->range_max);

    range_pubs_[index - 1]->publish(range_msg);
  }

  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> laser_subs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> range_pubs_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserToRangeNode>());
  rclcpp::shutdown();
  return 0;
}
