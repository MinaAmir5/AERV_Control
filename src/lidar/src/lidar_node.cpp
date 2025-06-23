// lidar_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <chrono>
#include <cstdio>
#include <memory>
#include <cstdlib>
#include <sstream>

using namespace std::chrono_literals;

class LidarNode : public rclcpp::Node {
public:
  LidarNode() : Node("lidar_node") {
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&LidarNode::publish_scan, this));

    std::string cmd = "../ldlidar_stl_sdk/build/ldlidar_stl_node LD06 serialcom /dev/ttyUSB0";
    lidar_proc_ = popen(cmd.c_str(), "r");
    if (!lidar_proc_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start LiDAR process.");
      rclcpp::shutdown();
    }
  }

  ~LidarNode() {
    if (lidar_proc_) {
      pclose(lidar_proc_);
    }
  }

private:
  void publish_scan() {
    if (!lidar_proc_) return;

    std::vector<float> ranges(360, std::numeric_limits<float>::infinity());
    std::vector<float> intensities(360, 0.0);

    char buffer[512];
    int points_read = 0;

    while (points_read < 500 && fgets(buffer, sizeof(buffer), lidar_proc_)) {
      std::string line(buffer);
      if (line.find("angle:") != std::string::npos) {
        try {
          float angle = std::stof(line.substr(line.find("angle:") + 6));
          float distance = std::stof(line.substr(line.find("distance(mm):") + 13));
          float intensity = std::stof(line.substr(line.find("intensity:") + 10));

          if (distance > 0 && distance < 12000 && angle >= 0 && angle < 360) {
            int index = static_cast<int>(angle);
            ranges[index] = distance / 1000.0;  // Convert mm to meters
            intensities[index] = intensity;
            ++points_read;
          }
        } catch (...) {
          continue;
        }
      }
    }

    auto scan_msg = sensor_msgs::msg::LaserScan();
    scan_msg.header.stamp = this->get_clock()->now();
    scan_msg.header.frame_id = "laser_frame";
    scan_msg.angle_min = -M_PI;
    scan_msg.angle_max = M_PI;
    scan_msg.angle_increment = M_PI / 180.0;  // 1 degree
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.1;
    scan_msg.range_min = 0.05;
    scan_msg.range_max = 12.0;
    scan_msg.ranges = ranges;
    scan_msg.intensities = intensities;

    publisher_->publish(scan_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  FILE* lidar_proc_ = nullptr;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
