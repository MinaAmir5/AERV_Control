cmake_minimum_required(VERSION 3.8)
project(aerv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf_tutorial REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PahoMqttCpp REQUIRED)

include_directories(include)
include_directories(/usr/include/jsoncpp)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

install (
  DIRECTORY meshes urdf launch rviz config maps src worlds include
  DESTINATION share/${PROJECT_NAME}
)

add_executable(vehicle_controller src/vehicle_controller.cpp)
ament_target_dependencies(vehicle_controller 
  rclcpp 
  std_msgs
  geometry_msgs
)

install(TARGETS
  vehicle_controller
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(imu_filter_node src/imu_filter_node.cpp)
ament_target_dependencies(imu_filter_node rclcpp sensor_msgs nav_msgs)
install(TARGETS imu_filter_node DESTINATION lib/${PROJECT_NAME})

add_executable(mqtt_bridge_node src/mqtt_bridge_node.cpp)
ament_target_dependencies(mqtt_bridge_node rclcpp std_msgs sensor_msgs PahoMqttCpp nav_msgs)
target_link_libraries(mqtt_bridge_node paho-mqttpp3 paho-mqtt3as)
install(TARGETS mqtt_bridge_node DESTINATION lib/${PROJECT_NAME})

add_executable(switch_node src/mode_switch.cpp)
ament_target_dependencies(switch_node rclcpp std_msgs geometry_msgs PahoMqttCpp)
target_link_libraries(switch_node paho-mqttpp3 paho-mqtt3as)
install(TARGETS switch_node DESTINATION lib/${PROJECT_NAME})

add_executable(laser_to_range_node src/laser_to_range_node.cpp)
ament_target_dependencies(laser_to_range_node rclcpp sensor_msgs)
install(TARGETS laser_to_range_node DESTINATION lib/${PROJECT_NAME})


add_executable(mqtt_app_node src/mqtt_app.cpp)
ament_target_dependencies(mqtt_app_node rclcpp sensor_msgs nav_msgs PahoMqttCpp)
target_link_libraries(mqtt_app_node paho-mqttpp3 paho-mqtt3as jsoncpp)
install(TARGETS mqtt_app_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

