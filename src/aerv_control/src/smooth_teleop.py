#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
import threading


class SmoothTeleopNode(Node):
    def __init__(self):
        super().__init__('smooth_teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_tele', 10)

        self.max_linear = 2.0
        self.max_angular = 1.0
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        self.increment = 0.002
        self.keys_pressed = set()

        self.create_timer(0.05, self.publish_cmd)

    def publish_cmd(self):
        twist = Twist()

        # Linear velocity
        if 'w' in self.keys_pressed:
            self.linear_speed = min(self.linear_speed + self.increment, self.max_linear)
        elif 's' in self.keys_pressed:
            self.linear_speed = max(self.linear_speed - self.increment, -self.max_linear)
        else:
            self.linear_speed = 0.0

        # Angular velocity
        if 'd' in self.keys_pressed:
            self.angular_speed = min(self.angular_speed - self.increment, self.max_angular)
        elif 'a' in self.keys_pressed:
            self.angular_speed = max(self.angular_speed + self.increment, -self.max_angular)
        else:
            self.angular_speed = 0.0

        print(f'Linear velocity: {self.linear_speed} m/s, Angular velocity: {self.angular_speed} rad/s')

        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher_.publish(twist)

    def on_press(self, key):
        try:
            k = key.char
        except AttributeError:
            return

        if k == 'f':
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            self.keys_pressed.clear()
        else:
            self.keys_pressed.add(k)

    def on_release(self, key):
        try:
            k = key.char
        except AttributeError:
            return
        self.keys_pressed.discard(k)


def main():
    rclpy.init()
    node = SmoothTeleopNode()

    # Start ROS spin in a background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Keyboard listener in main thread
    with keyboard.Listener(
        on_press=node.on_press,
        on_release=node.on_release
    ) as listener:
        listener.join()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
