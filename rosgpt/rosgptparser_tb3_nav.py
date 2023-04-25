#!/usr/bin/env python3
# This file is part of rosgpt package.
#
# Copyright (c) 2023 Anis Koubaa.
# All rights reserved.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

import rclpy
import json
import threading
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


# Sample JSON database
locations_json = """
[
    {"name": "kitchen", "x": 6.5, "y": 1.0, "theta": 0.0},
    {"name": "living_room", "x": 0.0, "y": 1.0, "theta": 0.0},
    {"name": "bedroom", "x": -3.6, "y": -1, "theta": 0.0}
]
"""

locations = json.loads(locations_json)

class LocationNavigationNode(Node):
    def __init__(self):
        super().__init__('location_navigation_node')
        self.subscription = self.create_subscription(
            String,
            'voice_cmd',
            self.voice_cmd_callback,
            10
        )
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Location Navigation Node is ready')

    def voice_cmd_callback(self, msg):
        voice_command = msg.data.lower()

        for location in locations:
            if location["name"] in voice_command:
                self.send_navigation_goal(location)
                break
            else:
                print("ERROR:No location found in the received JSON command")

    def send_navigation_goal(self, location: dict) -> None:
        """
        Sends a navigation goal to the action server.

        :param location: A dictionary containing the location information (name, x, y, theta).
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = location["x"]
        goal_msg.pose.pose.position.y = location["y"]
        goal_msg.pose.pose.orientation.z = location["theta"]

        self.get_logger().info(f'Sending navigation goal: {location["name"]}')
        self.action_client.wait_for_server()
        goal_handle = self.action_client.send_goal_async(goal_msg)
        goal_handle.add_done_callback(self.navigation_goal_done_callback)

    def navigation_goal_done_callback(self, future) -> None:
        """
        Callback function that is called when a navigation goal is done.

        :param future: A Future object that contains the result of the goal.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

    def read_keyboard_input(self) -> None:
        """
        Reads keyboard input from the user and sends a navigation goal accordingly.
        """
        while True:
            try:
                location_number = int(input("Enter a location number (1 - kitchen, 2 - living room, 3 - bedroom): "))
                if 1 <= location_number <= len(locations):
                    self.send_navigation_goal(locations[location_number - 1])
                else:
                    self.get_logger().info("Invalid location number. Please try again.")
            except ValueError:
                self.get_logger().info("Invalid input. Please enter a number.")

def main(args=None) -> None:
    rclpy.init(args=args)
    location_navigation_node = LocationNavigationNode()

    # Start keyboard input thread
    keyboard_input_thread = threading.Thread(target=location_navigation_node.read_keyboard_input)
    keyboard_input_thread.daemon = True
    keyboard_input_thread.start()

    rclpy.spin(location_navigation_node)
    location_navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
