#!/usr/bin/env python3
# This file is part of rosgpt package.
#
# Copyright (c) 2023 Anis Koubaa.
# All rights reserved.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.
# ================================================== DESCRIPTION ===============================================================
#This code defines a ROS (Robot Operating System) client node that sends text commands to the ROSGPT system, 
# which is a natural language processing system based on the GPT (Generative Pre-trained Transformer) model. 
#The ROSGPT system processes the text command using the ChatGPT language model and returns a response that is printed to the console. 
#The ROSGPTClient class is defined as a subclass of the Node class.
#The client node uses the requests library to send HTTP requests to the ROSGPT server and receive responses. 
#The ROS2 spin() function is called to run the ROSGPT client node.
#=============================================================================================================================
import json
import rclpy
from rclpy.node import Node
import requests


class ROSGPTClient(Node):
    def __init__(self):
        super().__init__('rosgpt_client')
        self.declare_parameter('server_url', 'http://localhost:5000/rosgpt')
        self.server_url = self.get_parameter('server_url').value

        self.get_logger().info('ROSGPT client node started')

        self.send_text_command()

    def send_text_command(self):
        """
        Sends a text command to the ROSGPT system and receives a response from the ChatGPT language model.
        """
        text_command = input("Enter a text command: ")
        data = {'text_command': text_command}

        response = requests.post(self.server_url, data=data)

        if response.status_code == 200:
            response_str = response.content.decode('utf-8')
            response_dict = json.loads(response_str)

            self.get_logger().info('Response: {}'.format(response_dict['text']))
            self.get_logger().info('JSON: {}'.format(json.loads(response_dict['json'])))
        else:
            self.get_logger().error('Error: {}'.format(response.status_code))


def main(args=None):
    rclpy.init(args=args)

    rosgpt_client = ROSGPTClient()

    rclpy.spin(rosgpt_client)

    rosgpt_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
