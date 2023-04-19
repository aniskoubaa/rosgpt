# RosGPT# rosGPT: Natural Language Commands for ROS

rosGPT is a ROS2 and ROS package that enables humans to send natural language commands to a robot. The package converts these commands to a JSON format and then to the appropriate ROS commands for execution.

## Overview

rosGPT consists of:

1. A `chatgpt_ros2.py` ROS2 node in the `ros2_chatgpt_python` package, which serves as a REST server that receives text commands and publishes them to the `voice_cmd` topic.
2. A `tb3.py` node that subscribes to the `voice_cmd` topic, parses the JSON message, and executes the received command.
3. An `index.html` page with `script.js` and `style.css` files, providing a user interface for humans to send voice commands. The voice command is converted to text and sent as a natural language command to the REST server in the `chatgpt_ros2` node.

## Installation

1. Clone the repository to your workspace:

```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/your_username/rosGPT.git
