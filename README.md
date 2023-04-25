# ROSGPT: ChatGPT Interface for ROS2 for Human-Robot Interaction

ROSGPT is a ROS2 2 package that enables humans to send natural language commands to a robot. 
The package uses ChatGPT to converte unstructured human commands into structured JSON commands, which will be Parsed by the ROSGPTParser to execute the corresponding task. 

## Reference Paper

Anis Koubaa. "ROSGPT: Next-Generation Human-Robot Interaction with ChatGPT and ROS"
Preprints.org 2023, 2023040827. 
https://doi.org/10.20944/preprints202304.0827.v2.

## Video Demo

This video shows a brief demonstration on how to get started with ROSGPT. 

[![ROSGPT VIDEO DEMO](https://img.youtube.com/vi/urkQD-hB5Hg/0.jpg)](https://www.youtube.com/watch?v=urkQD-hB5Hg)


## About ROSGPT ROS2 Package

# ROSGPT ROS2 Package

The ROSGPT ROS2 package contains the following scripts:

- **rosgpt.py**: Implements the ROSGPT node, which is a ROS2 node with a REST server that accepts POST requests containing natural human language text. It translates the text into structured JSON commands through an API call to ChatGPT. The script also defines an ontology-based prompt to guide ChatGPT in transforming the human command into a JSON command. The ROSGPT node publishes the JSON command on the `/voice_cmd` topic.

- **rosgpt_client_node.py**: Implements a ROS2 client node that sends POST requests with natural human language text to the ROSGPT REST server. It waits for the structured JSON commands and displays them when received. Execute this node using the `ros2 run` command.

- **rosgpt_client.py**: Functions similarly to `rosgpt_client_node.py` but without implementing a ROS2 node. Instead, it only implements a REST client for ROSGPT. Execute this script using the `python` command, not `ros2 run`.

- **rosgptparser_turtlesim.py**: Implements a ROSGPTParser that subscribes to the `/voice_cmd` topic and receives JSON commands. The node parses the JSON command and extracts the ROS2 primitives that must be executed to accomplish the tasks. This script considers a simple navigation task for the Turtlesim robot, including move and rotate functions.

- **rosgptparser_tb3_nav.py**: Implements a ROSGPTParser that subscribes to the `/voice_cmd` topic and receives JSON commands. The JSON commands are parsed and converted into navigation goal tasks for the Turtlebot3 robot.


## License

This project is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License. You are free to use, share, and adapt this material for non-commercial purposes, as long as you provide attribution to the original author(s) and the source.

## Contribute

As this project is still under progress, contributions are welcome! To contribute, please follow these steps:

1. Fork the repository on GitHub.
2. Create a new branch for your feature or bugfix.
3. Commit your changes and push them to your fork.
4. Create a pull request to the main repository.

Before submitting your pull request, please ensure that your changes do not break the build and adhere to the project's coding style.

For any questions or suggestions, please open an issue on the [GitHub issue tracker](https://github.com/aniskoubaa/rosgpt/issues).


