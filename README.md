# ROSGPT: ChatGPT Interface for ROS2 for Human-Robot Interaction

ROSGPT is a pioneering approach that combines the power of ChatGPT and ROS (Robot Operating System) to redefine human-robot interaction. By leveraging large language models like ChatGPT, ROSGPT enables the conversion of unstructured human language into actionable robotic commands. This repository contains the implementation of ROSGPT, allowing developers to explore and contribute to the project.

## Reference Paper

[![DOI](https://img.shields.io/badge/DOI-10.20944%2Fpreprints202304.0827.v2-blue)](https://www.preprints.org/manuscript/202304.0827/v2)

**Author**: Anis Koubaa

**Citation**: Koubaa, A. (2023). ROSGPT: Next-Generation Human-Robot Interaction with ChatGPT and ROS. Preprints.org, 2023, 2023040827.
https://www.preprints.org/manuscript/202304.0827/v2

**BibTeX Citation**:

```bibtex
@article{koubaa2023rosgpt,
  title={ROSGPT: Next-Generation Human-Robot Interaction with ChatGPT and ROS},
  author={Koubaa, Anis},
  journal={Preprints.org},
  year={2023},
  volume={2023},
  pages={2023040827},
  doi={10.20944/preprints202304.0827.v2}
}
```

## Video Demo

Explore ROSGPT in action with this video demonstration, showcasing the process of getting started and the capabilities of the system.

[![ROSGPT Video Demonstration](https://img.youtube.com/vi/urkQD-hB5Hg/0.jpg)](https://www.youtube.com/watch?v=urkQD-hB5Hg)

## ROSGPT ROS2 Package Description

The ROSGPT ROS2 package includes a collection of scripts that work together to provide a convenient way of translating natural human language text into structured JSON commands, which can be utilized by robots like Turtlesim and Turtlebot3. Below is a brief overview of each script:

- **rosgpt.py**: This script creates the ROSGPT node, which is a ROS2 node with a REST server that takes in POST requests containing natural human language text. It then translates the text into structured JSON commands via an API call to ChatGPT. The script also defines an ontology-based prompt that helps ChatGPT convert human commands into JSON commands. The ROSGPT node publishes the JSON command on the `/voice_cmd` topic.
- **rosgpt_client_node.py**: This script establishes a ROS2 client node that sends POST requests with natural human language text to the ROSGPT REST server. It waits for the structured JSON commands and displays them upon receipt. Use the `ros2 run` command to execute this node.
- **rosgpt_client.py**: Similar to `rosgpt_client_node.py`, this script sends POST requests with natural human language text to the ROSGPT REST server, but without implementing a ROS2 node. It solely functions as a REST client for ROSGPT. Use the `python` command, not `ros2 run`, to execute this script.
- **rosgptparser_turtlesim.py**: This script implements the ROSGPTParser, which subscribes to the `/voice_cmd` topic and receives JSON commands. The node parses the JSON command and determines the ROS2 primitives required to execute the specified tasks. In this script, a simple navigation task for the Turtlesim robot is considered, including move and rotate functions.
- **rosgptparser_tb3_nav.py**: This script also implements the ROSGPTParser, subscribing to the `/voice_cmd` topic and receiving JSON commands. The JSON commands are parsed and transformed into navigation goal tasks for the Turtlebot3 robot.

## Getting Started

To get started with ROSGPT, follow these steps:

1. Clone the repository to your local machine.
2. Install the dependencies listed in the environment setup section.
3. After following the environment setup steps, run the ROSGPT flask server using
   ```bash
   ros2 run rosgpt rosgpt
   ```
4. Run the turtlesim node using
   ```bash
   ros2 run turtlesim turtlesim_node
   ```
5. Run the rosgptparser_turtlesim.py using
   ```bash
   ros2 run rosgpt rosgptparser_turtlesim 
   ```
6. Run the rosgpt_client_node.py using
   ```bash
   ros2 run rosgpt rosgpt_client_node 
   ```
7. Now you can start giving commands to the robot using the rosgpt_client_node terminal .for example you can say "ı want that you move forward 1 meter speed 1" and the robot will move forward 1 meter with speed 1.

### Environment Setup

This ROS 2 package was tested using ROS 2 Humble with Ubuntu 22.04. It should also work with ROS 2 Foxy and other ROS 2 versions.
You need to install the following dependencies:

- Add your OpenAI API Key in your `.bashrc` as an environment variable

  ```bash
  echo 'export OPENAI_API_KEY=your_api_key' >> ~/.bashrc

  ```
- Install the dependencies required for the text-to-speech functionality

  ```bash
  sudo apt-get install libespeak1
  sudo apt install ros-humble-turtlesim*
  ```
- For ros2 Humble verison (Ubuntu 22.04) downgrading the setuptools is required

  ```bash
  pip3 install --upgrade setuptools==58.0.2
  ```
- Install python dependencies

  ```bash
  cd rosgpt
  pip3 install -r requirements.txt
  ```
- then build the package

  ```bash
  colcon build --packages-select rosgpt
  ```
- source the workspace

  ```bash
  source install/setup.bash
  ```

To get started with ROSGPT, got this section [Getting Started](##Getting-Started)

## ROSGPT REST API

The ROSGPT REST API is a convenient way of interacting with ROSGPT. It allows you to send POST requests with natural human language text to the ROSGPT server, which will then translate the text into structured JSON commands. The JSON commands can be used to control robots like Turtlesim and Turtlebot3.

To use the ROSGPT REST API, follow these steps:

1. Run the ROSGPT flask server using `ros2 run rosgpt rosgpt`.
2. Run the turtlesim node using `ros2 run turtlesim turtlesim_node`.
3. Run the rosgptparser_turtlesim.py using `ros2 run rosgpt rosgptparser_turtlesim`.
4. Run the rosgpt_client.py using `python rosgpt_client.py`.
5. Send a POST request to the ROSGPT server using `curl -X POST -H "Content-Type: application/json" -d '{"text":"move forward"}' http://localhost:5000/rosgpt`. You can replace "move forward" with any natural human language text you want. The ROSGPT server will translate the text into structured JSON commands and send them back to the client.
6. The client will display the JSON commands on the terminal. You can use these commands to control the Turtlesim robot.

## ROS1 Support

ROS1 is an earlier version of the Robot Operating System (ROS), which is still widely used in many robotics applications. While ROSGPT was originally developed for ROS 2 Humble on Ubuntu 22.04, we recognize the importance of supporting ROS1 as well.

To use ROSGPT with ROS1, you will need to modify the ROS 2 code in the scripts to the corresponding ROS 1 code. We are actively working on developing this functionality, but it is still a work-in-progress.

If you have already developed an extension to enable ROSGPT to work with ROS1, we would love to hear from you! Please create a pull request in a new branch and we will review it for inclusion in the ROSGPT repository.

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
