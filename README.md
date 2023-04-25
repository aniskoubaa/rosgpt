# ROSGPT: ChatGPT Interface for ROS2 for Human-Robot Interaction

ROSGPT is a pioneering approach that combines the power of ChatGPT and ROS (Robot Operating System) to redefine human-robot interaction. By leveraging large language models like ChatGPT, ROSGPT enables the conversion of unstructured human language into actionable robotic commands. This repository contains the implementation of ROSGPT, allowing developers to explore and contribute to the project.

## Reference Paper
[![DOI](https://img.shields.io/badge/DOI-10.20944%2Fpreprints202304.0827.v2-blue)](https://doi.org/10.20944/preprints202304.0827.v2)

**Author**: Anis Koubaa

**Citation**: Koubaa, A. (2023). ROSGPT: Next-Generation Human-Robot Interaction with ChatGPT and ROS. Preprints.org, 2023, 2023040827. [https://doi.org/10.20944/preprints202304.0827.v2](https://doi.org/10.20944/preprints202304.0827.v2)

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


