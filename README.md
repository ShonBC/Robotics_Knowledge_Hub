# Robotics_Knowledge_Hub
This repository is a comprehensive collection of robotics concepts, with detailed explanations and hands-on implementations in both Python and C++. It serves as a learning journey to refresh and expand knowledge on key robotics topics, ranging from foundational theories to advanced algorithms. Each concept is paired with example code and resources.


## Table of Contents

1. [Introduction to Robotics](#introduction-to-robotics)
2. [Robotics Software (ROS2)](#robotics-software-ros2)
3. [Path Planning](#path-planning)
4. [Robot Modeling (Kinematics)](#robot-modeling-kinematics)
4. [Perception (Computer Vision)](#perception-computer-vision)
5. [Machine Learning and Artificial Intelligence (AI)](#machine-learning-and-artificial-intelligence-ai)
6. [Control Systems](#control-systems)

## Introduction to Robotics


## Robotics Software (ROS2)
ROS (Robot Operating System) was originally created in 2007 at Stanford University's Artificial Intelligence Laboratory, and later developed by Willow Garage. It was designed to be a flexible framework for writing robot software, providing a collection of tools, libraries, and conventions to simplify the creation of complex and robust robot behavior across various robotic platforms.

As robotics technology advanced and new requirements emerged, the need for a more modern, real-time capable system became apparent. This led to the development of ROS2, which began around 2014. ROS2 was designed to address limitations in the original ROS, such as better support for real-time systems, improved security, and enhanced performance in multi-robot scenarios.

ROS2 (Robot Operating System 2) builds upon the success of ROS1 while introducing significant improvements. Like its predecessor, it's a flexible framework for writing robot software, offering a comprehensive set of tools, libraries, and conventions. These resources aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. [For more information on the history and evolution of ROS, see the official ROS2 Documentation page](https://docs.ros.org/en/iron/index.html). Key concepts in ROS2 include:

1. [Nodes](https://docs.ros.org/en/iron/Concepts/Basic/About-Nodes.html): Modular processes that perform computation.
2. [Topics](https://docs.ros.org/en/iron/Concepts/Basic/About-Topics.html): Named buses over which nodes exchange messages.
3. [Services](https://docs.ros.org/en/iron/Concepts/Basic/About-Services.html): Request/reply interactions between nodes.
4. [Actions](https://docs.ros.org/en/iron/Concepts/Basic/About-Actions.html): For longer tasks, similar to services but preemptable.
5. [Parameters](https://docs.ros.org/en/iron/Concepts/Basic/About-Parameters.html): Configuration values for nodes.
6. [Packages](https://docs.ros.org/en/iron/Package-Docs.html): Organization units for ROS2 code.
7. [Workspaces](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html): Collections of packages sharing a common build system.
8. [DDS (Data Distribution Service)](https://docs.ros.org/en/iron/Concepts/About-Different-Middleware-Vendors.html): The communication middleware used by ROS2.
9. [QoS (Quality of Service)](https://docs.ros.org/en/iron/Concepts/About-Quality-of-Service-Settings.html): Policies to configure communication reliability and history.
10. [Launch files](https://docs.ros.org/en/iron/Tutorials/Intermediate/Launch/Launch-Main.html): XML or Python files to start and configure multiple nodes.
11. [ROS_DOMAIN_ID](https://docs.ros.org/en/iron/Concepts/About-Domain-ID.html): An environment variable used to logically isolate ROS2 networks.
12. [QoS (Quality of Service)](https://docs.ros.org/en/iron/Concepts/About-Quality-of-Service-Settings.html): Policies to configure communication reliability and history.
13. [RQt](https://docs.ros.org/en/iron/Concepts/About-RQt.html): A Qt-based framework for GUI development for ROS.
14. [Tf2 (Transform Library)](https://docs.ros.org/en/iron/Concepts/About-Tf2.html): A package for keeping track of multiple coordinate frames over time.
15. [Bag Files](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html): A file format for storing ROS2 message data and can be used for data analysis.
16. [Rviz](https://github.com/ros2/rviz?tab=readme-ov-file): A 3D visualization tool for ROS (No ROS2 wiki page yet, but the [ROS1 wiki page](https://wiki.ros.org/rviz) is a good resource).
17. [Gazebo](https://docs.ros.org/en/iron/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html): A physics simulator for ROS.


ROS2 improves upon its predecessor with better performance, enhanced security features, and increased support for real-time systems and small embedded platforms.


### Pros and Cons of ROS2

#### Pros:
1. **Improved Performance**: ROS2 offers better performance compared to ROS1, especially in real-time systems and multi-robot scenarios.
2. **Enhanced Security**: Built-in DDS (Data Distribution Service) provides improved security features.
3. **Cross-Platform Support**: Better support for different operating systems, including Windows.
4. **Real-Time Systems**: Designed with real-time systems in mind, making it suitable for time-critical applications.
5. **Modularity**: Highly modular architecture allows for easier integration of new functionalities.
6. **Quality of Service (QoS)**: Configurable communication settings for different network conditions.
7. **Improved Build System**: Uses modern CMake practices, making it easier to build and integrate with other software.
8. **Better Documentation**: Generally more comprehensive and up-to-date documentation compared to ROS1.

#### Cons:
1. **Learning Curve**: Steeper learning curve, especially for those familiar with ROS1.
2. **Compatibility Issues**: Not directly compatible with ROS1, requiring bridges for integration with legacy systems.
3. **Smaller Community**: While growing, the ROS2 community is still smaller than ROS1, potentially leading to fewer resources and packages.
4. **Ongoing Development**: Some features and tools are still in development or not as mature as their ROS1 counterparts.
5. **Resource Intensity**: Can be more resource-intensive than ROS1, especially on smaller embedded systems.
6. **Complexity**: The added features and flexibility can make ROS2 more complex to set up and configure properly.
7. **Limited Packages**: Fewer available packages compared to ROS1, though this gap is closing over time.
8. **Debugging Challenges**: The distributed nature of ROS2 can make debugging more challenging in some scenarios.

Despite these challenges, the benefits of ROS2 often outweigh the drawbacks for many modern robotics applications, especially those requiring real-time performance, enhanced security, or cross-platform support.


## Path Planning
Path planning is a crucial aspect of robotics, enabling robots to navigate efficiently and safely through their environment. Here are some key algorithms and concepts in robotics path planning:

1. **Dijkstra's Algorithm**: 
   - A graph search algorithm that finds the shortest path between nodes in a graph.
   - Guarantees the optimal path but can be slow in large environments.
   - [More on Dijkstra's Algorithm](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)

2. **A* (A-Star) Algorithm**:
   - An extension of Dijkstra's algorithm that uses heuristics to guide the search.
   - Generally faster than Dijkstra's, especially in large spaces.
   - Guarantees the optimal path if the heuristic is admissible.
   - [More on A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)

3. **Weighted A***:
   - A variant of A* that allows for a trade-off between optimality and speed.
   - Uses a weight to bias the search towards the goal, potentially finding a suboptimal path faster.
   - Useful in time-critical applications where a good path is needed quickly.

4. **RRT (Rapidly-exploring Random Tree)**:
   - A sampling-based algorithm for quickly searching high-dimensional spaces.
   - Effective in environments with obstacles and many degrees of freedom.
   - Does not guarantee optimality but can find a feasible path quickly.
   - [More on RRT](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree)

5. **RRT* (RRT-Star)**:
   - An optimizing extension of RRT that converges to an optimal solution.
   - Continues to improve the path even after finding an initial solution.
   - More computationally intensive than RRT but produces better paths.
   - [More on RRT*](https://arxiv.org/abs/1105.1186)

6. **SLAM (Simultaneous Localization and Mapping)**:
   - Not strictly a path planning algorithm, but crucial for navigation in unknown environments.
   - Allows a robot to build a map of an unknown environment while simultaneously keeping track of its location within it.
   - Essential for autonomous navigation in new or changing environments.
   - [More on SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)

Each of these algorithms has its strengths and is suited to different types of environments and constraints. The choice of algorithm depends on factors such as the known information about the environment, computational resources, time constraints, and the specific requirements of the robotic application.

In ROS2, these algorithms are often implemented in packages such as `nav2` (Navigation2) for general navigation tasks, and specialized packages for specific algorithms or applications.


## Robot Modeling (Kinematics)

## Perception (Computer Vision)

## Machine Learning and Artificial Intelligence (AI)

## Control Systems

