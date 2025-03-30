
<h1 align="center">Gymbrot - ROS 2</h1>

<p align="center">
  <img alt="Github top language" src="https://img.shields.io/github/languages/top/Heur-a/gymbrot?color=56BEB8">
  <img alt="Github language count" src="https://img.shields.io/github/languages/count/Heur-a/gymbrot?color=56BEB8">
  <img alt="Repository size" src="https://img.shields.io/github/repo-size/Heur-a/gymbrot?color=56BEB8">
  <img alt="License" src="https://img.shields.io/github/license/Heur-a/gymbrot?color=56BEB8">
</p>

<p align="center">
  <a href="#dart-about">About</a> &#xa0; | &#xa0; 
  <a href="#sparkles-features">Features</a> &#xa0; | &#xa0;
  <a href="#rocket-technologies">Technologies</a> &#xa0; | &#xa0;
  <a href="#white_check_mark-requirements">Requirements</a> &#xa0; | &#xa0;
  <a href="#checkered_flag-starting">Getting Started</a> &#xa0; | &#xa0;
  <a href="#memo-license">License</a> &#xa0; | &#xa0;
  <a href="https://github.com/Heur-a" target="_blank">Author</a>
</p>

<br>

## :dart: About ##

Gymbrot is an autonomous robot developed with ROS 2 designed to perform tasks in a gym environment, such as transporting
equipment, guiding users, or monitoring spaces.

## :sparkles: Features ##

- Autonomous navigation using LiDAR sensors
- Object and person recognition
- Voice communication and graphical interface
- Remote control via ROS 2

## :rocket: Technologies ##

This project uses the following tools:

- [ROS 2 Galactic](https://docs.ros.org/en/galactic/index.html)
- [Gazebo](https://gazebosim.org/)
- [RViz](https://wiki.ros.org/rviz)
- [Python](https://www.python.org/)
- [C++](https://isocpp.org/)

## :white_check_mark: Requirements ##

Before starting, make sure you have the following installed:

- [ROS 2 Galactic](https://docs.ros.org/en/galactic/Installation.html)
- [Colcon](https://colcon.readthedocs.io/en/released/)
- [Gazebo](https://gazebosim.org/)

## :checkered_flag: Getting Started ##

```bash
# Clone turtlebot3_simulations repository
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations

# Navigate to src directory
$ cd turtlebot3_ws/src

# Clone this repository
$ git clone https://github.com/Heur-a/gymbrot

# Go back to turtlebot3_ws root
$ cd ..

# Build the project
$ colcon build

# Source the ROS 2 environment
$ source install/setup.bash

# Launch the robot
$ ros2 launch gymbrot bringup.launch.py #NOT YET
```

## :memo: License ##

This project is licensed under the GNU License. For more details, see the [LICENSE](LICENSE.md) file.

Made with :heart: by <a href="https://github.com/Heur-a" target="_blank">Heur-a</a>

&#xa0;

<a href="#top">Back to top</a>

