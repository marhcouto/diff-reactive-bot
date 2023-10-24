# Diff-reactive-bot
Differential reactive robot for RI course.

## Directory Structure
- **lib:** flatland2 directory
- **src:** reactive robot source code and flatland2 environment configuration
	- **world:** environment definition
    - **rviz:** rviz graphical interface configuration
    - **launch:** launch file
    - **wall_following_diff_robot:** differential drive robot code
- **docs:** paper
- **results (empty):** statistics generated from 'generate_stats.py' script



## Dependencies

- ROS 2 Humble
    - [Official installation guide](https://docs.ros.org/en/humble/Installation.html)
- Flatland2 - included as git submodule, follow instructions to clone repo

## Installation

### Clone Repository
```
$ git clone git@github.com:marhcouto/diff-reactive-bot.git
$ cd diff-reactive-bot/
$ git submodule update --init --recursive
```

### Setup and build project

In the root directory:

```
$ rosdep install -i --from-path src --rosdistro humble -y
$ colcon build
$ source install/setup.bash
```

## Launch Application
```
$ ros2 launch wall_following_diff_robot wall_following_diff_robot.launch.py
```

## Authors

- Marcelo Couto
- José Costa
- Fernando Rego
- Bruno Mendes
