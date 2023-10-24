# Diff-reactive-bot
Differential reactive robot for RI course.

# Requirements

- ROS 2 Humble
    - [Official installation guide](https://docs.ros.org/en/humble/Installation.html)

# Installation

## Clone Repository
```
$ git clone git@github.com:marhcouto/diff-reactive-bot.git
$ cd diff-reactive-bot/
$ git submodule update --init --recursive
```

## Setup and build project

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
