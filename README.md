# diff-reactive-bot
Differential reactive robot for RI course

# Requirements

- ROS 2 Humble
    - [Official installation guide](https://docs.ros.org/en/humble/Installation.html)

# Installation
```
$ git clone git@github.com:marhcouto/diff-reactive-bot.git
$ cd diff-reactive-bot/
$ git submodule update --init --recursive
```

## Setup and build flatland

```
$ cd lib/flatland2/
$ rosdep install -i --from-path src --rosdistro humble -y
$ sudo rosdep init
$ colcon build
$ source install/setup.bash
$ cd ../../
```

## Setup and build project

```
$ rosdep install -i --from-path src --rosdistro humble -y
$ colcon build
$ source install/setup.bash
```

## Launch Application
```
$ ros2 launch wall_following_diff_robot wall_following_diff_robot.launch.py
```