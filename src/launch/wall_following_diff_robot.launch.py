from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
import launch.conditions as conditions
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare("wall_following_diff_robot")

    # Flatland parameters declaration
    world_path = LaunchConfiguration("world_path")
    update_rate = LaunchConfiguration("update_rate")
    step_size = LaunchConfiguration("step_size")
    show_viz = LaunchConfiguration("show_viz")
    viz_pub_rate = LaunchConfiguration("viz_pub_rate")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Robot parameters declaration
    linear_speed = LaunchConfiguration("linear_speed")
    ideal_distance = LaunchConfiguration("ideal_distance")
    ideal_angle = LaunchConfiguration("ideal_angle")
    k = LaunchConfiguration("k")
    radius = LaunchConfiguration("radius")

    ld = LaunchDescription(
        [
            # Flatland parameters.
            # You can either change these values directly here or override them in the launch command default values. Example:
            #   ros2 launch serp_teleop serp_teleop.launch.py update_rate:="20.0"
            DeclareLaunchArgument(
                name="world_path",
                default_value=PathJoinSubstitution([pkg_share, "world/world.yaml"]),
            ),
            DeclareLaunchArgument(name="update_rate", default_value="100.0"),
            DeclareLaunchArgument(name="step_size", default_value="0.01"),
            DeclareLaunchArgument(name="show_viz", default_value="true"),
            DeclareLaunchArgument(name="viz_pub_rate", default_value="30.0"),
            DeclareLaunchArgument(name="use_sim_time", default_value="true"),
            SetEnvironmentVariable(name="ROSCONSOLE_FORMAT", value="[${severity} ${time} ${logger}]: ${message}"),

            # Robot parameters. Expected by the robot node.
            DeclareLaunchArgument(name="linear_speed", default_value="0.2"),
            DeclareLaunchArgument(name="ideal_distance", default_value="0.2"),
            DeclareLaunchArgument(name="ideal_angle", default_value="1.57"),
            DeclareLaunchArgument(name="k", default_value="2.0"),
            DeclareLaunchArgument(name="radius", default_value="0.075"),

            # **** Nodes launched by this file ****
            # launch flatland server
            Node(
                name="flatland_server",
                package="flatland_server",
                executable="flatland_server",
                output="screen",
                parameters=[
                    # use the arguments passed into the launchfile for this node
                    {"world_path": world_path},
                    {"update_rate": update_rate},
                    {"step_size": step_size},
                    {"show_viz": show_viz},
                    {"viz_pub_rate": viz_pub_rate},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            # runs the code that is used to control the robot
            Node(
                name="wall_following_diff_robot",
                package="wall_following_diff_robot",
                executable="wall_following_diff_robot",
                output="screen",
                parameters=[
                    {"linear_speed": linear_speed},
                    {"ideal_distance": ideal_distance},
                    {"ideal_angle": ideal_angle},
                    {"k": k},
                    {"radius": radius},
                ]
            ),

            # maps
            Node(
                name="tf",
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),

            # visualisation 
            Node(
                name="rviz",
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([pkg_share, "rviz/robot_navigation.rviz"])],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=conditions.IfCondition(show_viz),
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
