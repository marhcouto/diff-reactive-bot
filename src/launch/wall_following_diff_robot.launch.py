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
    target_velocity = LaunchConfiguration("target_velocity")
    ideal_distance = LaunchConfiguration("ideal_distance")
    invert_direction = LaunchConfiguration("invert_direction")
    detection_by_line = LaunchConfiguration("detection_by_line")
    k_ang = LaunchConfiguration("k_ang")
    k_lin = LaunchConfiguration("k_lin")
    old_controller = LaunchConfiguration("old_controller")

    ld = LaunchDescription(
        [
            # Flatland parameters.
            # You can either change these values directly here or override them in the launch command default values. Example:
            #   ros2 launch serp_teleop serp_teleop.launch.py update_rate:="20.0"
            DeclareLaunchArgument(
                name="world_path",
                default_value=PathJoinSubstitution([pkg_share, "world/world.yaml"]),
            ),
            DeclareLaunchArgument(name="update_rate", default_value="200.0"),
            DeclareLaunchArgument(name="step_size", default_value="0.01"),
            DeclareLaunchArgument(name="show_viz", default_value="true"),
            DeclareLaunchArgument(name="viz_pub_rate", default_value="30.0"),
            DeclareLaunchArgument(name="use_sim_time", default_value="true"),
            SetEnvironmentVariable(name="ROSCONSOLE_FORMAT", value="[${severity} ${time} ${logger}]: ${message}"),

            # Robot parameters. Expected by the robot node.
            DeclareLaunchArgument(name="target_velocity", default_value="0.5"),
            DeclareLaunchArgument(name="ideal_distance", default_value="0.6"),
            DeclareLaunchArgument(name="invert_direction", default_value="false"),
            DeclareLaunchArgument(name="detection_by_line", default_value="false"),
            DeclareLaunchArgument(name="k_ang", default_value="8.0"),
            DeclareLaunchArgument(name="k_lin", default_value="1.0"),
            DeclareLaunchArgument(name="old_controller", default_value="false"),

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
                    {"target_velocity": target_velocity},
                    {"ideal_distance": ideal_distance},
                    {"invert_direction": invert_direction},
                    {"detection_by_line": detection_by_line},
                    {"k_ang": k_ang},
                    {"k_lin": k_lin},
                    {"old_controller": old_controller},
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
