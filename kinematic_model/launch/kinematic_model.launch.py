from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    wheel_vel_pub_node = Node(
        name="wheel_velocities_publisher",
        package="wheel_velocities_publisher",
        executable="wheel_velocities_publisher",
        output="screen",
    )

    kinematic_model_node = Node(
        name="kinematic_model",
        package="kinematic_model",
        executable="kinematic_model",
        output="screen",
    )

    return LaunchDescription(
        [wheel_vel_pub_node,
        kinematic_model_node]
    )