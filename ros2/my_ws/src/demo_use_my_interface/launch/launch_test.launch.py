from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    action_my_server = Node(package = "demo_use_my_interface", executable = "server")
    action_my_client = Node(package = "demo_use_my_interface", executable = "client")

    launchDescription = LaunchDescription([action_my_server, action_my_client])
    return launchDescription