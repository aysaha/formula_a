from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  container = ComposableNodeContainer(
    node_name = 'container',
    node_namespace = 'drivers',
    package = 'rclcpp_components',
    node_executable = 'component_container',
    composable_node_descriptions = [
      ComposableNode(
        node_name = 'xbox_controller',
        package = 'drivers',
        node_plugin = 'drivers::XboxController',
      )
    ],
    output = 'screen',
  )

  return LaunchDescription([container])
