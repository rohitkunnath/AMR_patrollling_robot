import os  # Import OS module for file path handling

from ament_index_python.packages import get_package_share_directory  # Get ROS2 package paths
from launch import LaunchDescription  # Create launch descriptions
from launch_ros.actions import Node  # Launch ROS2 nodes

def generate_launch_description():  # Main function to generate launch description
  pkg_tb3_sim = get_package_share_directory('tb3_sim')  # Get tb3_sim package path
  pkg_tb3_autonomy = get_package_share_directory('tb3_autonomy')  # Get tb3_autonomy package path

  autonomy_node_cmd = Node(  # Define ROS2 node launch action
      package="tb3_autonomy",  # Package name
      executable="autonomy_node",  # Executable name
      name="autonomy_node",  # Node name
      parameters=[{  # Node parameters
          "location_file": os.path.join(pkg_tb3_sim, "config", "sim_house_locations.yaml")  # YAML file path
      }]
  )

  ld = LaunchDescription()  # Create launch description container
  ld.add_action(autonomy_node_cmd)  # Add node launch to description
  return ld  # Return the launch description
