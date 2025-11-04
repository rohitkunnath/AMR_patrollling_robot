import os  # To work with file paths in operating system
from time import sleep  # To pause the program for some time
from ament_index_python.packages import get_package_share_directory  # To find package folder paths in ROS2
from launch import LaunchDescription  # To create a launch description in ROS2
from launch.actions import IncludeLaunchDescription, ExecuteProcess  # To include other launch files and run processes
from launch.launch_description_sources import PythonLaunchDescriptionSource  # To use Python launch files as source
from launch.substitutions import LaunchConfiguration  # To use launch arguments/configurations

from launch_ros.actions import Node  # To launch ROS2 nodes like robot software modules

def generate_launch_description():  # Main function to set up the launch steps
  pkg_nav2_dir = get_package_share_directory('nav2_bringup')  # Get folder path for nav2_bringup package
  pkg_tb3_sim = get_package_share_directory('tb3_sim')  # Get folder path for tb3_sim package

  use_sim_time = LaunchConfiguration('use_sim_time', default='True')  # Get or set simulation time usage flag
  autostart = LaunchConfiguration('autostart', default='True')  # Get or set autostart flag for navigation

  nav2_launch_cmd = IncludeLaunchDescription(  # Include and run nav2 bringup launch file
      PythonLaunchDescriptionSource(
          os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')  # Path to nav2 bringup launch file
      ),
      launch_arguments={
          'use_sim_time': use_sim_time,  # Pass the use_sim_time argument
          'autostart': autostart,  # Pass autostart argument
          'map': os.path.join(pkg_tb3_sim, 'maps', 'map.yaml')  # Pass map file path for nav2
      }.items()
  )

  rviz_launch_cmd = Node(  # Launch RViz visualization node
      package="rviz2",  # RViz package
      executable="rviz2",  # RViz executable
      name="rviz2",  # Node name
      arguments=[
          '-d' + os.path.join(  # Load RViz config file for nav2 default view
              get_package_share_directory('nav2_bringup'),
              'rviz',
              'nav2_default_view.rviz'
          )]
  )

  set_init_amcl_pose_cmd = Node(  # Launch a node to set initial AMCL pose in simulation
      package="tb3_sim",  # The package name containing the node
      executable="amcl_init_pose_publisher",  # The executable node to publish initial pose
      name="amcl_init_pose_publisher",  # Node name
      parameters=[{  # Parameters to set initial position
          "x": -2.0,
          "y": -0.5,
      }]
  )

  ld = LaunchDescription()  # Create a launch description object

  # Add all three commands (nav2, AMCL pose, RViz) to launch description to run
  ld.add_action(nav2_launch_cmd)
  ld.add_action(set_init_amcl_pose_cmd)
  ld.add_action(rviz_launch_cmd)

  return ld  # Return the full launch description so ROS2 can execute it
