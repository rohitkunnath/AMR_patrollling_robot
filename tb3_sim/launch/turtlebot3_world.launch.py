import os  # To use operating system functions like joining file paths
from ament_index_python.packages import get_package_share_directory  # To find and get ROS2 package path from system
from launch import LaunchDescription  # To create a launch file description in ROS2
from launch.actions import IncludeLaunchDescription  # To include another launch file inside this launch file
from launch.launch_description_sources import PythonLaunchDescriptionSource  # To tell ROS2 that included launch file is Python type
from launch.substitutions import LaunchConfiguration  # To use configurable values (like variables) in launch file


def generate_launch_description():  # Main function that creates and returns the launch setup
  launch_file_dir = os.path.join(
      get_package_share_directory('turtlebot3_gazebo'), 'launch')  # Get the "launch" folder path from turtlebot3_gazebo package
  pkg_gazebo_ros = get_package_share_directory('gazebo_ros')  # Get the package path of gazebo_ros
  pkg_tb3_sim = get_package_share_directory('tb3_sim')  # Get the package path of tb3_sim package

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')  # Configurable value to use simulated time (default true)
  x_pose = LaunchConfiguration('x_pose', default='-2.0')  # Configurable robot starting position on x-axis
  y_pose = LaunchConfiguration('y_pose', default='-0.5')  # Configurable robot starting position on y-axis

  world = os.path.join(
      get_package_share_directory('turtlebot3_gazebo'),
      'worlds',
      'turtlebot3_world.world'
  )  # Get path of default turtlebot3 simulation world file

  gzserver_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
      ),
      launch_arguments={'world': world}.items()
  )  # Start Gazebo server with given world file

  gzclient_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
      )
  )  # Start Gazebo client (GUI window)

  robot_state_publisher_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
      ),
      launch_arguments={'use_sim_time': use_sim_time}.items()
  )  # Start robot state publisher (publishes robot joint and state info)

  spawn_turtlebot_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
      ),
      launch_arguments={
          'x_pose': x_pose,
          'y_pose': y_pose
      }.items()
  )  # Spawn (place) the TurtleBot3 robot at given x and y position

  ld = LaunchDescription()  # Create the main launch description object

  ld.add_action(gzserver_cmd)  # Add Gazebo server to launch sequence
  ld.add_action(gzclient_cmd)  # Add Gazebo client to launch sequence
  ld.add_action(robot_state_publisher_cmd)  # Add robot state publisher to launch sequence
  ld.add_action(spawn_turtlebot_cmd)  # Add TurtleBot spawn command to launch sequence

  return ld  # Return the final launch description to ROS2 to run
