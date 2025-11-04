import time  # To use time-related functions like sleep
import rclpy  # Import ROS2 Python client library
from rclpy.node import Node  # Import Node class for creating ROS2 nodes
import transforms3d  # To work with 3D rotations and convert angles
from geometry_msgs.msg import PoseWithCovarianceStamped  # Import message type for robot pose with uncertainty


class InitAmclPosePublisher(Node):  # Define a new ROS2 node class for publishing initial robot pose
  def __init__(self):
    super().__init__("init_amcl_pose_publisher")  # Initialize the node with a name

    self.declare_parameter("x", value=0.0)  # Define parameter 'x' with default value 0.0 (robot X position)
    self.declare_parameter("y", value=0.0)  # Define parameter 'y' with default value 0.0 (robot Y position)
    self.declare_parameter("theta", value=0.0)  # Define parameter 'theta' with default 0.0 (robot angle)
    self.declare_parameter("cov", value=0.5**2)  # Define parameter 'cov' for position uncertainty (variance)

    self.publisher = self.create_publisher(  # Create a publisher to send robot pose messages
        PoseWithCovarianceStamped,  # Message type to publish
        "/initialpose",  # Topic name where message will be sent
        10,  # Message queue size (buffer)
    )

    while(self.publisher.get_subscription_count() == 0):  # Wait until at least one subscriber is connected
      self.get_logger().info("Waiting for AMCL Initial Pose subscriber")  # Print info message repeatedly
      time.sleep(1.0)  # Pause for 1 second before checking again

  def send_init_pose(self):  # Function to send initial pose message
    x = self.get_parameter("x").value  # Read parameter 'x' value
    y = self.get_parameter("y").value  # Read parameter 'y' value
    theta = self.get_parameter("theta").value  # Read parameter 'theta' value
    cov = self.get_parameter("cov").value  # Read uncertainty parameter

    msg = PoseWithCovarianceStamped()  # Create a new message object
    msg.header.frame_id = "map"  # Set coordinate frame to map
    msg.pose.pose.position.x = x  # Set pose position x
    msg.pose.pose.position.y = y  # Set pose position y
    quat = transforms3d.euler.euler2quat(0, 0, theta)  # Convert angle theta to quaternion format (for rotation)
    msg.pose.pose.orientation.w = quat[0]  # Set quaternion w part
    msg.pose.pose.orientation.x = quat[1]  # Set quaternion x part
    msg.pose.pose.orientation.y = quat[2]  # Set quaternion y part
    msg.pose.pose.orientation.z = quat[3]  # Set quaternion z part

    msg.pose.covariance = [  # Set uncertainty covariance matrix for pose (position and rotation)
        cov, 0.0, 0.0, 0.0, 0.0, 0.0,  # Position X variance
        0.0, cov, 0.0, 0.0, 0.0, 0.0,  # Position Y variance
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Position Z variance (not used)
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Rotation X variance (not used)
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Rotation Y variance (not used)
        0.0, 0.0, 0.0, 0.0, 0.0, cov   # Rotation Z variance
    ]

    self.publisher.publish(msg)  # Publish the message to the topic


def main(args=None):  # Main function to start the program
  rclpy.init()  # Initialize ROS2 client library
  initAmclPosePublisher = InitAmclPosePublisher()  # Create the node object

  future = initAmclPosePublisher.send_init_pose()  # Send the initial pose once

  rclpy.spin(initAmclPosePublisher)  # Keep node running until interrupted

  initAmclPosePublisher.destroy_node()  # Clean up and destroy the node when done
  rclpy.shutdown()  # Shut down ROS2 client library cleanly
