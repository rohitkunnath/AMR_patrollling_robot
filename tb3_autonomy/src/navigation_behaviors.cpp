#include "navigation_behaviors.h" // Include the header for our navigation behavior tree nodes.
#include "yaml-cpp/yaml.h" // Include YAML parser to read location coordinates from a file.
#include <string> // Include string support.

GoToPose::GoToPose(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
// Constructor sets up GoToPose, saves pointer to ROS2 node.
{
  action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
  // Connect to Nav2's "navigate_to_pose" action server.
  done_flag_ = false; // Make sure we know a goal hasn't finished.
}

BT::PortsList GoToPose::providedPorts()
{
  return {BT::InputPort<std::string>("loc")}; // Declare that this node accepts an input called "loc" (location name/key).
}

BT::NodeStatus GoToPose::onStart()
{
  // When the BT starts this node:

  BT::Optional<std::string> loc = getInput<std::string>("loc"); // Get the location key (like "home" or "table1") from the behavior tree port.
  const std::string location_file = node_ptr_->get_parameter("location_file").as_string(); // Get the locations YAML filename from parameters.

  YAML::Node locations = YAML::LoadFile(location_file); // Open and parse the YAML file.

  std::vector<float> pose = locations[loc.value()].as<std::vector<float>>(); // Find the pose (x, y, yaw) for the chosen location.

  // Prepare to send the goal.
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1); // Set up a callback for when nav completes.

  // Fill a goal message with position and orientation.
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = pose[0];
  goal_msg.pose.pose.position.y = pose[1];

  tf2::Quaternion q;
  q.setRPY(0, 0, pose[2]); // Convert yaw (rotation around z) into a quaternion.
  q.normalize(); // Make sure quaternion is valid (unit length), needed for ROS compatibility.
  goal_msg.pose.pose.orientation = tf2::toMsg(q); // Save orientation in message format.

  done_flag_ = false; // Mark as not finished.
  action_client_ptr_->async_send_goal(goal_msg, send_goal_options); // Send goal to Nav2 server.
  RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2\n"); // Log for debugging.
  return BT::NodeStatus::RUNNING; // Tell BT we are still working.
}

BT::NodeStatus GoToPose::onRunning()
{
  if (done_flag_)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached\n", this->name());
    return BT::NodeStatus::SUCCESS; // If goal finished, report success.
  }
  else
  {
    return BT::NodeStatus::RUNNING; // Otherwise, keep running.
  }
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
  // Called when Nav2 action finishes.
  if (result.result)
  {
    done_flag_ = true; // Mark navigation as done.
  }
}
