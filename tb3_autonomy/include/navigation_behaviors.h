#include "rclcpp/rclcpp.hpp" // Use ROS2 C++ classes for nodes and logging.
#include "behaviortree_cpp_v3/behavior_tree.h" // Use BehaviorTree.CPP base classes for behavior tree nodes.
#include "rclcpp_action/rclcpp_action.hpp" // Use ROS2 Action client, required for sending goals to actions.
#include "nav2_msgs/action/navigate_to_pose.hpp" // Use the NavigateToPose action, which makes the robot move to a set location.

#include <tf2/LinearMath/Quaternion.h> // Handle rotation math for navigation.
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // Help convert between ROS2 geometry types.

class GoToPose : public BT::StatefulActionNode // Define a new behavior tree action type for "GoToPose".
{
public:
  GoToPose(const std::string &name,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr node_ptr);
  // Constructor: Sets up the action node and gets access to the parent ROS2 node.

  using NavigateToPose = nav2_msgs::action::NavigateToPose; // Shortcut for the navigation action type.
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>; // Shortcut for handling navigation action goals.

  rclcpp::Node::SharedPtr node_ptr_; // Shared pointer to the ROS2 Node (for making service/action calls, logging, etc).
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_; // Shared pointer for the action client — communicates with the navigation server.
  bool done_flag_; // Flag to keep track if the navigation action is finished.

  // These methods are called by the behavior tree as the action executes.
  BT::NodeStatus onStart() override; // Called when the action is started.
  BT::NodeStatus onRunning() override; // Called repeatedly while the action is still in progress.
  void onHalted() override{}; // Called if the action is stopped or interrupted.

  static BT::PortsList providedPorts(); // Returns the list of input/output ports for this tree node (e.g., goal pose).

  // Called when the navigation action finishes (success or failure).
  void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);
};
