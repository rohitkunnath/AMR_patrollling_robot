#include "rclcpp/rclcpp.hpp" // Include ROS2 C++ main API for creating nodes and handling communication.
#include "behaviortree_cpp_v3/bt_factory.h" // Include the BehaviorTree.CPP library header to use behavior trees.
#include "navigation_behaviors.h" // Include user-defined navigation behavior nodes/functions.
#include "ament_index_cpp/get_package_share_directory.hpp" // Include function to find ROS2 package share directories.
#include <tf2/LinearMath/Quaternion.h> // Include quaternion math for handling 3D orientation.

class AutonomyNode : public rclcpp::Node // Define a class named AutonomyNode that inherits from the main ROS2 Node class.
{
public:
  explicit AutonomyNode(const std::string &node_name); // Declare a constructor that creates the ROS2 node with a specific name.
  void setup(); // Declare a method to perform setup tasks (e.g., configuring timers or parameters).
  void create_behavior_tree(); // Declare a method to create and configure the behavior tree.
  void update_behavior_tree(); // Declare a method to update or tick the behavior tree in a loop (e.g., per timer tick).

private:
  rclcpp::TimerBase::SharedPtr timer_; // Timer to periodically trigger functions (like ticking the behavior tree).
  BT::Tree tree_; // Variable to hold the behavior tree structure (from BehaviorTree.CPP).
};
