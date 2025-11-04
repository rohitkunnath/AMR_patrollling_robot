#include "autonomy_node.h" // Include the header file that defines the AutonomyNode class and related functions.

using namespace std::chrono_literals; // Allow using time units like 500ms for timers.

const std::string bt_xml_dir =
    ament_index_cpp::get_package_share_directory("tb3_autonomy") + "/bt_xml"; // Get the directory path to the behavior tree XML files in the "tb3_autonomy" package.

AutonomyNode::AutonomyNode(const std::string &nodeName) : Node(nodeName)
{
  this->declare_parameter("location_file","none"); // Add a ROS2 parameter named "location_file" with a default value "none".
  RCLCPP_INFO(get_logger(), "Init done"); // Print "Init done" in the ROS2 log.
}

void AutonomyNode::setup()
{
  RCLCPP_INFO(get_logger(), "Setting up"); // Print "Setting up" in the ROS2 log.
  create_behavior_tree(); // Call a function to create the behavior tree.
  RCLCPP_INFO(get_logger(), "BT created"); // Print "BT created" in the ROS2 log.

  const auto timer_period = 500ms;
  timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&AutonomyNode::update_behavior_tree, this)); // Start a timer that runs every 500ms, calling update_behavior_tree.

  rclcpp::spin(shared_from_this()); // Keep the node alive and processing events until it's shut down.
  rclcpp::shutdown(); // Shutdown ROS2 when the node stops running.
}

void AutonomyNode::create_behavior_tree()
{
  BT::BehaviorTreeFactory factory; // Create a factory to build behavior trees.

  // register bt node
  BT::NodeBuilder builder =
      [=](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<GoToPose>(name, config, shared_from_this());
  }; // Define how to construct the GoToPose behavior tree node.

  factory.registerBuilder<GoToPose>("GoToPose", builder); // Register the builder with the factory so it knows how to build GoToPose nodes.

  RCLCPP_INFO(get_logger(), bt_xml_dir.c_str()); // Print the path of the behavior tree XML directory.

  tree_ = factory.createTreeFromFile(bt_xml_dir + "/tree.xml"); // Build the behavior tree from the XML file called tree.xml.
  RCLCPP_INFO(get_logger(), "3");  // Print "3" in the ROS2 log (possibly for debugging).
}

void AutonomyNode::update_behavior_tree()
{
  BT::NodeStatus tree_status = tree_.tickRoot(); // Run the root of the behavior tree, which returns its current status.

  if (tree_status == BT::NodeStatus::RUNNING)
  {
    return; // If the tree is still running, do nothing and wait for the next tick.
  }
  else if (tree_status == BT::NodeStatus::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Finished Navigation"); // If the tree finished successfully, print that navigation is done.
  }
  else if (tree_status == BT::NodeStatus::FAILURE)
  {
    RCLCPP_INFO(this->get_logger(), "Navigation Failed");
    timer_->cancel(); // If the tree failed, print an error and stop the timer.
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);// Initialize ROS2 communication.

  auto node = std::make_shared<AutonomyNode>("autonomy_node"); // Create the AutonomyNode object named "autonomy_node".

  node->setup(); // Run the setup code (build behavior tree, start timer, spin).

  return 0; // End the program.
}
