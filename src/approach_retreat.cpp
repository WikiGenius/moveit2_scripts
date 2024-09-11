#include "moveit2_scripts/approach_and_retreat_node.hpp"
#include <map>

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(
      true); // Automatically declare parameters from external sources

  // Create an instance of the ApproachAndRetreatNode
  ApproachAndRetreatNode approach_and_retreat(node_options);

  // Define the pre-grasp pose
  double x = 0.232;
  double y = -0.010;
  double z = 0.334;
  double w = 1.0;
  geometry_msgs::msg::Pose pre_grasp_pose =
      approach_and_retreat.createTargetPose(x, y, z, w);

  // Define motion parameters for each task stage
  std::map<std::string, MotionParams> motion_params = {
      {"home",
       {1, 1, 5.0}}, // Home position: moderate speed and planning time
      {"pre_grasp", {0.2, 0.2, 10.0}}, // Pre-grasp: slower for precision
      {"approach", {0.1, 0.1, 15.0}},  // Approach: very slow for safety
      {"retreat", {0.3, 0.3, 10.0}}    // Retreat: moderate speed
  };

  double effort = 5.0;
  // Execute the task
  approach_and_retreat.executeTask(pre_grasp_pose, motion_params, effort);

  return 0;
}