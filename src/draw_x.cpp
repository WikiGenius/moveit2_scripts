#include "moveit2_scripts/draw_x_node.hpp"
#include <map>

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(
      true); // Automatically declare parameters from external sources

  // Create an instance of the DrawXtNode
  DrawXtNode draw_x(node_options);

  // Define the pre-grasp pose
  double x = 0.232;
  double y = -0.010;
  double z = 0.334;
  double w = 1.0;
  geometry_msgs::msg::Pose pre_grasp_pose = draw_x.createTargetPose(x, y, z, w);

  // Define motion parameters for each task stage
  std::map<std::string, MotionParams> motion_params = {
      {"home", {1, 1, 5.0}}, // Home position: moderate speed and planning time
      {"pre_grasp", {0.2, 0.2, 10.0}}, // Pre-grasp: slower for precision
      {"draw_x", {0.01, 0.01, 15.0}},  // draw_x: very slow for safety
  };

  double effort = 5.0;
  // Execute the task
  draw_x.executeTask(pre_grasp_pose, motion_params, effort);

// Note: One of the key challenges in executing comprehensive motion planning 
// is that there can be multiple solutions for each motion or task. 
// In certain cases, a specific task may succeed (e.g., grasping an object), 
// but the overall sequence of tasks (e.g., grasp, lift, move, place) might fail 
// due to limitations in joint configurations, workspace constraints, or collision avoidance.
//
// As such, there's a need to implement recursive task planning. By evaluating 
// the success or failure of each task in real-time and adjusting the plan dynamically, 
// we can ensure that the robot is more adaptable. Recursive task planning allows 
// the system to revisit incomplete or failed tasks, refine the motion plan, 
// and ultimately ensure successful execution of the entire motion sequence.
//
// For example: If a grasp action fails due to an unreachable object, the planner can
// recursively adjust the approach or object orientation, rather than failing the entire operation.
  return 0;
}