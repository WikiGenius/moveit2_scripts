#include "moveit2_scripts/draw_x_node.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>

// Constructor
DrawXtNode::DrawXtNode(const rclcpp::NodeOptions &node_options)
    : node_options_(node_options) {
  // Initialize the ROS node
  node_ =
      std::make_shared<rclcpp::Node>("approach_and_retrieve", node_options_);

  // Initialize the executor and run it in a separate thread
  executor_.add_node(node_);
  node_thread_ = std::thread([this]() { executor_.spin(); });

  // Initialize MoveGroup interfaces for the robot arm and gripper
  move_group_arm_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          node_, PLANNING_GROUP_ARM);
  move_group_gripper_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          node_, PLANNING_GROUP_GRIPPER);

  initializeMoveGroup();
}
// Destructor
DrawXtNode::~DrawXtNode() {
  // Stop the executor and join the thread to ensure proper cleanup
  executor_.cancel();
  if (node_thread_.joinable()) {
    node_thread_.join();
  }
}
// Initialize the MoveGroup interfaces
void DrawXtNode::initializeMoveGroup() {
  move_group_arm_->setPoseReferenceFrame("base_footprint");
}

// Main Task Execution
void DrawXtNode::executeTask(
    const geometry_msgs::msg::Pose &pre_grasp_pose,
    const std::map<std::string, MotionParams> &motion_params,
    const double & /*effort*/) {
  RCLCPP_INFO(LOGGER, "Starting task execution...");

  // 1. Move to the home position
  RCLCPP_INFO(LOGGER, "Moving to home position...");
  if (!planAndMoveToJointPosition(HOME_POSITION, motion_params.at("home"))) {
    RCLCPP_ERROR(LOGGER, "Failed to move to home position.");
    return;
  }
  RCLCPP_INFO(LOGGER, "Successfully moved to home position.");

  // 2. Open the gripper before approaching the object
  RCLCPP_INFO(LOGGER, "Opening gripper...");
  if (!controlGripper(GRIPPER_OPEN_ACTION)) {
    RCLCPP_ERROR(LOGGER, "Failed to open the gripper.");
    return;
  }
  RCLCPP_INFO(LOGGER, "Gripper opened successfully.");

  // 3. Move to the pre-grasp position near the object
  if (!moveToPreGraspPosition(pre_grasp_pose, motion_params.at("pre_grasp"))) {
    return;
  }

  // 4. Draw X
  if (!draw_X(pre_grasp_pose, motion_params.at("draw_x"))) {
    return;
  }

  // 5. Move back to the home position after grasp
  RCLCPP_INFO(LOGGER, "Returning to home position...");
  if (!planAndMoveToJointPosition(HOME_POSITION, motion_params.at("home"))) {
    RCLCPP_ERROR(LOGGER,
                 "Failed to move to home position after releasing object.");
    return;
  }
  RCLCPP_INFO(LOGGER, "Successfully returned to home position.");

  RCLCPP_INFO(LOGGER, "Task execution completed successfully.");
}

// Plan and move the arm to a specific joint position
bool DrawXtNode::planAndMoveToJointPosition(
    const std::vector<double> &joint_position, const MotionParams &params) {
  move_group_arm_->setMaxVelocityScalingFactor(params.velocity_scaling);
  move_group_arm_->setMaxAccelerationScalingFactor(params.acceleration_scaling);
  move_group_arm_->setPlanningTime(params.planning_time);

  move_group_arm_->setJointValueTarget(joint_position);
  bool success =
      (move_group_arm_->move() == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_ERROR(LOGGER, "Failed to move to joint position.");
  }
  return success;
}

// Plan and move the arm to a specific target pose
bool DrawXtNode::planAndMoveToPose(const geometry_msgs::msg::Pose &target_pose,
                                   const MotionParams &params) {
  move_group_arm_->setMaxVelocityScalingFactor(params.velocity_scaling);
  move_group_arm_->setMaxAccelerationScalingFactor(params.acceleration_scaling);
  move_group_arm_->setPlanningTime(params.planning_time);

  move_group_arm_->setPoseTarget(target_pose);
  bool success =
      (move_group_arm_->move() == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_ERROR(LOGGER, "Failed to move to target pose.");
  }
  return success;
}

// Control the gripper (open or close) using predefined actions
bool DrawXtNode::controlGripper(const std::string &action) {
  move_group_gripper_->setNamedTarget(action);
  bool success =
      (move_group_gripper_->move() == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_ERROR(LOGGER, "Failed to %s the gripper.", action.c_str());
  }
  return success;
}
// Plan and move to pre-grasp position
bool DrawXtNode::moveToPreGraspPosition(
    const geometry_msgs::msg::Pose &pre_grasp_pose,
    const MotionParams &params) {
  RCLCPP_INFO(LOGGER, "Moving to pre-grasp position...");
  if (!planAndMoveToPose(pre_grasp_pose, params)) {
    RCLCPP_ERROR(LOGGER, "Failed to move to pre-grasp position.");
    return false;
  }
  RCLCPP_INFO(LOGGER, "Successfully moved to pre-grasp position.");
  return true;
}

// Execute a Cartesian path based on a series of waypoints
bool DrawXtNode::performCartesianPath(
    const std::vector<geometry_msgs::msg::Pose> &waypoints) {

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_arm_->computeCartesianPath(
      waypoints, EEF_STEP, JUMP_THRESHOLD, trajectory);
  if (fraction < 1) {
    RCLCPP_WARN(
        LOGGER,
        "Failed to perform Cartesian path execution.. Only covered %.2f%% "
        "of the path.",
        fraction * 100.0);
    return false;
  }
  return move_group_arm_->execute(trajectory) ==
         moveit::core::MoveItErrorCode::SUCCESS;
}

// draw X
bool DrawXtNode::draw_X(const geometry_msgs::msg::Pose &pre_grasp_pose,
                        const MotionParams &params) {
  RCLCPP_INFO(LOGGER, "Drawing with delta: %.2f", DELTA);

  move_group_arm_->setMaxVelocityScalingFactor(params.velocity_scaling);
  move_group_arm_->setMaxAccelerationScalingFactor(params.acceleration_scaling);
  move_group_arm_->setPlanningTime(params.acceleration_scaling);

  // Create waypoints for Cartesian movement
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose = pre_grasp_pose;

  // waypoint 1
  target_pose.position.x -= DELTA;
  waypoints.push_back(target_pose);

  // waypoint 2
  target_pose.position.x += DELTA;
  target_pose.position.y += DELTA;
  waypoints.push_back(target_pose);

  // waypoint 3
  target_pose.position.x -= DELTA;
  waypoints.push_back(target_pose);

  // waypoint 4
  target_pose.position.x += DELTA;
  target_pose.position.y -= DELTA;
  waypoints.push_back(target_pose);

  // Execute the Cartesian path

  if (!performCartesianPath(waypoints)) {
    RCLCPP_ERROR(LOGGER, "Failed to draw X.");
    return false;
  }

  RCLCPP_INFO(LOGGER, "Successfully completed Draw X.");
  return true;
}

// Create a target pose based on position and orientation
geometry_msgs::msg::Pose DrawXtNode::createTargetPose(double x, double y,
                                                      double z, double w) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = w;
  return pose;
}