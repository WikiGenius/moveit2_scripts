#include "moveit2_scripts/approach_and_retreat_node.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>

// Constructor
ApproachAndRetreatNode::ApproachAndRetreatNode(
    const rclcpp::NodeOptions &node_options)
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
ApproachAndRetreatNode::~ApproachAndRetreatNode() {
  // Stop the executor and join the thread to ensure proper cleanup
  executor_.cancel();
  if (node_thread_.joinable()) {
    node_thread_.join();
  }
}
// Initialize the MoveGroup interfaces
void ApproachAndRetreatNode::initializeMoveGroup() {
  move_group_arm_->setPoseReferenceFrame("base_footprint");
}

// Main Task Execution
void ApproachAndRetreatNode::executeTask(
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

  // 4. Approach the object (move down by delta_z for approach)
  RCLCPP_INFO(LOGGER, "Approaching ..");
  double approach_delta_z = -APPROACH_DELTA_Z; // Approach by moving down
  if (!performCartesianMove(pre_grasp_pose, approach_delta_z,
                            motion_params.at("approach"))) {
    RCLCPP_ERROR(LOGGER, "Failed to Approach.");

    return;
  }
  RCLCPP_INFO(LOGGER, "successfully Approach.");

  // 5. Close the gripper to grasp the object
  RCLCPP_INFO(LOGGER, "Closing gripper to grasp object...");
  if (!controlGripper(GRIPPER_CLOSE_ACTION)) {
    RCLCPP_ERROR(LOGGER, "Failed to close the gripper.");
    return;
  }
  RCLCPP_INFO(LOGGER, "Object successfully grasped.");

  // 6. Retreat after grasp (move up by delta_z for retreat)
  RCLCPP_INFO(LOGGER, "Retreating ..");

  double retreat_delta_z = RETREAT_DELTA_Z; // Retreat by moving up
  if (!performCartesianMove(pre_grasp_pose, retreat_delta_z,
                            motion_params.at("retreat"))) {
    RCLCPP_ERROR(LOGGER, "Failed to Retreat.");

    return;
  }
  RCLCPP_INFO(LOGGER, "successfully Retreat.");

  // 7. Open the gripper to release the object
  RCLCPP_INFO(LOGGER, "Opening gripper to release object...");
  if (!controlGripper(GRIPPER_OPEN_ACTION)) {
    RCLCPP_ERROR(LOGGER, "Failed to open the gripper after retreat.");
    return;
  }
  RCLCPP_INFO(LOGGER, "Object released successfully.");

  // 8. Move back to the home position after grasp
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
bool ApproachAndRetreatNode::planAndMoveToJointPosition(
    const std::vector<double> &joint_position, const MotionParams &params) {
  move_group_arm_->setMaxVelocityScalingFactor(params.velocity_scaling);
  move_group_arm_->setMaxAccelerationScalingFactor(params.acceleration_scaling);
  move_group_arm_->setPlanningTime(params.planning_time);

  move_group_arm_->setJointValueTarget(joint_position);
  bool success = (move_group_arm_->move() ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_ERROR(LOGGER, "Failed to move to joint position.");
  }
  return success;
}

// Plan and move the arm to a specific target pose
bool ApproachAndRetreatNode::planAndMoveToPose(
    const geometry_msgs::msg::Pose &target_pose, const MotionParams &params) {
  move_group_arm_->setMaxVelocityScalingFactor(params.velocity_scaling);
  move_group_arm_->setMaxAccelerationScalingFactor(params.acceleration_scaling);
  move_group_arm_->setPlanningTime(params.planning_time);

  move_group_arm_->setPoseTarget(target_pose);
  bool success = (move_group_arm_->move() ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_ERROR(LOGGER, "Failed to move to target pose.");
  }
  return success;
}

// Execute a Cartesian path based on a series of waypoints
bool ApproachAndRetreatNode::performCartesianPath(
    const std::vector<geometry_msgs::msg::Pose> &waypoints,
    const MotionParams &params) {
  move_group_arm_->setMaxVelocityScalingFactor(params.velocity_scaling);
  move_group_arm_->setMaxAccelerationScalingFactor(params.acceleration_scaling);

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_arm_->computeCartesianPath(
      waypoints, EEF_STEP, JUMP_THRESHOLD, trajectory);
  if (fraction < 1.0) {
    RCLCPP_WARN(LOGGER,
                "Cartesian path execution was incomplete. Only covered %.2f%% "
                "of the path.",
                fraction * 100.0);
  }
  return move_group_arm_->execute(trajectory) ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

// Control the gripper (open or close) using predefined actions
bool ApproachAndRetreatNode::controlGripper(const std::string &action) {
  move_group_gripper_->setNamedTarget(action);
  bool success = (move_group_gripper_->move() ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_ERROR(LOGGER, "Failed to %s the gripper.", action.c_str());
  }
  return success;
}
// Plan and move to pre-grasp position
bool ApproachAndRetreatNode::moveToPreGraspPosition(
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

// Perform a Cartesian move (either approach or retreat) based on delta_z
bool ApproachAndRetreatNode::performCartesianMove(
    const geometry_msgs::msg::Pose &pre_grasp_pose, const double &delta_z,
    const MotionParams &params) {
  // Create waypoints for Cartesian movement
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose = pre_grasp_pose;
  target_pose.position.z +=
      delta_z; // Apply the delta (positive for retreat, negative for approach)

  waypoints.push_back(target_pose);

  // Execute the Cartesian path
  RCLCPP_INFO(LOGGER, "Performing Cartesian move with delta_z: %.2f", delta_z);
  if (!performCartesianPath(waypoints, params)) {
    RCLCPP_ERROR(LOGGER, "Failed to execute Cartesian move.");
    return false;
  }
  RCLCPP_INFO(LOGGER, "Successfully completed Cartesian move.");
  return true;
}
// Create a target pose based on position and orientation
geometry_msgs::msg::Pose ApproachAndRetreatNode::createTargetPose(double x,
                                                                  double y,
                                                                  double z,
                                                                  double w) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = w;
  return pose;
}