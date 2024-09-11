#ifndef APPROACH_AND_RETREAT_NODE_HPP
#define APPROACH_AND_RETREAT_NODE_HPP

#include <chrono>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>

// Structure to hold motion parameters for velocity, acceleration, and planning
// time
struct MotionParams {
  double velocity_scaling;
  double acceleration_scaling;
  double planning_time;
};

class ApproachAndRetreatNode {
public:
  /**
   * @brief Constructor: Initializes the node and sets up the MoveGroup
   * interfaces.
   *
   * @param options NodeOptions for configuring the node (e.g., parameter
   * handling)
   */
  ApproachAndRetreatNode(const rclcpp::NodeOptions &options);

  /**
   * @brief Destructor: Joins the executor thread when the object is destroyed.
   */
  ~ApproachAndRetreatNode();
  /**
   * @brief Executes the main task workflow with adjustable speed and planning
   * time. This function coordinates the robot's entire sequence:
   * - Move to home position
   * - Open the gripper
   * - Approach the object
   * - Grasp the object
   * - Retreat after grasp
   * - Open gripper to release object
   * - Move back to home position
   * @param pre_grasp_pose The pose for pre-grasp positioning.
   * @param motion_params A map containing velocity, acceleration, and planning
   * time for each task.
   */
  void executeTask(const geometry_msgs::msg::Pose &pre_grasp_pose,
                   const std::map<std::string, MotionParams> &motion_params,
                   const double &effort);

  /**
   * @brief Creates a target pose for the robot's end-effector.
   *
   * @param x X-coordinate of the pose
   * @param y Y-coordinate of the pose
   * @param z Z-coordinate of the pose
   * @param w Orientation (quaternion W-component) of the pose
   * @return geometry_msgs::msg::Pose The generated pose object
   */
  geometry_msgs::msg::Pose createTargetPose(double x, double y, double z,
                                            double w);

private:
  // ROS 2 Node
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::NodeOptions node_options_;
  std::thread node_thread_;
  rclcpp::executors::SingleThreadedExecutor
      executor_; // Single-threaded executor

  // Move group interfaces for planning and controlling the arm and gripper
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_arm_; ///< Interface for controlling the robot arm
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_gripper_; ///< Interface for controlling the gripper

  // Motion planning constants
  const std::string PLANNING_GROUP_ARM =
      "ur_manipulator"; ///< Name of the arm's planning group
  const std::string PLANNING_GROUP_GRIPPER =
      "gripper";                ///< Name of the gripper's planning group
  const double EEF_STEP = 0.01; ///< End-effector step size for Cartesian paths
  const double JUMP_THRESHOLD =
      0.0; ///< Threshold to prevent large joint-space jumps
  const double APPROACH_DELTA_Z =
      0.05; ///< Distance to move down for the approach (in meters)
  const double RETREAT_DELTA_Z =
      0.05; ///< Distance to move up for the retreat (in meters)

  // Named positions and actions
  const std::vector<double> HOME_POSITION = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; ///< Joint angles for the home position
  const std::string GRIPPER_OPEN_ACTION =
      "open"; ///< Named target for opening the gripper
  const std::string GRIPPER_CLOSE_ACTION =
      "close"; ///< Named target for closing the gripper

  // Logging setup
  const rclcpp::Logger LOGGER = rclcpp::get_logger("approach_retreat");
  // -------------------------- Private Methods ----------------------------

  /**
   * @brief Initialize the MoveGroup interfaces for both the arm and the
   * gripper. Configures velocity scaling factors, acceleration scaling, and
   * planning settings.
   */
  void initializeMoveGroup();

  /**
   * @brief Plans and moves the arm to a specific joint position.
   *
   * @param joint_position Vector of joint angles to move the arm to
   * @return true if the movement succeeded, false otherwise
   */
  bool planAndMoveToJointPosition(const std::vector<double> &joint_position,
                                  const MotionParams &params);

  /**
   * @brief Plans and moves the arm to a specific target pose.
   *
   * @param target_pose The 3D target pose for the end-effector
   * @return true if the movement succeeded, false otherwise
   */
  bool planAndMoveToPose(const geometry_msgs::msg::Pose &target_pose,
                         const MotionParams &params);

  /**
   * @brief Executes a Cartesian path (straight-line movement) based on
   * waypoints.
   *
   * @param waypoints Vector of poses defining the Cartesian path
   * @return true if the path was executed successfully, false otherwise
   */
  bool
  performCartesianPath(const std::vector<geometry_msgs::msg::Pose> &waypoints,
                       const MotionParams &params);

  /**
   * @brief Controls the gripper (open or close) using predefined actions.
   *
   * @param action A string specifying the gripper action (either "open" or
   * "close")
   * @return true if the action succeeded, false otherwise
   */
  bool controlGripper(const std::string &action);

  /**
   * @brief APlan and move to pre-grasp position
   *
   * @param pre_grasp_pose The initial pre-grasp pose
   * @return true if the approach succeeded, false otherwise
   */

  bool moveToPreGraspPosition(const geometry_msgs::msg::Pose &pre_grasp_pose,
                              const MotionParams &params);
  /**
   * @brief  Perform a Cartesian move (either approach or retreat) based on
   * delta_z
   *
   * @param pre_grasp_pose The initial pre-grasp pose used for retreat
   * reference
   * @return true if the retreat succeeded, false otherwise
   */
  bool performCartesianMove(const geometry_msgs::msg::Pose &pre_grasp_pose,
                            const double &delta_z, const MotionParams &params);
};
#endif // APPROACH_AND_RETREAT_NODE_HPP
