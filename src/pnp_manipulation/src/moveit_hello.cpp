#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <thread>
#include <vector>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Parameters (robot_description, SRDF, kinematics) come from the launch file.
  // This option makes the node accept them without declaring each one.
  auto node = std::make_shared<rclcpp::Node>(
    "moveit_hello",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  // MoveGroupInterface needs the node to be "spinning" (processing messages)
  // in the background, e.g. to receive joint states. So spin it in a thread.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  // One interface per planning group from the SRDF
  MoveGroupInterface arm(node, "ur_manipulator");
  MoveGroupInterface gripper(node, "gripper");
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);

  RCLCPP_INFO(logger, "Planning frame: %s, end effector: %s",
    arm.getPlanningFrame().c_str(), arm.getEndEffectorLink().c_str());

  // Helper: plan a motion and execute it only if planning succeeded
  auto plan_and_execute = [&](MoveGroupInterface & group, const char * what) {
      MoveGroupInterface::Plan plan;
      if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Planning failed: %s", what);
        return false;
      }
      RCLCPP_INFO(logger, "Executing: %s", what);
      return group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    };

  // 1) Named target from the SRDF
  arm.setNamedTarget("home");
  if (!plan_and_execute(arm, "home")) {rclcpp::shutdown(); spinner.join(); return 1;}

  // 2) Pose goal: 20 cm above a point on the table, gripper pointing straight down.
  //    The quaternion (x=1, y=0, z=0, w=0) is a 180 deg rotation about X, which
  //    turns the gripper's Z axis to point down.
  geometry_msgs::msg::Pose above;
  above.position.x = 0.50;
  above.position.y = -0.15;
  above.position.z = 0.60;
  above.orientation.x = 1.0;
  above.orientation.y = 0.0;
  above.orientation.z = 0.0;
  above.orientation.w = 0.0;
  arm.setPoseTarget(above);
  if (!plan_and_execute(arm, "pose above target")) {rclcpp::shutdown(); spinner.join(); return 1;}

  // 3) Cartesian path: straight down 15 cm, in 5 mm steps
  geometry_msgs::msg::Pose down = above;
  down.position.z -= 0.15;
  std::vector<geometry_msgs::msg::Pose> waypoints{down};
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = arm.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
  RCLCPP_INFO(logger, "Cartesian path down: %.0f%% achieved", fraction * 100.0);
  if (fraction > 0.95) {
    arm.execute(trajectory);
  }

  // 4) Close the gripper (named target from the SRDF)
  gripper.setNamedTarget("closed");
  plan_and_execute(gripper, "close gripper");

  // 5) Straight back up
  waypoints = {above};
  if (arm.computeCartesianPath(waypoints, 0.005, 0.0, trajectory) > 0.95) {
    RCLCPP_INFO(logger, "Executing: lift");
    arm.execute(trajectory);
  }

  // 6) Open the gripper and go home
  gripper.setNamedTarget("open");
  plan_and_execute(gripper, "open gripper");
  arm.setNamedTarget("home");
  plan_and_execute(arm, "home");

  RCLCPP_INFO(logger, "Done!");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
