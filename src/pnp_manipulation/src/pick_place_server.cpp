#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <pnp_interfaces/action/pick_place.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "pnp_manipulation/grasp_planner.hpp"

namespace pnp_manipulation
{

using PickPlace = pnp_interfaces::action::PickPlace;
using GoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;
using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

// Read a parameter, declaring it with a default if it wasn't given in the YAML.
// (The node auto-declares everything from the YAML, so declare_parameter would
// throw "already declared" for those.)
template<typename T>
T param(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & def)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, def);
  }
  return node->get_parameter(name).get_value<T>();
}

class PickPlaceServer
{
public:
  explicit PickPlaceServer(const rclcpp::Node::SharedPtr & node)
  : node_(node), logger_(node->get_logger())
  {
    // ---- Parameters
    tcp_link_ = param<std::string>(node_, "tcp_link", "gripper_tcp");
    world_frame_ = param<std::string>(node_, "world_frame", "world");
    surface_z_ = param<double>(node_, "place_surface_z", 0.40);
    eef_step_ = param<double>(node_, "cartesian_eef_step", 0.005);
    min_fraction_ = param<double>(node_, "min_cartesian_fraction", 0.95);
    plan_retries_ = static_cast<int>(param<int64_t>(node_, "plan_retries", 3));
    params_.approach_distance = param<double>(node_, "approach_distance", 0.12);
    params_.lift_distance = param<double>(node_, "lift_distance", 0.15);
    params_.grasp_depth = param<double>(node_, "grasp_depth", 0.015);
    params_.place_clearance = param<double>(node_, "place_clearance", 0.005);
    table_size_ = param<std::vector<double>>(node_, "table_size", {0.8, 1.2, 0.40});
    table_center_ = param<std::vector<double>>(node_, "table_center", {0.55, 0.0, 0.20});

    // ---- MoveIt interfaces
    arm_ = std::make_shared<MoveGroupInterface>(node_, "ur_manipulator");
    gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
    arm_->setEndEffectorLink(tcp_link_);
    arm_->setPoseReferenceFrame(world_frame_);
    arm_->setMaxVelocityScalingFactor(0.4);
    arm_->setMaxAccelerationScalingFactor(0.4);
    arm_->setPlanningTime(5.0);
    arm_->setNumPlanningAttempts(5);

    // Tells the simulation helper (Step 18) when to hold / release the cube
    grasp_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/pnp/gripper/grasp", 10);

    addTable();

    // ---- The action server: accept a goal, allow cancel, run it in a thread
    server_ = rclcpp_action::create_server<PickPlace>(
      node_, "pick_place",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const PickPlace::Goal> goal) {
        if (busy_) {
          RCLCPP_WARN(logger_, "Rejecting %s: busy", goal->object_id.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        arm_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> gh) {
        std::thread{[this, gh]() {execute(gh);}}.detach();
      });

    RCLCPP_INFO(logger_, "PickPlace server ready");
  }

private:
  // ------------------------------------------------------------ scene helpers
  moveit_msgs::msg::CollisionObject box(
    const std::string & id, const std::vector<double> & size, const geometry_msgs::msg::Pose & pose)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = world_frame_;
    obj.id = id;
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {size.at(0), size.at(1), size.at(2)};   // BoundedVector<double, 3>
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    return obj;
  }

  void addTable()
  {
    geometry_msgs::msg::Pose p;
    p.position.x = table_center_.at(0);
    p.position.y = table_center_.at(1);
    p.position.z = table_center_.at(2);
    p.orientation.w = 1.0;
    scene_.applyCollisionObject(box("table", table_size_, p));
  }

  // ----------------------------------------------------------- motion helpers
  bool gripperTo(const std::string & named)
  {
    gripper_->setNamedTarget(named);
    return gripper_->move() == moveit::core::MoveItErrorCode::SUCCESS;
  }

  // Plan + execute, with retries (OMPL is randomised; a retry often succeeds)
  bool planAndExecute(const std::function<void()> & set_target)
  {
    for (int attempt = 1; attempt <= plan_retries_; ++attempt) {
      arm_->setStartStateToCurrentState();
      set_target();
      MoveGroupInterface::Plan plan;
      if (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        return arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      }
      RCLCPP_WARN(logger_, "Planning attempt %d/%d failed", attempt, plan_retries_);
    }
    return false;
  }

  bool moveTo(const Eigen::Isometry3d & target)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = world_frame_;
    ps.pose = tf2::toMsg(target);
    return planAndExecute([&] {arm_->setPoseTarget(ps, tcp_link_);});
  }

  bool straightTo(const Eigen::Isometry3d & target)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints{tf2::toMsg(target)};
    moveit_msgs::msg::RobotTrajectory traj;
    arm_->setStartStateToCurrentState();
    const double fraction = arm_->computeCartesianPath(waypoints, eef_step_, 0.0, traj, true);
    if (fraction < min_fraction_) {
      RCLCPP_WARN(logger_, "Cartesian path only %.0f%% feasible", fraction * 100.0);
      return false;
    }
    return arm_->execute(traj) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  void setGrasp(bool on)
  {
    std_msgs::msg::Bool b;
    b.data = on;
    grasp_pub_->publish(b);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  // ---------------------------------------------------------------- execute
  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    busy_ = true;
    const auto goal = gh->get_goal();
    auto feedback = std::make_shared<PickPlace::Feedback>();
    auto result = std::make_shared<PickPlace::Result>();
    const double h = goal->object_height > 0.0 ? goal->object_height : 0.04;
    const std::string id = goal->object_id.empty() ? "target" : goal->object_id;

    // Wait for a fresh robot state, or planning starts from garbage
    if (!arm_->getCurrentState(10.0)) {
      result->success = false;
      result->failed_stage = "wait_for_state";
      result->message = "no current robot state (is joint_states published?)";
      gh->abort(result);
      busy_ = false;
      return;
    }

    // ---- Geometry from YOUR grasp planner
    Eigen::Isometry3d obj;
    tf2::fromMsg(goal->pick_pose.pose, obj);
    const double wrist_yaw = yawOf(
      Eigen::Quaterniond(arm_->getCurrentPose(tcp_link_).pose.orientation.w,
      arm_->getCurrentPose(tcp_link_).pose.orientation.x,
      arm_->getCurrentPose(tcp_link_).pose.orientation.y,
      arm_->getCurrentPose(tcp_link_).pose.orientation.z));
    const auto pick_options = pickCandidates(obj, h, params_, wrist_yaw);
    GraspSequence pick = pick_options.front();
    GraspSequence place;
    const Eigen::Vector3d place_xy(
      goal->place_pose.pose.position.x, goal->place_pose.pose.position.y, 0.0);

    // The cube as a collision object, so MoveIt avoids it on the way
    const std::vector<double> cube{h, h, h};
    scene_.applyCollisionObject(box(id, cube, goal->pick_pose.pose));

    // ---- The stages
    struct Stage
    {
      std::string name;
      std::function<bool()> run;
    };
    const std::vector<Stage> stages = {
      {"open_gripper", [&] {return gripperTo("open");}},
      {"move_to_pregrasp", [&] {
          for (const auto & option : pick_options) {        // try all 4 symmetric yaws
            if (moveTo(option.pre)) {pick = option; return true;}
            RCLCPP_WARN(logger_, "Grasp yaw unreachable, trying the next one");
          }
          return false;
        }},
      // The fingers must be allowed to reach the cube: remove it from the scene first
      {"approach", [&] {scene_.removeCollisionObjects({id}); return straightTo(pick.grasp);}},
      {"close_gripper", [&] {return gripperTo("closed");}},
      {"attach", [&] {
          setGrasp(true);
          scene_.applyCollisionObject(box(id, cube, goal->pick_pose.pose));
          return arm_->attachObject(id, tcp_link_,
          {"left_finger_link", "right_finger_link", "gripper_base_link"});
        }},
      {"lift", [&] {return straightTo(pick.retreat);}},
      {"move_to_preplace", [&] {
          const double carry_yaw = yawOf(Eigen::Quaterniond(pick.grasp.rotation()));
          for (double dk : {0.0, 1.0, -1.0, 2.0}) {           // 4 equivalent place yaws
            auto option = computePlaceSequence(
              place_xy, surface_z_, h, wrapAngle(carry_yaw + dk * M_PI / 2.0), params_);
            if (moveTo(option.pre)) {place = option; return true;}
          }
          return false;
        }},
      {"lower", [&] {return straightTo(place.grasp);}},
      {"release", [&] {
          setGrasp(false);
          arm_->detachObject(id);
          scene_.removeCollisionObjects({id});
          return gripperTo("open");
        }},
      {"retreat", [&] {return straightTo(place.retreat);}},
      // Non-fatal: the object is already placed
      {"move_to_observe", [&] {
          if (!planAndExecute([&] {arm_->setNamedTarget("observe");})) {
            RCLCPP_WARN(logger_, "Could not reach 'observe'; camera view maybe blocked");
          }
          return true;
        }},
    };

    for (size_t i = 0; i < stages.size(); ++i) {
      if (gh->is_canceling()) {
        cleanup(id);
        result->success = false;
        result->message = "canceled";
        result->failed_stage = stages[i].name;
        gh->canceled(result);
        busy_ = false;
        return;
      }
      feedback->stage = stages[i].name;
      feedback->progress = static_cast<float>(i) / static_cast<float>(stages.size());
      gh->publish_feedback(feedback);
      RCLCPP_INFO(logger_, "[%s] %s", id.c_str(), stages[i].name.c_str());

      if (!stages[i].run()) {
        RCLCPP_ERROR(logger_, "[%s] stage '%s' failed", id.c_str(), stages[i].name.c_str());
        cleanup(id);
        result->success = false;
        result->failed_stage = stages[i].name;
        result->message = "stage failed: " + stages[i].name;
        gh->abort(result);
        busy_ = false;
        return;
      }
    }
    result->success = true;
    result->message = "placed " + id;
    gh->succeed(result);
    busy_ = false;
  }

  void cleanup(const std::string & id)
  {
    setGrasp(false);
    arm_->detachObject(id);
    scene_.removeCollisionObjects({id});
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::shared_ptr<MoveGroupInterface> arm_, gripper_;
  PlanningSceneInterface scene_;
  rclcpp_action::Server<PickPlace>::SharedPtr server_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasp_pub_;

  GraspParams params_;
  std::string tcp_link_, world_frame_;
  double surface_z_{0.40}, eef_step_{0.005}, min_fraction_{0.95};
  int plan_retries_{3};
  std::vector<double> table_size_, table_center_;
  std::atomic<bool> busy_{false};
};

}  // namespace pnp_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "pick_place_server",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // MoveGroupInterface needs the node spinning; a multi-threaded executor lets the
  // action callbacks and MoveIt's internal callbacks run at the same time.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() {exec.spin();});

  auto server = std::make_shared<pnp_manipulation::PickPlaceServer>(node);

  spinner.join();
  rclcpp::shutdown();
  return 0;
}
