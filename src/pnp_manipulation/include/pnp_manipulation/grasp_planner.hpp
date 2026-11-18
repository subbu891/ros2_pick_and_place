#pragma once

#include <Eigen/Geometry>

#include <vector>

namespace pnp_manipulation
{

struct GraspParams
{
  double approach_distance{0.12};  ///< pre-grasp height above the grasp pose [m]
  double lift_distance{0.15};      ///< lift height after closing [m]
  double grasp_depth{0.015};       ///< TCP this far below the object's top face [m]
  double place_clearance{0.005};   ///< gap left under the object when placing [m]
};

/// Three TCP poses (world frame) for one pick or one place.
struct GraspSequence
{
  Eigen::Isometry3d pre;      ///< above the object, gripper pointing down
  Eigen::Isometry3d grasp;    ///< at the object
  Eigen::Isometry3d retreat;  ///< straight up afterwards
};

/// Wrap an angle to (-pi, pi].
double wrapAngle(double a);

/// Rotation about world Z of an orientation.
double yawOf(const Eigen::Quaterniond & q);

/// Among object_yaw + k*90deg, pick the yaw closest to reference_yaw.
double chooseGraspYaw(double object_yaw, double reference_yaw);

/// TCP orientation pointing straight down (tool Z = -world Z) with the given yaw.
Eigen::Quaterniond topDownOrientation(double yaw);

/// Pick poses for an object whose CENTRE pose is given (only its yaw is used).
GraspSequence computePickSequence(
  const Eigen::Isometry3d & object_center, double object_height,
  const GraspParams & params, double reference_yaw = 0.0);

/// All four symmetric grasps of a cube, least wrist rotation first.
std::vector<GraspSequence> pickCandidates(
  const Eigen::Isometry3d & object_center, double object_height,
  const GraspParams & params, double reference_yaw = 0.0);

/// Place poses: put the object centre at place_xy, resting on a surface at surface_z.
GraspSequence computePlaceSequence(
  const Eigen::Vector3d & place_xy, double surface_z, double object_height, double yaw,
  const GraspParams & params);

}  // namespace pnp_manipulation
