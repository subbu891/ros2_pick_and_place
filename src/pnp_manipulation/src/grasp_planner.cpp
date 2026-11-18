#include "pnp_manipulation/grasp_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace pnp_manipulation
{

double wrapAngle(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a <= 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

double yawOf(const Eigen::Quaterniond & q)
{
  // Where does the rotated X axis point, seen from above?
  const Eigen::Vector3d x = q * Eigen::Vector3d::UnitX();
  return std::atan2(x.y(), x.x());
}

double chooseGraspYaw(double object_yaw, double reference_yaw)
{
  double best = object_yaw;
  double best_err = std::numeric_limits<double>::max();
  for (int k = -2; k <= 2; ++k) {
    const double cand = wrapAngle(object_yaw + k * M_PI / 2.0);
    const double err = std::abs(wrapAngle(cand - reference_yaw));
    if (err < best_err) {
      best_err = err;
      best = cand;
    }
  }
  return best;
}

Eigen::Quaterniond topDownOrientation(double yaw)
{
  // First flip the tool upside down (180 deg about X), then turn it about world Z.
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).normalized();
}

namespace
{
Eigen::Isometry3d makePose(const Eigen::Vector3d & p, const Eigen::Quaterniond & q)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() = p;
  return T;
}
}  // namespace

GraspSequence computePickSequence(
  const Eigen::Isometry3d & object_center, double object_height,
  const GraspParams & params, double reference_yaw)
{
  const double obj_yaw = yawOf(Eigen::Quaterniond(object_center.rotation()));
  const Eigen::Quaterniond q = topDownOrientation(chooseGraspYaw(obj_yaw, reference_yaw));

  const Eigen::Vector3d c = object_center.translation();
  const double top_z = c.z() + object_height / 2.0;
  const Eigen::Vector3d grasp_p(c.x(), c.y(), top_z - params.grasp_depth);

  GraspSequence s;
  s.grasp = makePose(grasp_p, q);
  s.pre = makePose(grasp_p + Eigen::Vector3d(0, 0, params.approach_distance), q);
  s.retreat = makePose(grasp_p + Eigen::Vector3d(0, 0, params.lift_distance), q);
  return s;
}

std::vector<GraspSequence> pickCandidates(
  const Eigen::Isometry3d & object_center, double object_height,
  const GraspParams & params, double reference_yaw)
{
  std::vector<GraspSequence> out;
  for (int k = 0; k < 4; ++k) {
    Eigen::Isometry3d rotated = object_center;
    rotated.linear() = object_center.linear() *
      Eigen::AngleAxisd(k * M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const double yaw_k = yawOf(Eigen::Quaterniond(rotated.rotation()));
    out.push_back(computePickSequence(rotated, object_height, params, yaw_k));
  }
  // Least wrist rotation first
  std::sort(out.begin(), out.end(), [reference_yaw](const GraspSequence & a,
    const GraspSequence & b) {
      const double ea = std::abs(wrapAngle(
        yawOf(Eigen::Quaterniond(a.grasp.rotation())) - reference_yaw));
      const double eb = std::abs(wrapAngle(
        yawOf(Eigen::Quaterniond(b.grasp.rotation())) - reference_yaw));
      return ea < eb;
    });
  return out;
}

GraspSequence computePlaceSequence(
  const Eigen::Vector3d & place_xy, double surface_z, double object_height, double yaw,
  const GraspParams & params)
{
  const Eigen::Quaterniond q = topDownOrientation(yaw);
  const double top_z = surface_z + params.place_clearance + object_height;
  const Eigen::Vector3d grasp_p(place_xy.x(), place_xy.y(), top_z - params.grasp_depth);

  GraspSequence s;
  s.grasp = makePose(grasp_p, q);
  s.pre = makePose(grasp_p + Eigen::Vector3d(0, 0, params.approach_distance), q);
  s.retreat = s.pre;   // after releasing, go back straight up
  return s;
}

}  // namespace pnp_manipulation
