#include <gtest/gtest.h>

#include <cmath>

#include "pnp_manipulation/grasp_planner.hpp"

using namespace pnp_manipulation;  // NOLINT

namespace
{
Eigen::Isometry3d objectAt(double x, double y, double z, double yaw)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << x, y, z;
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return T;
}
constexpr double kDeg = M_PI / 180.0;
}  // namespace

TEST(WrapAngle, StaysInRange)
{
  EXPECT_NEAR(wrapAngle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(wrapAngle(M_PI), M_PI, 1e-12);
  EXPECT_NEAR(wrapAngle(-M_PI), M_PI, 1e-12);
  EXPECT_NEAR(wrapAngle(2.5 * M_PI), 0.5 * M_PI, 1e-9);
}

TEST(ChooseGraspYaw, PicksEquivalentYawClosestToWrist)
{
  // Cube at 80 deg == -10 deg; with the wrist at 0, choose -10 deg
  EXPECT_NEAR(chooseGraspYaw(80 * kDeg, 0.0), -10 * kDeg, 1e-9);
  // Wrist already near 90 deg -> keep 80 deg
  EXPECT_NEAR(chooseGraspYaw(80 * kDeg, M_PI / 2.0), 80 * kDeg, 1e-9);
}

TEST(TopDown, ToolZPointsDown)
{
  for (double yaw : {0.0, 0.3, -1.2, M_PI / 2.0}) {
    const Eigen::Quaterniond q = topDownOrientation(yaw);
    EXPECT_NEAR((q * Eigen::Vector3d::UnitZ()).z(), -1.0, 1e-9);
    EXPECT_NEAR(wrapAngle(yawOf(q) - yaw), 0.0, 1e-9);
  }
}

TEST(PickSequence, GeometryIsConsistent)
{
  GraspParams p;
  p.approach_distance = 0.1;
  p.lift_distance = 0.2;
  p.grasp_depth = 0.02;
  const auto s = computePickSequence(objectAt(0.5, -0.2, 0.42, 0.1), 0.04, p, 0.0);

  // top face = 0.42 + 0.02 = 0.44; TCP 0.02 below = 0.42
  EXPECT_NEAR(s.grasp.translation().x(), 0.5, 1e-12);
  EXPECT_NEAR(s.grasp.translation().y(), -0.2, 1e-12);
  EXPECT_NEAR(s.grasp.translation().z(), 0.42, 1e-12);
  EXPECT_NEAR(s.pre.translation().z() - s.grasp.translation().z(), 0.1, 1e-12);
  EXPECT_NEAR(s.retreat.translation().z() - s.grasp.translation().z(), 0.2, 1e-12);
  // The approach is purely vertical: same XY, same orientation
  EXPECT_TRUE(s.pre.linear().isApprox(s.grasp.linear(), 1e-12));
  EXPECT_NEAR((s.pre.translation() - s.grasp.translation()).head<2>().norm(), 0.0, 1e-12);
}

TEST(PlaceSequence, ObjectEndsUpOnSurface)
{
  GraspParams p;
  p.grasp_depth = 0.02;
  p.place_clearance = 0.005;
  const auto s = computePlaceSequence({0.1, 0.6, 0.0}, 0.40, 0.04, 0.0, p);
  // top of object = 0.40 + 0.005 + 0.04 = 0.445; TCP 0.02 below = 0.425
  EXPECT_NEAR(s.grasp.translation().z(), 0.425, 1e-12);
  EXPECT_GT(s.pre.translation().z(), s.grasp.translation().z());
}

TEST(PickCandidates, FourDistinctYawsBestFirst)
{
  const auto c = pickCandidates(objectAt(0.5, 0.0, 0.42, 0.2), 0.04, GraspParams{}, 0.0);
  ASSERT_EQ(c.size(), 4u);
  EXPECT_NEAR(yawOf(Eigen::Quaterniond(c[0].grasp.rotation())), 0.2, 1e-9);
  for (size_t i = 0; i < c.size(); ++i) {
    for (size_t j = i + 1; j < c.size(); ++j) {
      const double d = std::abs(wrapAngle(
          yawOf(Eigen::Quaterniond(c[i].grasp.rotation())) -
          yawOf(Eigen::Quaterniond(c[j].grasp.rotation()))));
      EXPECT_GT(d, 1.0);   // at least ~90 deg apart
    }
    EXPECT_TRUE(c[i].grasp.translation().isApprox(c[0].grasp.translation(), 1e-12));
  }
}
