#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <limits>

#include "pnp_perception/color_detector.hpp"

using namespace pnp_perception;  // NOLINT

namespace
{
// A grey "table" image
cv::Mat greyTable()
{
  return cv::Mat(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
}

// Draw a filled square of a given size, rotation and colour
void drawSquare(cv::Mat & img, cv::Point2f c, float side, float deg, cv::Scalar bgr)
{
  cv::RotatedRect r(c, cv::Size2f(side, side), deg);
  cv::Point2f p[4];
  r.points(p);
  std::vector<cv::Point> poly(p, p + 4);
  cv::fillConvexPoly(img, poly, bgr);
}
}  // namespace

TEST(ColorDetector, FindsOneBlobPerColour)
{
  cv::Mat img = greyTable();
  drawSquare(img, {100, 100}, 40, 0, cv::Scalar(0, 0, 255));   // red (BGR!)
  drawSquare(img, {320, 240}, 40, 0, cv::Scalar(0, 255, 0));   // green
  drawSquare(img, {500, 380}, 40, 0, cv::Scalar(255, 0, 0));   // blue

  const auto out = ColorDetector(defaultParams()).detect(img);
  ASSERT_EQ(out.size(), 3u);
  int r = 0, g = 0, b = 0;
  for (const auto & d : out) {
    r += d.color == "red";
    g += d.color == "green";
    b += d.color == "blue";
  }
  EXPECT_EQ(r, 1);
  EXPECT_EQ(g, 1);
  EXPECT_EQ(b, 1);
}

TEST(ColorDetector, CentroidIsAccurate)
{
  cv::Mat img = greyTable();
  drawSquare(img, {200.f, 150.f}, 40, 0, cv::Scalar(0, 255, 0));
  const auto out = ColorDetector(defaultParams()).detect(img);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].centroid.x, 200.f, 1.0f);
  EXPECT_NEAR(out[0].centroid.y, 150.f, 1.0f);
}

TEST(ColorDetector, RedHueWrapAroundIsDetected)
{
  // A red with H = 175 (the "upper" red range) must still be classified as red
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(175, 220, 220));
  cv::Mat bgr_px;
  cv::cvtColor(hsv, bgr_px, cv::COLOR_HSV2BGR);
  const cv::Vec3b c = bgr_px.at<cv::Vec3b>(0, 0);

  cv::Mat img = greyTable();
  drawSquare(img, {300, 300}, 40, 0, cv::Scalar(c[0], c[1], c[2]));
  const auto out = ColorDetector(defaultParams()).detect(img);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].color, "red");
}

TEST(ColorDetector, RecoversRotation)
{
  cv::Mat img = greyTable();
  drawSquare(img, {320, 240}, 50, 30, cv::Scalar(255, 0, 0));
  const auto out = ColorDetector(defaultParams()).detect(img);
  ASSERT_EQ(out.size(), 1u);
  // 30 deg and -60 deg look identical for a square; we expect +-30 deg
  EXPECT_NEAR(std::abs(out[0].yaw_image), 30.0 * M_PI / 180.0, 3.0 * M_PI / 180.0);
}

TEST(ColorDetector, RejectsNoiseAndEmptyInput)
{
  cv::Mat img = greyTable();
  cv::circle(img, {50, 50}, 2, cv::Scalar(0, 0, 255), -1);   // tiny red speck
  ColorDetector det(defaultParams());
  EXPECT_TRUE(det.detect(img).empty());
  EXPECT_TRUE(det.detect(cv::Mat()).empty());
}

TEST(ColorDetector, RejectsThinStripsAndOversizedBlobs)
{
  cv::Mat img = greyTable();
  cv::rectangle(img, {50, 50}, {130, 56}, cv::Scalar(0, 255, 0), -1);    // thin strip
  cv::rectangle(img, {300, 200}, {380, 280}, cv::Scalar(0, 0, 255), -1);  // too big
  drawSquare(img, {500, 400}, 22, 10, cv::Scalar(255, 0, 0));             // a real cube
  const auto out = ColorDetector(defaultParams()).detect(img);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].color, "blue");
}

TEST(ColorDetector, ThrowsWithoutClasses)
{
  DetectorParams p;
  EXPECT_THROW(ColorDetector{p}, std::invalid_argument);
}

TEST(Geometry, NormalizeQuarterTurn)
{
  EXPECT_NEAR(normalizeQuarterTurn(0.0), 0.0, 1e-9);
  EXPECT_NEAR(normalizeQuarterTurn(M_PI / 2.0), 0.0, 1e-9);
  EXPECT_NEAR(normalizeQuarterTurn(M_PI / 3.0), -M_PI / 6.0, 1e-9);
  EXPECT_NEAR(normalizeQuarterTurn(-M_PI / 3.0), M_PI / 6.0, 1e-9);
}

TEST(Geometry, SampleDepthIgnoresInvalidPixels)
{
  cv::Mat d(10, 10, CV_32FC1, cv::Scalar(0.8f));
  d.at<float>(5, 5) = std::numeric_limits<float>::quiet_NaN();
  d.at<float>(5, 6) = 0.0f;
  const auto z = sampleDepth(d, {5.f, 5.f}, 2);
  ASSERT_TRUE(z.has_value());
  EXPECT_NEAR(*z, 0.8, 1e-6);

  cv::Mat all_nan(10, 10, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  EXPECT_FALSE(sampleDepth(all_nan, {5.f, 5.f}).has_value());
}

TEST(Geometry, SampleDepthMillimetres)
{
  cv::Mat d(10, 10, CV_16UC1, cv::Scalar(750));
  const auto z = sampleDepth(d, {3.f, 3.f});
  ASSERT_TRUE(z.has_value());
  EXPECT_NEAR(*z, 0.75, 1e-9);
}

TEST(Geometry, Deproject)
{
  Intrinsics k{500.0, 500.0, 320.0, 240.0};
  const auto p = deproject(k, {320.f, 240.f}, 1.0);   // the image centre -> on the axis
  EXPECT_NEAR(p.x, 0.0, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
  EXPECT_NEAR(p.z, 1.0, 1e-9);

  const auto q = deproject(k, {420.f, 240.f}, 2.0);   // 100 px right at 2 m -> 0.4 m
  EXPECT_NEAR(q.x, 0.4, 1e-9);
  EXPECT_THROW(deproject(Intrinsics{}, {0.f, 0.f}, 1.0), std::invalid_argument);
}
