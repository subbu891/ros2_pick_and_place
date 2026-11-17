#include "pnp_perception/color_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace pnp_perception
{

DetectorParams defaultParams()
{
  DetectorParams p;
  // S and V lower bounds keep the grey table, grey bins and shadows out.
  p.classes = {
    {"red", {{cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255)},        // H near 0
        {cv::Scalar(170, 120, 70), cv::Scalar(179, 255, 255)}}},         // H near 179
    {"green", {{cv::Scalar(40, 100, 60), cv::Scalar(85, 255, 255)}}},
    {"blue", {{cv::Scalar(100, 120, 60), cv::Scalar(130, 255, 255)}}},
  };
  return p;
}

double normalizeQuarterTurn(double angle_rad)
{
  constexpr double kQuarter = M_PI / 2.0;
  double a = std::fmod(angle_rad, kQuarter);   // now in (-pi/2, pi/2)
  if (a > kQuarter / 2.0) {
    a -= kQuarter;
  } else if (a <= -kQuarter / 2.0) {
    a += kQuarter;
  }
  return a;
}

ColorDetector::ColorDetector(DetectorParams params)
: params_(std::move(params))
{
  if (params_.classes.empty()) {
    throw std::invalid_argument("ColorDetector: at least one colour class is required");
  }
  const int k = std::max(1, params_.morph_kernel_px);
  kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
}

cv::Mat ColorDetector::maskFor(const cv::Mat & hsv, const ColorClass & cls) const
{
  cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
  cv::Mat part;
  for (const auto & r : cls.ranges) {
    cv::inRange(hsv, r.lower, r.upper, part);   // 255 where the pixel is in range
    mask |= part;                               // combine ranges (for red)
  }
  // Opening removes small specks; closing fills small holes.
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_);
  return mask;
}

std::vector<Detection2D> ColorDetector::detect(const cv::Mat & bgr) const
{
  std::vector<Detection2D> out;
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    return out;
  }

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  for (const auto & cls : params_.classes) {
    const cv::Mat mask = maskFor(hsv, cls);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto & c : contours) {
      // Filter 1: size
      const double area = cv::contourArea(c);
      if (area < params_.min_area_px || area > params_.max_area_px) {
        continue;
      }
      // Filter 2: solidity (a cube is a convex, filled shape)
      std::vector<cv::Point> hull;
      cv::convexHull(c, hull);
      const double hull_area = cv::contourArea(hull);
      const double solidity = hull_area > 0.0 ? area / hull_area : 0.0;
      if (solidity < params_.min_solidity) {
        continue;
      }
      // Filter 3: aspect ratio (a cube seen from above is square)
      const cv::RotatedRect rect = cv::minAreaRect(c);
      const double w = std::max(rect.size.width, rect.size.height);
      const double h = std::min(rect.size.width, rect.size.height);
      if (h <= 0.0 || w / h > params_.max_aspect_ratio) {
        continue;
      }

      const cv::Moments m = cv::moments(c);
      Detection2D d;
      d.color = cls.name;
      d.centroid = cv::Point2f(
        static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
      d.min_rect = rect;
      d.area_px = area;
      d.solidity = solidity;
      d.yaw_image = normalizeQuarterTurn(rect.angle * M_PI / 180.0);
      out.push_back(d);
    }
  }

  std::sort(out.begin(), out.end(), [](const Detection2D & a, const Detection2D & b) {
      return a.area_px > b.area_px;
    });
  return out;
}

std::optional<double> sampleDepth(const cv::Mat & depth, cv::Point2f uv, int radius)
{
  if (depth.empty()) {
    return std::nullopt;
  }
  const int u0 = static_cast<int>(std::lround(uv.x));
  const int v0 = static_cast<int>(std::lround(uv.y));

  std::vector<double> samples;
  for (int v = v0 - radius; v <= v0 + radius; ++v) {
    if (v < 0 || v >= depth.rows) {continue;}
    for (int u = u0 - radius; u <= u0 + radius; ++u) {
      if (u < 0 || u >= depth.cols) {continue;}
      double z = 0.0;
      if (depth.type() == CV_32FC1) {
        z = static_cast<double>(depth.at<float>(v, u));
      } else if (depth.type() == CV_16UC1) {
        z = static_cast<double>(depth.at<uint16_t>(v, u)) / 1000.0;
      } else {
        return std::nullopt;
      }
      if (std::isfinite(z) && z > 0.0) {   // skip NaN / missing pixels
        samples.push_back(z);
      }
    }
  }
  if (samples.empty()) {
    return std::nullopt;
  }
  // The median ignores outliers, e.g. a pixel on the cube's edge seeing the table.
  auto mid = samples.begin() + static_cast<std::ptrdiff_t>(samples.size() / 2);
  std::nth_element(samples.begin(), mid, samples.end());
  return *mid;
}

cv::Point3d deproject(const Intrinsics & k, cv::Point2f uv, double z)
{
  if (k.fx == 0.0 || k.fy == 0.0) {
    throw std::invalid_argument("deproject: focal length must be non-zero");
  }
  // Pinhole model: u = fx * x / z + cx  ->  x = (u - cx) * z / fx
  return {(uv.x - k.cx) * z / k.fx, (uv.y - k.cy) * z / k.fy, z};
}

}  // namespace pnp_perception
