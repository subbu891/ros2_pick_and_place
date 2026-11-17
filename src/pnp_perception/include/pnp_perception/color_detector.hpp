#pragma once

#include <opencv2/core.hpp>

#include <optional>
#include <string>
#include <vector>

namespace pnp_perception
{

/// Inclusive HSV range (OpenCV: H in [0,179], S and V in [0,255]).
struct HsvRange
{
  cv::Scalar lower;
  cv::Scalar upper;
};

/// A colour class may need several ranges (red wraps around H = 0).
struct ColorClass
{
  std::string name;
  std::vector<HsvRange> ranges;
};

struct DetectorParams
{
  std::vector<ColorClass> classes;
  double min_area_px{150.0};      ///< smaller blobs are noise
  double max_area_px{3000.0};     ///< bigger blobs are not cubes
  int morph_kernel_px{5};         ///< size of the noise-removal filter
  double min_solidity{0.85};      ///< blob area / convex hull area (rejects ragged shapes)
  double max_aspect_ratio{1.6};   ///< long side / short side (cubes from above are square)
};

/// One blob found in the image.
struct Detection2D
{
  std::string color;
  cv::Point2f centroid;           ///< pixel centre (u, v)
  cv::RotatedRect min_rect;       ///< oriented bounding box
  double area_px{0.0};
  double solidity{0.0};
  double yaw_image{0.0};          ///< in-plane rotation [rad], in (-pi/4, pi/4]
};

/// Pinhole camera intrinsics (from sensor_msgs/CameraInfo's K matrix).
struct Intrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
};

/// Red, green and blue classes tuned for the Gazebo world.
DetectorParams defaultParams();

/// A cube looks the same every 90 deg: map any angle to (-pi/4, pi/4].
double normalizeQuarterTurn(double angle_rad);

class ColorDetector
{
public:
  explicit ColorDetector(DetectorParams params);

  /// Detect coloured blobs in a BGR8 image, largest first.
  std::vector<Detection2D> detect(const cv::Mat & bgr) const;

  /// Binary mask for one colour class (public for debugging and tests).
  cv::Mat maskFor(const cv::Mat & hsv, const ColorClass & cls) const;

private:
  DetectorParams params_;
  cv::Mat kernel_;
};

/// Robust depth at (u, v): median of valid pixels in a (2r+1)x(2r+1) window.
/// Accepts 32FC1 (metres) or 16UC1 (millimetres). Returns metres.
std::optional<double> sampleDepth(const cv::Mat & depth, cv::Point2f uv, int radius = 3);

/// Pixel (u, v) at depth z -> 3D point in the camera optical frame.
cv::Point3d deproject(const Intrinsics & k, cv::Point2f uv, double z);

}  // namespace pnp_perception
