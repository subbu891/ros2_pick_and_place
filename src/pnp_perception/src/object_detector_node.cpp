#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/imgproc.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pnp_interfaces/msg/detected_object_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "pnp_perception/color_detector.hpp"

namespace pnp_perception
{

using sensor_msgs::msg::Image;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

class ObjectDetectorNode : public rclcpp::Node
{
public:
  ObjectDetectorNode()
  : Node("object_detector"),
    detector_(defaultParams()),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ---- Parameters (defaults here, real values from config/object_detector.yaml)
    target_frame_ = declare_parameter<std::string>("target_frame", "world");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "");  // "" = use image header
    object_height_ = declare_parameter<double>("object_height", 0.04);
    const auto rgb_topic = declare_parameter<std::string>("rgb_topic", "/camera/image");
    const auto depth_topic = declare_parameter<std::string>("depth_topic", "/camera/depth_image");
    const auto info_topic = declare_parameter<std::string>("info_topic", "/camera/camera_info");

    DetectorParams p = defaultParams();
    p.min_area_px = declare_parameter<double>("min_area_px", p.min_area_px);
    p.max_area_px = declare_parameter<double>("max_area_px", p.max_area_px);
    p.min_solidity = declare_parameter<double>("min_solidity", p.min_solidity);
    p.max_aspect_ratio = declare_parameter<double>("max_aspect_ratio", p.max_aspect_ratio);
    detector_ = ColorDetector(p);

    // ---- Publishers ("~/" puts them under the node name: /object_detector/...)
    objects_pub_ = create_publisher<pnp_interfaces::msg::DetectedObjectArray>("~/objects", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 10);
    debug_pub_ = create_publisher<Image>("~/debug_image", 10);

    // ---- Camera intrinsics: stored whenever a CameraInfo arrives
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        intrinsics_ = Intrinsics{msg->k[0], msg->k[4], msg->k[2], msg->k[5]};
      });

    // ---- Colour + depth, paired by timestamp
    rgb_sub_.subscribe(this, rgb_topic, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(
      std::bind(&ObjectDetectorNode::onImages, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Detector ready: %s + %s -> objects in '%s'",
      rgb_topic.c_str(), depth_topic.c_str(), target_frame_.c_str());
  }

private:
  void onImages(const Image::ConstSharedPtr & rgb_msg, const Image::ConstSharedPtr & depth_msg)
  {
    // 1) Need intrinsics first
    std::optional<Intrinsics> k;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      k = intrinsics_;
    }
    if (!k) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for camera_info...");
      return;
    }

    // 2) ROS images -> OpenCV
    cv_bridge::CvImageConstPtr rgb, depth;
    try {
      rgb = cv_bridge::toCvShare(rgb_msg, "bgr8");
      depth = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    // 3) Where is the camera relative to the world, at the time of this image?
    const std::string cam_frame =
      camera_frame_.empty() ? rgb_msg->header.frame_id : camera_frame_;
    geometry_msgs::msg::TransformStamped cam_to_target;
    try {
      cam_to_target = tf_buffer_.lookupTransform(
        target_frame_, cam_frame, rgb_msg->header.stamp, rclcpp::Duration::from_seconds(0.2));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF: %s", e.what());
      return;
    }

    // 4) Detect, then lift each blob into 3D
    const auto detections = detector_.detect(rgb->image);
    pnp_interfaces::msg::DetectedObjectArray out;
    out.header.stamp = rgb_msg->header.stamp;
    out.header.frame_id = target_frame_;
    std::map<std::string, int> count_per_color;

    for (const auto & d : detections) {
      const auto z = sampleDepth(depth->image, d.centroid);
      if (!z) {
        continue;   // no valid depth at that pixel
      }
      // The camera sees the cube's top face; add half its height to reach the centre
      const cv::Point3d p_cam = deproject(*k, d.centroid, *z + object_height_ / 2.0);

      geometry_msgs::msg::PoseStamped pose_cam;
      pose_cam.header.stamp = rgb_msg->header.stamp;
      pose_cam.header.frame_id = cam_frame;
      pose_cam.pose.position.x = p_cam.x;
      pose_cam.pose.position.y = p_cam.y;
      pose_cam.pose.position.z = p_cam.z;
      tf2::Quaternion q_cam;
      q_cam.setRPY(0.0, 0.0, d.yaw_image);   // rotation about the camera's viewing axis
      pose_cam.pose.orientation = tf2::toMsg(q_cam);

      // camera frame -> world frame
      geometry_msgs::msg::PoseStamped pose_world;
      tf2::doTransform(pose_cam, pose_world, cam_to_target);

      // Cubes lie flat on the table: keep only the yaw about world Z
      tf2::Quaternion q_w;
      tf2::fromMsg(pose_world.pose.orientation, q_w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q_w).getRPY(roll, pitch, yaw);
      tf2::Quaternion q_flat;
      q_flat.setRPY(0.0, 0.0, normalizeQuarterTurn(yaw));
      pose_world.pose.orientation = tf2::toMsg(q_flat);

      pnp_interfaces::msg::DetectedObject obj;
      obj.color = d.color;
      obj.id = d.color + "_" + std::to_string(count_per_color[d.color]++);
      obj.pose = pose_world;
      obj.pixel_area = static_cast<float>(d.area_px);
      obj.confidence = static_cast<float>(d.solidity);
      out.objects.push_back(obj);
    }

    objects_pub_->publish(out);
    publishMarkers(out);
    publishDebug(rgb, detections, rgb_msg->header);
  }

  void publishMarkers(const pnp_interfaces::msg::DetectedObjectArray & arr)
  {
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;   // remove old markers
    ma.markers.push_back(clear);
    int id = 0;
    for (const auto & o : arr.objects) {
      visualization_msgs::msg::Marker m;
      m.header = arr.header;
      m.ns = "detections";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.pose = o.pose.pose;
      m.scale.x = m.scale.y = m.scale.z = object_height_;
      m.color.a = 0.7f;
      m.color.r = o.color == "red" ? 1.0f : 0.0f;
      m.color.g = o.color == "green" ? 1.0f : 0.0f;
      m.color.b = o.color == "blue" ? 1.0f : 0.0f;
      m.lifetime = rclcpp::Duration::from_seconds(3.0);
      ma.markers.push_back(m);
    }
    markers_pub_->publish(ma);
  }

  void publishDebug(
    const cv_bridge::CvImageConstPtr & rgb, const std::vector<Detection2D> & dets,
    const std_msgs::msg::Header & header)
  {
    if (debug_pub_->get_subscription_count() == 0) {
      return;   // nobody is watching: skip the work
    }
    cv::Mat img = rgb->image.clone();
    for (const auto & d : dets) {
      cv::Point2f pts[4];
      d.min_rect.points(pts);
      for (int i = 0; i < 4; ++i) {
        cv::line(img, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 255), 2);
      }
      cv::putText(img, d.color, d.centroid + cv::Point2f(8.f, -8.f),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
    debug_pub_->publish(*cv_bridge::CvImage(header, "bgr8", img).toImageMsg());
  }

  ColorDetector detector_;
  std::string target_frame_;
  std::string camera_frame_;
  double object_height_{0.04};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex mutex_;
  std::optional<Intrinsics> intrinsics_;

  message_filters::Subscriber<Image> rgb_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  rclcpp::Publisher<pnp_interfaces::msg::DetectedObjectArray>::SharedPtr objects_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_pub_;
};

}  // namespace pnp_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pnp_perception::ObjectDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
