#include "so101_station_vulkan/frame_adapter.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cstring>
#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>

namespace so101_station_vulkan {
using namespace std::chrono_literals;
// This component is an explicit experimental acquisition/compute boundary.
// Vendor CPU alignment is exposed in status; strict full-GPU camera capture is
// rejected until the separate alignment/distortion parity gate passes.
class CameraComponent : public rclcpp::Node {
  rs2::context cameras_;
  rs2::pipeline camera_;
  rs2::align alignment_{RS2_STREAM_COLOR};
  std::unique_ptr<Pipeline> compute_;
  rclcpp::Publisher<FrameLease>::SharedPtr frames_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
  rclcpp::TimerBase::SharedPtr timer_;
  FrameLease active_, pending_;
  std::string serial_;
  uint64_t drops_ = 0, replacements_ = 0, completed_ = 0;
  float scale_ = 0;
  bool running_ = false;
  void publish_status(const std::string &message) {
    std_msgs::msg::String s;
    s.data = message;
    status_->publish(s);
  }
  void dispatch_pending() {
    if (!active_ && pending_) {
      active_ = std::move(pending_);
      compute_->submit(active_);
    }
  }
  void tick() {
    if (active_ && compute_->ready(active_)) {
      compute_->finish(active_);
      frames_->publish(active_);
      ++completed_;
      active_ = {};
      dispatch_pending();
    }
    rs2::frameset native;
    if (!camera_.poll_for_frames(&native))
      return;
    // Replace only unsubmitted pending storage; submitted and published leases
    // retain their own slots until both fence completion and last consumer.
    if (pending_) {
      pending_ = {};
      ++replacements_;
    }
    auto lease = compute_->acquire();
    if (!lease) {
      ++drops_;
      return;
    }
    auto aligned = alignment_.process(native).as<rs2::frameset>();
    auto color = aligned.get_color_frame();
    auto depth = aligned.get_depth_frame();
    auto intrinsics =
        color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    auto &slot = lease.slot();
    auto *bytes = static_cast<uint8_t *>(slot.upload.mapped);
    // Copy native USB plane rows exactly once; padding is never metric data.
    for (unsigned y = 0; y < 480; y++) {
      std::memcpy(bytes + y * 640 * 3,
                  static_cast<const uint8_t *>(color.get_data()) +
                      y * color.get_stride_in_bytes(),
                  640 * 3);
      std::memcpy(bytes + 640 * 480 * 3 + y * 640 * 2,
                  static_cast<const uint8_t *>(depth.get_data()) +
                      y * depth.get_stride_in_bytes(),
                  640 * 2);
    }
    slot.calibration = {intrinsics.fx,  intrinsics.fy, intrinsics.ppx,
                        intrinsics.ppy, scale_,        1};
    slot.metadata.frame_number = color.get_frame_number();
    slot.metadata.timestamp_ns = uint64_t(now().nanoseconds());
    slot.metadata.hardware_timestamp_ms = color.get_timestamp();
    slot.metadata.depth_frame_number = depth.get_frame_number();
    slot.metadata.depth_hardware_timestamp_ms = depth.get_timestamp();
    slot.metadata.depth_timestamp_domain = int(depth.get_frame_timestamp_domain());
    slot.metadata.timestamp_domain = int(color.get_frame_timestamp_domain());
    slot.metadata.calibration_generation = 1;
    slot.metadata.serial = serial_;
    pending_ = std::move(lease);
    dispatch_pending();
    if (completed_ % 30 == 0)
      publish_status("experimental; backend=" + compute_->backend() +
                     "; alignment=vendor_cpu; distortion_geometry=unqualified; "
                     "recording=disabled; dropped=" +
                     std::to_string(drops_) +
                     "; replaced=" + std::to_string(replacements_));
  }

public:
  explicit CameraComponent(const rclcpp::NodeOptions &options)
      : Node("resident_camera", options), camera_(cameras_) {
    auto mode = declare_parameter<std::string>("backend", "auto");
    serial_ = declare_parameter<std::string>("serial", "");
    status_ = create_publisher<std_msgs::msg::String>(
        "/station/resident/status", rclcpp::QoS(1).transient_local());
    auto devices = cameras_.query_devices();
    if (devices.size() == 0)
      throw std::runtime_error(
          "resident camera: no RealSense device; no camera was started");
    rs2::device selected;
    for (auto device : devices)
      if (serial_.empty() ||
          serial_ == device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
        selected = device;
        break;
      }
    if (!selected)
      throw std::runtime_error(
          "resident camera: requested RealSense serial not found");
    serial_ = selected.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    bool rgb = false, z16 = false;
    for (auto sensor : selected.query_sensors())
      for (auto profile : sensor.get_stream_profiles()) {
        auto video = profile.as<rs2::video_stream_profile>();
        if (!video || video.width() != 640 || video.height() != 480 ||
            video.fps() != 30)
          continue;
        rgb |= video.stream_type() == RS2_STREAM_COLOR &&
               video.format() == RS2_FORMAT_RGB8;
        z16 |= video.stream_type() == RS2_STREAM_DEPTH &&
               video.format() == RS2_FORMAT_Z16;
      }
    if (!rgb || !z16)
      throw std::runtime_error("resident camera: enumerated 640x480x30 "
                               "RGB8+Z16 profiles unavailable");
    if (mode == "vulkan")
      throw std::runtime_error(
          "strict Vulkan camera refused: raw-depth alignment/distortion "
          "numerical parity gate is unqualified; aligned offline fixtures can "
          "use strict Vulkan compute");
    auto shaders =
        ament_index_cpp::get_package_share_directory("so101_station_vulkan") +
        "/shaders";
    compute_ = std::make_unique<Pipeline>(640, 480, mode, shaders);
    rs2::config config;
    config.enable_device(serial_);
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    auto profile = camera_.start(config);
    running_ = true;
    scale_ = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
    frames_ =
        create_publisher<FrameLease>("/station/resident/frame", rclcpp::QoS(1));
    info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "/station/resident/color/camera_info",
        rclcpp::QoS(1).transient_local());
    auto k = profile.get_stream(RS2_STREAM_COLOR)
                 .as<rs2::video_stream_profile>()
                 .get_intrinsics();
    sensor_msgs::msg::CameraInfo ci;
    ci.width = 640;
    ci.height = 480;
    ci.header.frame_id = "camera_color_optical_frame";
    ci.distortion_model = "plumb_bob";
    ci.k = {k.fx, 0, k.ppx, 0, k.fy, k.ppy, 0, 0, 1};
    ci.d.assign(k.coeffs, k.coeffs + 5);
    info_->publish(ci);
    publish_status("experimental; backend=" + compute_->backend() +
                   "; fallback_reason=" + compute_->fallback_reason() +
                   "; alignment=vendor_cpu; full_station_qualified=false");
    timer_ = create_wall_timer(2ms, [this] {
      try {
        tick();
      } catch (const std::exception &e) {
        publish_status(std::string("fatal: ") + e.what());
        RCLCPP_ERROR(get_logger(), "%s", e.what());
        timer_->cancel();
        if (running_) {
          camera_.stop();
          running_ = false;
        }
      }
    });
  }
  ~CameraComponent() override {
    if (running_)
      camera_.stop();
  }
};

class ObjectsComponent : public rclcpp::Node {
  rclcpp::Subscription<FrameLease>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr measurements_;

public:
  explicit ObjectsComponent(const rclcpp::NodeOptions &options)
      : Node("resident_measurements", options) {
    measurements_ = create_publisher<std_msgs::msg::String>(
        "/station/resident/measurements", rclcpp::QoS(1));
    subscription_ = create_subscription<FrameLease>(
        "/station/resident/frame", rclcpp::QoS(1),
        [this](const FrameLease &frame) {
          const auto &slot = frame.slot();
          std_msgs::msg::String message;
          message.data =
              "source_frame=" + std::to_string(slot.metadata.frame_number) +
              "; components=" + std::to_string(slot.component_count) +
              "; geometry_admission=unqualified; execution_eligible=false";
          measurements_->publish(message);
        });
  }
};
} // namespace so101_station_vulkan
RCLCPP_COMPONENTS_REGISTER_NODE(so101_station_vulkan::CameraComponent)
RCLCPP_COMPONENTS_REGISTER_NODE(so101_station_vulkan::ObjectsComponent)
