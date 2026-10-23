#pragma once
#include "so101_station_vulkan/pipeline.hpp"
#include <cstring>
#include <rclcpp/type_adapter.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rclcpp {
// Only intra-process consumers can recover GPU ownership. Ordinary DDS
// subscribers receive a deliberate RGB8 compatibility copy on serialization.
template <>
struct TypeAdapter<so101_station_vulkan::FrameLease, sensor_msgs::msg::Image> {
  using is_specialized = std::true_type;
  using custom_type = so101_station_vulkan::FrameLease;
  using ros_message_type = sensor_msgs::msg::Image;
  static void convert_to_ros_message(const custom_type &source,
                                     ros_message_type &destination) {
    auto &slot = source.slot();
    destination.header.stamp.sec =
        int32_t(slot.metadata.timestamp_ns / 1000000000);
    destination.header.stamp.nanosec =
        uint32_t(slot.metadata.timestamp_ns % 1000000000);
    destination.header.frame_id = "camera_color_optical_frame";
    destination.height = slot.metadata.height;
    destination.width = slot.metadata.width;
    destination.encoding = "rgb8";
    destination.step = destination.width * 3;
    destination.is_bigendian = false;
    destination.data.resize(size_t(destination.step) * destination.height);
    std::memcpy(destination.data.data(), slot.upload.mapped,
                destination.data.size());
  }
  static void convert_to_custom(const ros_message_type &, custom_type &) {
    throw std::runtime_error(
        "resident frame lease requires composed intra-process transport; DDS "
        "images cannot import device ownership");
  }
};
} // namespace rclcpp
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(so101_station_vulkan::FrameLease,
                                           sensor_msgs::msg::Image);
