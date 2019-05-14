#pragma once

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include <encoder_msgs/Int64Stamped.h>

namespace odometry_wheels {

constexpr int64_t kMinTicks = 0;
constexpr int64_t kMaxTicks = 1677216;
constexpr int64_t kMaxEncoderDelta = 100000;
constexpr int64_t kInvalidEncoder = std::numeric_limits<int64_t>::max();
constexpr double kInvalidWheelDistance = std::numeric_limits<double>::max();

int64_t CalculateEncoderDelta(int64_t last_encoder_position,
                              int64_t current_encoder_position) {
  if (last_encoder_position < kMinTicks ||
      last_encoder_position > kMaxTicks) {
    return kInvalidEncoder;
  }
  if (current_encoder_position < kMinTicks ||
      current_encoder_position > kMaxTicks) {
    return kInvalidEncoder;
  }

  // Overrun.
  if (last_encoder_position > kMaxTicks - kMaxEncoderDelta &&
      current_encoder_position < kMaxEncoderDelta) {
    return (kMaxTicks - last_encoder_position) +
        current_encoder_position;
  }

  // Underrun.
  if (last_encoder_position < kMaxEncoderDelta &&
      current_encoder_position > kMaxTicks - kMaxEncoderDelta) {
    return (kMaxTicks - current_encoder_position) +
        last_encoder_position;
  }
  return current_encoder_position - last_encoder_position;
}

encoder_msgs::Int64Stamped InterpolateTicks(
    const ros::Time& interpolation_time,
    const encoder_msgs::Int64Stamped last_ticks,
    const encoder_msgs::Int64Stamped current_ticks) {
  if (interpolation_time < last_ticks.header.stamp ||
      interpolation_time > current_ticks.header.stamp) {
    return encoder_msgs::Int64Stamped();
  }
  if (interpolation_time == last_ticks.header.stamp) {
    return last_ticks;
  }
  if (interpolation_time == current_ticks.header.stamp) {
    return current_ticks;
  }
  double per_value = (interpolation_time - last_ticks.header.stamp).toSec() /
      (current_ticks.header.stamp - last_ticks.header.stamp).toSec();
  int64_t encoder_delta = CalculateEncoderDelta(
      last_ticks.ticks.data, current_ticks.ticks.data);
  if (encoder_delta == kInvalidEncoder) {
    return encoder_msgs::Int64Stamped();
  }
  int64_t interpolated_encoder_position =
      last_ticks.ticks.data + (per_value * encoder_delta);
  if (interpolated_encoder_position > kMaxTicks) {
    interpolated_encoder_position -= kMaxTicks;
  }
  encoder_msgs::Int64Stamped ret;
  ret.header.stamp = interpolation_time;
  ret.ticks.data = interpolated_encoder_position;
  return ret;
}

// Return value in meters.
double ConvertWheelTicksToWheelDistance(double ticks_per_meter,
                                     int64_t last_encoder_position,
                                     int64_t current_encoder_position) {
  int64_t encoder_distance = CalculateEncoderDelta(
      last_encoder_position, current_encoder_position);
  if (encoder_distance == kInvalidEncoder) {
    return kInvalidWheelDistance;
  }

  return encoder_distance / ticks_per_meter;
}


// Return value is in m/s.
double ConvertWheelSpeedsToLinearSpeed(double left_wheel_speed_m_s,
                                       double right_wheel_speed_m_s) {
  return (left_wheel_speed_m_s + right_wheel_speed_m_s) / 2.0;
}

double ConvertWheelDistancesToHeadingIncrement(
    double incremental_left_wheel_distance,
    double incremental_right_wheel_distance,
    double base_width) {
  return (incremental_right_wheel_distance - incremental_left_wheel_distance)
      / base_width;
}
}  // namespace odometry_wheels
