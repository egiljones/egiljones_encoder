#include "encoder_odometry/odometry_wheels.h"

#include <tf/transform_datatypes.h>

#include "encoder_odometry/odometry_wheels_helpers.h"

namespace odometry_wheels {

OdometryWheels::OdometryWheels(
    double left_ticks_per_meter, double right_ticks_per_meter,
    double wheel_base_width)
    : left_ticks_per_meter_(left_ticks_per_meter),
      right_ticks_per_meter_(right_ticks_per_meter),
      wheel_base_width_m_(wheel_base_width) {
  current_odometry_.header.frame_id = "odom";
  // Assuming we start with zero heading. 
  current_odometry_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
}

void OdometryWheels::UpdateFromLeftWheelTicks(
    const encoder_msgs::Int64Stamped& left) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_left_wheel_initialized_) {
    last_left_wheel_initialized_ = true;
    last_left_ticks_ = left;
    return;
  }
  if (!current_left_wheel_initialized_) {
    current_left_wheel_initialized_ = true;
  }
  current_left_ticks_ = left;
  ProduceUpdatedOdometry();
}

void OdometryWheels::UpdateFromRightWheelTicks(
    const encoder_msgs::Int64Stamped& right) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_right_wheel_initialized_) {
    last_right_wheel_initialized_ = true;
    last_right_ticks_ = right;
    return;
  }
  if (!current_right_wheel_initialized_) {
    current_right_wheel_initialized_ = true;
  }
  current_right_ticks_ = right;
  ProduceUpdatedOdometry();
}

bool OdometryWheels::GetCurrentOdometry(nav_msgs::Odometry* odom) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!odometry_valid_) return false;
  *odom = current_odometry_;
  return true;
}

void OdometryWheels::ProduceUpdatedOdometry() {
  if (!current_left_wheel_initialized_ || !current_right_wheel_initialized_) {
    return;
  }

  // This is the first time we've run.
  if (current_odometry_.header.stamp < last_left_ticks_.header.stamp ||
      current_odometry_.header.stamp < last_right_ticks_.header.stamp) {
    // Left side arrived first, replace with interpolation.
    if (last_left_ticks_.header.stamp < last_right_ticks_.header.stamp) {
      last_left_ticks_ =
          InterpolateTicks(last_right_ticks_.header.stamp,
                           last_left_ticks_, current_left_ticks_);
    } else if (last_right_ticks_.header.stamp < last_left_ticks_.header.stamp) {
      last_right_ticks_ =
          InterpolateTicks(last_left_ticks_.header.stamp,
                           last_right_ticks_, current_right_ticks_);
    }
    // Otherwise, times are equal and we don't have to replace anything.
  }

  // We should always be initialized here, so we can start.
  if (last_left_ticks_.header.stamp != last_right_ticks_.header.stamp) {
    ROS_ERROR_STREAM("Mismatch on last tick timestamps");
    assert(last_left_ticks_.header.stamp == last_right_ticks_.header.stamp);
  }
  
  encoder_msgs::Int64Stamped left_calc, right_calc;

  // Current data is synchronized.
  if (current_right_ticks_.header.stamp == current_left_ticks_.header.stamp) {
    left_calc = current_left_ticks_;
    right_calc = current_right_ticks_;
  } else if (current_right_ticks_.header.stamp
             < current_left_ticks_.header.stamp) {
    right_calc = current_right_ticks_;
    left_calc = InterpolateTicks(current_right_ticks_.header.stamp,
                                 last_left_ticks_, current_left_ticks_);
  } else {
    left_calc = current_left_ticks_;
    right_calc = InterpolateTicks(
        current_left_ticks_.header.stamp,
        last_right_ticks_, current_right_ticks_);
  }
  // At this point, both last times and the times for both calc times should be
  // equal.

  // If we haven't advanced the clock on what we can calculate, we don't have
  // anything we need to do.
  if (left_calc.header.stamp == current_odometry_.header.stamp) {
    return;
  }

  double left_distance_m = ConvertWheelTicksToWheelDistance(
      left_ticks_per_meter_, last_left_ticks_.ticks.data,
      left_calc.ticks.data);
  double right_distance_m = ConvertWheelTicksToWheelDistance(
      right_ticks_per_meter_, last_right_ticks_.ticks.data,
      right_calc.ticks.data);
  ros::Duration dur = left_calc.header.stamp -
      last_left_ticks_.header.stamp;
  double left_speed_m_s = left_distance_m / dur.toSec();
  double right_speed_m_s = right_distance_m / dur.toSec();

  double linear_speed_m_s = ConvertWheelSpeedsToLinearSpeed(left_speed_m_s,
                                                            right_speed_m_s);
  double linear_distance_m = linear_speed_m_s * dur.toSec();
  double heading_increment = ConvertWheelDistancesToHeadingIncrement(
      left_distance_m, right_distance_m, wheel_base_width_m_);

  double updated_heading = tf::getYaw(current_odometry_.pose.pose.orientation)
      + heading_increment;

  odometry_valid_ = true;
  current_odometry_.pose.pose.position.x =
      current_odometry_.pose.pose.position.x +
      linear_distance_m * cos(updated_heading);
  current_odometry_.pose.pose.position.y =
      current_odometry_.pose.pose.position.y +
      linear_distance_m * sin(updated_heading);
  current_odometry_.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(updated_heading);
  current_odometry_.twist.twist.linear.x =
      linear_speed_m_s * cos(updated_heading);
  current_odometry_.twist.twist.linear.y =
      linear_speed_m_s * sin(updated_heading);
  current_odometry_.twist.twist.angular.z = heading_increment / dur.toSec();
  current_odometry_.header.stamp = left_calc.header.stamp;

  last_left_ticks_ = left_calc;
  last_right_ticks_ = right_calc;
}

}  // odometry_wheels
