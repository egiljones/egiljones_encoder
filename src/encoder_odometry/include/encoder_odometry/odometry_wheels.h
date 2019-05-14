#pragma once

#include <mutex>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include <encoder_msgs/Int64Stamped.h>

namespace odometry_wheels {

// Class is thread-safe - will lock in the update functions.
class OdometryWheels {
 public:
  OdometryWheels(double left_ticks_per_meter, double right_ticks_per_meter,
                 double wheel_base_width);

  void UpdateFromLeftWheelTicks(const encoder_msgs::Int64Stamped& left);
  void UpdateFromRightWheelTicks(const encoder_msgs::Int64Stamped& right);

  // Will block until update finishes.
  bool GetCurrentOdometry(nav_msgs::Odometry* odom) const;

 private:
  // Assumes that we've locked.
  void ProduceUpdatedOdometry();

  // Protects all variables.
  mutable std::mutex mutex_;

  const double left_ticks_per_meter_;
  const double right_ticks_per_meter_;
  const double wheel_base_width_m_;

  bool last_left_wheel_initialized_ = false;
  bool last_right_wheel_initialized_ = false;
  bool current_left_wheel_initialized_ = false;
  bool current_right_wheel_initialized_ = false;

  encoder_msgs::Int64Stamped last_left_ticks_;
  encoder_msgs::Int64Stamped last_right_ticks_;
  encoder_msgs::Int64Stamped current_left_ticks_;
  encoder_msgs::Int64Stamped current_right_ticks_;

  bool odometry_valid_ = false;
  nav_msgs::Odometry current_odometry_;
};

}  // odometry_wheels
