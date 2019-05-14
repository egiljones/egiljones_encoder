#include <gtest/gtest.h>

#include <tf/transform_datatypes.h>

#include "encoder_odometry/odometry_wheels_helpers.h"
#include "encoder_odometry/odometry_wheels.h"

TEST(OdometryWheelHelpers, CalculateEndcoderDelta) {
  EXPECT_EQ(150u, odometry_wheels::CalculateEncoderDelta(
      50, odometry_wheels::kMaxTicks - 100));
  EXPECT_EQ(150u, odometry_wheels::CalculateEncoderDelta(
      odometry_wheels::kMaxTicks - 100, 50));
}

TEST(OdometryWheelHelpers, TickInterpolation) {
  encoder_msgs::Int64Stamped last_tick;
  last_tick.header.stamp = ros::Time(1.0);
  last_tick.ticks.data = 1000;

  encoder_msgs::Int64Stamped current_tick;
  current_tick.header.stamp = ros::Time(2.0);
  current_tick.ticks.data = 2000;

  encoder_msgs::Int64Stamped ret_tick =
    odometry_wheels::InterpolateTicks(ros::Time(1.0), last_tick, current_tick);
  EXPECT_EQ(ros::Time(1.0), ret_tick.header.stamp);
  EXPECT_EQ(1000, ret_tick.ticks.data);

  ret_tick =
    odometry_wheels::InterpolateTicks(ros::Time(2.0), last_tick, current_tick);
  EXPECT_EQ(ros::Time(2.0), ret_tick.header.stamp);
  EXPECT_EQ(2000, ret_tick.ticks.data);

  ret_tick =
    odometry_wheels::InterpolateTicks(ros::Time(1.6), last_tick, current_tick);
  EXPECT_EQ(ros::Time(1.6), ret_tick.header.stamp);
  EXPECT_EQ(1600, ret_tick.ticks.data);
}

TEST(OdometryWheel, OdometryWheelSync) {

  odometry_wheels::OdometryWheels odom_wheel(1000, 1000, 1.0);
  nav_msgs::Odometry odom;

  encoder_msgs::Int64Stamped left_tick;
  left_tick.header.stamp = ros::Time(1.0);
  left_tick.ticks.data = 1000;
  odom_wheel.UpdateFromLeftWheelTicks(left_tick);
  EXPECT_FALSE(odom_wheel.GetCurrentOdometry(&odom));

  encoder_msgs::Int64Stamped right_tick;
  right_tick.header.stamp = ros::Time(1.0);
  right_tick.ticks.data = 1000;
  odom_wheel.UpdateFromRightWheelTicks(right_tick);
  EXPECT_FALSE(odom_wheel.GetCurrentOdometry(&odom));

  left_tick.header.stamp = ros::Time(2.0);
  left_tick.ticks.data = 2000;
  odom_wheel.UpdateFromLeftWheelTicks(left_tick);
  EXPECT_FALSE(odom_wheel.GetCurrentOdometry(&odom));

  right_tick.header.stamp = ros::Time(2.0);
  right_tick.ticks.data = 2000;
  odom_wheel.UpdateFromRightWheelTicks(right_tick);
  EXPECT_TRUE(odom_wheel.GetCurrentOdometry(&odom));

  EXPECT_EQ(ros::Time(2.0), odom.header.stamp);
  EXPECT_EQ(1.0, odom.pose.pose.position.x);
  EXPECT_EQ(0.0, odom.pose.pose.position.y);
  EXPECT_EQ(0.0, tf::getYaw(odom.pose.pose.orientation));
  EXPECT_EQ(1.0, odom.twist.twist.linear.x);
  EXPECT_EQ(0.0, odom.twist.twist.angular.z);

  right_tick.header.stamp = ros::Time(2.5);
  right_tick.ticks.data = 1000;
  odom_wheel.UpdateFromRightWheelTicks(right_tick);
  EXPECT_TRUE(odom_wheel.GetCurrentOdometry(&odom));
  // This won't actually cause an update.
  EXPECT_EQ(ros::Time(2.0), odom.header.stamp);

  left_tick.header.stamp = ros::Time(2.5);
  left_tick.ticks.data = 1000;
  odom_wheel.UpdateFromLeftWheelTicks(left_tick);
  // This should update.
  EXPECT_TRUE(odom_wheel.GetCurrentOdometry(&odom));

  EXPECT_EQ(ros::Time(2.5), odom.header.stamp);
  EXPECT_EQ(0.0, odom.pose.pose.position.x);
  EXPECT_EQ(0.0, odom.pose.pose.position.y);
  EXPECT_EQ(0.0, tf::getYaw(odom.pose.pose.orientation));
  EXPECT_EQ(-2.0, odom.twist.twist.linear.x);
  EXPECT_EQ(0.0, odom.twist.twist.angular.z);

  left_tick.header.stamp = ros::Time(3.5);
  left_tick.ticks.data = 1900;
  odom_wheel.UpdateFromLeftWheelTicks(left_tick);
  // This won't actually cause an update.
  EXPECT_EQ(ros::Time(2.5), odom.header.stamp);

  right_tick.header.stamp = ros::Time(3.5);
  right_tick.ticks.data = 2000;
  odom_wheel.UpdateFromRightWheelTicks(right_tick);
  EXPECT_TRUE(odom_wheel.GetCurrentOdometry(&odom));

  // This should update.
  EXPECT_TRUE(odom_wheel.GetCurrentOdometry(&odom));
  EXPECT_EQ(ros::Time(3.5), odom.header.stamp);

  // Straight, and slightly to left (positive y, positive yaw).
  EXPECT_LT(.9, odom.pose.pose.position.x);
  EXPECT_LT(0.0, odom.pose.pose.position.y);
  EXPECT_LT(0.0, tf::getYaw(odom.pose.pose.orientation));
  EXPECT_LT(.9, odom.twist.twist.linear.x);
  EXPECT_LT(0.0, odom.twist.twist.angular.z);

  left_tick.header.stamp = ros::Time(4.5);
  left_tick.ticks.data = 3300;
  odom_wheel.UpdateFromLeftWheelTicks(left_tick);
  // This won't actually cause an update.
  EXPECT_EQ(ros::Time(3.5), odom.header.stamp);

  right_tick.header.stamp = ros::Time(4.5);
  right_tick.ticks.data = 3000;
  odom_wheel.UpdateFromRightWheelTicks(right_tick);
  EXPECT_TRUE(odom_wheel.GetCurrentOdometry(&odom));

  // This should update.
  EXPECT_TRUE(odom_wheel.GetCurrentOdometry(&odom));
  EXPECT_EQ(ros::Time(4.5), odom.header.stamp);

  // Straight, and now more to right (negative y, negative yaw).
  EXPECT_LT(1.9, odom.pose.pose.position.x);
  EXPECT_GT(0.0, odom.pose.pose.position.y);
  EXPECT_GT(0.0, tf::getYaw(odom.pose.pose.orientation));
  EXPECT_GT(1.9, odom.twist.twist.linear.x);
  EXPECT_GT(0.0, odom.twist.twist.angular.z);
}

TEST(OdometryWheel, OdometryWheelDesync) {

  odometry_wheels::OdometryWheels odom_wheel(1000, 1000, 1.0);
  nav_msgs::Odometry odom;

  encoder_msgs::Int64Stamped left_tick;
  left_tick.header.stamp = ros::Time(1.0);
  left_tick.ticks.data = 1000;
  odom_wheel.UpdateFromLeftWheelTicks(left_tick);
  EXPECT_FALSE(odom_wheel.GetCurrentOdometry(&odom));

  encoder_msgs::Int64Stamped right_tick;
  right_tick.header.stamp = ros::Time(1.5);
  right_tick.ticks.data = 1500;
  odom_wheel.UpdateFromRightWheelTicks(right_tick);
  EXPECT_FALSE(odom_wheel.GetCurrentOdometry(&odom));

  left_tick.header.stamp = ros::Time(2.0);
  left_tick.ticks.data = 2000;
  odom_wheel.UpdateFromLeftWheelTicks(left_tick);
  EXPECT_FALSE(odom_wheel.GetCurrentOdometry(&odom));

  right_tick.header.stamp = ros::Time(2.5);
  right_tick.ticks.data = 2500;
  odom_wheel.UpdateFromRightWheelTicks(right_tick);
  EXPECT_TRUE(odom_wheel.GetCurrentOdometry(&odom));

  // Latest time of left tick should be the update time.
  EXPECT_EQ(ros::Time(2.0), odom.header.stamp);
  EXPECT_EQ(.5, odom.pose.pose.position.x);
  EXPECT_EQ(0.0, odom.pose.pose.position.y);
  EXPECT_EQ(0.0, tf::getYaw(odom.pose.pose.orientation));
  EXPECT_EQ(1.0, odom.twist.twist.linear.x);
  EXPECT_EQ(0.0, odom.twist.twist.angular.z);

  left_tick.header.stamp = ros::Time(3.0);
  left_tick.ticks.data = 3000;
  odom_wheel.UpdateFromLeftWheelTicks(left_tick);
  EXPECT_TRUE(odom_wheel.GetCurrentOdometry(&odom));

  // Latest time of right tick should be the update time.
  EXPECT_EQ(ros::Time(2.5), odom.header.stamp);
  EXPECT_EQ(1.0, odom.pose.pose.position.x);
  EXPECT_EQ(0.0, odom.pose.pose.position.y);
  EXPECT_EQ(0.0, tf::getYaw(odom.pose.pose.orientation));
  EXPECT_EQ(1.0, odom.twist.twist.linear.x);
  EXPECT_EQ(0.0, odom.twist.twist.angular.z);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
