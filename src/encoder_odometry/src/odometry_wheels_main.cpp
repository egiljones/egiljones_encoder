#include <memory>
#include <thread>

#include <ros/ros.h>

#include "encoder_odometry/odometry_wheels.h"

namespace {
constexpr double kDefaultWheelBase = 1.0;
constexpr double kDefaultTicksPerMeter = 1000;
constexpr double kDefaultPubRateHz = 10;

class OdometryWheelsROS {
 public:
  OdometryWheelsROS() : priv_nh_("~") {

    double wheel_base_m;
    priv_nh_.param<double>(
        "robot_wheel_base_m", wheel_base_m, kDefaultWheelBase);

    double left_ticks_per_m;
    priv_nh_.param<double>(
        "left_wheel_ticks_per_m", left_ticks_per_m, kDefaultTicksPerMeter);

    double right_ticks_per_m;
    priv_nh_.param<double>(
        "right_wheel_ticks_per_m", right_ticks_per_m, kDefaultTicksPerMeter);

    priv_nh_.param<double>("odom_pub_rate_hz", pub_rate_hz_, kDefaultPubRateHz);

    odometry_wheels_.reset(new odometry_wheels::OdometryWheels(
        left_ticks_per_m, right_ticks_per_m, wheel_base_m));

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1000);

    std::function<void(const encoder_msgs::Int64StampedConstPtr&)>
        left_function =
        [this](const encoder_msgs::Int64StampedConstPtr& msg) {
      this->odometry_wheels_->UpdateFromLeftWheelTicks(*msg);
    };

    left_sub_ = nh_.subscribe<encoder_msgs::Int64Stamped>(
        "/encoders/left_wheel", 100, left_function);

    std::function<void(const encoder_msgs::Int64StampedConstPtr&)>
        right_function =
        [this](const encoder_msgs::Int64StampedConstPtr& msg) {
      this->odometry_wheels_->UpdateFromRightWheelTicks(*msg);
    };

    right_sub_ = nh_.subscribe<encoder_msgs::Int64Stamped>(
        "/encoders/right_wheel", 100, right_function);

    pub_thread_ =
        std::thread(std::bind(&OdometryWheelsROS::PublishThread, this));
  }

  ~OdometryWheelsROS() {
    pub_thread_.join();
  }

 private:

  void PublishThread() {
    ros::Rate r(pub_rate_hz_);
    while(ros::ok()) {
      nav_msgs::Odometry odom;
      if (odometry_wheels_->GetCurrentOdometry(&odom)) {
        odom_pub_.publish(odom);
      }
      r.sleep();
    }
  }

  std::unique_ptr<odometry_wheels::OdometryWheels> odometry_wheels_;
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  ros::Publisher odom_pub_;
  ros::Subscriber left_sub_;
  ros::Subscriber right_sub_;

  std::thread pub_thread_;
  double pub_rate_hz_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_wheels");

  OdometryWheelsROS wheels;

  ros::spin();
}
