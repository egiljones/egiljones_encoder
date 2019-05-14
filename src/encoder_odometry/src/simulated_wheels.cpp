#include <ros/ros.h>
#include <encoder_msgs/Int64Stamped.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulated_wheels");

  ros::NodeHandle nh;
  ros::Publisher left_pub =
      nh.advertise<encoder_msgs::Int64Stamped>("/encoders/left_wheel", 100);
  ros::Publisher right_pub =
      nh.advertise<encoder_msgs::Int64Stamped>("/encoders/right_wheel", 100);

  ros::Rate r(50);
  encoder_msgs::Int64Stamped left_msg;
  encoder_msgs::Int64Stamped right_msg;
  ros::Time start_time = ros::Time::now();
  while(ros::ok()) {
    left_msg.header.stamp = ros::Time::now();
    right_msg.header.stamp = left_msg.header.stamp;

    if (left_msg.header.stamp - start_time < ros::Duration(5.0)) {
      left_msg.ticks.data += 20;
      right_msg.ticks.data += 20;
    } else if (left_msg.header.stamp - start_time < ros::Duration(10.0)) {
      left_msg.ticks.data += 10;
      right_msg.ticks.data += 20;
    } else if (left_msg.header.stamp - start_time < ros::Duration(15.0)) {
      left_msg.ticks.data += 20;
      right_msg.ticks.data += 10;
    } else {
      start_time = ros::Time::now();
    }
    left_pub.publish(left_msg);
    right_pub.publish(right_msg);
    r.sleep();
  }
}
