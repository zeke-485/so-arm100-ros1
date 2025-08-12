#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "so_arm_100_hardware/so_arm_100_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "so_arm_100_hw_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  so_arm_100_hardware::SOARM100Interface robot;
  if (!robot.init(nh, pnh)) {
    ROS_ERROR("Failed to initialize hardware interface");
    return 1;
  }

  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(50.0);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev = ros::Time::now();
  while (ros::ok()) {
    ros::Time now = ros::Time::now();
    ros::Duration period = now - prev;
    prev = now;

    robot.read(now, period);
    cm.update(now, period);
    robot.write(now, period);

    rate.sleep();
  }
  spinner.stop();
  return 0;
}
