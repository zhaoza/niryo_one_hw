#include "niryo_one_hw/niryo_one_hw_interface.hpp"
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "niryo_one_hw_node");

  // Run the whole node under /niryo_robot so controller_manager services are:
  //   /niryo_robot/controller_manager/*
  ros::NodeHandle nh("/niryo_robot");

  niryo_one_hw::NiryoOneHW robot(nh);         // robot reads /niryo_robot/hardware/*
  if (!robot.init()) return 1;

  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  const double loop_hz = 200.0;
  ros::Rate rate(loop_hz);
  ros::Time last = ros::Time::now();

  while (ros::ok()) {
    const ros::Time now = ros::Time::now();
    const ros::Duration period = now - last;
    last = now;

    robot.read(period);
    cm.update(now, period);
    robot.write(period);

    rate.sleep();
  }
  return 0;
}
