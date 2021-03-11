#include <iostream>
#include <memory>
#include <cstring>
#include <ros/ros.h>

#include <lwr4p/robot.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "$project_name$");

  ros::NodeHandle nh("~");

  std::string robot_desc;
  if (!nh.getParam("robot_description",robot_desc)) throw std::runtime_error("Failed to load param \"robot_description\"...");

  std::shared_ptr<lwr4p_::Robot> robot;
  robot.reset(new lwr4p_::Robot(robot_desc));
  robot->publishState();

  robot->assignJointsPosition({0.24, -0.36, 0, 1.32, -0.12, -1.21, 0});

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  robot->startSim();

  double mo = robot->getObjectMass();
  arma::vec u = arma::vec().zeros(6,1);
  // u(2) = mo*9.81;

  while (ros::ok())
  {
    robot->setInputWrench(u);
    robot->waitSimCycle();
  }

  return 0;
}
