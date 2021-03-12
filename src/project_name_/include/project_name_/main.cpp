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

  bool comp_load;
  if (!nh.getParam("comp_load",comp_load)) comp_load = false;

  std::shared_ptr<lwr4p_::Robot> robot;
  robot.reset(new lwr4p_::Robot(robot_desc));
  robot->publishState();

  std::vector<double> q0;
  if (!nh.getParam("q0",q0)) throw std::runtime_error("Failed to load param \"q0\"...\n");
  robot->assignJointsPosition(q0);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  robot->startSim();

  arma::vec u = arma::vec().zeros(6,1);
  arma::vec p_oe = robot->get_pos_ee_obj();
  double mo = robot->getObjectMass();
  arma::vec Fo_minus = {0, 0, mo*9.81, 0, 0, 0};

  while (ros::ok())
  {
    if (comp_load)
    {
      arma::mat R = robot->getTaskRotm();
      u.subvec(0,2) = Fo_minus.subvec(0,2);
      u.subvec(3,5) = arma::cross(R*p_oe, Fo_minus.subvec(0,2)) + Fo_minus.subvec(3,5);
      robot->setInputWrench(u);
    }

    robot->waitNextCycle();
  }

  return 0;
}
