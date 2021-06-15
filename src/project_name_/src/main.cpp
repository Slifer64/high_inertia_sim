#include <iostream>
#include <memory>
#include <cstring>
#include <ros/ros.h>
#include <csignal>

#include <project_name_/main_ctrl.h>

using namespace as64_;

// ========================================================

void terminateProgram(int )
{
  exit(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "$project_name$");

  signal(SIGINT, terminateProgram);

  MainController main_prog;

  while (ros::ok());

  return 0;
}
