#include <iostream>
#include <memory>
#include <cstring>
#include <ros/ros.h>

#include <lwr4p/robot_obj_sim.h>

#include <gui_lib/utils.h>
#include <thread_lib/thread_lib.h>
#include <io_lib/print_utils.h>

#include <csignal>

#include <project_name_/gui/main_gui.h>

using namespace as64_;

class MainController
{
public:

  MainController();
  ~MainController();

  void launchGui();

  void run();

  void setFreedriveMode();
  void setIdleMode();
  void runSimulation();

  void gotoStartPose();
  void biasFTSensors();

  void saveLoggedData(const std::string &filename);

  void setLogging(bool set);

  std::string default_data_path;

  bool gui_finished;
  MainWindow *gui;

  bool comp_load;

  std::shared_ptr<lwr4p_::RobotObjSim> robot;

  thr_::Semaphore gui_created_sem;
};
