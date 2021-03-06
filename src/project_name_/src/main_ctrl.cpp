#include <iostream>
#include <memory>
#include <cstring>
#include <ros/ros.h>

#include <project_name_/robot_obj_sim.h>

#include <gui_lib/utils.h>
#include <thread_lib/thread_lib.h>
#include <io_lib/print_utils.h>

#include <csignal>

#include <ros/package.h>

#include <project_name_/main_ctrl.h>

using namespace as64_;

MainController::MainController()
{
  ros::NodeHandle nh("~");

  default_data_path = ros::package::getPath("project_name_") + "/data/";

  std::string robot_desc;
  if (!nh.getParam("robot_obj_description",robot_desc)) throw std::runtime_error("Failed to load param \"robot_obj_description\"...");

  if (!nh.getParam("comp_load",comp_load)) comp_load = false;

  bool use_gui;
  if (!nh.getParam("use_gui",use_gui)) use_gui = false;
  if (use_gui) launchGui();

  bool read_wrench_from_gui;
  if (!nh.getParam("read_wrench_from_gui",read_wrench_from_gui)) read_wrench_from_gui = false;
  if (!use_gui && read_wrench_from_gui)
  {
    PRINT_WARNING_MSG("\"read_wrench_from_gui\"=true, however \"use_gui\"=false...\n Setting \"read_wrench_from_gui\" to false!");
    read_wrench_from_gui = false;
  }

  robot.reset(new RobotObjSim(robot_desc));
  robot->publishState();

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

MainController::~MainController()
{
  PRINT_INFO_MSG("[MainController::~MainController]: Destructor called!\n");
}

void MainController::launchGui()
{
  gui_finished = false;
  std::thread gui_thr = std::thread([this]()
  {
    gui_::launchGui([this](){
      gui=new MainWindow(this);
      this->gui_created_sem.notify();
      return gui;
    }, &gui_finished, QThread::LowestPriority);
  });

  int err_code = thr_::setThreadPriority(gui_thr, SCHED_OTHER, 0);
  gui_thr.detach();

  gui_created_sem.wait();

  // if (err_code) PRINT_WARNING_MSG("[MainController::launch]: Failed to set thread priority! Reason:\n" + thr_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  // else PRINT_INFO_MSG("[MainController::launch]: Set thread priority successfully!\n", std::cerr);
}

void MainController::setFreedriveMode()
{
  robot->stopSim();
  robot->ur_wrap->setFreedriveMode();
  emit gui->modeChangedSignal("FREEDRIVE");
}

void MainController::setIdleMode()
{
  robot->stopSim();
  robot->ur_wrap->setIdleMode();
  emit gui->modeChangedSignal("IDLE");
}

void MainController::runSimulation()
{
  robot->setCtrlSignalFun([this]()
  {
    arma::vec u = arma::vec().zeros(6,1);
    arma::vec p_eo = robot->get_pos_ee_obj();
    double mo = robot->getObjectMass();
    arma::vec Fo_minus = {0, 0, mo*9.81, 0, 0, 0};
    arma::mat R = robot->getTaskRotm();
    u.subvec(0,2) = Fo_minus.subvec(0,2);
    u.subvec(3,5) = arma::cross(R*p_eo, Fo_minus.subvec(0,2)) + Fo_minus.subvec(3,5);
    //robot->setInputWrench(u);
    return u;
  });

  robot->startSim();
  emit gui->modeChangedSignal("CART_VEL_CTRL");

}

void MainController::gotoStartPose()
{
  std::thread([this]()
  {
    emit gui->modeChangedSignal("GOTO START");
    this->robot->gotoStartPose();
    emit gui->modeChangedSignal("IDLE");
  }).detach();
}

void MainController::biasFTSensors()
{
  robot->ur_wrap->biasFTSensors();
}

void MainController::saveLoggedData(const std::string &filename)
{
  robot->saveLogData(filename);
}

void MainController::setLogging(bool set)
{
  robot->log_data_ = set;
}
