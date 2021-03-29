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

class MainProgram
{
public:

  ~MainProgram()
  {
    PRINT_INFO_MSG("[MainProgram::~MainProgram]: Destructor called!\n");
  }

  // QMainWindow *createMainWindow()
  // {
  //   gui = new MainWindow();
  //   return gui;
  // }

  void launchGui()
  {
    gui_finished = false;
    std::thread gui_thr = std::thread([this]()
    {
      gui_::launchGui([this](){
        gui=new MainWindow();
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

  void run()
  {
    ros::NodeHandle nh("~");

    std::string robot_desc;
    if (!nh.getParam("robot_obj_description",robot_desc)) throw std::runtime_error("Failed to load param \"robot_obj_description\"...");

    bool comp_load;
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

    std::shared_ptr<lwr4p_::RobotObjSim> robot;
    robot.reset(new lwr4p_::RobotObjSim(robot_desc));
    robot->publishState();

    if (read_wrench_from_gui)
    {
      robot->setLHandleWrenchReadFun([this](){ return gui->lh_ctrl_->getWrench(); });
      robot->setRHandleWrenchReadFun([this](){ return gui->rh_ctrl_->getWrench(); });
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    robot->startSim();

    arma::vec u = arma::vec().zeros(6,1);
    arma::vec p_eo = robot->get_pos_ee_obj();
    double mo = robot->getObjectMass();
    arma::vec Fo_minus = {0, 0, mo*9.81, 0, 0, 0};

    while (ros::ok())
    {
      if (comp_load)
      {
        arma::mat R = robot->getTaskRotm();
        u.subvec(0,2) = Fo_minus.subvec(0,2);
        u.subvec(3,5) = arma::cross(R*p_eo, Fo_minus.subvec(0,2)) + Fo_minus.subvec(3,5);
        robot->setInputWrench(u);
      }

      robot->waitNextCycle();
    }
  }

  bool gui_finished;
  MainWindow *gui;

  thr_::Semaphore gui_created_sem;
};

// ========================================================
// ========================================================

void terminateProgram(int )
{
  exit(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "$project_name$");

  signal(SIGINT, terminateProgram);

  MainProgram main_prog;
  main_prog.run();

  return 0;
}
