#include <lwr4p/ur_wrapper.h>

#define Ur_Wrapper_fun_ std::string("[Ur_Wrapper::") + __func__ + "]: "

Ur_Wrapper::Ur_Wrapper()
{
  R_.resize(2);
  R_[0] = R_[1] = arma::mat().eye(4,4);

  ros::NodeHandle nh("~");
  std::string robot_desc;
  if (!nh.getParam("ur_robot_description",robot_desc)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"ur_robot_description\".");

  std::string base_link = "base_link";
  std::string tool_link = "tool_link";
  std::vector<std::string> robot_ip = {"10.0.0.1", "10.0.0.1"};
  std::vector<std::string> host_ip = {"10.0.0.2", "10.0.0.2"};
  std::vector<int> reverse_port = {8080, 8081};

  // Initialize generic robot with the kuka-lwr model
  std::cerr << "=======> Creating ur-robot wrapper...\n";

  robot.resize(2);
  for (int i=0; i<2; i++)
    robot[i].reset(new ur_::Robot(robot_desc, base_link, tool_link, host_ip[i], robot_ip[i], reverse_port[i]));

  std::cerr << "=======> ur-robot wrapper created successfully!\n";

  moveToStartPose();
  biasFTSensors();

  //
  // N_JOINTS = robot->getNumJoints();
  //
  // Ts = robot->getCtrlCycle();
  //
  // mode.set(rw_::STOPPED);
  // cmd_mode.set(rw_::IDLE);
  // jpos_cmd.set(robot->getJointsPosition());
  //
  // run_ = true;
  // std::thread robot_ctrl_thread = std::thread(&Ur_Wrapper::commandThread,this);
  // int err_code = thr_::setThreadPriority(robot_ctrl_thread, SCHED_FIFO, 99);
  // if (err_code) PRINT_WARNING_MSG(Ur_Wrapper_fun_ + "Failed to set thread priority! Reason:\n" + thr_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  // else PRINT_INFO_MSG(Ur_Wrapper_fun_ + "Set thread priority successfully!\n", std::cerr);
  // robot_ctrl_thread.detach();
  //
  // mode_change.wait(); // wait for mode to be set
}

Ur_Wrapper::~Ur_Wrapper()
{
  // run_ = false;
}

void Ur_Wrapper::setCartVelCtrl()
{
  robot[0]->setNormalMode();
  robot[1]->setNormalMode();
}

void Ur_Wrapper::biasFTSensors()
{
  robot[0]->biasFtSensor();
  robot[1]->biasFtSensor();
}

arma::vec Ur_Wrapper::getWrench(int robot_ind) const
{
  // get the wrench in base frame
  arma::vec wrench = robot[robot_ind]->getTcpWrench();

  // express the wrench w.r.t. new frame
  wrench.subvec(0,2) = R_[robot_ind]*wrench.subvec(0,2);
  wrench.subvec(3,5) = R_[robot_ind]*wrench.subvec(3,5);

  return wrench;
}

arma::mat Ur_Wrapper::getRotm(int robot_ind) const
{
  return robot[robot_ind]->getTaskRotm();
}

void Ur_Wrapper::setWrenchRotTransform(const arma::mat &R, int robot_ind)
{
  R_[robot_ind] = R;
}

void Ur_Wrapper::moveToStartPose()
{
  ros::NodeHandle nh("~");
  std::vector<double> ur_robot1_q0, ur_robot2_q0;
  if (!nh.getParam("ur_robot1_q0",ur_robot1_q0)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"ur_robot1_q0\".");
  if (!nh.getParam("ur_robot2_q0",ur_robot2_q0)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"ur_robot2_q0\".");

  arma::mat qT_mat = arma::join_horiz(arma::vec{ur_robot1_q0}, arma::vec{ur_robot2_q0});
  std::vector<std::thread> thr(2);
  for (int i=0; i<2; i++)
  {
    robot[i]->update();
    arma::vec q0 = robot[i]->getJointsPosition();
    arma::vec qT = qT_mat.col(i);
    double duration = std::max(arma::max(arma::abs(qT - q0)) * 10.0 / 3.14159, 2.5);
    thr[i] = std::thread(&Ur_Wrapper::setJointsTrajectory, this, qT, duration, robot[i].get());
  }
  for (int i=0; i<2; i++) thr[i].join();
}

arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime)
{
  arma::mat retTemp = arma::zeros<arma::mat>(p0.n_rows, 3);

  if (t < 0)
  {
    // before start
    retTemp.col(0) = p0;
  }
  else if (t > totalTime)
  {
    // after the end
    retTemp.col(0) = pT;
  }
  else
  {
    // somewhere betweeen ...
    // position
    retTemp.col(0) = p0 +
                     (pT - p0) * (10 * pow(t / totalTime, 3) -
                     15 * pow(t / totalTime, 4) +
                     6 * pow(t / totalTime, 5));
    // vecolity
    retTemp.col(1) = (pT - p0) * (30 * pow(t, 2) / pow(totalTime, 3) -
                     60 * pow(t, 3) / pow(totalTime, 4) +
                     30 * pow(t, 4) / pow(totalTime, 5));
    // acceleration
    retTemp.col(2) = (pT - p0) * (60 * t / pow(totalTime, 3) -
                     180 * pow(t, 2) / pow(totalTime, 4) +
                     120 * pow(t, 3) / pow(totalTime, 5));
  }

  // return vector
  return retTemp;
}

void Ur_Wrapper::setJointsTrajectory(const arma::vec &qT, double duration, ur_::Robot *robot_)
{
  robot_->setNormalMode();
  robot_->update();
  arma::vec q0 = robot_->getJointsPosition();
  arma::vec qref = q0;
  double t = 0.0;

  while (t < duration)
  {
    if (!robot_->isOk())
    {
      std::cerr << "An error occured on the robot!\n";
      return;
    }

    t += 0.002;
    robot_->setJointsPosition(get5thOrder(t, q0, qT, duration).col(0));
    robot_->update();
  }
}

/*
void Ur_Wrapper::setMode(const Mode &mode)
{
  if (mode == cmd_mode.get()) return;

  cmd_mode.set(mode);
  while (getMode()!=mode && isOk()) // to avoid spurious wake ups
  {
    mode_change.wait(); // wait to get notification from commandThread
  }
}

void Ur_Wrapper::commandThread()
{
  unsigned long long count = 0;

  arma::mat J;
  arma::vec q_dot;
  arma::vec adm_vel;

  arma::wall_clock timer;

  std::string limit_msg;

  while (run_)
  {
    if (!isOk())
    {
      // notify all to avoid deadlocks
      KRC_tick.notify();
      //mode_change.notify();
      continue;
    }

    timer.tic();

    Mode new_mode = cmd_mode.read();
    // check if we have to switch mode
    if (new_mode != mode.read())
    {
      switch (new_mode)
      {
        case rw_::Mode::JOINT_POS_CONTROL:
          robot->setNormalMode();
          jpos_cmd.set(robot->getJointsPosition());
          break;
        case rw_::Mode::JOINT_TORQUE_CONTROL:
          throw std::runtime_error(Ur_Wrapper_fun_ + "Unsupported mode \"JOINT_TORQUE_CONTROL\" for ur-robot...");
          break;
        case rw_::Mode::FREEDRIVE:
          robot->setFreedriveMode();
          break;
        case rw_::Mode::ADMITTANCE:
          robot->setNormalMode();
          adm_ctrl->init();
          break;
        case rw_::Mode::CART_VEL_CTRL:
          robot->setNormalMode();
          cart_vel_cmd.set(arma::vec().zeros(6));
          break;
        case rw_::Mode::IDLE:
          robot->setNormalMode();
          jpos_cmd.set(robot->getJointsPosition());
          break;
        case rw_::Mode::STOPPED:
          robot->setNormalMode();
          // robot->setExternalStop(true);
          mode.set(new_mode);
          mode_change.notify(); // unblock in case wait was called from another thread
          KRC_tick.notify(); // unblock in case wait was called from another thread
          return;
        case PROTECTIVE_STOP:
          mode.set(new_mode);
          continue;
          // if (main_ctrl) emit main_ctrl->gui->emergencyStopSignal();
          break;
      }
      mode.set(new_mode);
      mode_change.notify();
      continue;
    }

    // send command according to current mode
    switch (mode.read())
    {
      case rw_::Mode::JOINT_POS_CONTROL:
        robot->setJointsPosition(jpos_cmd.get());
        break;  //
  // N_JOINTS = robot->getNumJoints();
  //
  // Ts = robot->getCtrlCycle();
  //
  // mode.set(rw_::STOPPED);
  // cmd_mode.set(rw_::IDLE);
  // jpos_cmd.set(robot->getJointsPosition());
  //
  // run_ = true;
  // std::thread robot_ctrl_thread = std::thread(&Ur_Wrapper::commandThread,this);
  // int err_code = thr_::setThreadPriority(robot_ctrl_thread, SCHED_FIFO, 99);
  // if (err_code) PRINT_WARNING_MSG(Ur_Wrapper_fun_ + "Failed to set thread priority! Reason:\n" + thr_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  // else PRINT_INFO_MSG(Ur_Wrapper_fun_ + "Set thread priority successfully!\n", std::cerr);
  // robot_ctrl_thread.detach();
  //
  // mode_change.wait(); // wait for mode to be set
      case CART_VEL_CTRL:
        if (!safety_.assertToolVelLim(cart_vel_cmd.get(), &limit_msg))
        {
          //setErrMsg("Tool velocity limit exceeded: " + std::to_string(arma::norm(cart_vel_cmd.get())) );
          setErrMsg(limit_msg);
          setExternalStop(true);
        }
        else robot->setTaskVelocity(cart_vel_cmd.get());
        break;
      case rw_::Mode::FREEDRIVE:
        // robot->setJointTorque(-robot->getRobotJacobian().t() * tool_estimator->getToolWrench(this->getTaskOrientation()));
        // robot->setJointsTorque(arma::vec().zeros(7));
        break;
      case rw_::Mode::ADMITTANCE:
        adm_ctrl->update();
        adm_vel = adm_ctrl->getVelocity();
        if (!safety_.assertToolVelLim(adm_vel))
        {
          setErrMsg("Tool velocity limit exceeded!");
          setExternalStop(true);
        }
        else
        {
          if (use_svf)
          {
            q_dot = getInvJacobian()*adm_vel;
            if (this->use_jlav) q_dot += jlav->getControlSignal(getJointsPosition());
            if (!safety_.assertJointVelLim(q_dot))
            {
              setErrMsg("Joint Velocity limit exceeded!");
              setExternalStop(true);
            }
            else robot->setJointsVelocity(q_dot);
          }  //
  // N_JOINTS = robot->getNumJoints();
  //
  // Ts = robot->getCtrlCycle();
  //
  // mode.set(rw_::STOPPED);
  // cmd_mode.set(rw_::IDLE);
  // jpos_cmd.set(robot->getJointsPosition());
  //
  // run_ = true;
  // std::thread robot_ctrl_thread = std::thread(&Ur_Wrapper::commandThread,this);
  // int err_code = thr_::setThreadPriority(robot_ctrl_thread, SCHED_FIFO, 99);
  // if (err_code) PRINT_WARNING_MSG(Ur_Wrapper_fun_ + "Failed to set thread priority! Reason:\n" + thr_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  // else PRINT_INFO_MSG(Ur_Wrapper_fun_ + "Set thread priority successfully!\n", std::cerr);
  // robot_ctrl_thread.detach();
  //
  // mode_change.wait(); // wait for mode to be set
          else robot->setTaskVelocity(adm_vel);
        }
        break;
      case rw_::Mode::IDLE:
        // std::cerr << "*** Send command in IDLE mode ***\n";
        // std::cerr << "Robot mode: " << robot->getModeName() << "\n";
        // robot->setJointsPosition(jpos_cmd.get());
        break;
      case rw_::Mode::STOPPED:
      case rw_::Mode::PROTECTIVE_STOP:
        // std::cerr << "***  MODE: STOPPED ***\n";
        break;
    }

    // sync with KRC
    if (robot->isOk()) robot->update();
    KRC_tick.notify();

    double elaps_time = timer.toc();
    // if (elaps_time > 2*getCtrlCycle())
    // {
    //   std::ostringstream oss;
    //   oss << elaps_time*1000;
    //   PRINT_WARNING_MSG(Ur_Wrapper_fun_ + "*** WARNING *** Elaps time = " + oss.str() + " ms\n");
    // }
  }

  // mode_change.notify(); // unblock in case wait was called from another thread
  KRC_tick.notify(); // unblock in case wait was called from another thread
}


void Ur_Wrapper::stop()
{
  setMode(STOPPED);
}
*/
