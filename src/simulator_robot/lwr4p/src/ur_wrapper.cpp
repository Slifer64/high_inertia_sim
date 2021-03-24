#include <lwr4p/ur_wrapper.h>

#include <math_lib/math_lib.h>
#include <io_lib/print_utils.h>

using namespace as64_;

#define Ur_Wrapper_fun_ std::string("[Ur_Wrapper::") + __func__ + "]: "

namespace ur_wrap_
{

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

arma::mat jacobQq(const arma::vec &Q1)
{
  arma::mat JQq(4,3);

  if ( (1-std::fabs(Q1(0))) <= 1e-6)
  {
    JQq.row(0) = arma::rowvec().zeros(3);
    JQq.submat(1,0,3,2) = arma::mat().eye(3,3);
    return JQq;
  }

  double w = Q1(0);
  arma::vec v = Q1.subvec(1,3);
  double norm_v = arma::norm(v);
  arma::vec eta = v / norm_v;
  double s_th = norm_v;
  double c_th = w;
  double th = std::atan2(s_th, c_th);
  arma::mat Eta = eta*eta.t();

  JQq.row(0) = -0.5 * s_th * eta.t();
  JQq.submat(1,0,3,2) = 0.5 * ( (arma::mat().eye(3,3) - Eta)*s_th/th + c_th*Eta );
  return JQq;
}

arma::vec quatlogDot2rotVel(const arma::vec &Qlog_dot, const arma::vec &Q)
{
  arma::mat J_Q_Qlog = jacobQq(Q);
  arma::vec rotVel = 2 * math_::quatProd( J_Q_Qlog*Qlog_dot, math_::quatInv(Q) );
  return rotVel.subvec(1,3);
}

};

// ==================================================================

Ur_Wrapper::Ur_Wrapper()
{
  Ts = 0.002;
  Fext_prev = arma::vec().zeros(6);

  R_.resize(2);
  R_[0] = R_[1] = arma::mat().eye(4,4);

  ros::NodeHandle nh("~");
  std::string robot_desc;
  if (!nh.getParam("ur_robot_description",robot_desc)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"ur_robot_description\".");

  if (!nh.getParam("a_f",a_f)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"a_f\".");
  std::vector<double> temp;
  if (!nh.getParam("Fext_deadzone",temp)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"Fext_deadzone\".");
  Fext_deadzone = temp;


  std::string left_robot_host_ip;
  std::string left_robot_robot_ip;
  int left_robot_reverse_port;

  if (!nh.getParam("left_robot_host_ip",left_robot_host_ip)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"left_robot_host_ip\".");
  if (!nh.getParam("left_robot_robot_ip",left_robot_robot_ip)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"left_robot_robot_ip\".");
  if (!nh.getParam("left_robot_reverse_port",left_robot_reverse_port)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"left_robot_reverse_port\".");

  // ------------------------------------
  std::string right_robot_host_ip;
  std::string right_robot_robot_ip;
  int right_robot_reverse_port;

  if (!nh.getParam("right_robot_host_ip",right_robot_host_ip)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"right_robot_host_ip\".");
  if (!nh.getParam("right_robot_robot_ip",right_robot_robot_ip)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"right_robot_robot_ip\".");
  if (!nh.getParam("right_robot_reverse_port",right_robot_reverse_port)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"right_robot_reverse_port\".");

  std::string base_link = "base_link";
  std::string tool_link = "tool_link";
  std::vector<std::string> robot_ip = {left_robot_robot_ip, right_robot_robot_ip}; // {"10.0.1.1", "10.0.0.1"};
  std::vector<std::string> host_ip = {left_robot_host_ip, right_robot_host_ip}; // {"10.0.1.3", "10.0.0.3"};
  std::vector<int> reverse_port = {left_robot_reverse_port, right_robot_reverse_port}; // {8081, 8080};


  PRINT_INFO_MSG("left_robot_robot_ip: " + left_robot_robot_ip + "\n");
  PRINT_INFO_MSG("left_robot_host_ip: " + left_robot_host_ip + "\n");

  PRINT_INFO_MSG("right_robot_robot_ip: " + right_robot_robot_ip + "\n");
  PRINT_INFO_MSG("right_robot_host_ip: " + right_robot_host_ip + "\n");

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

arma::vec Ur_Wrapper::getWrench(int robot_ind)
{
  // get the wrench in base frame
  arma::vec Fext = robot[robot_ind]->getTcpWrench();

  Fext = applyFextDeadZone(Fext);
  Fext = a_f*Fext + (1-a_f)*Fext_prev;
  Fext_prev = Fext;

  // express the wrench w.r.t. new frame
  Fext.subvec(0,2) = R_[robot_ind]*Fext.subvec(0,2);
  Fext.subvec(3,5) = R_[robot_ind]*Fext.subvec(3,5);

  return Fext;
}

arma::vec Ur_Wrapper::applyFextDeadZone(const arma::vec &F_ext) const
{
  arma::vec sign_Fext = arma::sign(F_ext);
  arma::vec Fext2 = F_ext - sign_Fext%Fext_deadzone;
  return 0.5*(arma::sign(Fext2)+sign_Fext)%arma::abs(Fext2);
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

  PRINT_INFO_MSG("Moving with joints trajectory...\n");
  // move with joints trajectory close to desired pose
  for (int i=0; i<2; i++) thr[i] = std::thread(&Ur_Wrapper::setJointsTrajectory, this, qT_mat.col(i), robot[i].get());
  for (int i=0; i<2; i++) thr[i].join();

  arma::mat R = {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}};
  arma::mat q = math_::rotm2quat(R);
  arma::mat poseT_mat(7,2);
  poseT_mat.col(0) = arma::vec({0.25, -0.6, 0.34, q(0), q(1), q(2), q(3)});
  poseT_mat.col(1) = arma::vec({-0.25, -0.6, 0.34, q(0), q(1), q(2), q(3)});

  PRINT_INFO_MSG("Moving with Cartesian trajectory...\n");
  // move with Cartesian trajectory to desired pose
  for (int i=0; i<2; i++) thr[i] = std::thread(&Ur_Wrapper::setCartTrajectory, this, poseT_mat.col(i), robot[i].get());
  for (int i=0; i<2; i++) thr[i].join();
}

void Ur_Wrapper::setJointsTrajectory(const arma::vec &qT, ur_::Robot *robot_)
{
  robot_->setNormalMode();
  robot_->update();

  arma::vec q0 = robot_->getJointsPosition();
  double duration = std::max(arma::max(arma::abs(qT - q0)) * 10.0 / 3.14159, 1.5);

  arma::vec qref = q0;
  double t = 0.0;

  while (t < duration)
  {
    if (!robot_->isOk())
    {
      PRINT_ERROR_MSG("[Ur_Wrapper::setJointsTrajectory]: Robot is not ok!\n");
      return;
    }

    t += Ts;
    robot_->setJointsPosition(ur_wrap_::get5thOrder(t, q0, qT, duration).col(0));
    robot_->update();
  }

  robot_->setTaskVelocity(arma::vec().zeros(6));
  robot_->update();

  double q_err = arma::norm(robot_->getJointsPosition()-qT);
  if (q_err > 5e-4)
  {
    std::ostringstream oss;
    oss << "[Ur_Wrapper::" << __func__ << "]: Joints error is above tolerance:\n"
        << "q_err: " << q_err << "\n";
    PRINT_WARNING_MSG(oss.str());
  }
}

void Ur_Wrapper::setCartTrajectory(const arma::vec &poseT, ur_::Robot *robot_)
{
  robot_->setNormalMode();
  robot_->update();

  arma::mat T = robot_->getTaskPose();
  arma::vec pose0 = arma::join_vert(T.submat(0,3,2,3), math_::rotm2quat(T.submat(0,0,2,2)));

  arma::vec p0 = pose0.subvec(0,2);
  arma::vec pT = poseT.subvec(0,2);
  double duration_p = std::max(arma::max(arma::abs(pT - p0)) * 1.1 / 0.1, 1.5);

  arma::vec Q0 = pose0.subvec(3,6);
  arma::vec QT = poseT.subvec(3,6);
  if (arma::dot(Q0,QT) < 0) QT = -QT;
  arma::vec logQ0 = math_::quatLog(Q0);
  arma::vec logQT = math_::quatLog(QT);
  double duration_o = std::max(arma::max(arma::abs(logQT - logQ0)) * 10.0 / 3.14159, 1.5);

  double duration = std::max(duration_p, duration_o);

  arma::vec pref = p0;
  arma::vec logQref = logQ0;
  double t = 0.0;
  while (t < duration)
  {
    if (!robot_->isOk())
    {
      PRINT_ERROR_MSG("[Ur_Wrapper::setCartTrajectory]: Robot is not ok!\n");
      return;
    }

    t += Ts;

    arma::mat Pos_ref = ur_wrap_::get5thOrder(t, p0, pT, duration);
    arma::vec pref = Pos_ref.col(0);
    arma::vec pref_dot = Pos_ref.col(1);
    arma::vec p_robot = robot_->getTaskPosition();
    arma::vec v_cmd = pref_dot + 4*(pref - p_robot);

    arma::mat Orient_ref = ur_wrap_::get5thOrder(t, logQ0, logQT, duration);
    arma::vec logQref = Orient_ref.col(0);
    arma::vec logQref_dot = Orient_ref.col(1);
    arma::vec Qref = math_::quatExp(logQref);
    arma::vec Q_robot = robot_->getTaskQuat();
    if (arma::dot(Q_robot, Qref) < 0) Q_robot = -Q_robot;
    arma::vec vRot_ref = ur_wrap_::quatlogDot2rotVel(logQref_dot, Qref);
    arma::vec vRot_cmd = vRot_ref + 4*math_::quatLog(math_::quatDiff(Qref, Q_robot));

    arma::vec V_cmd = arma::join_vert(v_cmd, vRot_cmd);
    robot_->setTaskVelocity(V_cmd);
    robot_->update();
  }

  robot_->setTaskVelocity(arma::vec().zeros(6));
  robot_->update();

  arma::vec p_robot = robot_->getTaskPosition();
  arma::vec Q_robot = robot_->getTaskQuat();
  if (arma::dot(Q_robot, QT) < 0) Q_robot = -Q_robot;
  double pos_err = arma::norm(p_robot-pT);
  double orient_err = arma::norm(math_::quatLog(math_::quatDiff(Q_robot,QT)));
  if (pos_err > 5e-4 || orient_err > 1e-2)
  {
    std::ostringstream oss;
    oss << "[Ur_Wrapper::" << __func__ << "]: Pos/Orient error is above tolerance:\n"
        << "err_p: " << pos_err << "\n"
        << "err_o: " << orient_err << "\n";
    PRINT_WARNING_MSG(oss.str());
  }
}

arma::vec Ur_Wrapper::getTaskPose(int robot_ind) const
{
  arma::mat T = robot[robot_ind]->getTaskPose();
  return arma::join_vert(T.submat(0,3,2,3), math_::rotm2quat(T.submat(0,0,2,2)));
}
