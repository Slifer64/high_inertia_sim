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

Ur_Wrapper::Ur_Wrapper(const arma::mat &T_lh_rh, const arma::mat &T_b_h1, const arma::mat &T_b_h2)
{
  R_.resize(2);
  R_[0] = R_[1] = arma::mat().eye(4,4);

  T_b1_b2 = arma::mat().eye(4,4);
  T_b1_b2(0,3) = 1.1;

  T_h1_h2 = T_lh_rh;

  arma::vec p = {0.39, -0.6, 0.34}; // h1 position w.r.t. left robot
  p(0) = (arma::norm(T_b1_b2.submat(0,3,2,3)) - arma::norm(T_h1_h2.submat(0,3,2,3))) / 2;

  arma::mat R = {{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}; // h1 orientation w.r.t. left robot

  arma::mat T_b1_h1 = arma::mat().eye(4,4);
  T_b1_h1.submat(0,0,2,3) = arma::join_horiz(R, p);

  arma::mat T_b2_h2 = arma::inv(T_b1_b2) * T_b1_h1 * T_h1_h2;

  T_b_b1 = T_b_h1 * arma::inv(T_b1_h1);
  T_b_b2 = T_b_h2 * arma::inv(T_b2_h2);

  setWrenchRotTransform(T_b_b1.submat(0,0,2,2), 0);
  setWrenchRotTransform(T_b_b2.submat(0,0,2,2), 1);

  T_b1_h1_0 = T_b1_h1;
  T_b2_h2_0 = T_b2_h2;

  Ts = 0.002;
  Fext_prev = arma::vec().zeros(6);

  // std::cout << "T_b1_b2 = \n" << T_b1_b2 << "\n";
  // std::cout << "T_h1_h2 = \n" << T_h1_h2 << "\n";
  // std::cout << "T_b1_h1 = \n" << T_b1_h1 << "\n";
  // std::cout << "T_b2_h2 = \n" << T_b2_h2 << "\n";
  // std::cout << "T_b_b1 = \n" << T_b_b1 << "\n";
  // std::cout << "T_b_b2 = \n" << T_b_b2 << "\n";
  // exit(-1);

  ros::NodeHandle nh("~");
  std::string robot_desc;
  if (!nh.getParam("ur_robot_description",robot_desc)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"ur_robot_description\".");

  if (!nh.getParam("a_f",a_f)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"a_f\".");
  std::vector<double> temp;
  if (!nh.getParam("Fext_deadzone",temp)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"Fext_deadzone\".");
  Fext_deadzone = temp;

  if (!nh.getParam("SINGULARITY_THRES",SINGULARITY_THRES)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"SINGULARITY_THRES\".");
  if (!nh.getParam("VEL_THRES",VEL_THRES)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"VEL_THRES\".");
  if (!nh.getParam("ROT_VEL_THRES",ROT_VEL_THRES)) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"ROT_VEL_THRES\".");

  std::vector<std::string> host_ip(2);
  std::vector<std::string> robot_ip(2);
  std::vector<int> reverse_port(2);

  if (!nh.getParam("left_robot_host_ip",host_ip[0])) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"left_robot_host_ip\".");
  if (!nh.getParam("left_robot_robot_ip",robot_ip[0])) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"left_robot_robot_ip\".");
  if (!nh.getParam("left_robot_reverse_port",reverse_port[0])) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"left_robot_reverse_port\".");

  if (!nh.getParam("right_robot_host_ip",host_ip[1])) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"right_robot_host_ip\".");
  if (!nh.getParam("right_robot_robot_ip",robot_ip[1])) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"right_robot_robot_ip\".");
  if (!nh.getParam("right_robot_reverse_port",reverse_port[1])) throw std::ios_base::failure(Ur_Wrapper_fun_ + "Failed to read parameter \"right_robot_reverse_port\".");

  PRINT_INFO_MSG("left_robot: host_ip: " + host_ip[0] + "\n");
  PRINT_INFO_MSG("left_robot: robot_ip: " + robot_ip[0] + "\n");
  PRINT_INFO_MSG("left_robot: reverse_port: " + std::to_string(reverse_port[0]) + "\n\n");

  PRINT_INFO_MSG("right_robot: host_ip: " + host_ip[1] + "\n");
  PRINT_INFO_MSG("right_robot: robot_ip: " + robot_ip[1] + "\n");
  PRINT_INFO_MSG("right_robot: reverse_port: " + std::to_string(reverse_port[1]) + "\n\n");

  // Initialize generic robot with the kuka-lwr model
  PRINT_INFO_MSG("=======> Creating ur-robot wrapper...\n");

  robot.resize(2);
  for (int i=0; i<2; i++)
    robot[i].reset(new ur_::Robot(robot_desc, "base_link", "tool_link", host_ip[i], robot_ip[i], reverse_port[i]));

  PRINT_INFO_MSG("=======> ur-robot wrapper created successfully!\n");

  // moveToStartPose();

  robot[0]->setNormalMode();
  robot[1]->setNormalMode();

  biasFTSensors();

  pose.resize(2);
  Vel.resize(2);
  for (int i=0; i<2; i++)
  {
    pose[i] = getTaskPose(i);
    Vel[i] = {0,0,0,0,0,0};
  }

  R_b1_b = T_b_b1.submat(0,0,2,2).t();
  R_b2_b = T_b_b2.submat(0,0,2,2).t();
}

Ur_Wrapper::~Ur_Wrapper()
{
  robot[0]->setTaskVelocity({0,0,0,0,0,0});
  robot[1]->setTaskVelocity({0,0,0,0,0,0});
  // run_ = false;
}


bool Ur_Wrapper::throwError(const std::string &msg)
{
  setIdleMode();
  // robot[0]->setJointsVelocity(arma::vec().zeros(6));
  // robot[1]->setJointsVelocity(arma::vec().zeros(6));
  PRINT_ERROR_MSG(msg);
  //throw std::runtime_error(msg);
  return false;
}

bool Ur_Wrapper::setVelocity(const arma::vec &V)
{
  for (int i=0; i<2; i++)
  {
      arma::mat J = robot[i]->robot_urdf->getJacobian(robot[i]->getJointsPosition());
      if (arma::rank(J, SINGULARITY_THRES) < 6)
      {
        robot[0]->setJointsVelocity(arma::vec().zeros(6));
        robot[1]->setJointsVelocity(arma::vec().zeros(6));
        return throwError("Robot " + std::to_string(i+1) + " is close to singular pose!\n");
      }
  }

  arma::vec V1 = V.subvec(0,5);
  V1.subvec(0,2) = R_b1_b*V1.subvec(0,2);
  V1.subvec(3,5) = R_b1_b*V1.subvec(3,5);

  if (arma::norm(V1.subvec(0,2)) > VEL_THRES)
    return throwError("Robot 1: Linear velocity limit exceeded: " + std::to_string(arma::norm(V1.subvec(0,2))) + "\n");
  if (arma::norm(V1.subvec(3,5)) > ROT_VEL_THRES)
    return throwError("Robot 1: Angular velocity limit exceeded: " + std::to_string(arma::norm(V1.subvec(3,5))) + "\n");
  if (pose[0](1) < -0.8)
    return throwError("Robot 1: Y-pos limit exceeded!\n");
  if (pose[0](2) < 0.15)
    return throwError("Robot 1: Z-pos limit exceeded!\n");

  arma::vec V2 = V.subvec(6,11);
  V2.subvec(0,2) = R_b2_b*V2.subvec(0,2);
  V2.subvec(3,5) = R_b2_b*V2.subvec(3,5);

  if (arma::norm(V2.subvec(0,2)) > VEL_THRES)
    return throwError("Robot 2: Linear velocity limit exceeded: " + std::to_string(arma::norm(V2.subvec(0,2))) + "\n");
  if (arma::norm(V2.subvec(3,5)) > ROT_VEL_THRES)
    return throwError("Robot 2: Angular velocity limit exceeded: " + std::to_string(arma::norm(V2.subvec(3,5))) + "\n");
  if (pose[1](1) < -0.8)
    return throwError("Robot 2: Y-pos limit exceeded!\n");
  if (pose[1](2) < 0.15)
    return throwError("Robot 2: Z-pos limit exceeded!\n");

  // robot[0]->setTaskVelocity(V1);
  // robot[1]->setTaskVelocity(V2);

  arma::vec V1_cmd = calcVelocity_with_click(V1, 0);
  arma::vec V2_cmd = calcVelocity_with_click(V2, 1);
  robot[0]->setTaskVelocity(V1_cmd);
  robot[1]->setTaskVelocity(V2_cmd);

  return true;
}

arma::vec Ur_Wrapper::calcVelocity_with_click(const arma::vec &V, int i)
{
  pose[i].subvec(0,2) += Vel[i].subvec(0,2)*Ts;
  pose[i].subvec(3,6) = math_::quatProd(math_::quatExp(Vel[i].subvec(3,5)*Ts) , pose[i].subvec(3,6));
  Vel[i] = V;

  arma::vec pose_robot = getTaskPose(i);

  arma::vec V_cmd = V;
  V_cmd.subvec(0,2) += 4*(pose[i].subvec(0,2) - pose_robot.subvec(0,2));
  V_cmd.subvec(3,5) += 3*math_::quatLog( math_::quatDiff(pose[i].subvec(3,6), pose_robot.subvec(3,6)) );

  return V_cmd;
}

void Ur_Wrapper::waitNextCycle()
{
  robot[0]->update();
  robot[1]->update();
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
  PRINT_INFO_MSG("Bias F/T sensors DONE!\n");
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

  arma::vec p_b1_h1 = T_b1_h1_0.submat(0,3,2,3);
  arma::mat Q_b1_h1 = math_::rotm2quat(T_b1_h1_0.submat(0,0,2,2));

  arma::mat T_b2_h2_0 = arma::inv(T_b1_b2) * T_b1_h1_0 * T_h1_h2;
  arma::vec p_b2_h2 = T_b2_h2_0.submat(0,3,2,3);
  arma::mat Q_b2_h2 = math_::rotm2quat(T_b2_h2_0.submat(0,0,2,2));

  arma::mat poseT_mat(7,2);
  poseT_mat.col(0) = arma::join_vert(p_b1_h1, Q_b1_h1);
  poseT_mat.col(1) = arma::join_vert(p_b2_h2, Q_b2_h2);

  PRINT_INFO_MSG("Moving with Cartesian trajectory...\n");
  // move with Cartesian trajectory to desired pose
  for (int i=0; i<2; i++) thr[i] = std::thread(&Ur_Wrapper::setCartTrajectory, this, poseT_mat.col(i), robot[i].get());
  for (int i=0; i<2; i++) thr[i].join();

  pose.resize(2);
  Vel.resize(2);
  for (int i=0; i<2; i++)
  {
    pose[i] = getTaskPose(i);
    Vel[i] = {0,0,0,0,0,0};
  }

  PRINT_INFO_MSG("DONE!\n");

  setIdleMode();
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
  robot_->setNormalMode();

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
  robot_->setNormalMode();

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

void Ur_Wrapper::setFreedriveMode()
{
  robot[0]->setFreedriveMode();
  robot[1]->setFreedriveMode();
}

void Ur_Wrapper::setIdleMode()
{
  robot[0]->setNormalMode();
  robot[1]->setNormalMode();
}
