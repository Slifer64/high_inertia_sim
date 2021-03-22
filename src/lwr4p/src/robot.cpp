#include <lwr4p/robot.h>

#include <thread_lib/thread_lib.h>

#include <exception>

#include <ros/ros.h>

namespace lwr4p_
{

Robot::Robot(std::string robot_desc_param)
{
  ros::NodeHandle nh("~");

  is_running = true;

  std::string base_link = "base_link";
  base_ee_chain.reset(new robo_::KinematicChain(robot_desc_param, base_link, "ee_link"));

  obj_ = RPoint( robo_::KinematicChain(robot_desc_param, "ee_link", "object_link").getTaskPose(arma::vec()) );
  lh_ = RPoint( robo_::KinematicChain(robot_desc_param, "ee_link", "left_handle_handle").getTaskPose(arma::vec()) );
  rh_ = RPoint( robo_::KinematicChain(robot_desc_param, "ee_link", "right_handle_handle").getTaskPose(arma::vec()) );

  g_ = arma::vec({0, 0, -9.81});

  std::vector<double> M_vec;
  if (!nh.getParam("M",M_vec)) throw std::runtime_error("Failed to load param \"M\"...\n");
  M = arma::diagmat(arma::vec(M_vec));
  std::vector<double> D_vec;
  if (!nh.getParam("D",D_vec)) throw std::runtime_error("Failed to load param \"D\"...\n");
  D = arma::diagmat(arma::vec(D_vec));

  if (!nh.getParam("mo",mo)) throw std::runtime_error("Failed to load param \"mo\"...\n");
  std::vector<double> Jo_vec;
  if (!nh.getParam("Jo",Jo_vec)) throw std::runtime_error("Failed to load param \"Jo\"...\n");
  Jo_o = arma::diagmat(arma::vec(Jo_vec));
  Mo_o = arma::diagmat(arma::vec({mo, mo, mo, 0,0,0}));
  Mo_o.submat(3,3,5,5) = Jo_o;

  if (!nh.getParam("Ts",Ts)) throw std::runtime_error("Failed to load param \"Ts\"...\n");
  if (!nh.getParam("ctrl_cycle",ctrl_cycle)) throw std::runtime_error("Failed to load param \"ctrl_cycle\"...\n");

  if (!nh.getParam("use_lwr_dynamics",use_lwr_dynamics)) use_lwr_dynamics=false;

  u = arma::vec().zeros(6);

  F_rh = arma::vec().zeros(6);
  F_lh = arma::vec().zeros(6);

  setLHandleWrenchReadFun([](){ return arma::vec().zeros(6); } );
  setRHandleWrenchReadFun([](){ return arma::vec().zeros(6); } );

  // Ji = Jo + mo*( obj_.p*obj_.p.t() - arma::dot(obj_.p,obj_.p)*arma::mat().eye(3,3) );
  //
  // Mi = Mo;
  // Mi.submat(3,3,5,5) = Ji;

  double pub_rate_ms=33;
  state_pub.reset(new robo_::RobotStatePublisher(robot_desc_param, base_ee_chain->joint_names, std::bind(&Robot::getJointsPosition, this), pub_rate_ms));

  assignJointsPosition(arma::vec().zeros(7));
}

Robot::~Robot()
{
  is_running = false;
  sim_sem.notify();
  state_pub->stop();
}

void Robot::startSim()
{
  std::thread sim_thread = std::thread(&Robot::simulationLoop, this);
  int err_code = thr_::setThreadPriority(sim_thread, SCHED_FIFO, 99);
  if (err_code) std::cerr << "[lwr4p_::Robot::Robot]: Failed to set thread priority! Reason:\n" << thr_::setThreadPriorErrMsg(err_code) << "\n";
  sim_thread.detach();
}

void Robot::waitNextCycle() const
{
  int cycle = 0;
  while (cycle < ctrl_cycle)
  {
    sim_sem.wait();
    cycle++;
  }
}

void Robot::publishState(bool set)
{
  if (set) state_pub->start();
  else state_pub->stop();
}


void Robot::assignJointsPosition(const arma::vec &j_pos)
{
  joint_pos = j_pos;

  arma::mat Tp = base_ee_chain->getTaskPose(joint_pos);

  p = Tp.submat(0,3,2,3);
  Q = math_::rotm2quat(Tp.submat(0,0,2,2));

  dp = ddp = arma::vec().zeros(3);
  vRot = dvRot = arma::vec().zeros(3);
}

void Robot::simulationLoop()
{
  Timer timer;
  unsigned long cycle = Ts*1e9;

  unsigned long long count = 0;

  arma::vec Fo = arma::join_vert(mo*g_ , arma::vec().zeros(3));
  arma::vec F_h1(6);
  arma::vec F_h2(6);

  arma::mat q_prev = joint_pos;
  arma::mat q_dot = arma::vec().zeros(7);

  while (is_running)
  {
    count++;

    timer.start();

    // solve dynamics
    arma::vec V = arma::join_vert(dp, vRot);

    arma::vec Ci = arma::vec().zeros(6,1);
    // Ci.subvec(3,5) = arma::cross(vRot, Ji*vRot);

    arma::mat R_br = math_::quat2rotm(Q); // base_ee_chain->getTaskRotm(getJointsPosition());
    arma::mat R_ro = obj_.R;

    arma::mat R_bo = R_br*R_ro;

    arma::vec r_rh1 = R_br*lh_.p;
    arma::mat G_rh1 = wrenchMat(r_rh1);

    arma::vec r_rh2 = R_br*rh_.p;
    arma::mat G_rh2 = wrenchMat(r_rh2);

    arma::vec r_ro = R_br*obj_.p;

    arma::mat G_ro = wrenchMat(r_ro);
    arma::mat Gamma_or = twistMat(-r_ro);

    arma::mat Jo = R_bo * Jo_o * R_bo.t();

    arma::mat Mo = Mo_o; // Mo'
    Mo.submat(3,3,5,5) = Jo;
    arma::mat Mo2 = Gamma_or.t() * Mo * Gamma_or;

    arma::vec temp(6);
    temp.subvec(0,2) = arma::dot(vRot,vRot)*r_ro - arma::dot(vRot,r_ro)*vRot; // arma::cross(vRot, arma::cross(r_ro,vRot) );
    temp.subvec(3,5) = arma::cross(vRot, Jo*vRot);
    arma::mat Co2 = G_ro*Mo*temp;

    F_h1 = get_lh_wrench_(); //F_lh;
    F_h2 = get_rh_wrench_(); //F_rh;

    arma::mat Mr;
    arma::mat Cr;

    if (use_lwr_dynamics)
    {
      arma::mat Jr = base_ee_chain->getJacobian(joint_pos);
      arma::mat Mq = lwr4p_inertia(joint_pos);
      arma::mat Cq = lwr4p_coriolis(joint_pos, q_dot);
      arma::mat inv_Mq = arma::inv(Mq);
      Mr = arma::inv(Jr*inv_Mq*Jr.t());
      Cr = Mr*Jr*inv_Mq*(Cq*q_dot + lwr4p_friction(q_dot) ); // - M_r * Jr_dot * q_dot
    }
    else
    {
      Mr = M;
      Cr = D*V;
    }

    arma::vec dV = solve( Mr + Mo2, ( - Co2 - Cr + u + G_ro*Fo + G_rh1*F_h1 + G_rh2*F_h2 ) , arma::solve_opts::likely_sympd );
    ddp = dV.subvec(0,2);
    dvRot = dV.subvec(3,5);

    // numerical integration
    p += dp*Ts;
    Q = math_::quatProd(math_::quatExp(vRot*Ts),Q);
    dp += ddp*Ts;
    vRot += dvRot*Ts;

    // update robot joints pos;
    // arma::mat J = base_ee_chain->getJacobian(getJointsPosition());
    // arma::vec p_current = base_ee_chain->getTaskPosition(getJointsPosition());
    // arma::vec Q_current = base_ee_chain->getTaskQuat(getJointsPosition());
    // arma::vec V_click = V;
    // V_click.subvec(0,2) += 2*(p - p_current);
    // V_click.subvec(3,5) += 0.5*math_::quatLog(math_::quatDiff(Q, Q_current));
    // arma::vec q_dot = arma::solve(J, V_click);
    // joint_pos += q_dot*Ts;

    arma::mat Pose = arma::mat().eye(4,4);
    Pose.submat(0,3,2,3) = p;
    Pose.submat(0,0,2,2) = math_::quat2rotm(Q);

    bool found_sol = false;
    joint_pos = base_ee_chain->getJointsPosition(Pose, getJointsPosition(), &found_sol);
    if (!found_sol) std::cerr << "[Robot::simulationLoop]: Failed to find inverse solution...\n";

    q_dot = (joint_pos - q_prev)/Ts;
    q_prev = joint_pos;

    // clock sync
    unsigned long tc = timer.elapsedNanoSec();
    if (tc < cycle) std::this_thread::sleep_for(std::chrono::nanoseconds(cycle-tc));
    else
    {
      std::cerr << "Control cycle exceeded! cycle: " << tc*1e-6 << " ms\n";
    }

    sim_sem.notify();
  }
}

}
