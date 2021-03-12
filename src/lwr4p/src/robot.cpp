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

  obj_ = IPoint( robo_::KinematicChain(robot_desc_param, "ee_link", "object_link").getTaskPose(arma::vec()) );
  lh_ = IPoint( robo_::KinematicChain(robot_desc_param, "ee_link", "left_handle_handle").getTaskPose(arma::vec()) );
  rh_ = IPoint( robo_::KinematicChain(robot_desc_param, "ee_link", "right_handle_handle").getTaskPose(arma::vec()) );

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
  Jo = arma::diagmat(arma::vec(Jo_vec));
  Mo = arma::diagmat(arma::vec({mo, mo, mo, 0,0,0}));
  Mo.submat(3,3,5,5) = Jo;

  if (!nh.getParam("Ts",Ts)) throw std::runtime_error("Failed to load param \"Ts\"...\n");
  if (!nh.getParam("ctrl_cycle",ctrl_cycle)) throw std::runtime_error("Failed to load param \"ctrl_cycle\"...\n");

  u = arma::vec().zeros(6);

  F_rh = arma::vec().zeros(6);
  F_lh = arma::vec().zeros(6);
  F_o = arma::vec().zeros(6);

  Ji = Jo + mo*( obj_.p*obj_.p.t() - arma::dot(obj_.p,obj_.p)*arma::mat().eye(3,3) );

  Mi = Mo;
  Mi.submat(3,3,5,5) = Ji;

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

  while (is_running)
  {
    count++;

    timer.start();

    // solve dynamics
    arma::vec V = arma::join_vert(dp, vRot);

    arma::vec Ci = arma::vec().zeros(6,1);
    Ci.subvec(3,5) = arma::cross(vRot, Ji*vRot);

    arma::mat R_be = math_::quat2rotm(Q); // base_ee_chain->getTaskRotm(getJointsPosition());

    F_o = obj_.graspMat(R_be) * arma::join_vert(mo*g_ , arma::vec().zeros(3));

    // F_lh = {0, 0, 0, 0, 0, 0.05};
    // F_rh = {0, 0, 0, 0, 0, 0};

    arma::vec F_lh2 = lh_.graspMat(R_be) * F_lh;
    arma::vec F_rh2 = rh_.graspMat(R_be) * F_rh;

    // F_lh = {0, 0, 2, 0, 0, 0};
    // std::cerr << (lh_.G*F_lh).t() << "\n";
    // std::cerr << (rh_.G*F_rh).t() << "\n";
    // std::cerr << (F_o).t() << "\n";
    // std::cerr << (R_be*obj_.p).t() << "\n";
    // std::cerr << (obj_.p).t() << "\n";
    //
    // std::cerr << "G = \n" << obj_.graspMat(R_be) << "\n";

    // if (count == 30) exit(-1);


    arma::vec dV = solve( M + Mi, ( - Ci - D*V + u + F_o + F_lh2 + F_rh2 ) , arma::solve_opts::likely_sympd );
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

    // clock sync
    unsigned long tc = timer.elapsedNanoSec();
    if (tc < cycle) std::this_thread::sleep_for(std::chrono::nanoseconds(cycle-tc));
    else
    {
      std::cerr << "Control cycle exceeded!!!\n";
    }

    sim_sem.notify();
  }
}

}
