#include <project_name_/robot_obj_sim.h>

#include <thread_lib/thread_lib.h>
#include <io_lib/print_utils.h>
#include <io_lib/file_io.h>

#include <exception>
#include <queue>

#include <ros/ros.h>
#include <ros/package.h>

#include <io_lib/xml_parser.h>

// #define CHECK_CTRL_CYCLE_DELAYS

using namespace as64_;

#define RobotObjSim_fun_ std::string("[RobotObjSim::") + __func__ + "]: "

RobotObjSim::RobotObjSim(std::string robot_desc_param)
{
  ros::NodeHandle nh("~");

  run_sim_ = false;
  log_data_ = false;

  log_cycle = 2;

  base_ee_chain.reset(new robo_::KinematicChain(robot_desc_param, "base_link", "ee_link"));

  obj_ = RPoint( robo_::KinematicChain(robot_desc_param, "ee_link", "object_link").getTaskPose(arma::vec()) );
  lh_ = RPoint( robo_::KinematicChain(robot_desc_param, "ee_link", "left_handle_frame").getTaskPose(arma::vec()) );
  rh_ = RPoint( robo_::KinematicChain(robot_desc_param, "ee_link", "right_handle_frame").getTaskPose(arma::vec()) );

  g_ = arma::vec({0, 0, -9.81});

  // std::vector<double> M_vec;
  // if (!nh.getParam("M",M_vec)) throw std::runtime_error("Failed to load param \"M\"...\n");
  // M = arma::diagmat(arma::vec(M_vec));
  // std::vector<double> D_vec;
  // if (!nh.getParam("D",D_vec)) throw std::runtime_error("Failed to load param \"D\"...\n");
  // D = arma::diagmat(arma::vec(D_vec));
  //
  // if (!nh.getParam("mo",mo)) throw std::runtime_error("Failed to load param \"mo\"...\n");
  // std::vector<double> Jo_vec;
  // if (!nh.getParam("Jo",Jo_vec)) throw std::runtime_error("Failed to load param \"Jo\"...\n");
  // Jo_o = arma::diagmat(arma::vec(Jo_vec));
  // Mo_o = arma::diagmat(arma::vec({mo, mo, mo, 0,0,0}));
  // Mo_o.submat(3,3,5,5) = Jo_o;

  if (!nh.getParam("Ts",Ts)) throw std::runtime_error("Failed to load param \"Ts\"...\n");
  if (!nh.getParam("ctrl_cycle",ctrl_cycle)) throw std::runtime_error("Failed to load param \"ctrl_cycle\"...\n");

  if (!nh.getParam("use_lwr_dynamics",use_lwr_dynamics)) use_lwr_dynamics=false;

  u = arma::vec().zeros(6);

  F_rh = arma::vec().zeros(6);
  F_lh = arma::vec().zeros(6);

  setLHandleWrenchReadFun([](){ return arma::vec().zeros(6); } );
  setRHandleWrenchReadFun([](){ return arma::vec().zeros(6); } );
  send_feedback = [](const arma::vec &){ return true; };

  wait_next_cycle = [this]()
  {
    static bool initialized  = false;
    static Timer timer; 
    static double cycle_ns = Ts*1e9;

    if (!initialized)
    {
      initialized = true;
      timer.start();
    }

    double elaps_t = timer.elapsedNanoSec();

    unsigned long long sleep_t = (cycle_ns - elaps_t);
    if (sleep_t > 0) std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_t));
    timer.start();
  };


  // Publish simulated lwr robot state
  double pub_rate_ms=33;
  state_pub.reset(new robo_::RobotStatePublisher(robot_desc_param, base_ee_chain->joint_names, std::bind(&RobotObjSim::getJointsPosition, this), pub_rate_ms));

  std::vector<double> q0;
  if (!nh.getParam("q0",q0)) throw std::runtime_error("Failed to load param \"q0\"...\n");
  lwr4p_q0 = q0;
  assignJointsPosition(lwr4p_q0);

  arma::mat T_b_h1 = robo_::KinematicChain(robot_desc_param, "base_link", "left_handle_frame").getTaskPose(joint_pos);
  arma::mat T_b_h2 = robo_::KinematicChain(robot_desc_param, "base_link", "right_handle_frame").getTaskPose(joint_pos);

  bool use_ur_robot;
  if (!nh.getParam("use_ur_robot",use_ur_robot)) use_ur_robot=false;

  if (use_ur_robot)
  {
    ur_wrap.reset(new Ur_Wrapper(get_transform_lh_rh(), T_b_h1, T_b_h2));

    // arma::mat R_b_lh = getBaseLeftHandleRotm();
    // arma::mat R_b1_lh = ur_wrap->getRotm(0);
    // arma::mat R_b_b1 = R_b_lh * R_b1_lh.t();
    // ur_wrap->setWrenchRotTransform(R_b_b1, 0);
    //
    // arma::mat R_b_rh = getBaseRightHandleRotm();
    // arma::mat R_b2_rh = ur_wrap->getRotm(1);
    // arma::mat R_b_b2 = R_b_rh * R_b2_rh.t();
    // ur_wrap->setWrenchRotTransform(R_b_b2, 1);

    setLHandleWrenchReadFun([this](){ return ur_wrap->getWrench(0); } );
    setRHandleWrenchReadFun([this](){ return ur_wrap->getWrench(1); } );

    send_feedback = [this](const arma::vec &V){ return this->ur_wrap->setVelocity(V); };

    wait_next_cycle = [this](){ this->ur_wrap->waitNextCycle(); };
  }
}

RobotObjSim::~RobotObjSim()
{
  run_sim_ = false;
  sim_cycle_sem.notify();
  sim_stopped_sem.notify();
  state_pub->stop();
}

void RobotObjSim::gotoStartPose()
{
  assignJointsPosition(lwr4p_q0);
  ur_wrap->moveToStartPose();
}

void RobotObjSim::startSim()
{
  if (run_sim_) return;
  if (!loadSimParams()) return;

  std::thread sim_thread = std::thread(&RobotObjSim::simulationLoop, this);
  int err_code = thr_::setThreadPriority(sim_thread, SCHED_FIFO, 99);
  if (err_code) std::cerr << "[RobotObjSim::Robot]: Failed to set thread priority! Reason:\n" << thr_::setThreadPriorErrMsg(err_code) << "\n";
  sim_thread.detach();
}

void RobotObjSim::stopSim()
{
  if (run_sim_)
  {
    run_sim_ = false;
    if (!sim_stopped_sem.wait_until(500)) throw std::runtime_error("[RobotObjSim::stopSim]: Timeout on waiting for simulation to stop...\n");
  }
}

void RobotObjSim::waitNextCycle() const
{
  int cycle = 0;
  while (cycle < ctrl_cycle)
  {
    sim_cycle_sem.wait();
    cycle++;
  }
}

void RobotObjSim::publishState(bool set)
{
  if (set) state_pub->start();
  else state_pub->stop();
}


void RobotObjSim::assignJointsPosition(const arma::vec &j_pos)
{
  joint_pos = j_pos;

  arma::mat Tp = base_ee_chain->getTaskPose(joint_pos);

  p = Tp.submat(0,3,2,3);
  Q = math_::rotm2quat(Tp.submat(0,0,2,2));

  dp = ddp = arma::vec().zeros(3);
  vRot = dvRot = arma::vec().zeros(3);
}

void RobotObjSim::simulationLoop()
{
  unsigned long cycle_ns = Ts*1e9;

  unsigned long long count = 0;

  arma::vec Fo = arma::join_vert(mo*g_ , arma::vec().zeros(3));
  arma::vec F_h1(6);
  arma::vec F_h2(6);

  arma::mat q_prev = joint_pos;
  arma::mat q_dot = arma::vec().zeros(7);

  wait_next_cycle();

  #ifdef CHECK_CTRL_CYCLE_DELAYS
    Timer timer;
  #endif

  arma::mat D2 = D_max;

  run_sim_ = true;

  bool log_on = log_data_;
  if (log_on) clearLogData();

  double t = 0;

  // assignJointsPosition(getJointsPosition());
  dp = ddp = arma::vec().zeros(3);
  vRot = dvRot = arma::vec().zeros(3);

  arma::vec p_cur = p;
  arma::vec Q_cur = Q;

  arma::mat D_plus = D_max - D_min;

  double Fp_norm = 0;
  double Fo_norm = 0;

  print_to_console = false;

  int buff_size = 1;
  std::queue<arma::vec> cmd_buffer;
  for (int i=0; i<buff_size; i++) cmd_buffer.push(arma::vec().zeros(6,1));

  while (run_sim_)
  {
    count++;

    #ifdef CHECK_CTRL_CYCLE_DELAYS
      timer.start();
    #endif

    // solve dynamics
    arma::vec V = arma::join_vert(dp, vRot);

    arma::vec Ci = arma::vec().zeros(6,1);
    // Ci.subvec(3,5) = arma::cross(vRot, Ji*vRot);

    arma::mat R_br = getBaseEeRotm();
    arma::mat R_ro = obj_.R;

    arma::mat R_bo = R_br*R_ro;

    arma::vec r_rh1 = R_br*lh_.p;
    arma::mat G_rh1 = wrenchMat(r_rh1);

    arma::vec r_rh2 = R_br*rh_.p;
    arma::mat G_rh2 = wrenchMat(r_rh2);

    arma::vec r_ro = R_br*obj_.p; // express the vector from 'r' to 'o' w.r.t. the lwr-base frame 'b'.

    // Grasp and twist matrices, expressed in the lwr-base frame 'b'.
    arma::mat G_ro = wrenchMat(r_ro); // grasp matrix from lwr end-effector 'r' to the object CoM 'o'
    arma::mat Gamma_or = twistMat(-r_ro);

    // Here, Mo_o and Jo are a portion of the actual plants inertia.
    // They are expressed in the frame defined by the object's CoM and principal axes.
    // We then tranfer the inertia to the lwr robot's end-effector.
    arma::mat Mo = Mo_o; // Mo'
    arma::mat Jo = R_bo * Jo_o * R_bo.t();
    Mo.submat(3,3,5,5) = Jo;
    arma::mat Mo2 = Gamma_or.t() * Mo * Gamma_or;

    // Transfer the object's coriolis in the lwr robot's end-effector.
    arma::vec temp(6);
    temp.subvec(0,2) = mo*( arma::dot(vRot,vRot)*r_ro - arma::dot(vRot,r_ro)*vRot ); // arma::cross(vRot, arma::cross(r_ro,vRot) );
    temp.subvec(3,5) = arma::cross(vRot, Jo*vRot);
    arma::mat Co2 = G_ro*temp;

    F_h1 = get_lh_wrench_(); // wrench of the left ur robot at its handle 'h1', expressed in the lwr-base frame.
    F_h2 = get_rh_wrench_(); // wrench of the right ur robot at its handle 'h2', expressed in the lwr-base frame.

    // Tranfer the wrenches from the ur-handles to the lwr end-effector. All quantities are expressed in the lwr base frame.
    arma::vec F_rh1 = G_rh1*F_h1; // tranfer the wrench from the the handle 'h1' of the left robot, to the end-effector 'r' of the lwr robot
    arma::vec F_rh2 = G_rh2*F_h2; // tranfer the wrench from the the handle 'h2' of the right robot, to the end-effector 'r' of the lwr robot

    // ===========  print some values in the terminal  ===========
    if (count == 200)
    {
      count = 0;
      std::cerr << "=========================\n";
      printf("F_h1 :  %6.3f  ,  %6.3f  ,  %6.3f  |  %6.3f  ,  %6.3f  ,  %6.3f\n", F_h1(0),F_h1(1),F_h1(2),F_h1(3),F_h1(4),F_h1(5));
      printf("F_h2 :  %6.3f  ,  %6.3f  ,  %6.3f  |  %6.3f  ,  %6.3f  ,  %6.3f\n", F_h2(0),F_h2(1),F_h2(2),F_h2(3),F_h2(4),F_h2(5));
      printf("Vel_r:  %6.3f  ,  %6.3f  ,  %6.3f  |  %6.3f  ,  %6.3f  ,  %6.3f\n", V(0),V(1),V(2),V(3),V(4),V(5));
      printf("Damp :  %6.3f  ,  %6.3f  ,  %6.3f  |  %6.3f  ,  %6.3f  ,  %6.3f\n", D2(0,0),D2(1,1),D2(2,2),D2(3,3),D2(4,4),D2(5,5));
    }

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

      arma::vec Fp = F_rh1.subvec(0,2) + F_rh2.subvec(0,2);
      arma::vec Fo = F_rh1.subvec(3,5) + F_rh2.subvec(3,5);
      // Fp_norm = 0.9*Fp_norm  + 0.1*arma::norm(Fp);
      // Fo_norm = 0.9*Fo_norm  + 0.1*arma::norm(Fo);

      // Fp_norm = std::max( arma::dot(V.subvec(0,2), Fp), 0.0 );
      // Fo_norm = std::max( arma::dot(V.subvec(3,5), Fo), 0.0 );

      D2 = D_min;

      double vp;
      double vo;

      if (damp_adapt_method == FORCE_ADAPT)
      {
        vp = arma::norm(Fp);
        vo = arma::norm(Fo);
      }
      else if (damp_adapt_method == VEL_ADAPT)
      {
        vp = arma::norm(V.subvec(0,2));
        vo = arma::norm(V.subvec(3,5));
      }
      else // (damp_adapt_method == POWER_ADAPT)
      {
        vp = std::max( arma::dot(V.subvec(0,2), Fp) + arma::dot(V.subvec(3,5), Fo), 0.0 );
        vo = vp;
      }

      D2.submat(0,0,2,2) += D_plus.submat(0,0,2,2)*exp(-a_Fp*vp);
      D2.submat(3,3,5,5) += D_plus.submat(3,3,5,5)*exp(-a_Fo*vo);

      Cr = D2*V;

      if (damp_adapt_method == VEL_ADAPT)
      {
        double Dp_ = std::max( D_min(0,0), D_max(0,0)*std::exp(-a_Fp*vp) );
        double Do_ = std::max( D_min(3,3), D_max(3,3)*std::exp(-a_Fo*vo) );
        arma::vec V_temp = V;
        V_temp.subvec(0,2) *= Dp_;
        V_temp.subvec(3,5) *= Do_;
        Cr = V_temp;
      }

    }

    // u = get_ctrl_signal_();
    // arma::vec dV = arma::solve( Mr + Mo2, ( - Co2 - Cr + u + G_ro*Fo + F_rh1 + F_rh2 ) , arma::solve_opts::likely_sympd );

    // dynamics of the lwr end-effector 'r' expressed in the lwr-base frame 'b'.
    arma::vec dV = arma::solve( Mr + Mo2, ( - Co2 - Cr + F_rh1 + F_rh2 ) , arma::solve_opts::likely_sympd );

    ddp = dV.subvec(0,2);
    dvRot = dV.subvec(3,5);

    // numerical integration
    t = t + Ts;
    p += dp*Ts;
    Q = math_::quatProd(math_::quatExp(vRot*Ts),Q);
    dp += ddp*Ts;
    vRot += dvRot*Ts;

    arma::vec V_cmd = arma::join_vert(dp, vRot);
    cmd_buffer.push(V_cmd);
    arma::vec V_cmd_current = cmd_buffer.front();
    cmd_buffer.pop();

    arma::vec V_h1 = twistMat(-r_rh1)*V_cmd_current;
    arma::vec V_h2 = twistMat(-r_rh2)*V_cmd_current;

    if ( !send_feedback(arma::join_vert(V_h1, V_h2)) )
    {
      run_sim_ = false;
      break;
    }

    p_cur += V_cmd_current.subvec(0,2)*Ts;
    Q_cur = math_::quatProd(math_::quatExp(V_cmd_current.subvec(3,5)*Ts),Q_cur);

    // ===========  data logging  ===========
    if (log_on)
    {
      log_Time = arma::join_horiz(log_Time, arma::vec({t}) );
      log_Vh1 = arma::join_horiz(log_Vh1, V_h1 );
      log_Vh2 = arma::join_horiz(log_Vh2, V_h2 );

      log_Fh1 = arma::join_horiz(log_Fh1, F_h1 );
      log_Fh2 = arma::join_horiz(log_Fh2, F_h2 );

      log_Damp = arma::join_horiz(log_Damp, arma::diagvec(D2) );
    }

    // ==================================================================
    // ===========  Update simulated lwr robot visualization  ===========
    // ==================================================================

    // update robot joints pos;
    // arma::mat J = base_ee_chain->getJacobian(getJointsPosition());
    // arma::vec p_current = base_ee_chain->getTaskPosition(getJointsPosition());
    // arma::vec Q_current = base_ee_chain->getTaskQuat(getJointsPosition());
    // arma::vec V_click = V;
    // V_click.subvec(0,2) += 2*(p - p_current);
    // V_click.subvec(3,5) += 0.5*math_::quatLog(math_::quatDiff(Q, Q_current));
    // arma::vec q_dot = arma::solve(J, V_click);
    // joint_pos += q_dot*Ts;

    // Find the inverse kinematic solution for the current pose, stored in 'p', 'Q'
    // assign the solution to 'joint_pos' which is returned by 'getJointsPosition()'
    // which is used in the 'state_pub' to read the current joints and publish the lwr robot states to the 'tf/'
    arma::mat Pose = arma::mat().eye(4,4);
    Pose.submat(0,3,2,3) = p_cur;
    Pose.submat(0,0,2,2) = math_::quat2rotm(Q_cur);
//
    bool found_sol = false;
    joint_pos = base_ee_chain->getJointsPosition(Pose, getJointsPosition(), &found_sol);
    if (!found_sol) std::cerr << "[RobotObjSim::simulationLoop]: Failed to find inverse solution...\n";

    q_dot = (joint_pos - q_prev)/Ts;
    q_prev = joint_pos;

    // --------------------------------------------------------------------

    // clock sync
    wait_next_cycle();

    // check delays
    #ifdef CHECK_CTRL_CYCLE_DELAYS
      double tc = timer.elapsedNanoSec();
      std::cout << "tc = " << tc*1e-6 << " ms\n";
    #endif
    // if (tc > 1.02*cycle_ns) PRINT_WARNING_MSG("Control cycle exceeded! cycle: " + std::to_string(tc*1e-6) + " ms\n");

    sim_cycle_sem.notify();
  }
  sim_stopped_sem.notify();
}

void RobotObjSim::clearLogData()
{
  log_Time.clear();
  log_Vh1.clear();
  log_Vh2.clear();
  log_Fh1.clear();
  log_Fh2.clear();
  log_Damp.clear();
}

void RobotObjSim::saveLogData(const std::string &filename)
{
  if (log_Time.size() == 0)
  {
    PRINT_WARNING_MSG(RobotObjSim_fun_ +"The data are empty...\n");
    return;
  }

  try
  {
    io_::FileIO fid(filename, io_::FileIO::out|io_::FileIO::trunc);
    fid.write("Time_data",log_Time);
    fid.write("Vh1_data",log_Vh1);
    fid.write("Vh2_data",log_Vh2);
    fid.write("Fh1_data",log_Fh1);
    fid.write("Fh2_data",log_Fh2);
    fid.write("Damp_data",log_Damp);
    fid.close();
    PRINT_INFO_MSG(RobotObjSim_fun_ + "Logged data saved successfully!\n");
  }
  catch(std::exception &e)
  {
    PRINT_ERROR_MSG(RobotObjSim_fun_ + e.what() + "\n");
  }
}

bool RobotObjSim::loadSimParams()
{
  try{
    std::string filename = ros::package::getPath("project_name_") + "/config/sim_params.yaml";
    io_::XmlParser parser(filename);

    arma::rowvec Jo_vec, M_vec, D_min_vec, D_max_vec;

    if (!parser.getParam("mo", mo)) throw std::runtime_error("Failed to read param \"mo\"...");
    if (!parser.getParam("Jo", Jo_vec))
    {
      PRINT_WARNING_MSG("Failed to read param \"Jo\"...");
      double obj_w;
      double obj_l;
      double obj_h;
      if (!parser.getParam("obj_w", obj_w)) throw std::runtime_error("Failed to read param \"obj_w\"...");
      if (!parser.getParam("obj_l", obj_l)) throw std::runtime_error("Failed to read param \"obj_l\"...");
      if (!parser.getParam("obj_h", obj_h)) throw std::runtime_error("Failed to read param \"obj_h\"...");
      Jo_vec = (mo/12) * arma::rowvec( { (std::pow(obj_l,2)+std::pow(obj_h,2)), (std::pow(obj_w,2)+std::pow(obj_h,2)), (std::pow(obj_l,2)+std::pow(obj_w,2)) } );
    }

    std::cerr << "Jo = " << Jo_vec << "\n";

    if (!parser.getParam("M", M_vec)) throw std::runtime_error("Failed to read param \"M\"...");
    if (!parser.getParam("D_min", D_min_vec)) throw std::runtime_error("Failed to read param \"D_min\"...");
    if (!parser.getParam("D_max", D_max_vec)) throw std::runtime_error("Failed to read param \"D_max\"...");


    std::string damp_adapt_meth_str;
    if (!parser.getParam("damp_adapt_method", damp_adapt_meth_str)) throw std::runtime_error("Failed to read param \"damp_adapt_method\"...");
    if (damp_adapt_meth_str.compare("force")==0)
    {
      damp_adapt_method = FORCE_ADAPT;
      if (!parser.getParam("a_Dp_f", a_Fp)) throw std::runtime_error("Failed to read param \"a_Dp_f\"...");
      if (!parser.getParam("a_Do_f", a_Fo)) throw std::runtime_error("Failed to read param \"a_Do_f\"...");
    }
    else if (damp_adapt_meth_str.compare("vel")==0)
    {
      damp_adapt_method = VEL_ADAPT;
      if (!parser.getParam("a_Dp_v", a_Fp)) throw std::runtime_error("Failed to read param \"a_Dp_v\"...");
      if (!parser.getParam("a_Do_v", a_Fo)) throw std::runtime_error("Failed to read param \"a_Do_v\"...");
    }
    else if (damp_adapt_meth_str.compare("power")==0)
    {
      damp_adapt_method = POWER_ADAPT;
      if (!parser.getParam("a_Dp_p", a_Fp)) throw std::runtime_error("Failed to read param \"a_Dp_p\"...");
      if (!parser.getParam("a_Do_p", a_Fo)) throw std::runtime_error("Failed to read param \"a_Do_p\"...");
    }
    else throw std::runtime_error("Unsupported damping adaptation method \"" + damp_adapt_meth_str + "\"...");

    std::cerr << "damping adapt params: ";
    if (damp_adapt_method == FORCE_ADAPT) std::cerr << "FORCE_ADAPT , ";
    else if (damp_adapt_method == VEL_ADAPT) std::cerr << "VEL_ADAPT , ";
    else if (damp_adapt_method == POWER_ADAPT) std::cerr << "POWER_ADAPT , ";
    std::cerr << a_Fp << " , " << a_Fo << "\n";

    M = arma::diagmat(M_vec);
    D_min = arma::diagmat(D_min_vec);
    D_max = arma::diagmat(D_max_vec);

    Jo_o = arma::diagmat(Jo_vec);
    Mo_o = arma::diagmat(arma::vec({mo, mo, mo, 0,0,0}));
    Mo_o.submat(3,3,5,5) = Jo_o;

    return true;
  }
  catch(std::exception &e)
  {
    PRINT_ERROR_MSG(RobotObjSim_fun_ + e.what() + "\n");
    return false;
  }
}
