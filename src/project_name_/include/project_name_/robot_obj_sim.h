#ifndef ROBOT_OBJ_SIM_H
#define ROBOT_OBJ_SIM_H

#include <cstdlib>
#include <vector>
#include <memory>
#include <cstring>

#include <armadillo>

#include <robo_lib/kinematic_chain.h>
#include <robo_lib/robot_state_publisher.h>
#include <math_lib/math_lib.h>

#include <project_name_/utils.h>
#include <project_name_/ur_wrapper.h>
#include <lwr4p/lwr4p_dynamics.h>

using namespace as64_;


enum DampAdaptMethod
{
  FORCE_ADAPT = 0,
  VEL_ADAPT = 1,
  POWER_ADAPT = 2
};

struct RPoint
{
  RPoint(const arma::mat &Tf=arma::mat().eye(4,4))
  {
    p = Tf.submat(0,3,2,3);
    R = Tf.submat(0,0,2,2);
    Q = math_::rotm2quat(R);
  }

  arma::mat getTransformFromBase() const
  {
    arma::mat T = arma::mat().eye(4,4);
    T.submat(0,0,2,2) = R;
    T.submat(0,3,2,3) = p;
    return T;
  }

  arma::vec p; // position
  arma::mat R; // rotation matrix
  arma::vec Q; // orientation as unit quat
};

class RobotObjSim
{
public:
  RobotObjSim(std::string robot_desc_param);
  ~RobotObjSim();

  void publishState(bool set=true);

  void waitNextCycle() const;

  arma::vec getJointsPosition() const { return joint_pos; }

  arma::mat getTaskRotm() const { return math_::quat2rotm(Q); }

  void assignJointsPosition(const arma::vec &j_pos);

  void setInputWrench(const arma::vec &u_cmd) { u = u_cmd; }

  // void setLeftHandleWrench(const arma::vec &wrench) { F_lh = wrench; }
  // void setRightHandleWrench(const arma::vec &wrench) { F_rh = wrench; }

  double getObjectMass() const { return mo; }

  void startSim();
  void stopSim();

  arma::vec get_pos_ee_obj() const { return obj_.p; }

  void setLHandleWrenchReadFun(std::function<arma::vec()> get_wrench_fun) { get_lh_wrench_ = get_wrench_fun; }
  void setRHandleWrenchReadFun(std::function<arma::vec()> get_wrench_fun) { get_rh_wrench_ = get_wrench_fun; }

  arma::mat get_transform_lh_rh() const { return arma::inv(lh_.getTransformFromBase()) * rh_.getTransformFromBase(); }

  std::shared_ptr<Ur_Wrapper> ur_wrap;

  bool isSimRunning() const { return run_sim_; }

  void setCtrlSignalFun(std::function<arma::vec()> ctrl_fun) { get_ctrl_signal_ = ctrl_fun; }

  bool log_data_;
  int log_cycle;

  void clearLogData();
  void saveLogData(const std::string &filename);

  void gotoStartPose();

  bool print_to_console;

  std::function<arma::vec()> get_lh_wrench_;
  std::function<arma::vec()> get_rh_wrench_;

private:

  Timer timer;

  enum DampAdaptMethod damp_adapt_method;

  arma::rowvec log_Time;
  arma::mat log_Vh1;
  arma::mat log_Vh2;
  arma::mat log_Fh1;
  arma::mat log_Fh2;
  arma::mat log_Damp;

  std::function<arma::vec()> get_ctrl_signal_;

  std::function<bool(arma::vec)> send_feedback;

  std::function<void()> wait_next_cycle;

  bool loadSimParams();

  arma::mat getBaseEeTransform()
  {
    arma::mat T = arma::mat().eye(4,4);
    T.submat(0,0,2,2) = math_::quat2rotm(Q);
    T.submat(0,3,2,3) = p;
    return T;
  }

  arma::mat getBaseEeRotm()
  {
    // base_ee_chain->getTaskRotm(getJointsPosition());
    return math_::quat2rotm(Q);
  }

  arma::mat getBaseObjRotm()
  {
    return getBaseEeRotm()*obj_.R;
  }

  arma::mat getBaseLeftHandleRotm()
  {
    return getBaseEeRotm()*lh_.R;
  }

  arma::mat getBaseRightHandleRotm()
  {
    return getBaseEeRotm()*rh_.R;
  }

  arma::mat wrenchMat(const arma::vec &r)
  {
    arma::mat G = arma::mat().eye(6,6);
    G.submat(3,0,5,2) = math_::vec2ssMat(r);
    return G;
  }

  arma::mat twistMat(const arma::vec &r)
  {
    arma::mat Gamma = arma::mat().eye(6,6);
    Gamma.submat(0,3,2,5) = math_::vec2ssMat(r);
    return Gamma;
  }

  RPoint obj_;
  RPoint lh_, rh_;

  bool run_sim_;

  arma::vec lwr4p_q0;

  void simulationLoop();

  arma::vec joint_pos;

  std::shared_ptr<robo_::KinematicChain> base_ee_chain;
  std::shared_ptr<robo_::RobotStatePublisher> state_pub;

  mutable Semaphore sim_cycle_sem;
  mutable Semaphore sim_stopped_sem;

  bool use_lwr_dynamics;

  arma::vec p, Q;
  arma::vec dp, vRot;
  arma::vec ddp, dvRot;
  double Ts;

  int ctrl_cycle;

  int ctrl_delay;

  // arma::mat M; // robot inertia matrix
  // arma::mat D; // robot damping
  arma::vec u; // input wrench

  arma::mat M;
  arma::mat D_min;
  arma::mat D_max;
  double a_Fp;
  double a_Fo;

  bool use_force;

  arma::vec F_rh;
  arma::vec F_lh;

  arma::mat Mo_o; // object mass-inertia matrix
  double mo; // object mass
  arma::mat Jo_o; // object inertia matrix of CoM expressed in the object local frame

  arma::vec g_; // gravity vector in the world frame

};


#endif // ROBOT_OBJ_SIM_H
