#include <cstdlib>
#include <vector>
#include <memory>
#include <cstring>

#include <armadillo>

#include <robo_lib/kinematic_chain.h>
#include <robo_lib/robot_state_publisher.h>

#include <lwr4p/utils.h>
#include <math_lib/math_lib.h>

using namespace as64_;

namespace lwr4p_
{

struct IPoint
{
  IPoint(const arma::mat &Tf=arma::mat().eye(4,4))
  {
    p = Tf.submat(0,3,2,3);
    R = Tf.submat(0,0,2,2);
    Q = math_::rotm2quat(R);
  }

  arma::mat graspMat(const arma::mat &R_base_i)
  {
    arma::mat G = arma::mat().eye(6,6);
    G.submat(3,0,5,2) = math_::vec2ssMat(R_base_i*p);
    return G;
  }

  arma::vec p; // position
  arma::mat R; // rotation matrix
  arma::vec Q; // orientation as unit quat
};

class Robot
{
public:
  Robot(std::string robot_desc_param);
  ~Robot();

  void publishState(bool set=true);

  void waitNextCycle() const;

  arma::vec getJointsPosition() const { return joint_pos; }

  arma::mat getTaskRotm() const { return math_::quat2rotm(Q); }

  void assignJointsPosition(const arma::vec &j_pos);

  void setInputWrench(const arma::vec &u_cmd) { u = u_cmd; }
  void setLeftHandleWrench(const arma::vec &wrench) { F_lh = wrench; }
  void setRightHandleWrench(const arma::vec &wrench) { F_rh = wrench; }

  double getObjectMass() const { return mo; }

  void startSim();

  arma::vec get_pos_ee_obj() const { return obj_.p; }

private:

  IPoint obj_;
  IPoint lh_, rh_;

  bool is_running;

  void simulationLoop();

  arma::vec joint_pos;

  std::shared_ptr<robo_::KinematicChain> base_ee_chain;
  std::shared_ptr<robo_::RobotStatePublisher> state_pub;

  mutable Semaphore sim_sem;

  arma::vec p, Q;
  arma::vec dp, vRot;
  arma::vec ddp, dvRot;
  double Ts;

  int ctrl_cycle;

  arma::mat M; // robot inertia matrix
  arma::mat D; // robot damping
  arma::vec u; // input wrench

  arma::vec F_rh;
  arma::vec F_lh;
  arma::vec F_o;

  arma::mat Mo; // object mass-inertia matrix
  double mo; // object mass
  arma::mat Jo; // object inertia matrix of CoM expressed in the object local frame
  arma::mat Ji; // inertia matrix of point i expressed in world frame
  arma::mat Mi;

  arma::vec g_; // gravity vector in the world frame

};


}
