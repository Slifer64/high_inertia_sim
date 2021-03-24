#ifndef UR_WRAPPER_H
#define UR_WRAPPER_H

#include <cstdlib>
#include <vector>
#include <string>
#include <memory>

#include <ros/ros.h>

#include <ur_robot/robot.h>

class Ur_Wrapper
{
public:
  Ur_Wrapper(const arma::mat &T_lh_rh, const arma::mat &T_b_h1, const arma::mat &T_b_h2);
  ~Ur_Wrapper();

  void setCartVelCtrl();

  void biasFTSensors();

  void moveToStartPose();

  void setJointsTrajectory(const arma::vec &qT, ur_::Robot *robot_);

  void setCartTrajectory(const arma::vec &poseT, ur_::Robot *robot_);

  arma::vec getWrench(int robot_ind);

  arma::mat getRotm(int robot_ind) const;

  void setWrenchRotTransform(const arma::mat &R, int robot_ind);

  arma::vec getTaskPose(int robot_ind) const;

  void setVelocity(const arma::vec &V);

  void waitNextCycle();

private:

  std::vector<arma::vec> pose, Vel;

  arma::mat T_b1_b2;
  arma::mat T_h1_h2;

  arma::mat T_b_b1, R_b1_b;
  arma::mat T_b_b2, R_b2_b;

  arma::mat T_b1_h1_0;
  arma::mat T_b2_h2_0;

  double Ts;

  arma::vec applyFextDeadZone(const arma::vec &F_ext) const;

  arma::vec calcVelocity_with_click(const arma::vec &V, int i);

  double a_f;
  arma::vec Fext_deadzone;
  arma::vec Fext_prev;

  std::vector<arma::mat> R_;

  // void setRobotIdle() override { robot->setNormalMode(); }
  //
  // arma::vec getTaskWrenchFromRobot() const override
  // { return robot->getTcpWrench(); }

  std::vector<std::shared_ptr<ur_::Robot>> robot;

  // thr_::Semaphore mode_change;
  //
  // bool run_;
};


#endif // UR_WRAPPER_H
