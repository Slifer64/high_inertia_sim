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
  Ur_Wrapper();
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

private:

  //arma::vec transfromPose(const arma::vec &pose, int from_robot_ind, int to_robot_ind)

  arma::vec p_r1_r2;
  arma::vec R_r1_r2;

  double Ts;

  arma::vec applyFextDeadZone(const arma::vec &F_ext) const;

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
