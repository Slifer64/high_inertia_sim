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

/*
  void commandThread() override;

  arma::vec getTaskPosition() const override
  { return robot->getTaskPosition(); }

  arma::mat getTaskRotMat() const override
  { return robot->getTaskRotm() * R_et; }

  arma::vec getTaskOrientation() const override
  { return rotm2quat(this->getTaskRotMat()); }

#include <ur_robot/robot.h>
  arma::vec getJointsPosition() const override
  { return robot->getJointsPosition(); }

  arma::mat getJacobian() const override
  { return robot->getJacobian(); }

  void update() override
  { KRC_tick.wait(); }

  arma::vec getJointPosLowLim() const override
  { return arma::vec(robot->robot_urdf->getJointsPosLowLim())*180/3.14159; }

  arma::vec getJointPosUpperLim() const override
  { return arma::vec(robot->robot_urdf->getJointsPosUpperLim())*180/3.14159; }

  void stop() override;

  void setMode(const Mode &mode) override;

  bool isOk() const override
  { return robot->isOk() && !external_stop_; }

  void setJointsPosition(const arma::vec &jpos) override { jpos_cmd.set(jpos); }
  void setJointsTorque(const arma::vec &jtorq) override { jtorque_cmd.set(jtorq); }
  void setTaskVelocity(const arma::vec &vel) override { cart_vel_cmd.set(vel); }
  bool setJointsTrajectory(const arma::vec &qT, double duration) override;

  std::vector<std::string> getJointNames() const
  { return robot->robot_urdf->getJointsName(); }

  void biasRobotSensor() override
  { robot->biasFtSensor(); }
*/

private:

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
