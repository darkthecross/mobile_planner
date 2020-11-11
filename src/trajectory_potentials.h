#ifndef TRAJECTORY_POTENTIALS_H
#define TRAJECTORY_POTENTIALS_H

#include <Eigen/Dense>

#include "kinematics.h"

namespace kinematics {

class StatePotentialInterface {
 public:
  virtual double value(const State& s) const = 0;
  virtual Eigen::Vector4d gradient(const State& s) const = 0;
  virtual Eigen::Matrix4d hessian(const State& s) const = 0;
};

class ObjectRepellerPotential : public StatePotentialInterface {
 public:
  ObjectRepellerPotential(double xx, double yy, double r, double eps)
      : x_(xx), y_(yy), r_(r), eps_(eps) {}

  double value(const State& s) const override;
  Eigen::Vector4d gradient(const State& s) const override;
  Eigen::Matrix4d hessian(const State& s) const override;

 private:
  double x_, y_, r_, eps_;
};

class TrajectoryOptimizer {
 public:
  TrajectoryOptimizer(const Trajectory& init_trajectory)
      : init_trajectory_(init_trajectory),
        num_states_(init_trajectory.x().size()),
        timestep_(init_trajectory.dt()),
        robot_params_(init_trajectory.robot_params()) {}

  void AddObject(const ObjectRepellerPotential& potential) {
    object_repeller_potentials_.push_back(potential);
  }

  void SetStateGains(Eigen::Vector4d state_gains) {
    state_gains_ = state_gains;
  }

  void SetControlGains(Eigen::Vector2d control_gains) {
    control_gains_ = control_gains;
  }

  Trajectory RunOptIteration(const kinematics::Trajectory& traj);

  Trajectory Optimize();

 private:
  Eigen::Matrix4d dfdx(const State& x, const Control& u);

  Eigen::MatrixXd dfdu(const State& x, const Control& u);

  const Trajectory init_trajectory_;
  const int num_states_;
  double timestep_;
  RobotParams robot_params_;

  // State gains.
  Eigen::Vector4d state_gains_;
  // diag(R).
  Eigen::Vector2d control_gains_;
  // Object repellers.
  std::vector<ObjectRepellerPotential> object_repeller_potentials_;
};

}  // namespace kinematics

#endif