#ifndef TRAJECTORY_SOLVER_H
#define TRAJECTORY_SOLVER_H

#include <Eigen/Dense>

#include "kinematics_trajectory.h"

namespace kinematics {

class TrajectoryFollower {
 public:
  TrajectoryFollower(Trajectory initial_traj)
      : initial_trajectory_(initial_traj),
        timestep_(initial_traj.dt()),
        robot_params_(initial_traj.robot_params()) {}

  Eigen::Matrix4d dfdx(const State& x, const Control& u);

  Eigen::MatrixXd dfdu(const State& x, const Control& u);

  Trajectory Optimize(Eigen::Matrix4d Q, Eigen::Matrix2d R);

 private:
  Trajectory initial_trajectory_;
  double timestep_;
  RobotParams robot_params_;
};

}  // namespace kinematics

#endif