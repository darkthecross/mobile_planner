#ifndef TRAJECTORY_FOLLOWER_H
#define TRAJECTORY_FOLLOWER_H

#include <Eigen/Dense>

#include "kinematics.h"

namespace kinematics {

class TrajectoryFollower {
 public:
  TrajectoryFollower(Trajectory initial_traj)
      : initial_trajectory_(initial_traj),
        timestep_(initial_traj.dt()),
        robot_params_(initial_traj.robot_params()) {}

  Trajectory Optimize(Eigen::Matrix4d Q, Eigen::Matrix2d R);

 private:
  Eigen::Matrix4d dfdx(const State& x, const Control& u);

  Eigen::MatrixXd dfdu(const State& x, const Control& u);

  Trajectory initial_trajectory_;
  double timestep_;
  RobotParams robot_params_;
};

}  // namespace kinematics

#endif