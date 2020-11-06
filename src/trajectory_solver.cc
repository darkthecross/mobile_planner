#include "trajectory_solver.h"

namespace kinematics {

Eigen::Matrix4d TrajectoryFollower::dfdx(const State& x, const Control& u) {
  const double beta =
      atan(robot_params_.l_r / (robot_params_.l_f + robot_params_.l_r) *
           tan(u.delta_f));
  Eigen::Matrix4d m;
  m << 1, 0, -x.v * sin(x.phi + beta) * timestep_,
      cos(x.phi + beta) * timestep_, 0, 1, x.v * cos(x.phi + beta) * timestep_,
      sin(x.phi + beta) * timestep_, 0, 0, 1,
      sin(beta) / robot_params_.l_r * timestep_, 0, 0, 0, 1;
  return m;
}

Eigen::MatrixXd TrajectoryFollower::dfdu(const State& x, const Control& u) {
  const double c = robot_params_.l_r / (robot_params_.l_f + robot_params_.l_r);
  const double beta = atan(c * tan(u.delta_f));
  const double secdeltaf = 1 / cos(u.delta_f);
  const double dbetaddeltaf =
      c * secdeltaf * secdeltaf / (c * c * tan(u.delta_f) * tan(u.delta_f) + 1);
  Eigen::MatrixXd m(4, 2);
  m << 0, -x.v * sin(x.phi + beta) * dbetaddeltaf * timestep_, 0,
      x.v * cos(x.phi + beta) * dbetaddeltaf * timestep_, 0,
      x.v * cos(beta) / robot_params_.l_r * dbetaddeltaf * timestep_, timestep_,
      0;
  return m;
}

Trajectory TrajectoryFollower::Optimize(Eigen::Matrix4d Q, Eigen::Matrix2d R) {
  // Run backfill with LQR.
  const auto& initial_states = initial_trajectory_.x();
  const auto& initial_controls = initial_trajectory_.u();
  Eigen::Matrix4d P = Q;
  Eigen::MatrixXd K(2, 4);
  std::vector<Control> optimized_controls;
  optimized_controls.resize(initial_controls.size());
  for (int i = initial_states.size() - 2; i >= 0; i--) {
    const auto A_t = dfdx(initial_states[i], initial_controls[i]);
    const auto B_t = dfdu(initial_states[i], initial_controls[i]);
    Eigen::MatrixXd RpBPB = R + B_t.transpose() * P * B_t;
    K = -RpBPB.inverse() * B_t.transpose() * P * A_t;
    Eigen::MatrixXd ApBK = A_t + B_t * K;
    P = Q + K.transpose() * R * K + ApBK.transpose() * P * ApBK;
    Eigen::Vector4d x_t;
    x_t << initial_states[i].x, initial_states[i].y, initial_states[i].v,
        initial_states[i].phi;
    Eigen::Vector2d u_t_opt = K * x_t;
    optimized_controls[i] = Control(u_t_opt(0) + initial_controls[i].a,
                                    u_t_opt(1) + initial_controls[i].delta_f);
  }
  Trajectory optimized_trajectory(timestep_, robot_params_);
  optimized_trajectory.SetInitialState(initial_states.at(0));
  for (const auto& ctrl : optimized_controls) {
    optimized_trajectory.AppendControl(ctrl);
  }
  return optimized_trajectory;
}

}  // namespace kinematics