#include "trajectory_follower.h"

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
  std::vector<Eigen::MatrixXd> Ks;
  Ks.resize(initial_controls.size());
  // Backward pass
  for (int i = initial_states.size() - 2; i >= 0; i--) {
    const auto A_t = dfdx(initial_states[i], initial_controls[i]);
    const auto B_t = dfdu(initial_states[i], initial_controls[i]);
    Eigen::MatrixXd RpBPB = R + B_t.transpose() * P * B_t;
    K = -RpBPB.inverse() * B_t.transpose() * P * A_t;
    Eigen::MatrixXd ApBK = A_t + B_t * K;
    P = Q + K.transpose() * R * K + ApBK.transpose() * P * ApBK;
    Ks[i] = K;
  }
  // Forward pass
  Trajectory optimized_trajectory(timestep_, robot_params_);
  optimized_trajectory.SetInitialState(initial_states.at(0));
  for (int i = 0; i < initial_controls.size(); i++) {
    Eigen::Vector4d x_t_star;
    x_t_star << initial_states[i].x, initial_states[i].y, initial_states[i].v,
        initial_states[i].phi;
    const auto& x_t_state = optimized_trajectory.x().back();
    Eigen::Vector4d x_t;
    x_t << x_t_state.x, x_t_state.y, x_t_state.v, x_t_state.phi;
    const auto& Ki = Ks[i];
    Eigen::Vector2d u_t_opt = Ki * (x_t - x_t_star);
    Control ctrl_t(u_t_opt(0) + initial_controls[i].a,
                   u_t_opt(1) + initial_controls[i].delta_f);
    optimized_trajectory.AppendControl(ctrl_t);
  }
  return optimized_trajectory;
}

}  // namespace kinematics