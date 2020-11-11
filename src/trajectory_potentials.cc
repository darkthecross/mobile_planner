#include "trajectory_potentials.h"

#include <iostream>

namespace kinematics {

namespace {

Eigen::MatrixXd FullQ(Eigen::Matrix4d hessian, Eigen::Vector4d gradient,
                      double g) {
  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(5, 5);
  q.block<4, 4>(0, 0) << hessian;
  q.block<4, 1>(0, 4) << gradient;
  q.block<1, 4>(4, 0) << gradient.transpose();
  q(4, 4) = g;
  return q;
}

Eigen::MatrixXd QFromPosStates(const std::vector<ObjectRepellerPotential>& ps,
                               const State& s) {
  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(5, 5);
  for (const auto& p : ps) {
    const auto hessian = p.hessian(s);
    const auto gradient = p.gradient(s);
    const auto g = p.value(s);
    auto qp = FullQ(hessian, gradient, g);
    q = q + qp;
  }
  return q;
}

Eigen::MatrixXd PSDQ(Eigen::MatrixXd Q0, Eigen::MatrixXd Qp) {
  Eigen::LLT<Eigen::MatrixXd> lltOfQ0(Q0);
  double alpha = 0.1;
  Eigen::MatrixXd A = Qp * (1 - alpha) + Q0 * alpha;
  Eigen::LLT<Eigen::MatrixXd> lltOfA(A);
  int loop_count = 0;
  while (lltOfA.info() == Eigen::NumericalIssue && loop_count < 10) {
    alpha = alpha + (1.0 - alpha) * 0.5;
    A = Qp * (1 - alpha) + Q0 * alpha;
    lltOfA = Eigen::LLT<Eigen::MatrixXd>(A);
    loop_count++;
  }
  std::cout << "PSDQ:" << std::endl;
  std::cout << A << std::endl;
  return A;
}

}  // namespace

double ObjectRepellerPotential::value(const State& s) const {
  const double d = sqrt((s.x - x_) * (s.x - x_) + (s.y - y_) * (s.y - y_));
  if (d > r_ + eps_ || d < r_ - eps_) {
    return 0.0;
  } else if (d < r_) {
    const double dre = d - (r_ - eps_);
    return 1.0 - 1.0 / 2.0 / eps_ / eps_ * dre * dre;
  } else {
    const double dre = d - (r_ + eps_);
    return 1.0 / 2.0 / eps_ / eps_ * dre * dre;
  }
}

Eigen::Vector4d ObjectRepellerPotential::gradient(const State& s) const {
  Eigen::Vector4d v;
  v << 0, 0, 0, 0;
  const double d = sqrt((s.x - x_) * (s.x - x_) + (s.y - y_) * (s.y - y_));
  if (d > r_ - eps_ && d < r_) {
    const double dre = d - (r_ - eps_);
    const double c = -1.0 / eps_ / eps_ * dre / d;
    v << c * (s.x - x_), c * (s.y - y_), 0, 0;
  } else if (d > r_ && d < r_ + eps_) {
    const double dre = d - (r_ + eps_);
    const double c = 1.0 / eps_ / eps_ * dre / d;
    v << c * (s.x - x_), c * (s.y - y_), 0, 0;
  }
  return v;
}

Eigen::Matrix4d ObjectRepellerPotential::hessian(const State& s) const {
  Eigen::Matrix4d m;
  m << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  const double d = sqrt((s.x - x_) * (s.x - x_) + (s.y - y_) * (s.y - y_));
  if (d > r_ - eps_ && d < r_) {
    const double d2fdx2 = -1.0 / eps_ / eps_ *
                          ((r_ - eps_) * (s.x - x_) * (s.x - x_) / d / d / d +
                           1.0 - (r_ - eps_) / d);
    const double d2fdy2 = -1.0 / eps_ / eps_ *
                          ((r_ - eps_) * (s.y - y_) * (s.y - y_) / d / d / d +
                           1.0 - (r_ - eps_) / d);
    const double d2fdxy = -1.0 / eps_ / eps_ *
                          ((r_ - eps_) * (s.x - x_) * (s.y - y_) / d / d / d);
    m << d2fdx2, d2fdxy, 0, 0, d2fdxy, d2fdy2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  } else if (d > r_ && d < r_ + eps_) {
    const double d2fdx2 = 1.0 / eps_ / eps_ *
                          ((r_ + eps_) * (s.x - x_) * (s.x - x_) / d / d / d +
                           1.0 - (r_ + eps_) / d);
    const double d2fdy2 = 1.0 / eps_ / eps_ *
                          ((r_ + eps_) * (s.y - y_) * (s.y - y_) / d / d / d +
                           1.0 - (r_ + eps_) / d);
    const double d2fdxy =
        1.0 / eps_ / eps_ * ((r_ + eps_) * (s.x - x_) * (s.y - y_) / d / d / d);
    m << d2fdx2, d2fdxy, 0, 0, d2fdxy, d2fdy2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  }
  return m;
}

Eigen::Matrix4d TrajectoryOptimizer::dfdx(const State& x, const Control& u) {
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

Eigen::MatrixXd TrajectoryOptimizer::dfdu(const State& x, const Control& u) {
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

Trajectory TrajectoryOptimizer::RunOptIteration(const kinematics::Trajectory& traj) {
  // Run backfill with LQR.
  const auto& initial_states = traj.x();
  const auto& initial_controls = traj.u();
  // Backward pass
  const auto& s_back = initial_states.back();

  std::cout << "state control gains." << std::endl;
  // Constant state / control gains.
  Eigen::MatrixXd Q0 = Eigen::MatrixXd::Zero(5, 5);
  Q0.diagonal().block<4,1>(0,0) = state_gains_;
  // hack.
  Q0(4,4) = 1.0;
  Eigen::Matrix2d R0;
  R0.diagonal() = control_gains_;

  std::cout << "Q for z_last." << std::endl;
  // Construct Q for z_{last}
  Eigen::MatrixXd Qp_last = QFromPosStates(object_repeller_potentials_, s_back);
  Eigen::MatrixXd psdQ_last = PSDQ(Q0, Qp_last);
  Eigen::MatrixXd P = psdQ_last;
  Eigen::MatrixXd K(2, 5);
  std::vector<Eigen::MatrixXd> Ks;
  Ks.resize(initial_controls.size());

  std::cout << "backfill." << std::endl;
  // Backfill pass.
  for (int i = initial_states.size() - 2; i >= 0; i--) {
    const auto dfdx_mat = dfdx(initial_states[i], initial_controls[i]);
    const auto dfdu_mat = dfdu(initial_states[i], initial_controls[i]);
    Eigen::MatrixXd A_t = Eigen::MatrixXd::Identity(5, 5);
    A_t.block<4, 4>(0, 0) << dfdx_mat;
    Eigen::MatrixXd B_t = Eigen::MatrixXd::Zero(5, 2);
    B_t.block<4, 2>(0, 0) << dfdu_mat;

    Eigen::MatrixXd Qp =
        QFromPosStates(object_repeller_potentials_, initial_states[i+1]);
    Eigen::MatrixXd Q = PSDQ(Q0, Qp);

    Eigen::MatrixXd RpBPB = R0 + B_t.transpose() * P * B_t;
    K = -RpBPB.inverse() * B_t.transpose() * P * A_t;
    Eigen::MatrixXd ApBK = A_t + B_t * K;
    P = Q + K.transpose() * R0 * K + ApBK.transpose() * P * ApBK;
    Ks[i] = K;
  }

  std::cout << "forward pass." << std::endl;
  // Forward pass
  Trajectory optimized_trajectory(timestep_, robot_params_);
  optimized_trajectory.SetInitialState(initial_states.at(0));
  for (int i = 0; i < initial_controls.size(); i++) {
    std::cout << "forward pass: step " << i << std::endl;
    const auto& x_t_state = optimized_trajectory.x().back();
    Eigen::VectorXd z_t(5);
    z_t << x_t_state.x - initial_states[i].x, x_t_state.y - initial_states[i].y, x_t_state.v - initial_states[i].v,
        x_t_state.phi - initial_states[i].phi, 1;
    std::cout << "Ks[i]: \n" << Ks[i] << std::endl;
    Eigen::Vector2d u_t_opt = Ks[i] * z_t;
    std::cout << "u_t_opt: \n" << u_t_opt << std::endl;
    Control ctrl_t(u_t_opt(0) + initial_controls[i].a,
                   u_t_opt(1) + initial_controls[i].delta_f);
    std::cout << "i_control: (" << initial_controls[i].a << ", " << initial_controls[i].delta_f << ")" << std::endl;
    std::cout << "o_control: (" << ctrl_t.a << ", " << ctrl_t.delta_f << ")" << std::endl;
    optimized_trajectory.AppendControl(ctrl_t);
  }
  return optimized_trajectory;
}

}  // namespace kinematics