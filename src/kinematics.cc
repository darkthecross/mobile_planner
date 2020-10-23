#include "kinematics.h"

#include <cmath>

namespace kinematics {

void Trajectory::StepControl(const State& cur_state, const Control& control,
                             const RobotParams& robot_params, double timestep,
                             State* stepped_state) {
  if(stepped_state == nullptr) {
      return;
  }
  const double lf = robot_params.l_f;
  const double lr = robot_params.l_r;
  const double beta = std::atan2(lr * std::tan(control.delta_f), lr + lf);

  stepped_state->v = cur_state.v + control.a * timestep;
  const double dphi_0 = cur_state.v / lr * std::sin(beta);
  const double dphi_2 = stepped_state->v / lr * std::sin(beta);
  const double dphi_1 =
      (cur_state.v + stepped_state->v) / 2 / lr * std::sin(beta);
  stepped_state->phi =
      cur_state.phi + 1.0 / 6 * (dphi_0 + dphi_1 * 4 + dphi_2) * timestep;

  const double phi_mid = (cur_state.phi + stepped_state->phi) / 2.0;

  const double dx0ov = std::cos(cur_state.phi + beta);
  const double dx1ov = std::cos(phi_mid + beta);
  const double dx2ov = std::cos(stepped_state->phi + beta);
  stepped_state->x =
      cur_state.x + 1.0 / 6 * (dx0ov + dx1ov * 4 + dx2ov) * timestep;

  const double dy0ov = std::sin(cur_state.phi + beta);
  const double dy1ov = std::sin(phi_mid + beta);
  const double dy2ov = std::sin(stepped_state->phi + beta);
  stepped_state->y =
      cur_state.y + 1.0 / 6 * (dy0ov + dy1ov * 4 + dy2ov) * timestep;
}

void Trajectory::AppendControl(const Control& control) {
  if(x_.empty()) {
      return;
  }
  u_.push_back(control);
  State next_state;
  StepControl(x_.back(), control, robot_params_, timestep_, &next_state);
  x_.push_back(next_state);
}

}  // namespace kinematics