#include <iostream>
#include <random>

#include "trajectory_solver.h"

int main(int argc, char** argv) {
  // Construct a trajectory with random controls.
  std::default_random_engine generator;
  std::uniform_real_distribution<double> delta_f_distribution(0.0, 1.0);
  std::uniform_real_distribution<double> a_distribution(-0.1, 0.5);

  kinematics::Trajectory traj(0.1);
  traj.SetInitialState(kinematics::State());

  for (auto i = 0; i < 10; i++) {
    kinematics::Control random_control;
    random_control.a = a_distribution(generator);
    random_control.delta_f = delta_f_distribution(generator);
    traj.AppendControl(random_control);
  }

  Eigen::Matrix4d Q;
  Q << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  Eigen::Matrix2d R;
  R << 0.1, 0, 0, 0.1;

  kinematics::TrajectoryFollower follower(traj);
  const auto optimized_traj = follower.Optimize(Q, R);

  // Exam states.

  std::cout << "Original traj:" << std::endl;
  for (int i = 0; i < traj.x().size(); i++) {
    const auto& s = traj.x().at(i);
    std::cout << "State: " << s.DebugString() << std::endl;
    if (i < traj.u().size()) {
      const auto& u = traj.u().at(i);
      std::cout << "Control: (" << u.a << ", " << u.delta_f << ")" << std::endl;
    }
  }

  std::cout << "Optimized traj:" << std::endl;
  for (int i = 0; i < optimized_traj.x().size(); i++) {
    const auto& s = optimized_traj.x().at(i);
    std::cout << "State: " << s.DebugString() << std::endl;
    if (i < optimized_traj.u().size()) {
      const auto& u = optimized_traj.u().at(i);
      std::cout << "Control: (" << u.a << ", " << u.delta_f << ")" << std::endl;
    }
  }

  return 0;
}