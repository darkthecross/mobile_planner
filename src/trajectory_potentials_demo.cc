#include <stdio.h>

#include <iostream>
#include <opencv2/opencv.hpp>

#include "kinematics.h"
#include "util.h"
#include "trajectory_potentials.h"

using namespace cv;

int main(int argc, char** argv) {
  // Construct a straight ahead trajectory.

  kinematics::Trajectory traj(0.1);
  traj.SetInitialState(kinematics::State());

  for (auto i = 0; i < 10; i++) {
    kinematics::Control ctrl(0.5, 0.0);
    traj.AppendControl(ctrl);
  }
  for (auto i = 0; i < 20; i++) {
    traj.AppendControl(kinematics::Control());
  }

  Mat image(IMG_H, IMG_W, CV_8UC3, Scalar(0, 0, 0));

  // Set up the optimizer.
  kinematics::TrajectoryOptimizer optimizer(traj);
  Eigen::Vector4d state_gains;
  state_gains << 1,1,1,1;
  optimizer.SetStateGains(state_gains);
  Eigen::Vector2d control_gains;
  control_gains << 0.5, 0.5;
  optimizer.SetControlGains(control_gains);

  // Add objects.
  util::DrawObject(image, 0.5, 0.2, 0.22);
  kinematics::ObjectRepellerPotential o1(0.5, 0.2, 0.22, 0.05);
  optimizer.AddObject(o1);
  util::DrawObject(image, 0.9, -0.3, 0.28);
  kinematics::ObjectRepellerPotential o2(0.9, -0.3, 0.28, 0.05);
  optimizer.AddObject(o2);

  auto optimized_traj = traj;

  cv::Mat img_cp = image.clone();
  util::DrawTrajectory(img_cp, traj);
  // imwrite("init_traj.png", image);

  for(auto i = 0; i<25; i++) {
    img_cp = image.clone();
    optimized_traj = optimizer.RunOptIteration(optimized_traj);
    util::DrawTrajectory(img_cp, optimized_traj);
    imshow("Display Image", img_cp);
    // imwrite("opt_traj_" + std::to_string(i) + ".png", img_cp);
    waitKey(0);
  }

  // imwrite("opt_traj.png", img_cp);

  return 0;
}
