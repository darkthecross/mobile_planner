#include <stdio.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>

#include "util.h"
#include "kinematics.h"

using namespace cv;

int main(int argc, char** argv) {
  // Construct a trajectory with random controls.
  std::default_random_engine generator;
  std::uniform_real_distribution<double> delta_f_distribution(0.0, 1.0);
  std::uniform_real_distribution<double> a_distribution(-0.1, 0.5);

  kinematics::Trajectory traj(0.1);
  traj.SetInitialState(kinematics::State());

  for (auto i = 0; i < 30; i++) {
    kinematics::Control random_control;
    random_control.a = a_distribution(generator);
    random_control.delta_f = delta_f_distribution(generator);
    traj.AppendControl(random_control);
  }

  for (auto& s : traj.x()) {
    std::cout << s.DebugString() << std::endl;
  }

  Mat image(IMG_H, IMG_W, CV_8UC3, Scalar(0, 0, 0));

  const auto& states = traj.x();
  int prev_x, prev_y;
  for (auto i = 0; i < states.size(); i++) {
    int xx, yy;
    double theta;
    const auto& s = states.at(i);
    // std::cout << "phi(" << i << "): " << s.phi;
    util::WorldCoordToImgCoord(s.x, s.y, &xx, &yy);
    util::WorldThetaToImgTheta(s.phi, &theta);
    util::StateEllipse(image, xx, yy, theta);
    if (i > 0) {
      util::StateLine(image, Point(prev_x, prev_y), Point(xx, yy));
    }
    prev_x = xx;
    prev_y = yy;
  }

  namedWindow("Display Image", WINDOW_AUTOSIZE);
  imshow("Display Image", image);
  waitKey(0);
  return 0;
}