#include <stdio.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>

#include "kinematics_trajectory.h"

using namespace cv;

#define IMG_W 1600
#define IMG_H 1200
#define ORIGIN_W 400
#define ORIGIN_H 600

// img_x -> col, img_y -> row
void WorldCoordToImgCoord(double world_x, double world_y, int* img_x,
                          int* img_y) {
  *img_y = IMG_H - static_cast<int>(world_y * 200 + ORIGIN_H);
  *img_y = std::min(std::max(*img_y, 0), IMG_H - 1);
  *img_x = static_cast<int>(world_x * 200 + ORIGIN_W);
  *img_x = std::min(std::max(*img_x, 0), IMG_W - 1);

  // std::cout << "world: (" << world_x << ", " << world_y << "), img: (" <<
  // *img_x << ", " << *img_y << ")" << std::endl;
}

void StateEllipse(Mat img, int img_x, int img_y) {
  int thickness = 2;
  int lineType = 8;
  ellipse(img, Point(img_x, img_y), Size(5, 5), 2 * M_PI, 0, 360,
          Scalar(255, 0, 0), thickness, lineType);
}

void StateLine(Mat img, Point start, Point end) {
  int thickness = 2;
  int lineType = LINE_8;
  line(img, start, end, Scalar(0, 255, 0), thickness, lineType);
}

void DrawRobot(Mat img, const mobile_planner::MobileRobot_RobotState& state) {}

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
    const auto& s = states.at(i);
    // std::cout << "phi(" << i << "): " << s.phi;
    WorldCoordToImgCoord(s.x, s.y, &xx, &yy);
    StateEllipse(image, xx, yy);
    if (i > 0) {
      StateLine(image, Point(prev_x, prev_y), Point(xx, yy));
    }
    prev_x = xx;
    prev_y = yy;
  }

  namedWindow("Display Image", WINDOW_AUTOSIZE);
  imshow("Display Image", image);
  waitKey(0);
  return 0;
}