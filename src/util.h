#ifndef UTIL_H
#define UTIL_H

#include "kinematics.h"

#include <opencv2/opencv.hpp>

namespace util{

using namespace cv;

#define IMG_W 1600
#define IMG_H 1200
#define ORIGIN_W 400
#define ORIGIN_H 600

// img_x -> col, img_y -> row
void WorldCoordToImgCoord(double world_x, double world_y, int* img_x,
                          int* img_y);

void WorldThetaToImgTheta(double world_theta, double * img_theta);

void StateEllipse(Mat img, int img_x, int img_y, double img_theta);

void StateLine(Mat img, Point start, Point end);

void DrawRobot(Mat img, const kinematics::State& state);

void DrawRobot(Mat img, const mobile_planner::MobileRobot_RobotState& state);

}  // namespace util

#endif