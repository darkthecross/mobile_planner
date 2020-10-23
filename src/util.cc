#include "util.h"
#include "src/proto/world.pb.h"

namespace util {

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

  // std::cout << "world: (" << world_x << ", " << world_y << "), img: (" << *img_x << ", " << *img_y << ")" << std::endl;
}

void WorldThetaToImgTheta(double world_theta, double * img_theta) {
  *img_theta = - world_theta / M_PI * 180;
}

void StateEllipse(Mat img, int img_x, int img_y, double img_theta) {
  int thickness = 2;
  int lineType = 8;
  ellipse(img, Point(img_x, img_y), Size(5, 5), img_theta, 30, 330,
          Scalar(255, 0, 0), thickness, lineType);
  ellipse(img, Point(img_x, img_y), Size(5, 5), img_theta, -30, 30,
          Scalar(0, 0, 255), thickness, lineType);
}

void StateLine(Mat img, Point start, Point end) {
  int thickness = 2;
  int lineType = LINE_8;
  line(img, start, end, Scalar(0, 255, 0), thickness, lineType);
}

void DrawRobot(Mat img, const kinematics::State& state) {
    int xx, yy;
    double theta;
    WorldCoordToImgCoord(state.x, state.y, &xx, &yy);
    WorldThetaToImgTheta(state.phi, &theta);
    StateEllipse(img, xx, yy, theta);
}

void DrawRobot(Mat img, const mobile_planner::MobileRobot_RobotState& state) {
  kinematics::State s = kinematics::State::FromProto(state);
  DrawRobot(img, s);
}

}  //namespace util