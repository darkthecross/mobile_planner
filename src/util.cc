#include "util.h"

#include "src/proto/world.pb.h"

namespace util {

using namespace cv;

// img_x -> col, img_y -> row
void WorldCoordToImgCoord(double world_x, double world_y, int* img_x,
                          int* img_y) {
  *img_y = IMG_H - static_cast<int>(world_y * WORLD_SCALE + ORIGIN_H);
  *img_y = std::min(std::max(*img_y, 0), IMG_H - 1);
  *img_x = static_cast<int>(world_x * WORLD_SCALE + ORIGIN_W);
  *img_x = std::min(std::max(*img_x, 0), IMG_W - 1);

  // std::cout << "world: (" << world_x << ", " << world_y << "), img: (" <<
  // *img_x << ", " << *img_y << ")" << std::endl;
}

void WorldThetaToImgTheta(double world_theta, double* img_theta) {
  *img_theta = -world_theta / M_PI * 180;
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

void DrawObject(Mat img, double world_x, double world_y, double world_r) {
  int img_x, img_y;
  util::WorldCoordToImgCoord(world_x, world_y, &img_x, &img_y);
  double img_r = WORLD_SCALE * world_r;
  ellipse(img, Point(img_x, img_y), Size(img_r, img_r), 0, 0, 360,
          Scalar(255, 255, 255), 2, 8);
}


void DrawTrajectory(Mat img, const kinematics::Trajectory& traj) {
  const auto& states = traj.x();
  int prev_x, prev_y;
  for (auto i = 0; i < states.size(); i++) {
    int xx, yy;
    double theta;
    const auto& s = states.at(i);
    // std::cout << "phi(" << i << "): " << s.phi;
    util::WorldCoordToImgCoord(s.x, s.y, &xx, &yy);
    util::WorldThetaToImgTheta(s.phi, &theta);
    util::StateEllipse(img, xx, yy, theta);
    if (i > 0) {
      util::StateLine(img, Point(prev_x, prev_y), Point(xx, yy));
    }
    prev_x = xx;
    prev_y = yy;
  }
}

}  // namespace util