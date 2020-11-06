#ifndef KINEMATICS_PARAMS_H
#define KINEMATICS_PARAMS_H

namespace kinematics {

namespace limits {

constexpr double kMaxVelocity = 1.0;
constexpr double kMinVelocity = -0.5;

constexpr double kMaxAcceleration = 1.0;
constexpr double kMinAcceleration = -1.0;

constexpr double kMaxSteer = 1.0;
constexpr double kMinSteer = -1.0;

}  // namespace limits

namespace model {

constexpr double kLr = 0.2;
constexpr double kLf = 0.2;

constexpr double kTimeStep = 0.1;
constexpr int kMaxTrajectorySteps = 100;

}  // namespace model

}  // namespace kinematics

#endif