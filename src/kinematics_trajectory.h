#include <vector>

#include "src/proto/world.pb.h"

namespace kinematics {

struct State {
  State(double xx = 0.0, double yy = 0.0, double phi_in = 0.0,
        double v_in = 0.0)
      : x(xx), y(yy), phi(phi_in), v(v_in) {}
  State(mobile_planner::MobileRobot_RobotState s) {
    x = s.x();
    y = s.y();
    phi = s.phi();
    v = s.v();
  }
  mobile_planner::MobileRobot_RobotState ToProto() {
    mobile_planner::MobileRobot_RobotState s;
    s.set_x(x);
    s.set_y(y);
    s.set_phi(phi);
    s.set_v(v);
    return s;
  }
  double x;
  double y;
  double phi;
  double v;
};

struct Control {
  Control(double aa = 0.0, double delta_f_in = 0.0)
      : a(aa), delta_f(delta_f_in) {}
  double a;
  double delta_f;
};

struct RobotParams {
  RobotParams(double lr, double lf) : l_r(lr), l_f(lf) {}
  double l_r;
  double l_f;
  static const RobotParams Defaults() { return RobotParams(1.0, 1.0); }
};

class Trajectory {
 public:
  explicit Trajectory(double timestep,
                      const RobotParams& robot_params = RobotParams::Defaults())
      : timestep_(timestep), robot_params_(robot_params) {}

  const std::vector<State> x() { return x_; }

  const std::vector<Control> u() { return u_; }

  void Reset() {
    x_.clear();
    u_.clear();
  }

  void SetInitialState(const State& initial_state) {
    Reset();
    x_.push_back(initial_state);
  }

  void AppendControl(const Control& control);

  // Kinematics model.
  static void StepControl(const State& cur_state, const Control& control,
                          const RobotParams& robot_params, double timestep,
                          State* stepped_state);

 private:
  double timestep_;
  RobotParams robot_params_;

  std::vector<State> x_;
  std::vector<Control> u_;
};

}  // namespace kinematics