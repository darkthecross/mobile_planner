#include <iostream>

#include "trajectory_potentials.h"

using kinematics::State;

int main(int argc, char** argv) {
  // Construct a ObjectRepellerPotential
  kinematics::ObjectRepellerPotential p(3.0, 2.0, 5.0, 1.0);

  State s1(3.1, 2.2);
  std::vector<State> states = {State(3.1, 2.2), State(-1.5, 0.0),
                               State(-2.0, 0.0), State(-5.0, 2.0)};

  for (const auto& s : states) {
    std::cout << "=======================================" << std::endl;
    std::cout << s.DebugString() << std::endl;
    std::cout << "Value: \n" << p.value(s) << std::endl;
    const auto g = p.gradient(s);
    std::cout << "Gradient: \n" << g << std::endl;
    if (g(1) != 0.0) {
      std::cout << "Validate gradient direction: " << (s.x - 3.0) / (s.y - 2.0)
                << " == " << g(0) / g(1) << std::endl;
    }
    std::cout << "Hessian: \n" << p.hessian(s) << std::endl;
    std::cout << "=======================================" << std::endl;
  }

  return 0;
}