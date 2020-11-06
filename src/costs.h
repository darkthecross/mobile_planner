#ifndef COSTS_H
#define COSTS_H

#include "kinematics.h"

namespace optimization {

class StateCostInterface {
  virtual double Cost(const kinematics::State& s) = 0;
};


}  // namespace optimization

#endif