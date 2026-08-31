#pragma once

#include "pendulum/control/PendulumStateEstimator.h"

#include <array>

namespace pendulum::control {

class LqrController final {
public:
    LqrController(std::array<double, 4> gain,
                  double maximumAbsoluteForceNewtons);

    double update(const State& state) const;

private:
    std::array<double, 4> gain_;
    double maximumAbsoluteForceNewtons_;
};

}  // namespace pendulum::control
