#pragma once

#include "pendulum/control/PendulumStateEstimator.h"

namespace pendulum::control {

class AnglePdController final {
public:
    AnglePdController(double angleGainRatedTorquePerRadian,
                      double angularRateGainRatedTorquePerRadianPerSecond,
                      double maximumAbsoluteRatedTorqueFraction);

    double ratedTorqueFraction(const State& state, int polarity) const;

private:
    double angleGain_;
    double angularRateGain_;
    double maximumAbsoluteRatedTorqueFraction_;
};

}  // namespace pendulum::control
