#include "pendulum/control/AnglePdController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pendulum::control {

AnglePdController::AnglePdController(
    double angleGainRatedTorquePerRadian,
    double angularRateGainRatedTorquePerRadianPerSecond,
    double maximumAbsoluteRatedTorqueFraction)
    : angleGain_(angleGainRatedTorquePerRadian),
      angularRateGain_(angularRateGainRatedTorquePerRadianPerSecond),
      maximumAbsoluteRatedTorqueFraction_(maximumAbsoluteRatedTorqueFraction) {
    if (!std::isfinite(angleGain_) || angleGain_ < 0.0 ||
        !std::isfinite(angularRateGain_) || angularRateGain_ < 0.0 ||
        !std::isfinite(maximumAbsoluteRatedTorqueFraction_) ||
        maximumAbsoluteRatedTorqueFraction_ <= 0.0 ||
        maximumAbsoluteRatedTorqueFraction_ > 1.0) {
        throw std::invalid_argument("Invalid angle PD controller settings");
    }
}

double AnglePdController::ratedTorqueFraction(const State& state,
                                              int polarity) const {
    if (polarity != -1 && polarity != 1) {
        throw std::invalid_argument("Balance polarity must be +1 or -1");
    }
    if (!std::isfinite(state.pendulumAngleRadians) ||
        !std::isfinite(state.pendulumAngularRateRadiansPerSecond)) {
        throw std::runtime_error("Pendulum state contains NaN or Inf");
    }
    const double correction = static_cast<double>(polarity) *
                              (angleGain_ * state.pendulumAngleRadians +
                               angularRateGain_ *
                                   state.pendulumAngularRateRadiansPerSecond);
    return std::clamp(correction, -maximumAbsoluteRatedTorqueFraction_,
                      maximumAbsoluteRatedTorqueFraction_);
}

}  // namespace pendulum::control
