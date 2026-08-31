#include "pendulum/control/LqrController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pendulum::control {

LqrController::LqrController(std::array<double, 4> gain,
                             double maximumAbsoluteForceNewtons)
    : gain_(gain), maximumAbsoluteForceNewtons_(maximumAbsoluteForceNewtons) {
    if (!std::isfinite(maximumAbsoluteForceNewtons) ||
        maximumAbsoluteForceNewtons <= 0.0) {
        throw std::invalid_argument("Invalid LQR force limit");
    }
    for (const double value : gain_) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("LQR gain must be finite");
        }
    }
}

double LqrController::update(const State& state) const {
    const std::array<double, 4> values{
        state.cartPositionMeters, state.cartVelocityMetersPerSecond,
        state.pendulumAngleRadians, state.pendulumAngularRateRadiansPerSecond};
    double force = 0.0;
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (!std::isfinite(values[index])) {
            throw std::runtime_error("LQR state contains NaN or Inf");
        }
        force -= gain_[index] * values[index];
    }
    return std::clamp(force, -maximumAbsoluteForceNewtons_,
                      maximumAbsoluteForceNewtons_);
}

}  // namespace pendulum::control
