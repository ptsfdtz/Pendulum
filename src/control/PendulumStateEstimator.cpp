#include "pendulum/control/PendulumStateEstimator.h"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace pendulum::control {

PendulumStateEstimator::PendulumStateEstimator(
    std::int64_t countsPerRevolution, double angularRateFilterAlpha)
    : countsPerRevolution_(countsPerRevolution),
      radiansPerCount_(2.0 * std::numbers::pi /
                       static_cast<double>(countsPerRevolution)),
      angularRateFilterAlpha_(angularRateFilterAlpha) {
    if (countsPerRevolution <= 0 || !std::isfinite(angularRateFilterAlpha) ||
        angularRateFilterAlpha <= 0.0 || angularRateFilterAlpha > 1.0) {
        throw std::invalid_argument("Invalid pendulum state-estimator settings");
    }
}

void PendulumStateEstimator::reset(
    std::int64_t uprightCount, std::int64_t currentCount,
    std::chrono::steady_clock::time_point time) {
    uprightCount_ = uprightCount;
    previousAngle_ = static_cast<double>(
                         wrappedCounts(currentCount - uprightCount_,
                                       countsPerRevolution_)) *
                     radiansPerCount_;
    filteredAngularRate_ = 0.0;
    previousTime_ = time;
    initialized_ = true;
}

State PendulumStateEstimator::update(
    std::int64_t currentCount, std::chrono::steady_clock::time_point time) {
    if (!initialized_) {
        throw std::logic_error("Pendulum state estimator has not been reset");
    }
    const double angle = static_cast<double>(
                             wrappedCounts(currentCount - uprightCount_,
                                           countsPerRevolution_)) *
                         radiansPerCount_;
    const double dt = std::chrono::duration<double>(time - previousTime_).count();
    if (!std::isfinite(dt) || dt <= 0.0) {
        throw std::runtime_error("Non-positive pendulum estimator time step");
    }
    const double wrappedAngleDelta =
        static_cast<double>(wrappedCounts(
            static_cast<std::int64_t>(std::llround(
                (angle - previousAngle_) / radiansPerCount_)),
            countsPerRevolution_)) *
        radiansPerCount_;
    const double rawRate = wrappedAngleDelta / dt;
    filteredAngularRate_ +=
        angularRateFilterAlpha_ * (rawRate - filteredAngularRate_);
    previousAngle_ = angle;
    previousTime_ = time;
    return State{0.0, 0.0, angle, filteredAngularRate_};
}

std::int64_t PendulumStateEstimator::wrappedCounts(
    std::int64_t countDelta, std::int64_t countsPerRevolution) {
    if (countsPerRevolution <= 0) {
        throw std::invalid_argument("countsPerRevolution must be positive");
    }
    const auto half = countsPerRevolution / 2;
    auto wrapped = countDelta % countsPerRevolution;
    if (wrapped >= half) {
        wrapped -= countsPerRevolution;
    } else if (wrapped < -half) {
        wrapped += countsPerRevolution;
    }
    return wrapped;
}

}  // namespace pendulum::control
