#include "pendulum/control/DoublePendulumLqrController.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace pendulum::control {
namespace {

double wrapToPi(double value) {
    return std::remainder(value, 2.0 * std::numbers::pi);
}

void validate(const DoublePendulumLqrSettings& s) {
    if (!std::isfinite(s.sampleSeconds) || s.sampleSeconds <= 0.0 ||
        !std::isfinite(s.cartMetersPerCount) || s.cartMetersPerCount <= 0.0 ||
        s.firstCountsPerRevolution <= 0 || s.secondCountsPerRevolution <= 0 ||
        !std::isfinite(s.velocityFilterHz) || s.velocityFilterHz <= 0.0 ||
        !std::isfinite(s.accelerationLimit) || s.accelerationLimit <= 0.0 ||
        !std::isfinite(s.velocityReferenceLimit) || s.velocityReferenceLimit <= 0.0 ||
        !std::isfinite(s.velocityProportionalGain) ||
        !std::isfinite(s.velocityIntegralGain) || s.velocityIntegralGain < 0.0 ||
        !std::isfinite(s.voltageLimit) || s.voltageLimit <= 0.0 ||
        !std::isfinite(s.stationaryVoltage)) {
        throw std::invalid_argument("Invalid double-pendulum LQR settings");
    }
    for (const auto gain : s.gain) {
        if (!std::isfinite(gain)) {
            throw std::invalid_argument("Double-pendulum LQR gain contains NaN or Inf");
        }
    }
}

}  // namespace

DoublePendulumLqrController::DoublePendulumLqrController(
    DoublePendulumLqrSettings settings)
    : settings_(std::move(settings)) {
    validate(settings_);
}

void DoublePendulumLqrController::reset() noexcept {
    initialized_ = false;
    previousX_ = 0.0;
    previousTheta1_ = 0.0;
    previousTheta2_ = 0.0;
    filteredXdot_ = 0.0;
    filteredTheta1dot_ = 0.0;
    filteredTheta2dot_ = 0.0;
    velocityReference_ = 0.0;
    velocityIntegral_ = 0.0;
}

const DoublePendulumLqrSettings& DoublePendulumLqrController::settings() const noexcept {
    return settings_;
}

DoublePendulumLqrOutput DoublePendulumLqrController::update(
    std::int64_t cartRelativeCounts, std::int64_t firstRelativeCounts,
    std::int64_t secondRelativeCounts) {
    DoublePendulumLqrOutput out;
    out.cartPositionMeters =
        -static_cast<double>(cartRelativeCounts) * settings_.cartMetersPerCount;
    out.firstAngleRadians = wrapToPi(
        -static_cast<double>(firstRelativeCounts) * 2.0 * std::numbers::pi /
        static_cast<double>(settings_.firstCountsPerRevolution));
    const double relativeSecondAngle = wrapToPi(
        -static_cast<double>(secondRelativeCounts) * 2.0 * std::numbers::pi /
        static_cast<double>(settings_.secondCountsPerRevolution));
    out.secondAngleRadians = wrapToPi(out.firstAngleRadians + relativeSecondAngle);

    if (!initialized_) {
        previousX_ = out.cartPositionMeters;
        previousTheta1_ = out.firstAngleRadians;
        previousTheta2_ = out.secondAngleRadians;
        initialized_ = true;
    }
    const double alpha = std::exp(-2.0 * std::numbers::pi *
                                  settings_.velocityFilterHz * settings_.sampleSeconds);
    filteredXdot_ = alpha * filteredXdot_ + (1.0 - alpha) *
        (out.cartPositionMeters - previousX_) / settings_.sampleSeconds;
    filteredTheta1dot_ = alpha * filteredTheta1dot_ + (1.0 - alpha) *
        wrapToPi(out.firstAngleRadians - previousTheta1_) / settings_.sampleSeconds;
    filteredTheta2dot_ = alpha * filteredTheta2dot_ + (1.0 - alpha) *
        wrapToPi(out.secondAngleRadians - previousTheta2_) / settings_.sampleSeconds;
    out.cartVelocityMetersPerSecond = filteredXdot_;
    out.firstAngularRateRadiansPerSecond = filteredTheta1dot_;
    out.secondAngularRateRadiansPerSecond = filteredTheta2dot_;

    const std::array<double, 6> state{
        out.cartPositionMeters, out.firstAngleRadians, out.secondAngleRadians,
        filteredXdot_, filteredTheta1dot_, filteredTheta2dot_};
    double acceleration = 0.0;
    for (std::size_t index = 0; index < state.size(); ++index) {
        acceleration -= settings_.gain[index] * state[index];
    }
    out.accelerationCommandMetersPerSecondSquared = std::clamp(
        acceleration, -settings_.accelerationLimit, settings_.accelerationLimit);
    velocityReference_ = std::clamp(
        velocityReference_ + settings_.sampleSeconds *
                                 out.accelerationCommandMetersPerSecondSquared,
        -settings_.velocityReferenceLimit, settings_.velocityReferenceLimit);
    out.velocityReferenceMetersPerSecond = velocityReference_;
    out.velocityErrorMetersPerSecond = velocityReference_ - filteredXdot_;

    const double candidateIntegral = velocityIntegral_ + settings_.sampleSeconds *
        out.velocityErrorMetersPerSecond;
    const double unconstrained = settings_.stationaryVoltage +
        settings_.velocityProportionalGain * out.velocityErrorMetersPerSecond +
        settings_.velocityIntegralGain * candidateIntegral;
    out.outputVoltage = std::clamp(
        unconstrained, -settings_.voltageLimit, settings_.voltageLimit);
    out.voltageSaturated = out.outputVoltage != unconstrained;
    if (!out.voltageSaturated ||
        std::signbit(out.velocityErrorMetersPerSecond) !=
            std::signbit(unconstrained - out.outputVoltage)) {
        velocityIntegral_ = candidateIntegral;
    }
    if (!std::isfinite(out.outputVoltage)) {
        throw std::runtime_error("Double-pendulum LQR produced NaN or Inf");
    }

    previousX_ = out.cartPositionMeters;
    previousTheta1_ = out.firstAngleRadians;
    previousTheta2_ = out.secondAngleRadians;
    return out;
}

}  // namespace pendulum::control
