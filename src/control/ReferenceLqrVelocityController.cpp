#include "pendulum/control/ReferenceLqrVelocityController.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace pendulum::control {
namespace {

double wrapToPi(double value) {
    while (value > std::numbers::pi) {
        value -= 2.0 * std::numbers::pi;
    }
    while (value < -std::numbers::pi) {
        value += 2.0 * std::numbers::pi;
    }
    return value;
}

bool oppositeNonPositiveSigns(double previousValue, double input) {
    return (previousValue <= 0.0) != (input <= 0.0);
}

}  // namespace

void ReferenceLqrVelocityController::reset() noexcept {
    previousFirstPendulumAngleRadians_ = 0.0;
    previousSecondPendulumAngleRadians_ = 0.0;
    previousCartPositionMeters_ = 0.0;
    previousVelocityIntegratorNotSaturated_ = false;
    previousVelocityIntegratorOutput_ = 0.0;
    velocityIntegratorState_ = 0.0;
    previousPiIntegratorNotSaturated_ = false;
    previousPiIntegratorOutput_ = 0.0;
    piIntegratorState_ = 0.0;
}

ReferenceLqrOutput ReferenceLqrVelocityController::update(
    std::int64_t firstPendulumRelativeCounts,
    std::int64_t secondPendulumRelativeCounts,
    std::int64_t cartRelativeCounts) {
    ReferenceLqrOutput output;
    output.firstPendulumAngleRadians = wrapToPi(
        -static_cast<double>(firstPendulumRelativeCounts) * 2.0 *
        std::numbers::pi /
        static_cast<double>(kFirstPendulumCountsPerRevolution));
    output.secondPendulumAngleRadians = wrapToPi(
        -static_cast<double>(secondPendulumRelativeCounts) * 2.0 *
            std::numbers::pi /
            static_cast<double>(kSecondPendulumCountsPerRevolution) +
        output.firstPendulumAngleRadians);
    output.cartPositionMeters =
        -static_cast<double>(cartRelativeCounts) * kCartMetersPerRevolution /
        static_cast<double>(kMotorCountsPerRevolution);

    output.firstPendulumAngularRateRadiansPerSecond =
        (output.firstPendulumAngleRadians -
         previousFirstPendulumAngleRadians_) /
        kControlSampleSeconds;
    output.secondPendulumAngularRateRadiansPerSecond =
        (output.secondPendulumAngleRadians -
         previousSecondPendulumAngleRadians_) /
        kControlSampleSeconds;
    output.cartVelocityMetersPerSecond =
        (output.cartPositionMeters - previousCartPositionMeters_) /
        kControlSampleSeconds;

    const double acceleration =
        kSecondPendulumAngleGain * output.secondPendulumAngleRadians +
        kSecondPendulumAngularRateGain *
            output.secondPendulumAngularRateRadiansPerSecond +
        kFirstPendulumAngleGain * output.firstPendulumAngleRadians +
        kFirstPendulumAngularRateGain *
            output.firstPendulumAngularRateRadiansPerSecond +
        kCartPositionGain * output.cartPositionMeters +
        kCartVelocityGain * output.cartVelocityMetersPerSecond;
    output.lqrAccelerationMetersPerSecondSquared = std::clamp(
        acceleration, -kAccelerationLimit, kAccelerationLimit);

    output.angleStopActive =
        std::abs(output.firstPendulumAngleRadians) >= kMaximumAngleRadians ||
        std::abs(output.secondPendulumAngleRadians) >= kMaximumAngleRadians;

    const double velocityIntegratorGain =
        previousVelocityIntegratorNotSaturated_ ||
                oppositeNonPositiveSigns(
                    previousVelocityIntegratorOutput_,
                    output.lqrAccelerationMetersPerSecondSquared)
            ? 1.0
            : 0.0;
    const double velocityIntegratorSum =
        kAcc2VolIntegrationSeconds *
            output.lqrAccelerationMetersPerSecondSquared *
            velocityIntegratorGain +
        velocityIntegratorState_;
    const double velocityIntegratorOutput =
        output.angleStopActive ? 0.0 : velocityIntegratorSum;
    output.velocityReferenceMetersPerSecond = std::clamp(
        velocityIntegratorOutput, -kVelocityLimit, kVelocityLimit);
    output.velocityErrorMetersPerSecond =
        output.velocityReferenceMetersPerSecond -
        output.cartVelocityMetersPerSecond;

    const double activeIntegralGain =
        previousPiIntegratorNotSaturated_ ||
                oppositeNonPositiveSigns(previousPiIntegratorOutput_,
                                         output.velocityErrorMetersPerSecond)
            ? kVelocityIntegralGain
            : 0.0;
    output.integralVoltage =
        kAcc2VolIntegrationSeconds * output.velocityErrorMetersPerSecond *
            activeIntegralGain +
        piIntegratorState_;
    output.proportionalVoltage =
        kVelocityProportionalGain * output.velocityErrorMetersPerSecond;
    output.outputVoltage = std::clamp(
        output.proportionalVoltage + output.integralVoltage,
        -kVoltageLimit, kVoltageLimit);
    if (!std::isfinite(output.outputVoltage)) {
        throw std::runtime_error("LQR_lp2 controller produced NaN or Inf");
    }

    previousFirstPendulumAngleRadians_ = output.firstPendulumAngleRadians;
    previousSecondPendulumAngleRadians_ = output.secondPendulumAngleRadians;
    previousCartPositionMeters_ = output.cartPositionMeters;
    previousVelocityIntegratorNotSaturated_ =
        output.velocityReferenceMetersPerSecond == velocityIntegratorOutput;
    previousVelocityIntegratorOutput_ = velocityIntegratorOutput;
    velocityIntegratorState_ = velocityIntegratorOutput;
    previousPiIntegratorNotSaturated_ =
        output.integralVoltage == output.integralVoltage;
    previousPiIntegratorOutput_ = output.integralVoltage;
    piIntegratorState_ = output.integralVoltage;
    return output;
}

}  // namespace pendulum::control
