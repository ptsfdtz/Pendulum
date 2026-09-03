#include "pendulum/control/ReferenceLqrVelocityController.h"

#include <algorithm>
#include <cmath>
#include <limits>
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
    previousPendulumAngleRadians_ = 0.0;
    previousEncoderAngleRadians_ = 0.0;
    previousCartPositionMeters_ = 0.0;
    previousVelocityIntegratorNotSaturated_ = false;
    previousVelocityIntegratorOutput_ = 0.0;
    velocityIntegratorState_ = 0.0;
    previousPiIntegratorNotSaturated_ = false;
    previousPiIntegratorOutput_ = 0.0;
    piIntegratorState_ = 0.0;
}

void ReferenceLqrVelocityController::resetCommandIntegrators() noexcept {
    previousVelocityIntegratorNotSaturated_ = false;
    previousVelocityIntegratorOutput_ = 0.0;
    velocityIntegratorState_ = 0.0;
    previousPiIntegratorNotSaturated_ = false;
    previousPiIntegratorOutput_ = 0.0;
    piIntegratorState_ = 0.0;
}

SoftwareTravelLimitOutput
ReferenceLqrVelocityController::applySoftwareTravelLimit(
    double outputVoltage, double positionFromCenterHalfTravel,
    double limitFraction, bool positiveVoltageMovesRight) {
    if (!std::isfinite(outputVoltage) ||
        !std::isfinite(positionFromCenterHalfTravel) ||
        !std::isfinite(limitFraction) || limitFraction <= 0.0 ||
        limitFraction > 1.0) {
        throw std::invalid_argument("Invalid software travel-limit input");
    }

    SoftwareTravelLimitOutput result{outputVoltage};
    if (positionFromCenterHalfTravel >= limitFraction) {
        result.side = SoftwareTravelLimitSide::Right;
    } else if (positionFromCenterHalfTravel <= -limitFraction) {
        result.side = SoftwareTravelLimitSide::Left;
    } else {
        return result;
    }

    const double rightwardVoltageSign = positiveVoltageMovesRight ? 1.0 : -1.0;
    const double rightwardCommand = outputVoltage * rightwardVoltageSign;
    const bool pointsOutward =
        (result.side == SoftwareTravelLimitSide::Right && rightwardCommand > 0.0) ||
        (result.side == SoftwareTravelLimitSide::Left && rightwardCommand < 0.0);
    if (pointsOutward) {
        result.outputVoltage = 0.0;
        result.outwardCommandBlocked = true;
    }
    return result;
}

ReferenceLqrOutput ReferenceLqrVelocityController::update(
    std::int64_t pendulumRelativeCounts, std::int64_t cartRelativeCounts,
    bool enableSwingUp) {
    constexpr double radiansPerCount =
        2.0 * std::numbers::pi /
        static_cast<double>(kEncoderCountsPerRevolution);
    constexpr double cartMetersPerCount =
        kCartMetersPerRevolution /
        static_cast<double>(kEncoderCountsPerRevolution);

    ReferenceLqrOutput output;
    const double encoderAngleRadians =
        static_cast<double>(pendulumRelativeCounts) * radiansPerCount;
    // Reference model: theta = wrap(0 - encoder1 * 2*pi/8000).
    output.pendulumAngleRadians = wrapToPi(-encoderAngleRadians);
    // Reference model: x = encoder2 * (-1) * 0.163/8000.
    output.cartPositionMeters =
        -static_cast<double>(cartRelativeCounts) * cartMetersPerCount;
    output.pendulumAngularRateRadiansPerSecond =
        (output.pendulumAngleRadians - previousPendulumAngleRadians_) /
        kControlSampleSeconds;
    output.cartVelocityMetersPerSecond =
        (output.cartPositionMeters - previousCartPositionMeters_) /
        kControlSampleSeconds;

    const double lqrAcceleration =
        kPendulumAngleGain * output.pendulumAngleRadians +
        kPendulumAngularRateGain *
            output.pendulumAngularRateRadiansPerSecond +
        kCartPositionGain * output.cartPositionMeters +
        kCartVelocityGain * output.cartVelocityMetersPerSecond;
    output.lqrAccelerationMetersPerSecondSquared = std::clamp(
        lqrAcceleration, -kAccelerationLimit, kAccelerationLimit);

    // Exact ETlabLibrary/Swing_up branch from the generated R2021a code.
    const double encoderAngleDelta =
        encoderAngleRadians - previousEncoderAngleRadians_;
    const double targetEnergy =
        2.0 * kSwingMassKilograms * kGravityMetersPerSecondSquared *
        kPendulumLengthMeters;
    const double measuredEnergy =
        kSwingMassKilograms * kGravityMetersPerSecondSquared *
            kPendulumLengthMeters * (1.0 - std::cos(encoderAngleRadians)) +
        kPendulumInertia / 0.0002 *
            (encoderAngleDelta * encoderAngleDelta);
    const double energyError = targetEnergy - measuredEnergy;
    const double signInput =
        std::cos(encoderAngleRadians) * encoderAngleDelta * energyError;
    double signValue = 0.0;
    if (signInput < 0.0) {
        signValue = -1.0;
    } else if (signInput > 0.0) {
        signValue = 1.0;
    } else if (signInput != 0.0) {
        signValue = std::numeric_limits<double>::quiet_NaN();
    }
    const double swingEnergyTerm = -kSwingEnergyGain * signValue;
    const double swingPositionTerm =
        std::log(1.0 - 0.8 / kSwingPositionLimitMeters *
                           std::abs(output.cartPositionMeters)) *
        output.cartPositionMeters * kSwingPositionGain;
    output.swingUpAccelerationMetersPerSecondSquared = std::clamp(
        swingPositionTerm - swingEnergyTerm,
        -kAccelerationLimit, kAccelerationLimit);
    output.swingUpActive =
        enableSwingUp &&
        std::abs(output.pendulumAngleRadians) >= kSwitchAngleRadians;
    output.accelerationCommandMetersPerSecondSquared =
        output.swingUpActive
            ? output.swingUpAccelerationMetersPerSecondSquared
            : output.lqrAccelerationMetersPerSecondSquared;

    // Exact ACC2VOL velocity-reference integrator and clamp logic.
    const double velocityIntegratorGain =
        previousVelocityIntegratorNotSaturated_ ||
                oppositeNonPositiveSigns(previousVelocityIntegratorOutput_,
                                         output.accelerationCommandMetersPerSecondSquared)
            ? 1.0
            : 0.0;
    const double velocityIntegratorOutput =
        kAcc2VolIntegrationSeconds *
            output.accelerationCommandMetersPerSecondSquared *
            velocityIntegratorGain +
        velocityIntegratorState_;
    output.velocityReferenceMetersPerSecond = std::clamp(
        velocityIntegratorOutput, -kVelocityLimit, kVelocityLimit);

    output.velocityErrorMetersPerSecond =
        output.velocityReferenceMetersPerSecond -
        output.cartVelocityMetersPerSecond;

    // Exact ACC2VOL PI integrator ordering. In the generated reference code
    // its saturation-equality signal reduces to (integrator == integrator).
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
    const double unconstrainedVoltage =
        output.proportionalVoltage + output.integralVoltage;
    output.outputVoltage = std::clamp(
        unconstrainedVoltage, -kVoltageLimit, kVoltageLimit);

    if (!std::isfinite(output.outputVoltage)) {
        throw std::runtime_error("Reference LQR produced NaN or Inf");
    }

    previousPendulumAngleRadians_ = output.pendulumAngleRadians;
    previousEncoderAngleRadians_ = encoderAngleRadians;
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
