#include "pendulum/control/AnglePdController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pendulum::control {

AnglePdController::AnglePdController(
    double angleGainPercentAtMaximumAngle,
    double angularRateGainPercentAtMaximumRate,
    double maximumBalanceAngleDegrees,
    double maximumBalanceAngularRateDegreesPerSecond,
    double maximumAbsoluteRatedTorqueFraction,
    double cartPositionGainRatedTorquePerHalfTravel,
    double cartVelocityGainRatedTorquePerHalfTravelPerSecond,
    double cartIntegralGainRatedTorquePerHalfTravelSecond,
    double maximumAbsoluteCartRatedTorqueFraction,
    double angleRelayBoostRatedTorqueFraction,
    double angleRelayBoostDeadbandRadians)
    : angleGain_(angleGainPercentAtMaximumAngle),
      angularRateGain_(angularRateGainPercentAtMaximumRate),
      maximumBalanceAngleDegrees_(maximumBalanceAngleDegrees),
      maximumBalanceAngularRateDegreesPerSecond_(
          maximumBalanceAngularRateDegreesPerSecond),
      maximumAbsoluteRatedTorqueFraction_(maximumAbsoluteRatedTorqueFraction),
      cartPositionGain_(cartPositionGainRatedTorquePerHalfTravel),
      cartVelocityGain_(cartVelocityGainRatedTorquePerHalfTravelPerSecond),
      cartIntegralGain_(cartIntegralGainRatedTorquePerHalfTravelSecond),
      maximumAbsoluteCartRatedTorqueFraction_(
          maximumAbsoluteCartRatedTorqueFraction),
      angleRelayBoostRatedTorqueFraction_(angleRelayBoostRatedTorqueFraction),
      angleRelayBoostDeadbandRadians_(angleRelayBoostDeadbandRadians) {
    if (!std::isfinite(angleGain_) || angleGain_ < 0.0 ||
        !std::isfinite(angularRateGain_) || angularRateGain_ < 0.0 ||
        !std::isfinite(maximumBalanceAngleDegrees_) ||
        maximumBalanceAngleDegrees_ <= 0.0 ||
        !std::isfinite(maximumBalanceAngularRateDegreesPerSecond_) ||
        maximumBalanceAngularRateDegreesPerSecond_ <= 0.0 ||
        !std::isfinite(maximumAbsoluteRatedTorqueFraction_) ||
        maximumAbsoluteRatedTorqueFraction_ <= 0.0 ||
        maximumAbsoluteRatedTorqueFraction_ > 1.0 ||
        !std::isfinite(cartPositionGain_) || cartPositionGain_ < 0.0 ||
        !std::isfinite(cartVelocityGain_) || cartVelocityGain_ < 0.0 ||
        !std::isfinite(cartIntegralGain_) || cartIntegralGain_ < 0.0 ||
        !std::isfinite(maximumAbsoluteCartRatedTorqueFraction_) ||
        maximumAbsoluteCartRatedTorqueFraction_ < 0.0 ||
        maximumAbsoluteCartRatedTorqueFraction_ >
            maximumAbsoluteRatedTorqueFraction_ ||
        !std::isfinite(angleRelayBoostRatedTorqueFraction_) ||
        angleRelayBoostRatedTorqueFraction_ < 0.0 ||
        angleRelayBoostRatedTorqueFraction_ > maximumAbsoluteRatedTorqueFraction_ ||
        !std::isfinite(angleRelayBoostDeadbandRadians_) ||
        angleRelayBoostDeadbandRadians_ < 0.0) {
        throw std::invalid_argument("Invalid angle PD controller settings");
    }
}

AnglePdOutput AnglePdController::update(
    const State& state, int polarity,
    const CartCenteringFeedback& cartFeedback) {
    if (polarity != -1 && polarity != 1) {
        throw std::invalid_argument("Balance polarity must be +1 or -1");
    }
    if (!std::isfinite(state.pendulumAngleRadians) ||
        !std::isfinite(state.pendulumAngularRateRadiansPerSecond) ||
        !std::isfinite(state.pendulumAngleDegrees) ||
        !std::isfinite(state.pendulumAngularRateRawDegreesPerSecond) ||
        !std::isfinite(state.pendulumAngularRateFilteredDegreesPerSecond) ||
        !std::isfinite(cartFeedback.positionHalfTravel) ||
        !std::isfinite(cartFeedback.velocityHalfTravelPerSecond) ||
        !std::isfinite(cartFeedback.samplePeriodSeconds) ||
        cartFeedback.samplePeriodSeconds < 0.0) {
        throw std::runtime_error("Balance feedback contains NaN or Inf");
    }

    cartPositionIntegral_ +=
        cartFeedback.positionHalfTravel * cartFeedback.samplePeriodSeconds;
    if (cartIntegralGain_ > 0.0) {
        const double integralLimit =
            maximumAbsoluteCartRatedTorqueFraction_ / cartIntegralGain_;
        cartPositionIntegral_ = std::clamp(
            cartPositionIntegral_, -integralLimit, integralLimit);
    } else {
        cartPositionIntegral_ = 0.0;
    }

    AnglePdOutput output;
    output.proportionalTermPercent = static_cast<double>(polarity) * angleGain_ *
        state.pendulumAngleDegrees / maximumBalanceAngleDegrees_;
    output.derivativeTermPercent = static_cast<double>(polarity) * angularRateGain_ *
        state.pendulumAngularRateFilteredDegreesPerSecond /
        maximumBalanceAngularRateDegreesPerSecond_;
    output.angleRatedTorqueFraction =
        (output.proportionalTermPercent + output.derivativeTermPercent) / 100.0;
    if (std::abs(state.pendulumAngleRadians) > angleRelayBoostDeadbandRadians_) {
        output.angleRelayBoostRatedTorqueFraction =
            static_cast<double>(polarity) *
            std::copysign(angleRelayBoostRatedTorqueFraction_,
                          state.pendulumAngleRadians);
    }
    output.cartRatedTorqueFraction = std::clamp(
        cartPositionGain_ * cartFeedback.positionHalfTravel +
            cartVelocityGain_ * cartFeedback.velocityHalfTravelPerSecond +
            cartIntegralGain_ * cartPositionIntegral_,
        -maximumAbsoluteCartRatedTorqueFraction_,
        maximumAbsoluteCartRatedTorqueFraction_);
    output.totalRatedTorqueFraction = std::clamp(
        output.angleRatedTorqueFraction +
            output.angleRelayBoostRatedTorqueFraction +
            output.cartRatedTorqueFraction,
        -maximumAbsoluteRatedTorqueFraction_,
        maximumAbsoluteRatedTorqueFraction_);
    return output;
}

double AnglePdController::ratedTorqueFraction(const State& state, int polarity) {
    return update(state, polarity).totalRatedTorqueFraction;
}

}  // namespace pendulum::control
