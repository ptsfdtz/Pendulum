#include "pendulum/control/AnglePdController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pendulum::control {

AnglePdController::AnglePdController(
    double angleGainRatedTorquePerRadian,
    double angularRateGainRatedTorquePerRadianPerSecond,
    double maximumAbsoluteRatedTorqueFraction,
    double cartPositionGainRatedTorquePerHalfTravel,
    double cartVelocityGainRatedTorquePerHalfTravelPerSecond,
    double cartIntegralGainRatedTorquePerHalfTravelSecond,
    double maximumAbsoluteCartRatedTorqueFraction)
    : angleGain_(angleGainRatedTorquePerRadian),
      angularRateGain_(angularRateGainRatedTorquePerRadianPerSecond),
      maximumAbsoluteRatedTorqueFraction_(maximumAbsoluteRatedTorqueFraction),
      cartPositionGain_(cartPositionGainRatedTorquePerHalfTravel),
      cartVelocityGain_(cartVelocityGainRatedTorquePerHalfTravelPerSecond),
      cartIntegralGain_(cartIntegralGainRatedTorquePerHalfTravelSecond),
      maximumAbsoluteCartRatedTorqueFraction_(
          maximumAbsoluteCartRatedTorqueFraction) {
    if (!std::isfinite(angleGain_) || angleGain_ < 0.0 ||
        !std::isfinite(angularRateGain_) || angularRateGain_ < 0.0 ||
        !std::isfinite(maximumAbsoluteRatedTorqueFraction_) ||
        maximumAbsoluteRatedTorqueFraction_ <= 0.0 ||
        maximumAbsoluteRatedTorqueFraction_ > 1.0 ||
        !std::isfinite(cartPositionGain_) || cartPositionGain_ < 0.0 ||
        !std::isfinite(cartVelocityGain_) || cartVelocityGain_ < 0.0 ||
        !std::isfinite(cartIntegralGain_) || cartIntegralGain_ < 0.0 ||
        !std::isfinite(maximumAbsoluteCartRatedTorqueFraction_) ||
        maximumAbsoluteCartRatedTorqueFraction_ < 0.0 ||
        maximumAbsoluteCartRatedTorqueFraction_ >
            maximumAbsoluteRatedTorqueFraction_) {
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
    output.angleRatedTorqueFraction =
        static_cast<double>(polarity) *
        (angleGain_ * state.pendulumAngleRadians +
         angularRateGain_ * state.pendulumAngularRateRadiansPerSecond);
    output.cartRatedTorqueFraction = std::clamp(
        -(cartPositionGain_ * cartFeedback.positionHalfTravel +
          cartVelocityGain_ * cartFeedback.velocityHalfTravelPerSecond +
          cartIntegralGain_ * cartPositionIntegral_),
        -maximumAbsoluteCartRatedTorqueFraction_,
        maximumAbsoluteCartRatedTorqueFraction_);
    output.totalRatedTorqueFraction = std::clamp(
        output.angleRatedTorqueFraction + output.cartRatedTorqueFraction,
        -maximumAbsoluteRatedTorqueFraction_,
        maximumAbsoluteRatedTorqueFraction_);
    return output;
}

double AnglePdController::ratedTorqueFraction(const State& state, int polarity) {
    return update(state, polarity).totalRatedTorqueFraction;
}

}  // namespace pendulum::control
