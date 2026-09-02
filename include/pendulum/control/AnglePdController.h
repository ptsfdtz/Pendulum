#pragma once

#include "pendulum/control/PendulumStateEstimator.h"

namespace pendulum::control {

struct CartCenteringFeedback {
    // Position and velocity are normalized so +/-1 is one half-travel from center.
    // Their positive direction is the direction produced by positive AO voltage.
    double positionHalfTravel{0.0};
    double velocityHalfTravelPerSecond{0.0};
    double samplePeriodSeconds{0.0};
};

struct AnglePdOutput {
    double proportionalTermPercent{0.0};
    double derivativeTermPercent{0.0};
    double angleRatedTorqueFraction{0.0};
    double angleRelayBoostRatedTorqueFraction{0.0};
    double cartRatedTorqueFraction{0.0};
    double totalRatedTorqueFraction{0.0};
};

class AnglePdController final {
public:
    AnglePdController(double angleGainPercentAtMaximumAngle,
                      double angularRateGainPercentAtMaximumRate,
                      double maximumBalanceAngleDegrees,
                      double maximumBalanceAngularRateDegreesPerSecond,
                      double maximumAbsoluteRatedTorqueFraction,
                      double cartPositionGainRatedTorquePerHalfTravel = 0.0,
                      double cartVelocityGainRatedTorquePerHalfTravelPerSecond = 0.0,
                      double cartIntegralGainRatedTorquePerHalfTravelSecond = 0.0,
                      double maximumAbsoluteCartRatedTorqueFraction = 0.0,
                      double angleRelayBoostRatedTorqueFraction = 0.0,
                      double angleRelayBoostDeadbandRadians = 0.0);

    AnglePdOutput update(const State& state, int polarity,
                         const CartCenteringFeedback& cartFeedback = {});
    double ratedTorqueFraction(const State& state, int polarity);

private:
    double angleGain_;
    double angularRateGain_;
    double maximumBalanceAngleDegrees_;
    double maximumBalanceAngularRateDegreesPerSecond_;
    double maximumAbsoluteRatedTorqueFraction_;
    double cartPositionGain_;
    double cartVelocityGain_;
    double cartIntegralGain_;
    double maximumAbsoluteCartRatedTorqueFraction_;
    double angleRelayBoostRatedTorqueFraction_;
    double angleRelayBoostDeadbandRadians_;
    double cartPositionIntegral_{0.0};
};

}  // namespace pendulum::control
