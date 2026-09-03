#pragma once

#include <cstdint>

namespace pendulum::control {

struct ReferenceLqrOutput {
    double firstPendulumAngleRadians{0.0};
    double firstPendulumAngularRateRadiansPerSecond{0.0};
    double secondPendulumAngleRadians{0.0};
    double secondPendulumAngularRateRadiansPerSecond{0.0};
    double cartPositionMeters{0.0};
    double cartVelocityMetersPerSecond{0.0};
    double lqrAccelerationMetersPerSecondSquared{0.0};
    double velocityReferenceMetersPerSecond{0.0};
    double velocityErrorMetersPerSecond{0.0};
    double proportionalVoltage{0.0};
    double integralVoltage{0.0};
    double outputVoltage{0.0};
    bool angleStopActive{false};
};

// Exact numerical control path from LQR_lp2.slx, model version 4.43.
// The model's absolute-angle equations are expressed relative to the upright
// encoder counts captured by the operator's "balance start" command.
class ReferenceLqrVelocityController final {
public:
    static constexpr std::int64_t kMotorCountsPerRevolution = 8000;
    static constexpr std::int64_t kFirstPendulumCountsPerRevolution = 10000;
    static constexpr std::int64_t kSecondPendulumCountsPerRevolution = 4000;
    static constexpr double kCartMetersPerRevolution = 0.163;
    static constexpr double kControlSampleSeconds = 0.005;

    static constexpr double kSecondPendulumAngleGain = 150.31;
    static constexpr double kSecondPendulumAngularRateGain = 23.63;
    static constexpr double kFirstPendulumAngleGain = -93.74;
    static constexpr double kFirstPendulumAngularRateGain = -3.25;
    static constexpr double kCartPositionGain = -10.0;
    static constexpr double kCartVelocityGain = -11.64;
    static constexpr double kAccelerationLimit = 30.0;
    static constexpr double kMaximumAngleRadians = 0.17453292519943295;

    static constexpr double kAcc2VolIntegrationSeconds = 0.005;
    static constexpr double kVelocityProportionalGain = 0.18;
    static constexpr double kVelocityIntegralGain = 27.0;
    static constexpr double kVelocityLimit = 0.6;
    static constexpr double kVoltageLimit = 2.0;

    void reset() noexcept;
    ReferenceLqrOutput update(std::int64_t firstPendulumRelativeCounts,
                              std::int64_t secondPendulumRelativeCounts,
                              std::int64_t cartRelativeCounts);

private:
    double previousFirstPendulumAngleRadians_{0.0};
    double previousSecondPendulumAngleRadians_{0.0};
    double previousCartPositionMeters_{0.0};
    bool previousVelocityIntegratorNotSaturated_{false};
    double previousVelocityIntegratorOutput_{0.0};
    double velocityIntegratorState_{0.0};
    bool previousPiIntegratorNotSaturated_{false};
    double previousPiIntegratorOutput_{0.0};
    double piIntegratorState_{0.0};
};

}  // namespace pendulum::control
