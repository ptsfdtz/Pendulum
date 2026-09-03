#pragma once

#include <cstdint>

namespace pendulum::control {

struct ReferenceLqrOutput {
    double pendulumAngleRadians{0.0};
    double pendulumAngularRateRadiansPerSecond{0.0};
    double cartPositionMeters{0.0};
    double cartVelocityMetersPerSecond{0.0};
    double accelerationCommandMetersPerSecondSquared{0.0};
    double velocityReferenceMetersPerSecond{0.0};
    double velocityErrorMetersPerSecond{0.0};
    double proportionalVoltage{0.0};
    double integralVoltage{0.0};
    double outputVoltage{0.0};
};

// Exact manual-upright branch of Copy_of_LQR_lp1_1.slx (model version 4.37).
// The constants and update ordering match the R2021a generated code. Swing-up
// is intentionally outside this class until it is explicitly enabled.
class ReferenceLqrVelocityController final {
public:
    static constexpr std::int64_t kEncoderCountsPerRevolution = 8000;
    static constexpr double kCartMetersPerRevolution = 0.163;
    static constexpr double kControlSampleSeconds = 0.01;
    static constexpr double kAcc2VolIntegrationSeconds = 0.005;
    static constexpr double kCartPositionGain = 10.0;
    static constexpr double kCartVelocityGain = 12.23;
    static constexpr double kPendulumAngleGain = -58.6;
    static constexpr double kPendulumAngularRateGain = -10.69;
    static constexpr double kAccelerationLimit = 10.0;
    static constexpr double kVelocityProportionalGain = 0.18;
    static constexpr double kVelocityIntegralGain = 54.0;
    static constexpr double kVelocityLimit = 0.6;
    static constexpr double kVoltageLimit = 1.0;

    void reset() noexcept;
    ReferenceLqrOutput update(std::int64_t pendulumRelativeCounts,
                              std::int64_t cartRelativeCounts);

private:
    double previousPendulumAngleRadians_{0.0};
    double previousCartPositionMeters_{0.0};

    bool previousVelocityIntegratorNotSaturated_{false};
    double previousVelocityIntegratorOutput_{0.0};
    double velocityIntegratorState_{0.0};

    bool previousPiIntegratorNotSaturated_{false};
    double previousPiIntegratorOutput_{0.0};
    double piIntegratorState_{0.0};
};

}  // namespace pendulum::control
