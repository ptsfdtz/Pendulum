#pragma once

#include <cstdint>

namespace pendulum::control {

struct ReferenceLqrOutput {
    double pendulumAngleRadians{0.0};
    double pendulumAngularRateRadiansPerSecond{0.0};
    double cartPositionMeters{0.0};
    double cartVelocityMetersPerSecond{0.0};
    double lqrAccelerationMetersPerSecondSquared{0.0};
    double swingUpAccelerationMetersPerSecondSquared{0.0};
    double accelerationCommandMetersPerSecondSquared{0.0};
    double velocityReferenceMetersPerSecond{0.0};
    double velocityErrorMetersPerSecond{0.0};
    double proportionalVoltage{0.0};
    double integralVoltage{0.0};
    double outputVoltage{0.0};
    bool swingUpActive{false};
};

enum class SoftwareTravelLimitSide {
    None,
    Left,
    Right,
};

struct SoftwareTravelLimitOutput {
    double outputVoltage{0.0};
    SoftwareTravelLimitSide side{SoftwareTravelLimitSide::None};
    bool outwardCommandBlocked{false};
};

// Exact controller from Copy_of_LQR_lp1_1.slx (model version 4.37). The
// constants and update ordering match the R2021a generated code. Callers may
// select the full Swing_up/LQR switch or the manual-upright LQR branch.
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
    static constexpr double kSwitchAngleRadians = 0.52359877559829882;
    static constexpr double kSwingMassKilograms = 0.134;
    static constexpr double kGravityMetersPerSecondSquared = 9.8;
    static constexpr double kPendulumLengthMeters = 0.223;
    static constexpr double kPendulumInertia = 0.0089;
    static constexpr double kSwingEnergyGain = 5.0;
    static constexpr double kSwingPositionGain = 6.0;
    static constexpr double kSwingPositionLimitMeters = 0.25;

    void reset() noexcept;
    void resetCommandIntegrators() noexcept;
    ReferenceLqrOutput update(std::int64_t pendulumRelativeCounts,
                              std::int64_t cartRelativeCounts,
                              bool enableSwingUp = false);
    static SoftwareTravelLimitOutput applySoftwareTravelLimit(
        double outputVoltage, double positionFromCenterHalfTravel,
        double limitFraction, bool positiveVoltageMovesRight);

private:
    double previousPendulumAngleRadians_{0.0};
    double previousEncoderAngleRadians_{0.0};
    double previousCartPositionMeters_{0.0};

    bool previousVelocityIntegratorNotSaturated_{false};
    double previousVelocityIntegratorOutput_{0.0};
    double velocityIntegratorState_{0.0};

    bool previousPiIntegratorNotSaturated_{false};
    double previousPiIntegratorOutput_{0.0};
    double piIntegratorState_{0.0};
};

}  // namespace pendulum::control
