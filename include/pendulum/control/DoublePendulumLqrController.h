#pragma once

#include <array>
#include <cstdint>

namespace pendulum::control {

enum class DoubleSoftwareTravelLimitSide { None, Left, Right };

struct DoubleSoftwareTravelLimitOutput {
    double outputVoltage{0.0};
    DoubleSoftwareTravelLimitSide side{DoubleSoftwareTravelLimitSide::None};
    bool outwardCommandBlocked{false};
};

struct DoublePendulumLqrSettings {
    double sampleSeconds{0.005};
    double cartMetersPerCount{0.734 / 33259.0};
    std::int64_t firstCountsPerRevolution{8000};
    std::int64_t secondCountsPerRevolution{4000};
    std::array<double, 6> gain{
        20.755626070279462, 136.68455751706179, -254.85837173801795,
        24.440647335478328, 3.3044801353394511, -40.750449384516202};
    double velocityFilterHz{20.0};
    double accelerationLimit{30.0};
    double velocityReferenceLimit{0.6};
    double velocityProportionalGain{0.18};
    double velocityIntegralGain{54.0};
    double voltageLimit{1.0};
    double stationaryVoltage{-0.00152587890625};
    double firstMassKilograms{0.3534};
    double secondMassKilograms{0.1016};
    double firstCenterOfMassMeters{0.12};
    double secondCenterOfMassMeters{0.23};
    double firstInertiaKilogramMetersSquared{0.005160863235};
    double secondInertiaKilogramMetersSquared{0.007028721867};
    double gravityMetersPerSecondSquared{9.81};
    double stage1AngleRadians{0.37};
    double stage1ReentryAngleRadians{0.70};
    double stage1CaptureRateRadiansPerSecond{2.0};
    double cartVelocityLimitMetersPerSecond{0.12};
    double stage1Sigma1{1.7824};
    double stage1Sigma2{1.8504};
    double stage1Gain1{2.1876};
    double stage1Gain2{4.9156};
    double stage1Gain3{8.2100};
    std::array<double, 4> singlePendulumGain{
        -4.7434164902525747, 59.170534176527148,
        -6.5584718328315056, 5.4718267547671294};
    double stage2FarAngleRadians{1.146};
    double stage2FarGain{1.40};
    double stage2NearGain{5.0};
    double secondEnergyTargetJoules{0.005};
    double captureAngle1Radians{0.12};
    double captureAngle2Radians{0.26};
    double captureRate1RadiansPerSecond{0.60};
    double captureRate2RadiansPerSecond{0.80};
    double captureCartVelocityMetersPerSecond{0.12};
    double balanceReentryAngle1Radians{0.40};
    double balanceReentryAngle2Radians{0.35};
    double assistAngle1Radians{0.20};
    double assistAngle2Radians{0.45};
    double assistRate1RadiansPerSecond{1.20};
    double assistRate2RadiansPerSecond{9.0};
    double softTrackLimitMeters{0.22};
    double trackLimitMeters{0.30};
    double trackBrakeMarginMeters{0.015};
    double minimumBrakeAccelerationMetersPerSecondSquared{2.0};
};

struct DoublePendulumLqrOutput {
    double cartPositionMeters{0.0};
    double firstAngleRadians{0.0};
    double secondAngleRadians{0.0};
    double cartVelocityMetersPerSecond{0.0};
    double firstAngularRateRadiansPerSecond{0.0};
    double secondAngularRateRadiansPerSecond{0.0};
    double accelerationCommandMetersPerSecondSquared{0.0};
    double velocityReferenceMetersPerSecond{0.0};
    double velocityErrorMetersPerSecond{0.0};
    double outputVoltage{0.0};
    bool voltageSaturated{false};
    int stage{3};
    bool swingUpActive{false};
    bool captureAssistActive{false};
};

class DoublePendulumLqrController final {
public:
    explicit DoublePendulumLqrController(DoublePendulumLqrSettings settings = {});
    void reset() noexcept;
    void resetCommandIntegrators() noexcept;
    const DoublePendulumLqrSettings& settings() const noexcept;
    static DoubleSoftwareTravelLimitOutput applySoftwareTravelLimit(
        double requestedVoltage, double cartPositionMeters,
        double limitMeters, bool positiveVoltageMovesRight,
        double recoveryVoltage);
    DoublePendulumLqrOutput update(std::int64_t cartRelativeCounts,
                                   std::int64_t firstRelativeCounts,
                                   std::int64_t secondRelativeCounts,
                                   bool automaticSwingUp = false);

private:
    DoublePendulumLqrSettings settings_;
    bool initialized_{false};
    double previousX_{0.0};
    double previousTheta1_{0.0};
    double previousTheta2_{0.0};
    double filteredXdot_{0.0};
    double filteredTheta1dot_{0.0};
    double filteredTheta2dot_{0.0};
    double velocityReference_{0.0};
    double velocityIntegral_{0.0};
    int stage_{1};
};

}  // namespace pendulum::control
