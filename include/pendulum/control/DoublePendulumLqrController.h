#pragma once

#include <array>
#include <cstdint>

namespace pendulum::control {

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
};

class DoublePendulumLqrController final {
public:
    explicit DoublePendulumLqrController(DoublePendulumLqrSettings settings = {});
    void reset() noexcept;
    const DoublePendulumLqrSettings& settings() const noexcept;
    DoublePendulumLqrOutput update(std::int64_t cartRelativeCounts,
                                   std::int64_t firstRelativeCounts,
                                   std::int64_t secondRelativeCounts);

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
};

}  // namespace pendulum::control
