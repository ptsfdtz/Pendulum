#pragma once

#include <chrono>
#include <cstdint>

namespace pendulum::control {

struct State {
    double cartPositionMeters{0.0};
    double cartVelocityMetersPerSecond{0.0};
    double pendulumAngleRadians{0.0};
    double pendulumAngularRateRadiansPerSecond{0.0};
};

class PendulumStateEstimator final {
public:
    PendulumStateEstimator(std::int64_t countsPerRevolution,
                           double angularRateFilterAlpha);

    void reset(std::int64_t uprightCount, std::int64_t currentCount,
               std::chrono::steady_clock::time_point time);
    State update(std::int64_t currentCount,
                 std::chrono::steady_clock::time_point time);

    static std::int64_t wrappedCounts(std::int64_t countDelta,
                                      std::int64_t countsPerRevolution);

private:
    std::int64_t countsPerRevolution_;
    double radiansPerCount_;
    double angularRateFilterAlpha_;
    std::int64_t uprightCount_{0};
    double previousAngle_{0.0};
    double filteredAngularRate_{0.0};
    std::chrono::steady_clock::time_point previousTime_{};
    bool initialized_{false};
};

}  // namespace pendulum::control
