#pragma once

#include <cstdint>
#include <vector>

namespace pendulum::calibration {

struct MotorEncoderCalibrationResult {
    std::uint32_t startRawCount{0};
    std::uint32_t endRawCount{0};
    std::int64_t signedDeltaCounts{0};
    double measuredDistanceMillimeters{0.0};
    double countsPerMillimeter{0.0};
};

class MotorEncoderCalibration {
public:
    static std::int64_t deltaWithRollover(std::uint32_t start, std::uint32_t end) noexcept;
    static std::uint32_t stableRepresentative(const std::vector<std::uint32_t>& samples,
                                              std::uint64_t maximumSpanCounts);
    static MotorEncoderCalibrationResult calculate(std::uint32_t start, std::uint32_t end,
                                                   double measuredDistanceMillimeters);
};

}  // namespace pendulum::calibration

