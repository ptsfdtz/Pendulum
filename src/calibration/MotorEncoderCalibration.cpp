#include "pendulum/calibration/MotorEncoderCalibration.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace pendulum::calibration {

std::int64_t MotorEncoderCalibration::deltaWithRollover(std::uint32_t start,
                                                        std::uint32_t end) noexcept {
    const std::uint32_t modularDifference = end - start;
    if (modularDifference <= static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())) {
        return static_cast<std::int64_t>(modularDifference);
    }
    const std::uint64_t reverseMagnitude =
        static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()) -
        modularDifference + 1ULL;
    return -static_cast<std::int64_t>(reverseMagnitude);
}

std::uint32_t MotorEncoderCalibration::stableRepresentative(
    const std::vector<std::uint32_t>& samples, std::uint64_t maximumSpanCounts) {
    if (samples.empty()) {
        throw std::invalid_argument("Encoder stability capture requires at least one sample");
    }
    std::vector<std::int64_t> offsets;
    offsets.reserve(samples.size());
    const auto anchor = samples.front();
    for (const auto sample : samples) {
        offsets.push_back(deltaWithRollover(anchor, sample));
    }
    std::sort(offsets.begin(), offsets.end());
    const auto span = static_cast<std::uint64_t>(offsets.back() - offsets.front());
    if (span > maximumSpanCounts) {
        throw std::runtime_error("Encoder is not stable; captured span is " +
                                 std::to_string(span) + " counts");
    }

    const auto medianOffset = offsets[offsets.size() / 2];
    constexpr std::int64_t modulus = 1LL << 32;
    std::int64_t representative = static_cast<std::int64_t>(anchor) + medianOffset;
    representative %= modulus;
    if (representative < 0) {
        representative += modulus;
    }
    return static_cast<std::uint32_t>(representative);
}

MotorEncoderCalibrationResult MotorEncoderCalibration::calculate(
    std::uint32_t start, std::uint32_t end, double measuredDistanceMillimeters) {
    if (!std::isfinite(measuredDistanceMillimeters) || measuredDistanceMillimeters <= 0.0) {
        throw std::invalid_argument("Measured distance must be a positive finite number");
    }
    const auto delta = deltaWithRollover(start, end);
    if (delta == 0) {
        throw std::runtime_error("Encoder delta is zero; calibration cannot be calculated");
    }
    const auto magnitude = delta < 0 ? -delta : delta;
    const double countsPerMillimeter =
        static_cast<double>(magnitude) / measuredDistanceMillimeters;
    if (!std::isfinite(countsPerMillimeter) || countsPerMillimeter <= 0.0) {
        throw std::runtime_error("Calculated counts_per_mm is invalid");
    }
    return MotorEncoderCalibrationResult{
        start, end, delta, measuredDistanceMillimeters, countsPerMillimeter};
}

}  // namespace pendulum::calibration
