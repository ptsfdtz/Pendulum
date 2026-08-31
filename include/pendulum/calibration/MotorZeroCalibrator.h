#pragma once

#include "pendulum/config/Config.h"

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace pendulum::calibration {

struct MotorZeroCalibrationResult {
    double voltage{0.0};
    double dacLsbVoltage{0.0};
    double finalSignedSpeedCountsPerSecond{0.0};
    double finalAbsoluteSpeedCountsPerSecond{0.0};
    bool accepted{false};
    bool targetMet{false};
};

struct VoltageInterval {
    double low{0.0};
    double high{0.0};
    bool found{false};
};

class MotorZeroCalibrator final {
public:
    using WriteVoltage = std::function<void(double)>;
    using ReadCount = std::function<std::uint32_t()>;
    using AbortRequested = std::function<bool()>;
    using Progress = std::function<void(const std::string&)>;

    MotorZeroCalibrator(config::MotorZeroCalibrationConfig settings,
                        double hardwareMinimumVoltage,
                        double hardwareMaximumVoltage,
                        WriteVoltage writeVoltage,
                        ReadCount readCount,
                        AbortRequested abortRequested,
                        Progress progress = {});

    MotorZeroCalibrationResult run();

    static double median(std::vector<double> values);
    static double findZeroCrossing(const std::vector<double>& voltages,
                                   const std::vector<double>& speeds);
    static VoltageInterval findLongestDeadband(const std::vector<double>& voltages,
                                               const std::vector<double>& speeds,
                                               double threshold);

private:
    double measureSignedSpeed(double sampleSeconds);
    double measureAtVoltage(double voltage, double settleSeconds,
                            double sampleSeconds, std::uint32_t repeats);
    void writeBounded(double voltage);
    void wait(double seconds);
    void checkAbort() const;
    void report(const std::string& message) const;

    config::MotorZeroCalibrationConfig settings_;
    double dacLsbVoltage_{0.0};
    WriteVoltage writeVoltage_;
    ReadCount readCount_;
    AbortRequested abortRequested_;
    Progress progress_;
};

}  // namespace pendulum::calibration
