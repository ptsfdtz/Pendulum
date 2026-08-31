#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace pendulum::config {

struct Ni6602Config {
    std::string device;
    std::string expectedProduct;
    std::string motorCounter;
    std::string motorEncoderATerminal;
    std::string motorEncoderBTerminal;
    std::uint32_t motorEncoderPulsesPerRevolution{0};
    std::string motorEncoderDecoding;
    std::uint32_t motorEncoderCountsPerRevolution{0};
    std::optional<double> motorEncoderCountsPerMillimeter;
    std::string pendulumCounter;
    std::string leftLimitLine;
    bool leftLimitActiveHigh{true};
    std::string rightLimitLine;
    bool rightLimitActiveHigh{true};
    std::string servoEnableLine;
    bool servoActiveHigh{true};
};

struct Pci1723Config {
    std::string deviceDescription;
    int aoChannel{0};
    double safeVoltage{0.0};
    std::optional<double> calibratedZeroVoltage;
    bool zeroVoltageRequiresCalibration{true};
    double minimumVoltage{-10.0};
    double maximumVoltage{10.0};
    std::string positiveVoltageCartDirection;
    std::string negativeVoltageCartDirection;
};

struct SafetyConfig {
    bool allowSafeOutputTest{false};
    std::string outputTestConfirmation;
};

struct ManualConsoleConfig {
    std::uint32_t monitorPeriodMilliseconds{5};
    std::uint32_t limitDebounceSamples{3};
};

struct LoggingConfig {
    std::filesystem::path directory{"logs"};
    std::size_t queueCapacity{4096};
};

struct MotorEncoderCalibrationRecord {
    std::uint32_t startRawCount{0};
    std::uint32_t endRawCount{0};
    std::int64_t signedDeltaCounts{0};
    double measuredDistanceMillimeters{0.0};
    double countsPerMillimeter{0.0};
};

struct AppConfig {
    int schemaVersion{0};
    Ni6602Config ni6602;
    Pci1723Config pci1723;
    SafetyConfig safety;
    ManualConsoleConfig manualConsole;
    LoggingConfig logging;

    static AppConfig load(const std::filesystem::path& path);
    static void saveMotorEncoderCalibration(const std::filesystem::path& path,
                                            const MotorEncoderCalibrationRecord& record);
    void validateForEnumeration() const;
    void validateForSafeOutputTest() const;
    void validateForMotorEncoderCalibration() const;
    void validateForManualConsole() const;
};

inline constexpr const char* kOutputTestConfirmation =
    "I_CONFIRM_ONLY_SAFE_OUTPUTS_MAY_CHANGE";
bool isConfirmed(const std::string& value);

}  // namespace pendulum::config
