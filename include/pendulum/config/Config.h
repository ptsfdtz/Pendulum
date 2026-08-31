#pragma once

#include <cstddef>
#include <array>
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
    std::string pendulumEncoderATerminal;
    std::string pendulumEncoderBTerminal;
    std::string pendulumEncoderDecoding;
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
    std::uint32_t dashboardRefreshMilliseconds{100};
};

struct MotorZeroCalibrationConfig {
    std::uint32_t dacBits{16};
    double voltageLimit{0.1};
    double servoSettleSeconds{3.0};
    double targetSpeedCountsPerSecond{2.0};
    double maximumAcceptedSpeedCountsPerSecond{5.0};
    double coarseStartVoltage{-0.005};
    double coarseEndVoltage{0.005};
    double coarseStepVoltage{0.001};
    double coarseSettleSeconds{0.5};
    double coarseSampleSeconds{0.7};
    std::uint32_t coarseRepeats{2};
    double fineHalfRangeVoltage{0.0015};
    std::uint32_t finePreconditionCodes{3};
    double finePreconditionSeconds{0.8};
    double fineSettleSeconds{0.45};
    double fineSampleSeconds{1.0};
    std::uint32_t fineRepeats{3};
    std::uint32_t finalPreconditionCodes{2};
    double finalPreconditionSeconds{0.5};
    double finalSettleSeconds{0.7};
    double finalSampleSeconds{1.5};
    std::uint32_t finalRepeats{5};
    double verificationSettleSeconds{1.5};
    double verificationSampleSeconds{2.0};
    std::uint32_t verificationRepeats{7};
};

struct HomeCenterConfig {
    double searchVoltage{0.10};
    double fineVoltage{0.020};
    double escapeVoltage{0.030};
    double centerFastVoltage{0.10};
    double centerMidVoltage{0.050};
    double centerSlowVoltage{0.015};
    std::int64_t escapeCounts{200};
    std::int64_t minimumTravelCounts{1000};
    std::int64_t maximumTravelCounts{33253};
    double centerToleranceFraction{0.0005};
    std::int64_t minimumCenterToleranceCounts{10};
    std::int64_t maximumReuseCenterErrorCounts{50};
    double searchTimeoutSeconds{60.0};
    double backoffTimeoutSeconds{5.0};
    std::uint32_t awayDirectionTestMilliseconds{300};
    double centerTimeoutSeconds{30.0};
    std::uint32_t centerSettleMilliseconds{200};
    std::uint32_t pollPeriodMilliseconds{5};
};

struct BalanceControlConfig {
    std::string driveModel{"SGD7S-180A00A002"};
    std::string driveControlMode{"ANALOG_TORQUE"};
    std::uint32_t pn400Setting{30};
    double ratedTorqueCommandVoltage{3.0};
    double analogTorqueZeroVoltage{-0.00135};
    bool analogTorqueZeroCalibrated{false};
    std::uint32_t frequencyHz{500};
    std::uint32_t pendulumPulsesPerRevolution{2000};
    std::uint32_t pendulumCountsPerRevolution{8000};
    double angularRateFilterAlpha{0.15};
    double angleGainRatedTorquePerRadian{2.0};
    double angularRateGainRatedTorquePerRadianPerSecond{0.1};
    int defaultPolarity{1};
    double maximumAbsoluteRatedTorqueFraction{1.0};
    std::uint32_t telemetryDivider{10};
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

struct MotorZeroCalibrationRecord {
    double voltage{0.0};
    double dacLsbVoltage{0.0};
    double finalSignedSpeedCountsPerSecond{0.0};
    double finalAbsoluteSpeedCountsPerSecond{0.0};
    double targetSpeedCountsPerSecond{0.0};
    double maximumAcceptedSpeedCountsPerSecond{0.0};
};

struct HomeCenterCalibrationRecord {
    std::int64_t leftBoundaryCounts{0};
    std::int64_t rightBoundaryCounts{0};
    std::int64_t centerCounts{0};
    std::int64_t travelCounts{0};
    std::int64_t finalPositionCounts{0};
    std::int64_t centerErrorCounts{0};
    bool reusedStoredTravel{false};
};

struct AppConfig {
    int schemaVersion{0};
    Ni6602Config ni6602;
    Pci1723Config pci1723;
    SafetyConfig safety;
    ManualConsoleConfig manualConsole;
    MotorZeroCalibrationConfig motorZeroCalibration;
    HomeCenterConfig homeCenter;
    std::optional<HomeCenterCalibrationRecord> homeCenterCalibration;
    BalanceControlConfig balanceControl;
    LoggingConfig logging;

    static AppConfig load(const std::filesystem::path& path);
    static void saveMotorEncoderCalibration(const std::filesystem::path& path,
                                            const MotorEncoderCalibrationRecord& record);
    static void saveMotorZeroCalibration(const std::filesystem::path& path,
                                         const MotorZeroCalibrationRecord& record);
    static void saveHomeCenterCalibration(const std::filesystem::path& path,
                                          const HomeCenterCalibrationRecord& record);
    void validateForEnumeration() const;
    void validateForSafeOutputTest() const;
    void validateForMotorEncoderCalibration() const;
    void validateForManualConsole() const;
};

inline constexpr const char* kOutputTestConfirmation =
    "I_CONFIRM_ONLY_SAFE_OUTPUTS_MAY_CHANGE";
bool isConfirmed(const std::string& value);

}  // namespace pendulum::config
