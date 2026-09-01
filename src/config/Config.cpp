#include "pendulum/config/Config.h"

#include <Windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace pendulum::config {
namespace {

using nlohmann::json;

const json& requireObject(const json& parent, const char* key) {
    if (!parent.contains(key) || !parent.at(key).is_object()) {
        throw std::runtime_error(std::string("Missing JSON object: ") + key);
    }
    return parent.at(key);
}

template <typename T>
T requireValue(const json& parent, const char* key) {
    if (!parent.contains(key)) {
        throw std::runtime_error(std::string("Missing configuration value: ") + key);
    }
    try {
        return parent.at(key).get<T>();
    } catch (const json::exception& error) {
        throw std::runtime_error(std::string("Invalid configuration value '") + key +
                                 "': " + error.what());
    }
}

void requireNotEmpty(const std::string& value, const char* field) {
    if (value.empty()) {
        throw std::runtime_error(std::string(field) + " must not be empty");
    }
}

std::int64_t calibrationMidpoint(std::int64_t first, std::int64_t second) {
    const auto low = std::min(first, second);
    const auto high = std::max(first, second);
    return low + (high - low) / 2;
}

std::string utcTimestamp(bool fileSafe = false) {
    const auto now = std::chrono::system_clock::now();
    const auto value = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
    gmtime_s(&utc, &value);
    std::ostringstream stream;
    stream << std::put_time(&utc, fileSafe ? "%Y%m%dT%H%M%SZ" : "%Y-%m-%dT%H:%M:%SZ");
    return stream.str();
}

json loadDocument(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Cannot open configuration file: " + path.string());
    }
    json document;
    try {
        input >> document;
    } catch (const json::exception& error) {
        throw std::runtime_error("Invalid JSON in " + path.string() + ": " + error.what());
    }
    return document;
}

void writeDocumentAtomically(const std::filesystem::path& path, const json& document) {
    const auto temporary = std::filesystem::path(path.string() + ".pending");
    const auto backup = std::filesystem::path(path.string() + ".backup_" + utcTimestamp(true));
    {
        std::ofstream output(temporary, std::ios::out | std::ios::trunc);
        if (!output) {
            throw std::runtime_error("Cannot create pending configuration: " + temporary.string());
        }
        output << document.dump(2) << '\n';
        output.flush();
        if (!output) {
            throw std::runtime_error("Failed while writing pending configuration");
        }
    }

    if (!ReplaceFileW(path.c_str(), temporary.c_str(), backup.c_str(),
                      REPLACEFILE_WRITE_THROUGH, nullptr, nullptr)) {
        const auto error = GetLastError();
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        throw std::runtime_error("Atomic config replacement failed with Windows error " +
                                 std::to_string(error));
    }
}

}  // namespace

bool isConfirmed(const std::string& value) {
    return !value.empty() && value != "UNCONFIRMED";
}

AppConfig AppConfig::load(const std::filesystem::path& path) {
    const json document = loadDocument(path);

    AppConfig config;
    config.schemaVersion = requireValue<int>(document, "schema_version");

    const auto& hardware = requireObject(document, "hardware");
    const auto& ni = requireObject(hardware, "ni6602");
    config.ni6602.device = requireValue<std::string>(ni, "device");
    config.ni6602.expectedProduct = requireValue<std::string>(ni, "expected_product");
    config.ni6602.motorCounter = requireValue<std::string>(ni, "motor_counter");
    config.ni6602.motorEncoderATerminal =
        requireValue<std::string>(ni, "motor_encoder_a_terminal");
    config.ni6602.motorEncoderBTerminal =
        requireValue<std::string>(ni, "motor_encoder_b_terminal");
    config.ni6602.motorEncoderPulsesPerRevolution =
        requireValue<std::uint32_t>(ni, "motor_encoder_pulses_per_revolution");
    config.ni6602.motorEncoderDecoding =
        requireValue<std::string>(ni, "motor_encoder_decoding");
    config.ni6602.motorEncoderCountsPerRevolution =
        requireValue<std::uint32_t>(ni, "motor_encoder_counts_per_revolution");
    config.ni6602.motorEncoderFilterMinPulseWidthMicroseconds =
        requireValue<double>(ni, "motor_encoder_filter_min_pulse_width_us");
    if (!ni.contains("motor_encoder_counts_per_mm")) {
        throw std::runtime_error("Missing configuration value: motor_encoder_counts_per_mm");
    }
    if (ni.at("motor_encoder_counts_per_mm").is_null()) {
        config.ni6602.motorEncoderCountsPerMillimeter = std::nullopt;
    } else {
        config.ni6602.motorEncoderCountsPerMillimeter =
            requireValue<double>(ni, "motor_encoder_counts_per_mm");
    }
    config.ni6602.pendulumCounter = requireValue<std::string>(ni, "pendulum_counter");
    config.ni6602.pendulumEncoderATerminal =
        requireValue<std::string>(ni, "pendulum_encoder_a_terminal");
    config.ni6602.pendulumEncoderBTerminal =
        requireValue<std::string>(ni, "pendulum_encoder_b_terminal");
    config.ni6602.pendulumEncoderDecoding =
        requireValue<std::string>(ni, "pendulum_encoder_decoding");
    config.ni6602.leftLimitLine = requireValue<std::string>(ni, "left_limit_line");
    config.ni6602.leftLimitActiveHigh = requireValue<bool>(ni, "left_limit_active_high");
    config.ni6602.rightLimitLine = requireValue<std::string>(ni, "right_limit_line");
    config.ni6602.rightLimitActiveHigh = requireValue<bool>(ni, "right_limit_active_high");
    config.ni6602.servoEnableLine = requireValue<std::string>(ni, "servo_enable_line");
    config.ni6602.servoActiveHigh = requireValue<bool>(ni, "servo_active_high");

    const auto& pci = requireObject(hardware, "pci1723");
    config.pci1723.deviceDescription = requireValue<std::string>(pci, "device_description");
    config.pci1723.aoChannel = requireValue<int>(pci, "ao_channel");
    config.pci1723.safeVoltage = requireValue<double>(pci, "safe_voltage");
    if (!pci.contains("calibrated_zero_voltage")) {
        throw std::runtime_error("Missing configuration value: calibrated_zero_voltage");
    }
    if (pci.at("calibrated_zero_voltage").is_null()) {
        config.pci1723.calibratedZeroVoltage = std::nullopt;
    } else {
        config.pci1723.calibratedZeroVoltage =
            requireValue<double>(pci, "calibrated_zero_voltage");
    }
    config.pci1723.zeroVoltageRequiresCalibration =
        requireValue<bool>(pci, "zero_voltage_requires_calibration");
    config.pci1723.minimumVoltage = requireValue<double>(pci, "minimum_voltage");
    config.pci1723.maximumVoltage = requireValue<double>(pci, "maximum_voltage");
    config.pci1723.positiveVoltageCartDirection =
        requireValue<std::string>(pci, "positive_voltage_cart_direction");
    config.pci1723.negativeVoltageCartDirection =
        requireValue<std::string>(pci, "negative_voltage_cart_direction");

    const auto& safety = requireObject(document, "safety");
    config.safety.allowSafeOutputTest = requireValue<bool>(safety, "allow_safe_output_test");
    config.safety.outputTestConfirmation =
        requireValue<std::string>(safety, "output_test_confirmation");

    const auto& manualConsole = requireObject(document, "manual_console");
    config.manualConsole.monitorPeriodMilliseconds =
        requireValue<std::uint32_t>(manualConsole, "monitor_period_ms");
    config.manualConsole.limitDebounceSamples =
        requireValue<std::uint32_t>(manualConsole, "limit_debounce_samples");
    config.manualConsole.dashboardRefreshMilliseconds =
        requireValue<std::uint32_t>(manualConsole, "dashboard_refresh_ms");

    const auto& zeroCalibration = requireObject(document, "motor_zero_calibration");
    auto& zero = config.motorZeroCalibration;
    zero.dacBits = requireValue<std::uint32_t>(zeroCalibration, "dac_bits");
    zero.voltageLimit = requireValue<double>(zeroCalibration, "voltage_limit");
    zero.servoSettleSeconds = requireValue<double>(zeroCalibration, "servo_settle_seconds");
    zero.targetSpeedCountsPerSecond =
        requireValue<double>(zeroCalibration, "target_speed_counts_per_second");
    zero.maximumAcceptedSpeedCountsPerSecond =
        requireValue<double>(zeroCalibration, "maximum_accepted_speed_counts_per_second");
    zero.coarseStartVoltage =
        requireValue<double>(zeroCalibration, "coarse_start_voltage");
    zero.coarseEndVoltage = requireValue<double>(zeroCalibration, "coarse_end_voltage");
    zero.coarseStepVoltage = requireValue<double>(zeroCalibration, "coarse_step_voltage");
    zero.coarseSettleSeconds =
        requireValue<double>(zeroCalibration, "coarse_settle_seconds");
    zero.coarseSampleSeconds =
        requireValue<double>(zeroCalibration, "coarse_sample_seconds");
    zero.coarseRepeats = requireValue<std::uint32_t>(zeroCalibration, "coarse_repeats");
    zero.fineHalfRangeVoltage =
        requireValue<double>(zeroCalibration, "fine_half_range_voltage");
    zero.finePreconditionCodes =
        requireValue<std::uint32_t>(zeroCalibration, "fine_precondition_codes");
    zero.finePreconditionSeconds =
        requireValue<double>(zeroCalibration, "fine_precondition_seconds");
    zero.fineSettleSeconds =
        requireValue<double>(zeroCalibration, "fine_settle_seconds");
    zero.fineSampleSeconds =
        requireValue<double>(zeroCalibration, "fine_sample_seconds");
    zero.fineRepeats = requireValue<std::uint32_t>(zeroCalibration, "fine_repeats");
    zero.finalPreconditionCodes =
        requireValue<std::uint32_t>(zeroCalibration, "final_precondition_codes");
    zero.finalPreconditionSeconds =
        requireValue<double>(zeroCalibration, "final_precondition_seconds");
    zero.finalSettleSeconds =
        requireValue<double>(zeroCalibration, "final_settle_seconds");
    zero.finalSampleSeconds =
        requireValue<double>(zeroCalibration, "final_sample_seconds");
    zero.finalRepeats = requireValue<std::uint32_t>(zeroCalibration, "final_repeats");
    zero.verificationSettleSeconds =
        requireValue<double>(zeroCalibration, "verification_settle_seconds");
    zero.verificationSampleSeconds =
        requireValue<double>(zeroCalibration, "verification_sample_seconds");
    zero.verificationRepeats =
        requireValue<std::uint32_t>(zeroCalibration, "verification_repeats");

    const auto& homing = requireObject(document, "home_center");
    auto& home = config.homeCenter;
    home.searchVoltage = requireValue<double>(homing, "search_voltage");
    home.fineVoltage = requireValue<double>(homing, "fine_voltage");
    home.escapeVoltage = requireValue<double>(homing, "escape_voltage");
    home.centerFastVoltage = requireValue<double>(homing, "center_fast_voltage");
    home.centerMidVoltage = requireValue<double>(homing, "center_mid_voltage");
    home.centerSlowVoltage = requireValue<double>(homing, "center_slow_voltage");
    home.escapeCounts = requireValue<std::int64_t>(homing, "escape_counts");
    home.minimumTravelCounts = requireValue<std::int64_t>(homing, "minimum_travel_counts");
    home.maximumTravelDisagreementFraction =
        requireValue<double>(homing, "maximum_travel_disagreement_fraction");
    home.centerToleranceFraction =
        requireValue<double>(homing, "center_tolerance_fraction");
    home.minimumCenterToleranceCounts =
        requireValue<std::int64_t>(homing, "minimum_center_tolerance_counts");
    home.maximumReuseCenterErrorCounts =
        requireValue<std::int64_t>(homing, "maximum_reuse_center_error_counts");
    home.searchTimeoutSeconds = requireValue<double>(homing, "search_timeout_seconds");
    home.backoffTimeoutSeconds = requireValue<double>(homing, "backoff_timeout_seconds");
    home.awayDirectionTestMilliseconds =
        requireValue<std::uint32_t>(homing, "away_direction_test_ms");
    home.centerTimeoutSeconds = requireValue<double>(homing, "center_timeout_seconds");
    home.centerSettleMilliseconds =
        requireValue<std::uint32_t>(homing, "center_settle_ms");
    home.pollPeriodMilliseconds =
        requireValue<std::uint32_t>(homing, "poll_period_ms");

    if (document.contains("home_center_calibration") &&
        document.at("home_center_calibration").is_object()) {
        const auto& saved = document.at("home_center_calibration");
        HomeCenterCalibrationRecord record;
        record.leftBoundaryCounts =
            requireValue<std::int64_t>(saved, "left_boundary_counts");
        record.rightBoundaryCounts =
            requireValue<std::int64_t>(saved, "right_boundary_counts");
        record.centerCounts = requireValue<std::int64_t>(saved, "center_counts");
        record.travelCounts = requireValue<std::int64_t>(saved, "travel_counts");
        record.finalPositionCounts =
            requireValue<std::int64_t>(saved, "final_position_counts");
        record.centerErrorCounts =
            requireValue<std::int64_t>(saved, "center_error_counts");
        record.reusedStoredTravel = saved.value("reused_stored_travel", false);
        config.homeCenterCalibration = record;
    }

    const auto& balance = requireObject(document, "balance_control");
    auto& control = config.balanceControl;
    control.driveModel = requireValue<std::string>(balance, "drive_model");
    control.driveControlMode =
        requireValue<std::string>(balance, "drive_control_mode");
    control.pn400Setting =
        requireValue<std::uint32_t>(balance, "pn400_setting");
    control.ratedTorqueCommandVoltage =
        requireValue<double>(balance, "rated_torque_command_voltage");
    control.analogTorqueZeroVoltage =
        requireValue<double>(balance, "analog_torque_zero_voltage");
    control.analogTorqueZeroCalibrated =
        requireValue<bool>(balance, "analog_torque_zero_calibrated");
    control.frequencyHz = requireValue<std::uint32_t>(balance, "frequency_hz");
    control.pendulumPulsesPerRevolution =
        requireValue<std::uint32_t>(balance, "pendulum_pulses_per_revolution");
    control.pendulumCountsPerRevolution =
        requireValue<std::uint32_t>(balance, "pendulum_counts_per_revolution");
    control.angularRateFilterAlpha =
        requireValue<double>(balance, "angular_rate_filter_alpha");
    control.angleGainRatedTorquePerRadian =
        requireValue<double>(balance, "angle_gain_rated_torque_per_radian");
    control.angularRateGainRatedTorquePerRadianPerSecond = requireValue<double>(
        balance, "angular_rate_gain_rated_torque_per_radian_per_second");
    control.defaultPolarity = requireValue<int>(balance, "default_polarity");
    control.maximumAbsoluteRatedTorqueFraction = requireValue<double>(
        balance, "maximum_absolute_rated_torque_fraction");
    control.telemetryDivider =
        requireValue<std::uint32_t>(balance, "telemetry_divider");

    const auto& logging = requireObject(document, "logging");
    config.logging.directory = requireValue<std::string>(logging, "directory");
    config.logging.queueCapacity = requireValue<std::size_t>(logging, "queue_capacity");

    config.validateForEnumeration();
    return config;
}

void AppConfig::saveMotorEncoderCalibration(const std::filesystem::path& path,
                                            const MotorEncoderCalibrationRecord& record) {
    if (record.signedDeltaCounts == 0 || !std::isfinite(record.measuredDistanceMillimeters) ||
        record.measuredDistanceMillimeters <= 0.0 || !std::isfinite(record.countsPerMillimeter) ||
        record.countsPerMillimeter <= 0.0) {
        throw std::invalid_argument("Refusing to save an invalid motor encoder calibration");
    }

    json document = loadDocument(path);
    auto& ni = document.at("hardware").at("ni6602");
    ni["motor_encoder_counts_per_mm"] = record.countsPerMillimeter;
    ni["motor_encoder_calibration"] = {
        {"timestamp_utc", utcTimestamp()},
        {"start_raw_count", record.startRawCount},
        {"end_raw_count", record.endRawCount},
        {"signed_delta_counts", record.signedDeltaCounts},
        {"absolute_delta_counts", std::abs(record.signedDeltaCounts)},
        {"measured_distance_mm", record.measuredDistanceMillimeters},
        {"counts_per_mm", record.countsPerMillimeter},
    };

    writeDocumentAtomically(path, document);
}

void AppConfig::saveMotorZeroCalibration(const std::filesystem::path& path,
                                         const MotorZeroCalibrationRecord& record) {
    if (!std::isfinite(record.voltage) || !std::isfinite(record.dacLsbVoltage) ||
        record.dacLsbVoltage <= 0.0 ||
        !std::isfinite(record.finalSignedSpeedCountsPerSecond) ||
        !std::isfinite(record.finalAbsoluteSpeedCountsPerSecond) ||
        record.finalAbsoluteSpeedCountsPerSecond < 0.0 ||
        !std::isfinite(record.targetSpeedCountsPerSecond) ||
        !std::isfinite(record.maximumAcceptedSpeedCountsPerSecond) ||
        record.targetSpeedCountsPerSecond <= 0.0 ||
        record.maximumAcceptedSpeedCountsPerSecond < record.targetSpeedCountsPerSecond ||
        record.finalAbsoluteSpeedCountsPerSecond >
            record.maximumAcceptedSpeedCountsPerSecond) {
        throw std::invalid_argument("Refusing to save an invalid motor zero calibration");
    }

    json document = loadDocument(path);
    auto& pci = document.at("hardware").at("pci1723");
    pci["calibrated_zero_voltage"] = record.voltage;
    pci["zero_voltage_requires_calibration"] = false;
    pci["motor_zero_calibration"] = {
        {"timestamp_utc", utcTimestamp()},
        {"voltage", record.voltage},
        {"dac_lsb_voltage", record.dacLsbVoltage},
        {"final_signed_speed_counts_per_second",
         record.finalSignedSpeedCountsPerSecond},
        {"final_absolute_speed_counts_per_second",
         record.finalAbsoluteSpeedCountsPerSecond},
        {"target_speed_counts_per_second", record.targetSpeedCountsPerSecond},
        {"maximum_accepted_speed_counts_per_second",
         record.maximumAcceptedSpeedCountsPerSecond},
    };
    writeDocumentAtomically(path, document);
}

void AppConfig::saveHomeCenterCalibration(
    const std::filesystem::path& path,
    const HomeCenterCalibrationRecord& record) {
    if (record.travelCounts <= 0 ||
        std::llabs(record.rightBoundaryCounts - record.leftBoundaryCounts) !=
            record.travelCounts ||
        record.centerCounts != calibrationMidpoint(
                                   record.leftBoundaryCounts,
                                   record.rightBoundaryCounts)) {
        throw std::invalid_argument("Refusing to save an invalid home-center calibration");
    }
    json document = loadDocument(path);
    document["home_center_calibration"] = {
        {"timestamp_utc", utcTimestamp()},
        {"left_boundary_counts", record.leftBoundaryCounts},
        {"right_boundary_counts", record.rightBoundaryCounts},
        {"center_counts", record.centerCounts},
        {"travel_counts", record.travelCounts},
        {"final_position_counts", record.finalPositionCounts},
        {"center_error_counts", record.centerErrorCounts},
        {"reused_stored_travel", record.reusedStoredTravel},
    };
    writeDocumentAtomically(path, document);
}

void AppConfig::validateForEnumeration() const {
    if (schemaVersion != 1) {
        throw std::runtime_error("Unsupported config schema_version; expected 1");
    }
    requireNotEmpty(ni6602.device, "hardware.ni6602.device");
    requireNotEmpty(ni6602.expectedProduct, "hardware.ni6602.expected_product");
    if (ni6602.motorEncoderPulsesPerRevolution == 0 ||
        ni6602.motorEncoderDecoding != "X4" ||
        ni6602.motorEncoderCountsPerRevolution !=
            ni6602.motorEncoderPulsesPerRevolution * 4ULL) {
        throw std::runtime_error(
            "Motor encoder must use X4 and counts_per_revolution must equal 4 * PPR");
    }
    if (ni6602.motorEncoderCountsPerMillimeter.has_value() &&
        (!std::isfinite(*ni6602.motorEncoderCountsPerMillimeter) ||
         *ni6602.motorEncoderCountsPerMillimeter <= 0.0)) {
        throw std::runtime_error("motor_encoder_counts_per_mm must be null or a positive number");
    }
    if (!std::isfinite(ni6602.motorEncoderFilterMinPulseWidthMicroseconds) ||
        ni6602.motorEncoderFilterMinPulseWidthMicroseconds < 0.0) {
        throw std::runtime_error(
            "motor_encoder_filter_min_pulse_width_us must be finite and non-negative");
    }
    requireNotEmpty(pci1723.deviceDescription, "hardware.pci1723.device_description");
    if (pci1723.aoChannel < 0) {
        throw std::runtime_error("hardware.pci1723.ao_channel must be non-negative");
    }
    if (!std::isfinite(pci1723.safeVoltage) || !std::isfinite(pci1723.minimumVoltage) ||
        !std::isfinite(pci1723.maximumVoltage)) {
        throw std::runtime_error("PCI-1723 voltage limits must be finite");
    }
    if (pci1723.minimumVoltage >= pci1723.maximumVoltage) {
        throw std::runtime_error("PCI-1723 minimum_voltage must be less than maximum_voltage");
    }
    const bool validDirectionMapping =
        (pci1723.positiveVoltageCartDirection == "LEFT" &&
         pci1723.negativeVoltageCartDirection == "RIGHT") ||
        (pci1723.positiveVoltageCartDirection == "RIGHT" &&
         pci1723.negativeVoltageCartDirection == "LEFT");
    if (!validDirectionMapping) {
        throw std::runtime_error(
            "PCI-1723 voltage directions must be opposite LEFT/RIGHT values");
    }
    if (pci1723.safeVoltage < pci1723.minimumVoltage ||
        pci1723.safeVoltage > pci1723.maximumVoltage) {
        throw std::runtime_error("PCI-1723 safe_voltage is outside configured limits");
    }
    if (std::abs(pci1723.safeVoltage) > 1e-12) {
        throw std::runtime_error("Phase 1 requires PCI-1723 safe_voltage to be exactly 0 V");
    }
    if (pci1723.calibratedZeroVoltage.has_value() &&
        (!std::isfinite(*pci1723.calibratedZeroVoltage) ||
         *pci1723.calibratedZeroVoltage < pci1723.minimumVoltage ||
         *pci1723.calibratedZeroVoltage > pci1723.maximumVoltage)) {
        throw std::runtime_error("calibrated_zero_voltage is outside configured AO limits");
    }
    if (logging.directory.empty() || logging.queueCapacity < 64) {
        throw std::runtime_error("Logging directory must be set and queue_capacity must be >= 64");
    }
}

void AppConfig::validateForSafeOutputTest() const {
    validateForEnumeration();
    if (!safety.allowSafeOutputTest) {
        throw std::runtime_error("Safe output test is disabled in config.json");
    }
    if (safety.outputTestConfirmation != kOutputTestConfirmation) {
        throw std::runtime_error("Safe output test confirmation token is missing or incorrect");
    }
    if (!isConfirmed(ni6602.servoEnableLine)) {
        throw std::runtime_error("Servo enable line is UNCONFIRMED; refusing to create a DO task");
    }
}

void AppConfig::validateForMotorEncoderCalibration() const {
    validateForEnumeration();
    if (!isConfirmed(ni6602.motorCounter)) {
        throw std::runtime_error("Motor counter is UNCONFIRMED; refusing encoder calibration");
    }
    if (!isConfirmed(ni6602.motorEncoderATerminal) ||
        !isConfirmed(ni6602.motorEncoderBTerminal)) {
        throw std::runtime_error("Motor encoder A/B terminals are UNCONFIRMED");
    }
    const bool useDefaultRouting = ni6602.motorEncoderATerminal == "DEFAULT" &&
                                   ni6602.motorEncoderBTerminal == "DEFAULT";
    const bool useExplicitRouting = ni6602.motorEncoderATerminal != "DEFAULT" &&
                                    ni6602.motorEncoderBTerminal != "DEFAULT" &&
                                    ni6602.motorEncoderATerminal !=
                                        ni6602.motorEncoderBTerminal;
    if (!useDefaultRouting && !useExplicitRouting) {
        throw std::runtime_error("Motor encoder A and B terminals must be different");
    }
}

void AppConfig::validateForManualConsole() const {
    validateForMotorEncoderCalibration();
    if (!isConfirmed(ni6602.pendulumCounter) ||
        !isConfirmed(ni6602.pendulumEncoderATerminal) ||
        !isConfirmed(ni6602.pendulumEncoderBTerminal) ||
        ni6602.pendulumEncoderDecoding != "X4") {
        throw std::runtime_error("Manual console requires a confirmed X4 pendulum encoder");
    }
    const bool pendulumDefaultRouting =
        ni6602.pendulumEncoderATerminal == "DEFAULT" &&
        ni6602.pendulumEncoderBTerminal == "DEFAULT";
    const bool pendulumExplicitRouting =
        ni6602.pendulumEncoderATerminal != "DEFAULT" &&
        ni6602.pendulumEncoderBTerminal != "DEFAULT" &&
        ni6602.pendulumEncoderATerminal != ni6602.pendulumEncoderBTerminal;
    if (!pendulumDefaultRouting && !pendulumExplicitRouting) {
        throw std::runtime_error("Pendulum encoder A/B routing is invalid");
    }
    if (!isConfirmed(ni6602.leftLimitLine) || !isConfirmed(ni6602.rightLimitLine) ||
        ni6602.leftLimitLine == ni6602.rightLimitLine) {
        throw std::runtime_error("Manual console requires two distinct confirmed limit lines");
    }
    if (!isConfirmed(ni6602.servoEnableLine)) {
        throw std::runtime_error("Manual console requires a confirmed Servo output line");
    }
    if (manualConsole.monitorPeriodMilliseconds == 0 ||
        manualConsole.monitorPeriodMilliseconds > 100 ||
        manualConsole.limitDebounceSamples == 0 ||
        manualConsole.limitDebounceSamples > 20 ||
        manualConsole.dashboardRefreshMilliseconds < 50 ||
        manualConsole.dashboardRefreshMilliseconds > 1000) {
        throw std::runtime_error("Manual console monitor settings are invalid");
    }

    const auto& zero = motorZeroCalibration;
    const auto positiveFinite = [](double value) {
        return std::isfinite(value) && value > 0.0;
    };
    if (zero.dacBits == 0 || zero.dacBits > 24 || !positiveFinite(zero.voltageLimit) ||
        zero.voltageLimit > std::abs(pci1723.minimumVoltage) ||
        zero.voltageLimit > std::abs(pci1723.maximumVoltage) ||
        !positiveFinite(zero.servoSettleSeconds) ||
        !positiveFinite(zero.targetSpeedCountsPerSecond) ||
        !positiveFinite(zero.maximumAcceptedSpeedCountsPerSecond) ||
        zero.targetSpeedCountsPerSecond > zero.maximumAcceptedSpeedCountsPerSecond ||
        !std::isfinite(zero.coarseStartVoltage) ||
        !std::isfinite(zero.coarseEndVoltage) ||
        zero.coarseStartVoltage >= zero.coarseEndVoltage ||
        std::abs(zero.coarseStartVoltage) > zero.voltageLimit ||
        std::abs(zero.coarseEndVoltage) > zero.voltageLimit ||
        !positiveFinite(zero.coarseStepVoltage) ||
        !positiveFinite(zero.coarseSettleSeconds) ||
        !positiveFinite(zero.coarseSampleSeconds) || zero.coarseRepeats == 0 ||
        !positiveFinite(zero.fineHalfRangeVoltage) || zero.finePreconditionCodes == 0 ||
        !positiveFinite(zero.finePreconditionSeconds) ||
        !positiveFinite(zero.fineSettleSeconds) ||
        !positiveFinite(zero.fineSampleSeconds) || zero.fineRepeats == 0 ||
        zero.finalPreconditionCodes == 0 ||
        !positiveFinite(zero.finalPreconditionSeconds) ||
        !positiveFinite(zero.finalSettleSeconds) ||
        !positiveFinite(zero.finalSampleSeconds) || zero.finalRepeats == 0 ||
        !positiveFinite(zero.verificationSettleSeconds) ||
        !positiveFinite(zero.verificationSampleSeconds) ||
        zero.verificationRepeats == 0) {
        throw std::runtime_error("Motor zero calibration settings are invalid");
    }

    const auto& home = homeCenter;
    const auto positiveFiniteHome = [](double value) {
        return std::isfinite(value) && value > 0.0;
    };
    if (!positiveFiniteHome(home.searchVoltage) ||
        !positiveFiniteHome(home.fineVoltage) ||
        !positiveFiniteHome(home.escapeVoltage) ||
        !positiveFiniteHome(home.centerFastVoltage) ||
        !positiveFiniteHome(home.centerMidVoltage) ||
        !positiveFiniteHome(home.centerSlowVoltage) ||
        home.escapeCounts <= 0 || home.minimumTravelCounts <= 0 ||
        !std::isfinite(home.maximumTravelDisagreementFraction) ||
        home.maximumTravelDisagreementFraction < 0.0 ||
        home.maximumTravelDisagreementFraction > 1.0 ||
        !positiveFiniteHome(home.centerToleranceFraction) ||
        home.centerToleranceFraction >= 0.5 ||
        home.minimumCenterToleranceCounts <= 0 ||
        home.maximumReuseCenterErrorCounts < home.minimumCenterToleranceCounts ||
        !positiveFiniteHome(home.searchTimeoutSeconds) ||
        !positiveFiniteHome(home.backoffTimeoutSeconds) ||
        home.awayDirectionTestMilliseconds == 0 ||
        home.awayDirectionTestMilliseconds > 2000 ||
        !positiveFiniteHome(home.centerTimeoutSeconds) ||
        home.centerSettleMilliseconds == 0 ||
        home.centerSettleMilliseconds > 5000 ||
        home.pollPeriodMilliseconds == 0 || home.pollPeriodMilliseconds > 100) {
        throw std::runtime_error("Home-center settings are invalid");
    }
    if (homeCenterCalibration.has_value()) {
        const auto& saved = *homeCenterCalibration;
        if (saved.travelCounts <= 0 ||
            std::llabs(saved.rightBoundaryCounts - saved.leftBoundaryCounts) !=
                saved.travelCounts ||
            saved.centerCounts != calibrationMidpoint(
                                      saved.leftBoundaryCounts,
                                      saved.rightBoundaryCounts) ||
            saved.finalPositionCounts - saved.centerCounts !=
                saved.centerErrorCounts) {
            throw std::runtime_error("Stored home-center calibration is invalid");
        }
    }


    const auto& control = balanceControl;
    if (control.driveModel.empty() ||
        control.driveControlMode != "ANALOG_TORQUE" ||
        control.pn400Setting == 0 ||
        !positiveFiniteHome(control.ratedTorqueCommandVoltage) ||
        !std::isfinite(control.analogTorqueZeroVoltage) ||
        control.frequencyHz < 50 || control.frequencyHz > 2000 ||
        control.pendulumPulsesPerRevolution == 0 ||
        control.pendulumCountsPerRevolution !=
            control.pendulumPulsesPerRevolution * 4ULL ||
        !std::isfinite(control.angularRateFilterAlpha) ||
        control.angularRateFilterAlpha <= 0.0 ||
        control.angularRateFilterAlpha > 1.0 ||
        !std::isfinite(control.angleGainRatedTorquePerRadian) ||
        control.angleGainRatedTorquePerRadian < 0.0 ||
        !std::isfinite(control.angularRateGainRatedTorquePerRadianPerSecond) ||
        control.angularRateGainRatedTorquePerRadianPerSecond < 0.0 ||
        (control.defaultPolarity != -1 && control.defaultPolarity != 1) ||
        !positiveFiniteHome(control.maximumAbsoluteRatedTorqueFraction) ||
        control.maximumAbsoluteRatedTorqueFraction > 1.0 ||
        control.telemetryDivider == 0) {
        throw std::runtime_error("Balance-control settings are invalid");
    }
    const double maximumCommandVoltage =
        control.ratedTorqueCommandVoltage *
        control.maximumAbsoluteRatedTorqueFraction;
    if (control.analogTorqueZeroVoltage - maximumCommandVoltage <
            pci1723.minimumVoltage ||
        control.analogTorqueZeroVoltage + maximumCommandVoltage >
            pci1723.maximumVoltage) {
        throw std::runtime_error("Balance voltage settings exceed the AO range");
    }
}

}  // namespace pendulum::config
