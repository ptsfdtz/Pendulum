#include "pendulum/config/Config.h"

#include <Windows.h>

#include <chrono>
#include <cmath>
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
        manualConsole.limitDebounceSamples > 20) {
        throw std::runtime_error("Manual console monitor settings are invalid");
    }
}

}  // namespace pendulum::config
