#pragma once
#include "pendulum/control/DoublePendulumLqrController.h"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <optional>
#include <numbers>
#include <cmath>
#include <algorithm>
#include <vector>
namespace pendulum::tools {
using json = nlohmann::json;
struct DoubleBalanceConfig {
    std::string secondCounter;
    std::string secondATerminal;
    std::string secondBTerminal;
    double secondFilterSeconds{10e-6};
    pendulum::control::DoublePendulumLqrSettings controller;
    double durationSeconds{20.0};
    double startAngleRadians{5.0 * std::numbers::pi / 180.0};
    double stopAngleRadians{10.0 * std::numbers::pi / 180.0};
    double positionWarningMeters{0.30};
    double positionStopMeters{0.35};
    double sampleTimeoutSeconds{0.05};
    std::int64_t cartJumpCounts{1000};
    std::int64_t firstJumpCounts{2000};
    std::int64_t secondJumpCounts{1000};
    double downwardZeroCaptureSeconds{1.5};
    double downwardZeroSettleTimeoutSeconds{30.0};
    std::int64_t firstDownwardMaximumSpanCounts{80};
    std::int64_t secondDownwardMaximumSpanCounts{40};
    double softwareLimitRecoveryVoltage{0.03};
};

inline json loadJson(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Cannot open configuration: " + path.string());
    }
    json value;
    input >> value;
    return value;
}

inline DoubleBalanceConfig loadDoubleConfig(const std::filesystem::path& path,
                                      std::optional<double> durationOverride) {
    const auto document = loadJson(path);
    const auto& ni = document.at("hardware").at("ni6602");
    const auto& source = document.at("double_balance_control");
    DoubleBalanceConfig config;
    config.secondCounter = ni.at("second_pendulum_counter").get<std::string>();
    config.secondATerminal = ni.at("second_pendulum_encoder_a_terminal").get<std::string>();
    config.secondBTerminal = ni.at("second_pendulum_encoder_b_terminal").get<std::string>();
    if (ni.at("second_pendulum_encoder_decoding").get<std::string>() != "X4") {
        throw std::runtime_error("Second pendulum encoder must use X4 decoding");
    }
    config.secondFilterSeconds =
        ni.at("second_pendulum_encoder_filter_min_pulse_width_us").get<double>() * 1e-6;

    const auto frequency = source.at("frequency_hz").get<double>();
    config.controller.sampleSeconds = 1.0 / frequency;
    const auto travelCounts = source.at("cart_travel_counts").get<double>();
    config.controller.cartMetersPerCount =
        source.at("cart_travel_meters").get<double>() / travelCounts;
    config.controller.firstCountsPerRevolution =
        source.at("first_counts_per_revolution").get<std::int64_t>();
    config.controller.secondCountsPerRevolution =
        source.at("second_counts_per_revolution").get<std::int64_t>();
    const auto gains = source.at("lqr_gain").get<std::vector<double>>();
    if (gains.size() != config.controller.gain.size()) {
        throw std::runtime_error("double_balance_control.lqr_gain must have 6 values");
    }
    std::copy(gains.begin(), gains.end(), config.controller.gain.begin());
    config.controller.velocityFilterHz = source.at("velocity_filter_hz").get<double>();
    config.controller.accelerationLimit =
        source.at("acceleration_limit_m_s2").get<double>();
    config.controller.velocityReferenceLimit =
        source.at("velocity_reference_limit_m_s").get<double>();
    config.controller.velocityProportionalGain = source.at("velocity_kp").get<double>();
    config.controller.velocityIntegralGain = source.at("velocity_ki").get<double>();
    config.controller.voltageLimit = source.at("voltage_limit").get<double>();
    config.controller.stationaryVoltage = source.at("stationary_voltage").get<double>();
    config.durationSeconds = durationOverride.value_or(
        source.at("experiment_duration_seconds").get<double>());
    config.startAngleRadians = source.at("start_angle_degrees").get<double>() *
                               std::numbers::pi / 180.0;
    config.stopAngleRadians = source.at("stop_angle_degrees").get<double>() *
                              std::numbers::pi / 180.0;
    config.positionWarningMeters = source.at("position_warning_meters").get<double>();
    config.positionStopMeters = source.at("position_stop_meters").get<double>();
    config.sampleTimeoutSeconds = source.at("sample_timeout_seconds").get<double>();
    config.cartJumpCounts = source.at("cart_jump_counts_per_sample").get<std::int64_t>();
    config.firstJumpCounts = source.at("first_jump_counts_per_sample").get<std::int64_t>();
    config.secondJumpCounts = source.at("second_jump_counts_per_sample").get<std::int64_t>();
    const auto& swing = source.at("swing_up");
    config.downwardZeroCaptureSeconds =
        swing.at("downward_zero_capture_seconds").get<double>();
    config.downwardZeroSettleTimeoutSeconds =
        swing.at("downward_zero_settle_timeout_seconds").get<double>();
    config.firstDownwardMaximumSpanCounts =
        swing.at("first_downward_maximum_span_counts").get<std::int64_t>();
    config.secondDownwardMaximumSpanCounts =
        swing.at("second_downward_maximum_span_counts").get<std::int64_t>();
    config.controller.cartVelocityLimitMetersPerSecond =
        swing.at("cart_velocity_limit_m_s").get<double>();
    config.controller.stage1Gain1 = swing.at("stage1_gain1").get<double>();
    config.controller.stage1Gain2 = swing.at("stage1_gain2").get<double>();
    config.controller.stage1Gain3 = swing.at("stage1_gain3").get<double>();
    config.controller.stage2FarGain = swing.at("stage2_far_gain").get<double>();
    config.controller.stage2NearGain = swing.at("stage2_near_gain").get<double>();
    config.controller.assistRate2RadiansPerSecond =
        swing.at("capture_assist_rate2_rad_s").get<double>();
    config.controller.secondEnergyTargetJoules =
        swing.at("second_energy_target_joules").get<double>();
    config.controller.balanceReentryAngle1Radians =
        swing.at("balance_reentry_angle1_degrees").get<double>() *
        std::numbers::pi / 180.0;
    config.controller.balanceReentryAngle2Radians =
        swing.at("balance_reentry_angle2_degrees").get<double>() *
        std::numbers::pi / 180.0;
    config.controller.softTrackLimitMeters =
        swing.at("soft_track_limit_meters").get<double>();
    config.controller.trackLimitMeters =
        swing.at("track_limit_meters").get<double>();
    config.softwareLimitRecoveryVoltage =
        swing.at("software_limit_recovery_voltage").get<double>();

    const auto motorCounter = ni.at("motor_counter").get<std::string>();
    const auto firstCounter = ni.at("pendulum_counter").get<std::string>();
    if (config.secondCounter.empty() || config.secondCounter == motorCounter ||
        config.secondCounter == firstCounter ||
        !std::isfinite(config.secondFilterSeconds) || config.secondFilterSeconds <= 0.0 ||
        config.durationSeconds < 0.0 ||
        frequency != 200.0 || travelCounts <= 0.0 ||
        config.controller.firstCountsPerRevolution != 8000 ||
        config.controller.secondCountsPerRevolution != 4000 ||
        config.startAngleRadians <= 0.0 ||
        config.stopAngleRadians <= config.startAngleRadians ||
        config.positionWarningMeters <= 0.0 ||
        config.positionStopMeters <= config.positionWarningMeters ||
        config.controller.voltageLimit > 1.0 ||
        config.controller.accelerationLimit > 30.0 ||
        config.sampleTimeoutSeconds <= config.controller.sampleSeconds ||
        config.cartJumpCounts <= 0 || config.firstJumpCounts <= 0 ||
        config.secondJumpCounts <= 0 || config.downwardZeroCaptureSeconds <= 0.0 ||
        config.downwardZeroSettleTimeoutSeconds <
            config.downwardZeroCaptureSeconds ||
        config.firstDownwardMaximumSpanCounts <= 0 ||
        config.secondDownwardMaximumSpanCounts <= 0 ||
        config.controller.trackLimitMeters > config.positionWarningMeters ||
        config.softwareLimitRecoveryVoltage <= 0.0 ||
        config.softwareLimitRecoveryVoltage > config.controller.voltageLimit) {
        throw std::runtime_error("Invalid or unsafe double_balance_control settings");
    }
    // Constructing validates all controller fields and gains.
    static_cast<void>(pendulum::control::DoublePendulumLqrController(config.controller));
    return config;
}

}
