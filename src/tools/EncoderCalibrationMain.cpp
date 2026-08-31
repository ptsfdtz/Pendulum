#include "pendulum/calibration/MotorEncoderCalibration.h"
#include "pendulum/config/Config.h"
#include "pendulum/hardware/NI6602.h"
#include "pendulum/logging/AsyncLogger.h"
#include "pendulum/safety/ProcessSafety.h"
#include "pendulum/safety/SafetyManager.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Options {
    std::filesystem::path configPath{"config/config.json"};
    std::size_t samples{25};
    std::uint32_t intervalMilliseconds{20};
    std::uint64_t maximumStabilitySpanCounts{20};
    bool probeOnly{false};
};

std::uint64_t parseUnsigned(const std::string& text, const char* name) {
    std::size_t consumed = 0;
    const auto value = std::stoull(text, &consumed);
    if (consumed != text.size() || value == 0) {
        throw std::invalid_argument(std::string(name) + " must be a positive integer");
    }
    return value;
}

Options parseOptions(int argc, char* argv[]) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        auto requireValue = [&]() -> std::string {
            if (++index >= argc) {
                throw std::invalid_argument(argument + " requires a value");
            }
            return argv[index];
        };
        if (argument == "--config") {
            options.configPath = requireValue();
        } else if (argument == "--probe") {
            options.probeOnly = true;
        } else if (argument == "--samples") {
            options.samples = static_cast<std::size_t>(parseUnsigned(requireValue(), "samples"));
        } else if (argument == "--interval-ms") {
            options.intervalMilliseconds =
                static_cast<std::uint32_t>(parseUnsigned(requireValue(), "interval-ms"));
        } else if (argument == "--max-stability-span-counts") {
            options.maximumStabilitySpanCounts =
                parseUnsigned(requireValue(), "max-stability-span-counts");
        } else if (argument == "--help" || argument == "-h") {
            std::cout
                << "Usage: pendulum_encoder_calibrate [options]\n\n"
                << "This tool never commands AO or Servo ON. Move the cart manually.\n\n"
                << "Options:\n"
                << "  --probe                             Print raw counts without calibration\n"
                << "  --config PATH                       Configuration file\n"
                << "  --samples N                         Samples per endpoint (default 25)\n"
                << "  --interval-ms N                     Sample interval (default 20 ms)\n"
                << "  --max-stability-span-counts N       Maximum endpoint span (default 20)\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("Unknown argument: " + argument);
        }
    }
    return options;
}

void waitForEnter(const std::string& message) {
    std::cout << message << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) {
        throw std::runtime_error("Input stream closed while waiting for confirmation");
    }
}

double readDistanceMillimeters() {
    std::cout << "Enter the measured travel distance in millimeters: " << std::flush;
    std::string text;
    if (!std::getline(std::cin, text)) {
        throw std::runtime_error("Input stream closed while reading distance");
    }
    std::size_t consumed = 0;
    const double distance = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(distance) || distance <= 0.0) {
        throw std::invalid_argument("Distance must be a positive finite number");
    }
    return distance;
}

std::uint32_t captureStableCount(pendulum::hardware::NI6602& ni, const Options& options) {
    std::vector<std::uint32_t> samples;
    samples.reserve(options.samples);
    for (std::size_t index = 0; index < options.samples; ++index) {
        samples.push_back(ni.readMotorEncoderRaw());
        if (index + 1 < options.samples) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(options.intervalMilliseconds));
        }
    }
    return pendulum::calibration::MotorEncoderCalibration::stableRepresentative(
        samples, options.maximumStabilitySpanCounts);
}

int run(const Options& options) {
    auto config = pendulum::config::AppConfig::load(options.configPath);
    config.validateForMotorEncoderCalibration();

    pendulum::logging::AsyncLogger logger(config.logging.directory, config.logging.queueCapacity);
    pendulum::safety::SafetyManager safety([&logger](const std::string& reason) {
        logger.log(pendulum::logging::Level::Critical, "Safety", reason);
    });
    pendulum::safety::ProcessSafetyHooks processHooks(safety);

    try {
        std::cout << "READ-ONLY ENCODER CALIBRATION\n"
                  << "Servo must be OFF. This program does not create AO or Servo output tasks.\n"
                  << "Counter: " << config.ni6602.motorCounter << '\n'
                  << "A terminal: " << config.ni6602.motorEncoderATerminal << '\n'
                  << "B terminal: " << config.ni6602.motorEncoderBTerminal << '\n'
                  << "Decoder: X4, PPR: "
                  << config.ni6602.motorEncoderPulsesPerRevolution << ", counts/rev: "
                  << config.ni6602.motorEncoderCountsPerRevolution << "\n\n";

        pendulum::hardware::NI6602 ni;
        ni.configureMotorEncoder(config.ni6602.motorCounter,
                                 config.ni6602.motorEncoderATerminal,
                                 config.ni6602.motorEncoderBTerminal,
                                 config.ni6602.motorEncoderPulsesPerRevolution);

        if (options.probeOnly) {
            const auto first = ni.readMotorEncoderRaw();
            std::cout << "sample,raw_count,delta_from_first\n"
                      << "0," << first << ",0\n";
            for (std::size_t index = 1; index < options.samples; ++index) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(options.intervalMilliseconds));
                const auto raw = ni.readMotorEncoderRaw();
                const auto delta =
                    pendulum::calibration::MotorEncoderCalibration::deltaWithRollover(first, raw);
                std::cout << index << ',' << raw << ',' << delta << '\n';
            }
            logger.log(pendulum::logging::Level::Info, "EncoderCalibration",
                       "Read-only motor encoder probe completed");
            return 0;
        }

        waitForEnter("Place the cart at the first measurement mark, keep it still, then press Enter: ");
        const auto start = captureStableCount(ni, options);
        std::cout << "Start raw count: " << start << '\n';

        waitForEnter("Move the cart manually by a measured distance, keep it still, then press Enter: ");
        const auto end = captureStableCount(ni, options);
        std::cout << "End raw count: " << end << '\n';

        const double distance = readDistanceMillimeters();
        const auto result = pendulum::calibration::MotorEncoderCalibration::calculate(
            start, end, distance);
        std::cout << "Signed encoder delta: " << result.signedDeltaCounts << " counts\n"
                  << "Calculated counts_per_mm: " << result.countsPerMillimeter << "\n"
                  << "Type SAVE to write this calibration to config.json: " << std::flush;
        std::string confirmation;
        if (!std::getline(std::cin, confirmation) || confirmation != "SAVE") {
            throw std::runtime_error("Calibration was not saved because SAVE was not entered");
        }

        pendulum::config::AppConfig::saveMotorEncoderCalibration(
            options.configPath,
            pendulum::config::MotorEncoderCalibrationRecord{
                result.startRawCount, result.endRawCount, result.signedDeltaCounts,
                result.measuredDistanceMillimeters, result.countsPerMillimeter});
        logger.log(pendulum::logging::Level::Info, "EncoderCalibration",
                   "Saved counts_per_mm=" + std::to_string(result.countsPerMillimeter));
        std::cout << "Calibration saved. Original config retained as a timestamped backup.\n";
        return 0;
    } catch (...) {
        safety.emergencyStop("Motor encoder calibration aborted");
        throw;
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        return run(parseOptions(argc, argv));
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
