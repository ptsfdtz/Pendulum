#include "pendulum/config/Config.h"
#include "pendulum/hardware/NI6602.h"
#include "pendulum/hardware/PCI1723.h"
#include "pendulum/logging/AsyncLogger.h"
#include "pendulum/safety/ProcessSafety.h"
#include "pendulum/safety/SafetyManager.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

enum class Mode { EnumerateOnly, InputProbe, EncoderProbe, SafeOutputTest };

struct Options {
    std::filesystem::path configPath{"config/config.json"};
    Mode mode{Mode::EnumerateOnly};
};

Options parseOptions(int argc, char* argv[]) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--config") {
            if (++index >= argc) {
                throw std::invalid_argument("--config requires a path");
            }
            options.configPath = argv[index];
        } else if (argument == "--enumerate-only") {
            options.mode = Mode::EnumerateOnly;
        } else if (argument == "--input-probe") {
            options.mode = Mode::InputProbe;
        } else if (argument == "--encoder-probe") {
            options.mode = Mode::EncoderProbe;
        } else if (argument == "--safe-output-test") {
            options.mode = Mode::SafeOutputTest;
        } else if (argument == "--help" || argument == "-h") {
            std::cout
                << "Usage: pendulum_self_test [--config PATH] [MODE]\n\n"
                << "Modes:\n"
                << "  --enumerate-only    Enumerate devices without creating output tasks (default)\n"
                << "  --input-probe       Read and interpret limit inputs without output tasks\n"
                << "  --encoder-probe     Read all three encoder counters without output tasks\n"
                << "  --safe-output-test  Write AO0=0 V and Servo OFF; config authorization required\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("Unknown argument: " + argument);
        }
    }
    return options;
}

void runEncoderProbe(const pendulum::config::AppConfig& config,
                     pendulum::logging::AsyncLogger& logger) {
    config.validateForManualConsole();
    pendulum::hardware::NI6602 ni;
    ni.configureMotorEncoder(
        config.ni6602.motorCounter, config.ni6602.motorEncoderATerminal,
        config.ni6602.motorEncoderBTerminal,
        config.ni6602.motorEncoderPulsesPerRevolution,
        config.ni6602.motorEncoderFilterMinPulseWidthMicroseconds * 1e-6);
    ni.configurePendulumEncoderRaw(
        config.ni6602.pendulumCounter,
        config.ni6602.pendulumEncoderATerminal,
        config.ni6602.pendulumEncoderBTerminal,
        config.ni6602.pendulumEncoderFilterMinPulseWidthMicroseconds * 1e-6);
    ni.configureSecondPendulumEncoderRaw(
        config.ni6602.secondPendulumCounter,
        config.ni6602.secondPendulumEncoderATerminal,
        config.ni6602.secondPendulumEncoderBTerminal,
        config.ni6602.secondPendulumEncoderFilterMinPulseWidthMicroseconds *
            1e-6);

    const auto motor = ni.readMotorEncoderRaw();
    const auto first = ni.readPendulumEncoderRaw();
    const auto second = ni.readSecondPendulumEncoderRaw();
    std::cout << "Cart encoder " << config.ni6602.motorCounter
              << ": raw=" << motor << '\n'
              << "First pendulum " << config.ni6602.pendulumCounter
              << ": raw=" << first << '\n'
              << "Second pendulum " << config.ni6602.secondPendulumCounter
              << ": raw=" << second << '\n';
    logger.log(pendulum::logging::Level::Info, "EncoderProbe",
               "motor=" + std::to_string(motor) +
                   ", first=" + std::to_string(first) +
                   ", second=" + std::to_string(second));
}

bool readStableDigitalLine(const std::string& line, std::size_t samples = 10) {
    const bool first = pendulum::hardware::NI6602::readDigitalLine(line);
    for (std::size_t index = 1; index < samples; ++index) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (pendulum::hardware::NI6602::readDigitalLine(line) != first) {
            throw std::runtime_error("Digital input is unstable during probe: " + line);
        }
    }
    return first;
}

void runInputProbe(const pendulum::config::AppConfig& config,
                   pendulum::logging::AsyncLogger& logger) {
    if (!pendulum::config::isConfirmed(config.ni6602.leftLimitLine) ||
        !pendulum::config::isConfirmed(config.ni6602.rightLimitLine)) {
        throw std::runtime_error("Limit input lines are not confirmed");
    }
    const bool leftRaw = readStableDigitalLine(config.ni6602.leftLimitLine);
    const bool rightRaw = readStableDigitalLine(config.ni6602.rightLimitLine);
    const bool leftTriggered = leftRaw == config.ni6602.leftLimitActiveHigh;
    const bool rightTriggered = rightRaw == config.ni6602.rightLimitActiveHigh;

    std::cout << "Left limit  " << config.ni6602.leftLimitLine << ": raw="
              << (leftRaw ? "HIGH" : "LOW") << ", state="
              << (leftTriggered ? "TRIGGERED" : "CLEAR") << '\n'
              << "Right limit " << config.ni6602.rightLimitLine << ": raw="
              << (rightRaw ? "HIGH" : "LOW") << ", state="
              << (rightTriggered ? "TRIGGERED" : "CLEAR") << '\n';
    logger.log(pendulum::logging::Level::Info, "LimitProbe",
               std::string("left=") + (leftTriggered ? "TRIGGERED" : "CLEAR") +
                   ", right=" + (rightTriggered ? "TRIGGERED" : "CLEAR"));
    if (leftTriggered && rightTriggered) {
        throw std::runtime_error("Both mechanical limits are triggered simultaneously");
    }
}

template <typename Range, typename Predicate>
bool containsMatching(const Range& values, Predicate predicate) {
    return std::find_if(values.begin(), values.end(), predicate) != values.end();
}

void verifyAndReportDevices(const pendulum::config::AppConfig& config,
                            pendulum::logging::AsyncLogger& logger) {
    const auto niDevices = pendulum::hardware::NI6602::enumerateDevices();
    std::cout << "NI-DAQmx devices:\n";
    for (const auto& device : niDevices) {
        std::cout << "  " << device.name << " = " << device.productType << '\n';
        logger.log(pendulum::logging::Level::Info, "NI6602",
                   "Found " + device.name + " = " + device.productType);
    }
    const bool niFound = containsMatching(niDevices, [&](const auto& device) {
        return device.name == config.ni6602.device &&
               device.productType == config.ni6602.expectedProduct;
    });
    if (!niFound) {
        throw std::runtime_error("Configured NI device/product pair was not found");
    }

    const auto advantechDevices = pendulum::hardware::PCI1723::enumerateDevices();
    std::cout << "Advantech DAQNavi devices:\n";
    for (const auto& device : advantechDevices) {
        std::cout << "  " << device << '\n';
        logger.log(pendulum::logging::Level::Info, "PCI1723", "Found " + device);
    }
    if (!containsMatching(advantechDevices, [&](const auto& device) {
            return device == config.pci1723.deviceDescription;
        })) {
        throw std::runtime_error("Configured Advantech device was not found");
    }
}

void runSafeOutputTest(const pendulum::config::AppConfig& config,
                       pendulum::safety::SafetyManager& safety,
                       pendulum::logging::AsyncLogger& logger) {
    config.validateForSafeOutputTest();
    logger.log(pendulum::logging::Level::Warning, "SelfTest",
               "Authorized safe output test starting");

    pendulum::hardware::PCI1723 analogOutput;
    analogOutput.open(config.pci1723.deviceDescription, config.pci1723.aoChannel,
                      config.pci1723.minimumVoltage, config.pci1723.maximumVoltage);
    auto aoRegistration = safety.registerAction(
        0, "AO0 zero", [&analogOutput] { analogOutput.forceZeroVolts(); });
    analogOutput.writeVoltage(config.pci1723.safeVoltage);
    logger.log(pendulum::logging::Level::Info, "PCI1723", "AO channel written to 0 V");

    pendulum::hardware::NI6602 ni;
    ni.configureServoOutput(config.ni6602.servoEnableLine, config.ni6602.servoActiveHigh);
    auto servoRegistration = safety.registerAction(
        10, "Servo OFF", [&ni] { ni.forceServoOff(); });
    pendulum::safety::SafetyGuard guard(safety);
    logger.log(pendulum::logging::Level::Info, "NI6602", "Servo output written OFF");

    std::cout << "Safe output test passed: AO" << config.pci1723.aoChannel
              << " = 0 V, Servo OFF.\n";
}

int runApplication(const Options& options) {
    auto config = pendulum::config::AppConfig::load(options.configPath);
    pendulum::logging::AsyncLogger logger(config.logging.directory, config.logging.queueCapacity);
    pendulum::safety::SafetyManager safety([&logger](const std::string& reason) {
        logger.log(pendulum::logging::Level::Critical, "Safety", reason);
    });
    pendulum::safety::ProcessSafetyHooks processHooks(safety);

    try {
        logger.log(pendulum::logging::Level::Info, "SelfTest", "Phase 1 self-test started");
        verifyAndReportDevices(config, logger);
        if (options.mode == Mode::SafeOutputTest) {
            runSafeOutputTest(config, safety, logger);
        } else if (options.mode == Mode::EncoderProbe) {
            runEncoderProbe(config, logger);
            std::cout << "Encoder probe passed. No output tasks were created.\n";
        } else if (options.mode == Mode::InputProbe) {
            runInputProbe(config, logger);
            std::cout << "Input probe passed. No output tasks were created.\n";
        } else {
            std::cout << "Enumeration-only self-test passed. No output tasks were created.\n";
            logger.log(pendulum::logging::Level::Info, "SelfTest",
                       "Enumeration-only self-test passed; no output tasks created");
        }
        std::cout << "Log: " << logger.path().string() << '\n';
        return 0;
    } catch (...) {
        safety.emergencyStop("Phase 1 self-test exception");
        logger.log(pendulum::logging::Level::Error, "SelfTest", "Self-test failed");
        throw;
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        return runApplication(parseOptions(argc, argv));
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    } catch (...) {
        std::cerr << "FAILED: unknown exception\n";
        return 2;
    }
}
