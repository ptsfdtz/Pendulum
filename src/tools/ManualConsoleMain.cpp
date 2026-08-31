#include "pendulum/calibration/MotorEncoderCalibration.h"
#include "pendulum/config/Config.h"
#include "pendulum/hardware/NI6602.h"
#include "pendulum/hardware/PCI1723.h"
#include "pendulum/logging/AsyncLogger.h"
#include "pendulum/safety/ProcessSafety.h"
#include "pendulum/safety/SafetyManager.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using namespace std::chrono_literals;

struct Options {
    std::filesystem::path configPath{"config/config.json"};
};

Options parseOptions(int argc, char* argv[]) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--config" && ++index < argc) {
            options.configPath = argv[index];
        } else if (argument == "--help" || argument == "-h") {
            std::cout << "Usage: pendulum_manual_console [--config PATH]\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("Unknown or incomplete argument: " + argument);
        }
    }
    return options;
}

std::string trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

class ManualConsole final {
public:
    ManualConsole(pendulum::config::AppConfig config,
                  pendulum::logging::AsyncLogger& logger,
                  pendulum::safety::SafetyManager& safety)
        : config_(std::move(config)), logger_(logger), safety_(safety) {}

    int run() {
        config_.validateForManualConsole();
        initializeHardware();
        monitor_ = std::jthread([this](std::stop_token stopToken) { monitor(stopToken); });
        waitForFirstSample();

        record("Console ready");
        printHelp();
        std::string line;
        while (!safety_.stopRequested()) {
            std::cout << "pendulum> " << std::flush;
            if (!std::getline(std::cin, line)) {
                break;
            }
            if (!execute(trim(line))) {
                break;
            }
        }
        stopOutputs("console exit");
        monitor_.request_stop();
        return safety_.stopRequested() ? 1 : 0;
    }

private:
    void initializeHardware() {
        analogOutput_.open(config_.pci1723.deviceDescription, config_.pci1723.aoChannel,
                           config_.pci1723.minimumVoltage, config_.pci1723.maximumVoltage);
        aoRegistration_ = safety_.registerAction(0, "AO0 zero", [this] {
            std::scoped_lock lock(outputMutex_);
            analogOutput_.forceZeroVolts();
            commandedVoltage_.store(0.0);
        });
        analogOutput_.writeVoltage(0.0);

        ni_.configureServoOutput(config_.ni6602.servoEnableLine,
                                 config_.ni6602.servoActiveHigh);
        servoRegistration_ = safety_.registerAction(10, "Servo OFF", [this] {
            std::scoped_lock lock(outputMutex_);
            ni_.forceServoOff();
            servoOn_.store(false);
        });
        ni_.setServoEnabled(false);
        ni_.configureLimitInputs(config_.ni6602.leftLimitLine,
                                 config_.ni6602.leftLimitActiveHigh,
                                 config_.ni6602.rightLimitLine,
                                 config_.ni6602.rightLimitActiveHigh);
        ni_.configureMotorEncoder(config_.ni6602.motorCounter,
                                  config_.ni6602.motorEncoderATerminal,
                                  config_.ni6602.motorEncoderBTerminal,
                                  config_.ni6602.motorEncoderPulsesPerRevolution);
        record("Hardware initialized at AO0=0 V and Servo OFF");
    }

    void waitForFirstSample() {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (!sampleReady_.load() && !faultLatched_.load() &&
               std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(10ms);
        }
        if (!sampleReady_.load()) {
            throw std::runtime_error("Hardware monitor did not produce an initial sample");
        }
    }

    void monitor(std::stop_token stopToken) noexcept {
        std::uint32_t leftCount = 0;
        std::uint32_t rightCount = 0;
        std::uint32_t samplesSeen = 0;
        try {
            while (!stopToken.stop_requested() && !safety_.stopRequested()) {
                const auto limits = ni_.readLimitInputs();
                const auto encoder = ni_.readMotorEncoderRaw();
                leftRawHigh_.store(limits.leftRawHigh);
                rightRawHigh_.store(limits.rightRawHigh);
                encoderRaw_.store(encoder);
                leftCount = limits.leftTriggered ? leftCount + 1 : 0;
                rightCount = limits.rightTriggered ? rightCount + 1 : 0;
                const bool leftStable =
                    leftCount >= config_.manualConsole.limitDebounceSamples;
                const bool rightStable =
                    rightCount >= config_.manualConsole.limitDebounceSamples;
                leftTriggered_.store(leftStable);
                rightTriggered_.store(rightStable);
                if (samplesSeen < config_.manualConsole.limitDebounceSamples) {
                    ++samplesSeen;
                }
                sampleReady_.store(samplesSeen >=
                                   config_.manualConsole.limitDebounceSamples);

                const bool anyLimit = leftStable || rightStable;
                if (anyLimit && !limitStopActive_.exchange(true)) {
                    stopOutputs(std::string("limit triggered: ") +
                                (leftStable ? "LEFT" : "RIGHT"));
                    record(std::string("LIMIT STOP: ") +
                               (leftStable ? "LEFT" : "RIGHT"),
                           pendulum::logging::Level::Critical);
                    std::cerr << "\nLIMIT: " << (leftStable ? "LEFT" : "RIGHT")
                              << "; AO0=0 V, Servo OFF\n";
                } else if (!anyLimit) {
                    limitStopActive_.store(false);
                }
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(config_.manualConsole.monitorPeriodMilliseconds));
            }
        } catch (const std::exception& error) {
            trip(std::string("hardware monitor failure: ") + error.what());
        } catch (...) {
            trip("unknown hardware monitor failure");
        }
    }

    void trip(const std::string& reason) noexcept {
        bool expected = false;
        if (!faultLatched_.compare_exchange_strong(expected, true)) {
            return;
        }
        stopOutputs(reason);
        record("FAULT: " + reason, pendulum::logging::Level::Critical);
        std::cerr << "\nFAULT: " << reason << "; AO0=0 V, Servo OFF\n";
    }

    void stopOutputs(const std::string& reason) noexcept {
        std::scoped_lock lock(outputMutex_);
        analogOutput_.forceZeroVolts();
        commandedVoltage_.store(0.0);
        ni_.forceServoOff();
        servoOn_.store(false);
        logger_.log(pendulum::logging::Level::Warning, "ManualConsole",
                    "Outputs stopped: " + reason);
    }

    bool outputsAllowed() const noexcept {
        return sampleReady_.load() && !faultLatched_.load() &&
               !leftTriggered_.load() && !rightTriggered_.load();
    }

    bool execute(const std::string& line) {
        if (line.empty()) {
            return true;
        }
        std::istringstream input(line);
        std::string command;
        input >> command;
        if (command == "help") {
            printHelp();
        } else if (command == "status") {
            printStatus();
        } else if (command == "limits") {
            printLimits();
        } else if (command == "encoder") {
            std::cout << "encoder_raw=" << encoderRaw_.load() << '\n';
        } else if (command == "log") {
            printLog();
        } else if (command == "servo") {
            std::string state;
            input >> state;
            setServo(state);
        } else if (command == "voltage") {
            double voltage = 0.0;
            if (!(input >> voltage)) {
                std::cout << "Usage: voltage <volts> [duration_ms]\n";
            } else {
                std::uint32_t duration = 0;
                if (input >> duration) {
                    setVoltage(voltage, duration);
                } else {
                    setVoltage(voltage, std::nullopt);
                }
            }
        } else if (command == "quit" || command == "exit") {
            return false;
        } else {
            std::cout << "Unknown command. Enter 'help'.\n";
        }
        return true;
    }

    void setServo(const std::string& state) {
        if (state == "off") {
            stopOutputs("operator servo off");
            record("Operator command: servo off");
            std::cout << "Servo OFF; AO0=0 V.\n";
            return;
        }
        if (state != "on") {
            std::cout << "Usage: servo on|off\n";
            return;
        }
        if (!outputsAllowed()) {
            std::cout << "Servo ON rejected: a limit is active or limit monitoring failed.\n";
            record("Rejected command: servo on", pendulum::logging::Level::Warning);
            return;
        }
        {
            std::scoped_lock lock(outputMutex_);
            if (!outputsAllowed()) {
                std::cout << "Servo ON rejected: safety state changed.\n";
                return;
            }
            analogOutput_.writeVoltage(0.0);
            commandedVoltage_.store(0.0);
            ni_.setServoEnabled(true);
            servoOn_.store(true);
        }
        record("Operator command: servo on");
        std::cout << "Servo ON; AO0=0 V.\n";
    }

    void setVoltage(double voltage, std::optional<std::uint32_t> durationMilliseconds) {
        if (!outputsAllowed() || !servoOn_.load()) {
            std::cout << "Voltage rejected: use 'servo on' first and ensure limits are clear.\n";
            return;
        }
        if (!std::isfinite(voltage) || voltage < config_.pci1723.minimumVoltage ||
            voltage > config_.pci1723.maximumVoltage) {
            std::cout << "Voltage must be within the configured hardware range "
                      << config_.pci1723.minimumVoltage << ".."
                      << config_.pci1723.maximumVoltage << " V.\n";
            return;
        }
        if (durationMilliseconds.has_value() && *durationMilliseconds == 0) {
            std::cout << "Duration must be greater than zero.\n";
            return;
        }

        const auto start = encoderRaw_.load();
        {
            std::scoped_lock lock(outputMutex_);
            if (!outputsAllowed() || !servoOn_.load()) {
                std::cout << "Voltage rejected: safety state changed.\n";
                return;
            }
            analogOutput_.writeVoltage(voltage);
            commandedVoltage_.store(voltage);
        }
        std::ostringstream event;
        event << "Voltage set to " << voltage << " V";
        if (voltage != 0.0) {
            event << "; expected cart direction="
                  << (voltage > 0.0 ? config_.pci1723.positiveVoltageCartDirection
                                    : config_.pci1723.negativeVoltageCartDirection);
        }
        if (durationMilliseconds.has_value()) {
            event << " for " << *durationMilliseconds << " ms";
        }
        record(event.str());

        if (!durationMilliseconds.has_value()) {
            std::cout << "AO0=" << voltage << " V; Servo remains ON";
            if (voltage != 0.0) {
                std::cout << "; expected cart direction="
                          << (voltage > 0.0
                                  ? config_.pci1723.positiveVoltageCartDirection
                                  : config_.pci1723.negativeVoltageCartDirection);
            }
            std::cout << ".\n";
            return;
        }

        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(*durationMilliseconds);
        while (std::chrono::steady_clock::now() < deadline && !faultLatched_.load() &&
               !safety_.stopRequested()) {
            std::this_thread::sleep_for(1ms);
        }
        stopOutputs("voltage pulse complete");
        const auto end = encoderRaw_.load();
        const auto delta = pendulum::calibration::MotorEncoderCalibration::deltaWithRollover(
            start, end);
        const char* direction = delta > 0 ? "positive counts" :
                                delta < 0 ? "negative counts" : "no detected movement";
        std::cout << "Timed voltage stopped; Servo OFF. encoder_start=" << start
                  << ", encoder_end=" << end << ", delta=" << delta
                  << " (" << direction << ")\n";
    }

    void printStatus() const {
        std::cout << std::boolalpha
                  << "servo_on=" << servoOn_.load()
                  << ", ao_voltage=" << commandedVoltage_.load()
                  << ", fault=" << faultLatched_.load() << '\n';
        printLimits();
        std::cout << "encoder_raw=" << encoderRaw_.load() << '\n';
    }

    void printLimits() const {
        std::cout << "left: raw=" << (leftRawHigh_.load() ? "HIGH" : "LOW")
                  << ", state=" << (leftTriggered_.load() ? "TRIGGERED" : "CLEAR")
                  << "; right: raw=" << (rightRawHigh_.load() ? "HIGH" : "LOW")
                  << ", state=" << (rightTriggered_.load() ? "TRIGGERED" : "CLEAR")
                  << '\n';
    }

    void printHelp() const {
        std::cout
            << "Commands:\n"
            << "  status | limits | encoder | log | help | quit\n"
            << "  servo on | servo off\n"
            << "  voltage <volts>                 Set and hold AO voltage\n"
            << "  voltage <volts> <duration_ms>   Timed voltage, then Servo OFF\n";
    }

    void record(const std::string& message,
                pendulum::logging::Level level = pendulum::logging::Level::Info) noexcept {
        logger_.log(level, "ManualConsole", message);
        try {
            std::scoped_lock lock(historyMutex_);
            history_.push_back(message);
            if (history_.size() > 50) {
                history_.pop_front();
            }
        } catch (...) {
        }
    }

    void printLog() const {
        std::scoped_lock lock(historyMutex_);
        for (const auto& entry : history_) {
            std::cout << "  " << entry << '\n';
        }
        std::cout << "CSV log: " << logger_.path().string() << '\n';
    }

    pendulum::config::AppConfig config_;
    pendulum::logging::AsyncLogger& logger_;
    pendulum::safety::SafetyManager& safety_;
    pendulum::hardware::NI6602 ni_;
    pendulum::hardware::PCI1723 analogOutput_;
    pendulum::safety::SafetyManager::Registration aoRegistration_;
    pendulum::safety::SafetyManager::Registration servoRegistration_;
    std::jthread monitor_;
    mutable std::mutex outputMutex_;
    mutable std::mutex historyMutex_;
    std::deque<std::string> history_;
    std::atomic<bool> sampleReady_{false};
    std::atomic<bool> leftRawHigh_{false};
    std::atomic<bool> rightRawHigh_{false};
    std::atomic<bool> leftTriggered_{false};
    std::atomic<bool> rightTriggered_{false};
    std::atomic<bool> faultLatched_{false};
    std::atomic<bool> limitStopActive_{false};
    std::atomic<bool> servoOn_{false};
    std::atomic<double> commandedVoltage_{0.0};
    std::atomic<std::uint32_t> encoderRaw_{0};
};

int runApplication(const Options& options) {
    auto config = pendulum::config::AppConfig::load(options.configPath);
    pendulum::logging::AsyncLogger logger(config.logging.directory,
                                           config.logging.queueCapacity);
    pendulum::safety::SafetyManager safety([&logger](const std::string& reason) {
        logger.log(pendulum::logging::Level::Critical, "Safety", reason);
    });
    pendulum::safety::ProcessSafetyHooks processHooks(safety);
    pendulum::safety::SafetyGuard guard(safety);
    ManualConsole console(std::move(config), logger, safety);
    const int result = console.run();
    std::cout << "Log: " << logger.path().string() << '\n';
    return result;
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
