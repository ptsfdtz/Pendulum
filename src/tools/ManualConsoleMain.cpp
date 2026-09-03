#include "pendulum/calibration/HomeCenterController.h"
#include "pendulum/calibration/MotorEncoderCalibration.h"
#include "pendulum/calibration/MotorZeroCalibrator.h"
#include "pendulum/config/Config.h"
#include "pendulum/control/ReferenceLqrVelocityController.h"
#include "pendulum/hardware/NI6602.h"
#include "pendulum/hardware/PCI1723.h"
#include "pendulum/logging/AsyncLogger.h"
#include "pendulum/safety/ProcessSafety.h"
#include "pendulum/safety/SafetyManager.h"

#include <Windows.h>
#include <conio.h>
#include <io.h>
#include <mmsystem.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numbers>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;


struct Options {
    std::filesystem::path configPath{"config/config.json"};
};

class WindowsTimerResolution final {
public:
    explicit WindowsTimerResolution(UINT milliseconds)
        : milliseconds_(milliseconds), active_(timeBeginPeriod(milliseconds_) == TIMERR_NOERROR) {}

    ~WindowsTimerResolution() {
        if (active_) {
            timeEndPeriod(milliseconds_);
        }
    }

    bool active() const noexcept { return active_; }

private:
    UINT milliseconds_;
    bool active_;
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

std::int64_t stableRepresentative(std::vector<std::int64_t> samples,
                                  std::int64_t maximumSpanCounts) {
    if (samples.empty() || maximumSpanCounts < 0) {
        throw std::invalid_argument("Invalid pendulum stability window");
    }
    std::sort(samples.begin(), samples.end());
    if (samples.back() - samples.front() > maximumSpanCounts) {
        throw std::runtime_error("Pendulum is not stationary");
    }
    return samples[samples.size() / 2];
}

std::int64_t wrappedCounts(std::int64_t countDelta,
                           std::int64_t countsPerRevolution) {
    const auto half = countsPerRevolution / 2;
    auto wrapped = countDelta % countsPerRevolution;
    if (wrapped >= half) {
        wrapped -= countsPerRevolution;
    } else if (wrapped < -half) {
        wrapped += countsPerRevolution;
    }
    return wrapped;
}

class ManualConsole final {
public:
    ManualConsole(pendulum::config::AppConfig config,
                  std::filesystem::path configPath,
                  pendulum::logging::AsyncLogger& logger,
                  pendulum::safety::SafetyManager& safety)
        : config_(std::move(config)), configPath_(std::move(configPath)),
          logger_(logger), safety_(safety) {}

    int run() {
        config_.validateForManualConsole();
        WindowsTimerResolution timerResolution(1);
        initializeHardware();
        monitor_ = std::jthread([this](std::stop_token stopToken) { monitor(stopToken); });
        waitForFirstSample();
        static_cast<void>(captureStablePendulumZero("startup"));

        record("Console ready");
        if (!timerResolution.active()) {
            record("Windows 1 ms timer resolution request failed",
                   pendulum::logging::Level::Warning);
        }
        dashboardMode_ = enableDashboardTerminal();
        if (dashboardMode_) {
            dashboard_ = std::jthread(
                [this](std::stop_token stopToken) { dashboardLoop(stopToken); });
            while (!safety_.stopRequested()) {
                const auto line = readDashboardCommand();
                if (!line.has_value() || !execute(trim(*line))) {
                    break;
                }
            }
            dashboard_.request_stop();
            if (dashboard_.joinable()) {
                dashboard_.join();
            }
            std::scoped_lock lock(renderMutex_);
            std::cout << "\x1b[0m\x1b[2J\x1b[H" << std::flush;
        } else {
            printHelp();
            std::string line;
            while (!safety_.stopRequested()) {
                std::cout << "pendulum> " << std::flush;
                if (!std::getline(std::cin, line) || !execute(trim(line))) {
                    break;
                }
            }
        }
        stopBalance("console exit");
        stopOutputs("console exit");
        monitor_.request_stop();
        if (monitor_.joinable()) {
            monitor_.join();
        }
        return safety_.stopRequested() ? 1 : 0;
    }

private:
    struct PendulumSample {
        std::int64_t positionCounts{0};
        std::int64_t motorPositionCounts{0};
        std::chrono::steady_clock::time_point time{};
        std::uint64_t sequence{0};
    };

    struct CartSessionState {
        double positionFromCenterHalfTravel{0.0};
        double velocityHalfTravelPerSecond{0.0};
    };

    CartSessionState currentCartSessionState(double samplePeriodSeconds) const {
        if (!homeResultAvailable_.load()) {
            throw std::runtime_error(
                "fresh 'home center' result is required for cart feedback");
        }
        const auto center = homeCalibrationCenter_.load();
        const auto rightBoundary = homeCalibrationRightBoundary_.load();
        const auto rightHalfTravel = rightBoundary - center;
        if (rightHalfTravel == 0 || homeCalibrationTravel_.load() <= 0) {
            throw std::runtime_error("home-center session geometry is invalid");
        }

        CartSessionState state;
        state.positionFromCenterHalfTravel =
            static_cast<double>(motorPositionCounts_.load() - center) /
            static_cast<double>(rightHalfTravel);
        state.velocityHalfTravelPerSecond =
            motorSpeedCountsPerSecond_.load() /
            static_cast<double>(rightHalfTravel);
        static_cast<void>(samplePeriodSeconds);
        return state;
    }

    PendulumSample latestPendulumSample() const {
        std::scoped_lock lock(pendulumSampleMutex_);
        return pendulumSample_;
    }

    static std::string fit(std::string value, std::size_t width) {
        if (value.size() > width) {
            if (width > 3) {
                value.resize(width - 3);
                value += "...";
            } else {
                value.resize(width);
            }
        } else {
            value.append(width - value.size(), ' ');
        }
        return value;
    }

    static std::string border(std::string title, std::size_t width) {
        if (width < 4) {
            return std::string(width, '-');
        }
        std::string value = "+- " + std::move(title) + ' ';
        if (value.size() < width - 1) {
            value.append(width - 1 - value.size(), '-');
        } else {
            value.resize(width - 1);
        }
        value += '+';
        return value;
    }

    static std::string row(const std::string& content, std::size_t width) {
        if (width < 4) {
            return fit(content, width);
        }
        return "| " + fit(content, width - 4) + " |";
    }

    bool enableDashboardTerminal() const noexcept {
        if (_isatty(_fileno(stdin)) == 0 || _isatty(_fileno(stdout)) == 0) {
            return false;
        }
        const HANDLE output = GetStdHandle(STD_OUTPUT_HANDLE);
        DWORD mode = 0;
        if (output == INVALID_HANDLE_VALUE || !GetConsoleMode(output, &mode)) {
            return false;
        }
        return SetConsoleMode(output, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING) != FALSE;
    }

    std::optional<std::string> readDashboardCommand() {
        while (!safety_.stopRequested()) {
            if (_kbhit() == 0) {
                std::this_thread::sleep_for(10ms);
                continue;
            }
            const int character = _getwch();
            if (character == 0 || character == 0xE0) {
                const int extended = _getwch();
                std::scoped_lock lock(inputMutex_);
                if (extended == 72 && !commandHistory_.empty()) {
                    if (commandHistoryCursor_ >= commandHistory_.size()) {
                        commandHistoryDraft_ = inputBuffer_;
                        commandHistoryCursor_ = commandHistory_.size();
                    }
                    if (commandHistoryCursor_ > 0) {
                        --commandHistoryCursor_;
                        inputBuffer_ = commandHistory_[commandHistoryCursor_];
                    }
                } else if (extended == 80 &&
                           commandHistoryCursor_ < commandHistory_.size()) {
                    ++commandHistoryCursor_;
                    inputBuffer_ = commandHistoryCursor_ < commandHistory_.size()
                                       ? commandHistory_[commandHistoryCursor_]
                                       : commandHistoryDraft_;
                }
                continue;
            }
            if (character == '\r' || character == '\n') {
                std::scoped_lock lock(inputMutex_);
                auto command = inputBuffer_;
                if (!command.empty() &&
                    (commandHistory_.empty() || commandHistory_.back() != command)) {
                    commandHistory_.push_back(command);
                    if (commandHistory_.size() > 100) {
                        commandHistory_.pop_front();
                    }
                }
                inputBuffer_.clear();
                commandHistoryCursor_ = commandHistory_.size();
                commandHistoryDraft_.clear();
                return command;
            }
            if (character == '\b') {
                std::scoped_lock lock(inputMutex_);
                if (!inputBuffer_.empty()) {
                    inputBuffer_.pop_back();
                }
                commandHistoryCursor_ = commandHistory_.size();
                continue;
            }
            if (character >= 32 && character <= 126) {
                std::scoped_lock lock(inputMutex_);
                if (inputBuffer_.size() < 200) {
                    inputBuffer_.push_back(static_cast<char>(character));
                }
                commandHistoryCursor_ = commandHistory_.size();
            }
        }
        return std::nullopt;
    }

    void dashboardLoop(std::stop_token stopToken) noexcept {
        while (!stopToken.stop_requested() && !safety_.stopRequested()) {
            renderDashboard();
            std::this_thread::sleep_for(std::chrono::milliseconds(
                config_.manualConsole.dashboardRefreshMilliseconds));
        }
    }

    void renderDashboard() noexcept {
        try {
            CONSOLE_SCREEN_BUFFER_INFO info{};
            const HANDLE outputHandle = GetStdHandle(STD_OUTPUT_HANDLE);
            int consoleWidth = 100;
            int consoleHeight = 25;
            if (GetConsoleScreenBufferInfo(outputHandle, &info)) {
                consoleWidth = info.srWindow.Right - info.srWindow.Left + 1;
                consoleHeight = info.srWindow.Bottom - info.srWindow.Top + 1;
            }
            const auto width = static_cast<std::size_t>(std::max(20, consoleWidth));
            const bool showHelp = dashboardHelpVisible_.load();
            const int eventRows = showHelp ? 0 : std::clamp(consoleHeight - 21, 0, 20);

            std::deque<std::string> events;
            {
                std::scoped_lock lock(historyMutex_);
                const auto first = history_.size() > static_cast<std::size_t>(eventRows)
                                       ? history_.size() - static_cast<std::size_t>(eventRows)
                                       : 0;
                for (std::size_t index = first; index < history_.size(); ++index) {
                    events.push_back(history_[index]);
                }
            }
            std::string input;
            {
                std::scoped_lock lock(inputMutex_);
                input = inputBuffer_;
            }
            std::string message;
            {
                std::scoped_lock lock(messageMutex_);
                message = lastMessage_;
            }

            const bool left = leftTriggered_.load();
            const bool right = rightTriggered_.load();
            const bool servo = servoOn_.load();
            const double voltage = commandedVoltage_.load();
            std::string motion;
            if (faultLatched_.load()) {
                motion = "FAULT / OUTPUT DISABLED";
            } else if (balanceRunning_.load()) {
                motion = "BALANCING / ANGLE + CART FEEDBACK";
            } else if (homingRunning_.load()) {
                std::scoped_lock lock(homingStateMutex_);
                motion = "HOMING / " + homingState_;
            } else if (left || right) {
                motion = std::string("LIMIT STOP / ") + (left ? "LEFT" : "RIGHT");
            } else if (calibrationRunning_.load()) {
                motion = "CALIBRATING MOTOR ZERO";
            } else if (!servo) {
                motion = "IDLE / SERVO OFF";
            } else if (std::abs(voltage) < 1e-12) {
                motion = "SERVO ON / ZERO SPEED COMMAND";
            } else {
                motion = "MOVING " +
                         (voltage > 0.0 ? config_.pci1723.positiveVoltageCartDirection
                                        : config_.pci1723.negativeVoltageCartDirection);
            }

            std::ostringstream screen;
            screen << "\x1b[H\x1b[1;36m"
                   << fit(" PendulumLab  /  Hardware Commissioning Console", width)
                   << "\x1b[0m\n";
            screen << border("ENCODERS / X4 SIGNED COUNTS", width) << '\n';
            std::ostringstream motor;
            motor << "MOTOR     position " << std::setw(12) << motorPositionCounts_.load()
                  << "    rate " << std::showpos << std::fixed << std::setprecision(1)
                  << std::setw(12) << motorSpeedCountsPerSecond_.load() << " counts/s";
            screen << row(motor.str(), width) << '\n';
            std::ostringstream pendulum;
            pendulum << "PENDULUM  position " << std::setw(12)
                     << pendulumPositionCounts_.load()
                     << "    rate " << std::showpos << std::fixed << std::setprecision(1)
                     << std::setw(12) << pendulumSpeedCountsPerSecond_.load() << " counts/s";
            screen << row(pendulum.str(), width) << '\n';
            screen << border("BALANCE CONTROL / LIVE TERMS", width) << '\n';
            std::ostringstream balance;
            balance << "STATE     "
                    << (balanceRunning_.load() ? "RUNNING" : "STOPPED")
                    << "    run " << balanceRunId_.load()
                    << "    down-zero "
                    << (pendulumZeroCaptured_.load() ? "CAPTURED" : "NOT SET")
                    << "    target " << pendulumUprightCount_.load()
                    << "    angle " << std::showpos << std::fixed
                    << std::setprecision(3) << balanceAngleDegrees_.load()
                    << " deg    rate " << balanceAngularRateDegrees_.load()
                    << " deg/s    fixed-sign reference model";
            screen << "\x1b[1;37m" << row(balance.str(), width)
                   << "\x1b[0m\n";
            std::ostringstream angleTerms;
            angleTerms << "LQR       acc " << std::showpos << std::fixed
                       << std::setprecision(4) << balanceReferenceAcceleration_.load()
                       << " m/s^2    vref " << balanceReferenceVelocity_.load()
                       << " m/s";
            screen << "\x1b[1;36m" << row(angleTerms.str(), width)
                   << "\x1b[0m\n";
            std::ostringstream cartTerms;
            cartTerms << "CART      x " << std::showpos << std::fixed
                      << std::setprecision(5) << balanceCartPositionMeters_.load()
                      << " m    v " << balanceCartVelocityMetersPerSecond_.load()
                      << " m/s    verr " << balanceVelocityError_.load();
            screen << "\x1b[1;33m" << row(cartTerms.str(), width)
                   << "\x1b[0m\n";
            std::ostringstream combinedTerms;
            combinedTerms << "ACC2VOL   P " << std::showpos << std::fixed
                          << std::setprecision(5) << balanceProportionalVoltage_.load()
                          << " V    I " << balancePiIntegralVoltage_.load()
                          << " V    AO0 " << voltage << " V";
            const bool nearVoltageLimit = std::abs(voltage) >= 0.8;
            screen << (nearVoltageLimit ? "\x1b[1;31m" : "\x1b[1;32m")
                   << row(combinedTerms.str(), width) << "\x1b[0m\n";

            screen << border("LIMITS / OUTPUT / MOTION", width) << '\n';
            const std::string limitLine =
                "LEFT " + std::string(left ? "TRIGGERED" : "CLEAR") +
                "  [raw " + (leftRawHigh_.load() ? "HIGH" : "LOW") + "]    RIGHT " +
                (right ? "TRIGGERED" : "CLEAR") + "  [raw " +
                (rightRawHigh_.load() ? "HIGH" : "LOW") + "]";
            screen << (left || right ? "\x1b[1;31m" : "\x1b[1;32m")
                   << row(limitLine, width) << "\x1b[0m\n";
            std::ostringstream output;
            output << "SERVO " << (servo ? "ON " : "OFF") << "    AO0 "
                   << std::showpos << std::fixed << std::setprecision(7) << voltage
                   << " V    STATE " << motion;
            screen << (servo ? "\x1b[1;33m" : "\x1b[0;37m")
                   << row(output.str(), width) << "\x1b[0m\n";
            std::ostringstream calibration;
            if (homeResultAvailable_.load()) {
                const auto error = homeCalibrationError_.load();
                calibration << "HOME LAST SESSION    travel "
                            << homeCalibrationTravel_.load()
                            << " counts    offset " << std::showpos << error;
            } else {
                calibration << "HOME LAST SESSION    NOT RUN";
            }
            screen << row(calibration.str(), width) << '\n';

            if (showHelp) {
                screen << border("HELP / COMMANDS", width) << '\n';
                screen << row("status  limits  encoder  log  help  quit", width) << '\n';
                screen << row("servo on|off    voltage <V> [duration_ms]", width) << '\n';
                screen << row("home measure|center    calibrate zero", width) << '\n';
                screen << row("balance zero (recapture)    balance start", width) << '\n';
                screen << row("balance stop|status    balance gains", width) << '\n';
                screen << row("balance gains (locked Simulink constants)", width) << '\n';
                screen << row("Up/Down command history    Backspace edit    Enter run", width)
                       << '\n';
            } else {
                screen << border("RECENT EVENTS", width) << '\n';
                for (int index = 0; index < eventRows; ++index) {
                    const std::string event = index < static_cast<int>(events.size())
                                                  ? events[static_cast<std::size_t>(index)]
                                                  : "";
                    screen << "\x1b[2;37m" << row(event, width) << "\x1b[0m\n";
                }
            }
            screen << border("COMMAND", width) << '\n';
            if (!showHelp) {
                screen << row("balance start    balance zero (recapture)    balance stop|status", width)
                       << '\n';
                screen << row("balance gains    show locked Simulink parameters", width)
                       << '\n';
                screen << row("servo on|off    voltage <V> [ms]    home measure|center", width)
                       << '\n';
                screen << row("status  limits  encoder  log  help  quit    Up/Down: history", width)
                       << '\n';
            }
            screen << row(message.empty() ? "Ready" : message, width) << '\n';
            screen << "\x1b[1;36m> \x1b[0m" << fit(input, width > 2 ? width - 2 : 0)
                   << "\x1b[J" << std::flush;

            std::scoped_lock lock(renderMutex_);
            std::cout << screen.str() << std::flush;
        } catch (...) {
        }
    }

    void notify(std::string message, bool error = false) const {
        if (dashboardMode_) {
            std::scoped_lock lock(messageMutex_);
            lastMessage_ = std::move(message);
        } else if (error) {
            std::cerr << message << '\n';
        } else {
            std::cout << message << '\n';
        }
    }

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
                                  config_.ni6602.motorEncoderPulsesPerRevolution,
                                  config_.ni6602.motorEncoderFilterMinPulseWidthMicroseconds *
                                      1e-6);
        ni_.configurePendulumEncoderRaw(config_.ni6602.pendulumCounter,
                                        config_.ni6602.pendulumEncoderATerminal,
                                        config_.ni6602.pendulumEncoderBTerminal,
                                        config_.ni6602.pendulumEncoderFilterMinPulseWidthMicroseconds *
                                            1e-6);
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
        std::uint32_t previousMotor = 0;
        std::uint32_t previousPendulum = 0;
        std::int64_t motorPosition = 0;
        std::int64_t pendulumPosition = 0;
        auto previousTime = std::chrono::steady_clock::now();
        bool havePreviousEncoderSample = false;
        try {
            while (!stopToken.stop_requested() && !safety_.stopRequested()) {
                const auto limits = ni_.readLimitInputs();
                const auto motorEncoder = ni_.readMotorEncoderRaw();
                const auto pendulumEncoder = ni_.readPendulumEncoderRaw();
                const auto sampleTime = std::chrono::steady_clock::now();
                leftRawHigh_.store(limits.leftRawHigh);
                rightRawHigh_.store(limits.rightRawHigh);
                encoderRaw_.store(motorEncoder);
                pendulumEncoderRaw_.store(pendulumEncoder);
                if (havePreviousEncoderSample) {
                    const double elapsed =
                        std::chrono::duration<double>(sampleTime - previousTime).count();
                    if (elapsed > 0.0) {
                        const double motorInstant = static_cast<double>(
                            pendulum::calibration::MotorEncoderCalibration::deltaWithRollover(
                                previousMotor, motorEncoder)) / elapsed;
                        const double pendulumInstant = static_cast<double>(
                            pendulum::calibration::MotorEncoderCalibration::deltaWithRollover(
                                previousPendulum, pendulumEncoder)) / elapsed;
                        motorSpeedCountsPerSecond_.store(
                            0.8 * motorSpeedCountsPerSecond_.load() + 0.2 * motorInstant);
                        pendulumSpeedCountsPerSecond_.store(
                            0.8 * pendulumSpeedCountsPerSecond_.load() +
                            0.2 * pendulumInstant);
                    }
                    motorPosition +=
                        pendulum::calibration::MotorEncoderCalibration::deltaWithRollover(
                            previousMotor, motorEncoder);
                    pendulumPosition +=
                        pendulum::calibration::MotorEncoderCalibration::deltaWithRollover(
                            previousPendulum, pendulumEncoder);
                } else {
                    motorPosition =
                        pendulum::calibration::MotorEncoderCalibration::deltaWithRollover(
                            0U, motorEncoder);
                    pendulumPosition =
                        pendulum::calibration::MotorEncoderCalibration::deltaWithRollover(
                            0U, pendulumEncoder);
                }
                motorPositionCounts_.store(motorPosition);
                pendulumPositionCounts_.store(pendulumPosition);
                previousMotor = motorEncoder;
                previousPendulum = pendulumEncoder;
                previousTime = sampleTime;
                havePreviousEncoderSample = true;
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
                    const bool expectedHomingLimit = homingRunning_.load();
                    if (!expectedHomingLimit) {
                        calibrationAbort_.store(true);
                    }
                    balanceAbort_.store(true);
                    stopOutputs(std::string("limit triggered: ") +
                                (leftStable ? "LEFT" : "RIGHT"));
                    if (expectedHomingLimit) {
                        record(std::string("HOME LIMIT REACHED: ") +
                               (leftStable ? "LEFT" : "RIGHT"));
                        notify(std::string("HOME LIMIT: ") +
                                   (leftStable ? "LEFT" : "RIGHT") +
                                   "; releasing inward",
                               true);
                    } else {
                        record(std::string("LIMIT STOP: ") +
                                   (leftStable ? "LEFT" : "RIGHT"),
                               pendulum::logging::Level::Critical);
                        notify(std::string("LIMIT: ") +
                                   (leftStable ? "LEFT" : "RIGHT") +
                                   "; AO0=0 V, Servo OFF",
                               true);
                    }
                } else if (!anyLimit) {
                    limitStopActive_.store(false);
                }
                {
                    std::scoped_lock lock(pendulumSampleMutex_);
                    pendulumSample_.positionCounts = pendulumPosition;
                    pendulumSample_.motorPositionCounts = motorPosition;
                    pendulumSample_.time = sampleTime;
                    ++pendulumSample_.sequence;
                }
                pendulumSampleCondition_.notify_all();
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
        if (reason.find("A710") != std::string::npos) {
            a710AlarmCount_.fetch_add(1);
        }
        bool expected = false;
        if (!faultLatched_.compare_exchange_strong(expected, true)) {
            return;
        }
        calibrationAbort_.store(true);
        balanceAbort_.store(true);
        stopOutputs(reason);
        record("FAULT: " + reason, pendulum::logging::Level::Critical);
        notify("FAULT: " + reason + "; AO0=0 V, Servo OFF", true);
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
        if (command != "help") {
            dashboardHelpVisible_.store(false);
        }
        if (command == "help") {
            printHelp();
        } else if (command == "status") {
            printStatus();
        } else if (command == "limits") {
            printLimits();
        } else if (command == "encoder") {
            notify("motor_position=" + std::to_string(motorPositionCounts_.load()) +
                   ", pendulum_position=" +
                   std::to_string(pendulumPositionCounts_.load()));
        } else if (command == "log") {
            printLog();
        } else if (command == "servo") {
            std::string state;
            input >> state;
            setServo(state);
        } else if (command == "voltage") {
            double voltage = 0.0;
            if (!(input >> voltage)) {
                notify("Usage: voltage <volts> [duration_ms]", true);
            } else {
                std::uint32_t duration = 0;
                if (input >> duration) {
                    setVoltage(voltage, duration);
                } else {
                    setVoltage(voltage, std::nullopt);
                }
            }
        } else if (command == "calibrate") {
            std::string target;
            input >> target;
            if (target == "zero") {
                calibrateMotorZero();
            } else {
                notify("Usage: calibrate zero", true);
            }
        } else if (command == "balance") {
            std::string action;
            input >> action;
            if (action == "zero") {
                capturePendulumZero();
            } else if (action == "start") {
                std::string polarity;
                input >> polarity;
                if (polarity.empty()) {
                    startBalance();
                } else {
                    notify("Reference-model signs are fixed; use 'balance start' without a polarity override.", true);
                }
            } else if (action == "stop") {
                stopBalance("operator balance stop");
            } else if (action == "status") {
                printBalanceStatus();
            } else if (action == "gains") {
                printBalanceGains();
            } else if (action == "kp" || action == "kd" || action == "kx" ||
                       action == "kv" || action == "ki") {
                notify("Reference-model gains are locked and cannot be changed at runtime.", true);
            } else {
                notify("Usage: balance zero|start|stop|status|gains|kp|kd|kx|kv|ki <value>", true);
            }
        } else if (command == "home") {
            std::string target;
            input >> target;
            if (target == "center") {
                runHomeOperation(false);
            } else if (target == "return") {
                runHomeReturn();
            } else if (target == "measure") {
                runHomeOperation(true);
            } else {
                notify("Usage: home measure|center|return", true);
            }
        } else if (command == "quit" || command == "exit") {
            return false;
        } else {
            notify("Unknown command. Enter 'help'.", true);
        }
        return true;
    }

    void setServo(const std::string& state) {
        if (state == "off") {
            if (balanceRunning_.load()) {
                stopBalance("operator servo off");
            }
            stopOutputs("operator servo off");
            record("Operator command: servo off");
            notify("Servo OFF; AO0=0 V.");
            return;
        }
        if (state != "on") {
            notify("Usage: servo on|off", true);
            return;
        }
        if (balanceRunning_.load()) {
            notify("Servo is owned by the running balance loop.", true);
            return;
        }
        if (!outputsAllowed()) {
            notify("Servo ON rejected: a limit is active or limit monitoring failed.", true);
            record("Rejected command: servo on", pendulum::logging::Level::Warning);
            return;
        }
        {
            std::scoped_lock lock(outputMutex_);
            if (!outputsAllowed()) {
                notify("Servo ON rejected: safety state changed.", true);
                return;
            }
            analogOutput_.writeVoltage(0.0);
            commandedVoltage_.store(0.0);
            ni_.setServoEnabled(true);
            servoOn_.store(true);
        }
        record("Operator command: servo on");
        std::ostringstream message;
        message << "Servo ON in analog velocity mode; AO0=0 V.";
        notify(message.str());
    }

    void setVoltage(double voltage, std::optional<std::uint32_t> durationMilliseconds) {
        if (balanceRunning_.load()) {
            notify("Manual voltage is disabled while balance control is running.", true);
            return;
        }
        if (!outputsAllowed() || !servoOn_.load()) {
            notify("Voltage rejected: use 'servo on' first and ensure limits are clear.", true);
            return;
        }
        if (!std::isfinite(voltage) || voltage < config_.pci1723.minimumVoltage ||
            voltage > config_.pci1723.maximumVoltage) {
            std::ostringstream message;
            message << "Voltage must be within the configured hardware range "
                    << config_.pci1723.minimumVoltage << ".."
                    << config_.pci1723.maximumVoltage << " V.";
            notify(message.str(), true);
            return;
        }
        if (durationMilliseconds.has_value() && *durationMilliseconds == 0) {
            notify("Duration must be greater than zero.", true);
            return;
        }

        const auto start = encoderRaw_.load();
        {
            std::scoped_lock lock(outputMutex_);
            if (!outputsAllowed() || !servoOn_.load()) {
                notify("Voltage rejected: safety state changed.", true);
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
            std::ostringstream message;
            message << "AO0=" << voltage << " V; Servo remains ON";
            if (voltage != 0.0) {
                message << "; expected cart direction="
                        << (voltage > 0.0
                                ? config_.pci1723.positiveVoltageCartDirection
                                : config_.pci1723.negativeVoltageCartDirection);
            }
            message << '.';
            notify(message.str());
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
        std::ostringstream message;
        message << "Timed voltage stopped; Servo OFF. encoder_start=" << start
                << ", encoder_end=" << end << ", delta=" << delta
                << " (" << direction << ')';
        notify(message.str());
    }

    void calibrateMotorZero() {
        if (balanceRunning_.load()) {
            notify("Stop balance control before motor calibration.", true);
            return;
        }
        if (!outputsAllowed()) {
            notify("Calibration cannot start while a limit or monitor fault is active.", true);
            return;
        }
        if (calibrationRunning_.exchange(true)) {
            notify("Motor zero calibration is already running.", true);
            return;
        }
        calibrationAbort_.store(false);

        try {
            {
                std::scoped_lock lock(outputMutex_);
                if (!outputsAllowed()) {
                    throw std::runtime_error("Safety state changed before calibration start");
                }
                analogOutput_.writeVoltage(0.0);
                commandedVoltage_.store(0.0);
                ni_.setServoEnabled(true);
                servoOn_.store(true);
            }
            record("Motor zero calibration started");
            notify("Motor zero calibration started; Servo ON, AO0=0 V.");

            pendulum::calibration::MotorZeroCalibrator calibrator(
                config_.motorZeroCalibration,
                config_.pci1723.minimumVoltage,
                config_.pci1723.maximumVoltage,
                [this](double voltage) {
                    if (calibrationAbort_.load() || !outputsAllowed() || !servoOn_.load()) {
                        throw std::runtime_error("Calibration output interrupted");
                    }
                    std::scoped_lock lock(outputMutex_);
                    if (calibrationAbort_.load() || !outputsAllowed() || !servoOn_.load()) {
                        throw std::runtime_error("Calibration safety state changed");
                    }
                    analogOutput_.writeVoltage(voltage);
                    commandedVoltage_.store(voltage);
                },
                [this] { return encoderRaw_.load(); },
                [this] {
                    return calibrationAbort_.load() || faultLatched_.load() ||
                           safety_.stopRequested() || leftTriggered_.load() ||
                           rightTriggered_.load();
                },
                [this](const std::string& message) {
                    record("ZeroCal: " + message);
                    notify("[zero-cal] " + message);
                });

            const auto result = calibrator.run();
            if (!result.accepted) {
                std::ostringstream message;
                message << "Calibration failed: median absolute speed "
                        << result.finalAbsoluteSpeedCountsPerSecond << " counts/s exceeds "
                        << config_.motorZeroCalibration.maximumAcceptedSpeedCountsPerSecond;
                throw std::runtime_error(message.str());
            }

            pendulum::config::AppConfig::saveMotorZeroCalibration(
                configPath_,
                pendulum::config::MotorZeroCalibrationRecord{
                    result.voltage,
                    result.dacLsbVoltage,
                    result.finalSignedSpeedCountsPerSecond,
                    result.finalAbsoluteSpeedCountsPerSecond,
                    config_.motorZeroCalibration.targetSpeedCountsPerSecond,
                    config_.motorZeroCalibration.maximumAcceptedSpeedCountsPerSecond});
            config_.pci1723.calibratedZeroVoltage = result.voltage;
            config_.pci1723.zeroVoltageRequiresCalibration = false;

            std::ostringstream message;
            message << std::showpos << std::fixed << std::setprecision(7)
                    << "Calibration complete: Vzero=" << result.voltage << " V, speed="
                    << std::setprecision(3) << result.finalSignedSpeedCountsPerSecond
                    << " counts/s, quality=" << (result.targetMet ? "TARGET" : "ACCEPTED")
                    << "; Servo remains ON at Vzero";
            record(message.str());
            notify(message.str());
        } catch (const std::exception& error) {
            stopOutputs("motor zero calibration failed or aborted");
            record(std::string("Motor zero calibration failed: ") + error.what(),
                   pendulum::logging::Level::Error);
            notify(std::string("Motor zero calibration failed: ") + error.what() +
                       "; AO0=0 V, Servo OFF",
                   true);
        }
        calibrationRunning_.store(false);
    }

    void capturePendulumZero() {
        if (balanceRunning_.load()) {
            notify("Stop balance control before capturing the downward zero.", true);
            return;
        }
        if (!sampleReady_.load() || faultLatched_.load()) {
            notify("Downward zero capture requires live encoder samples.", true);
            return;
        }
        if (servoOn_.load() || calibrationRunning_.load() || homingRunning_.load()) {
            notify("Downward zero capture requires Servo OFF and no active motion.", true);
            return;
        }
        static_cast<void>(captureStablePendulumZero("manual"));
    }

    bool captureStablePendulumZero(const std::string& reason) {
        constexpr auto samplePeriod = 10ms;
        const auto requiredSamples = std::max<std::size_t>(
            2, static_cast<std::size_t>(std::ceil(
                   config_.balanceControl.downwardZeroCaptureSeconds / 0.010)));
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration<double>(
                                  config_.balanceControl.downwardZeroSettleTimeoutSeconds);
        std::deque<std::int64_t> window;
        pendulumZeroCaptured_.store(false);
        while (std::chrono::steady_clock::now() < deadline &&
               !faultLatched_.load() && !safety_.stopRequested()) {
            window.push_back(pendulumPositionCounts_.load());
            if (window.size() > requiredSamples) {
                window.pop_front();
            }
            if (window.size() == requiredSamples) {
                const std::vector<std::int64_t> samples(window.begin(), window.end());
                try {
                    const auto count = stableRepresentative(
                        samples,
                        config_.balanceControl.downwardZeroMaximumSpanCounts);
                    const auto [minimum, maximum] =
                        std::minmax_element(samples.begin(), samples.end());
                    pendulumDownCount_.store(count);
                    pendulumZeroCaptured_.store(true);
                    balanceAngleDegrees_.store(0.0);
                    balanceAngularRateDegrees_.store(0.0);
                    const auto message =
                        "Pendulum downward zero captured (" + reason + "): count=" +
                        std::to_string(count) + ", span=" +
                        std::to_string(*maximum - *minimum) + " counts";
                    record(message);
                    notify(message);
                    return true;
                } catch (const std::runtime_error&) {
                    // Keep observing until a complete stationary window is available.
                }
            }
            std::this_thread::sleep_for(samplePeriod);
        }
        const auto message = "Pendulum downward zero not captured (" + reason +
                             "): no stable window before timeout";
        record(message, pendulum::logging::Level::Warning);
        notify(message, true);
        return false;
    }

    void startBalance() {
        if (balanceRunning_.load()) {
            notify("Balance control is already running.", true);
            return;
        }
        if (!sampleReady_.load() || faultLatched_.load() ||
            leftTriggered_.load() || rightTriggered_.load()) {
            notify("Balance start requires live samples and clear limits.", true);
            return;
        }
        if (calibrationRunning_.load() || homingRunning_.load()) {
            notify("Finish calibration or homing before balance control.", true);
            return;
        }
        if (!homeResultAvailable_.load()) {
            notify("Run 'home center' in this process before balance start; saved positions are not reused.",
                   true);
            return;
        }
        try {
            const auto cart = currentCartSessionState(0.0);
            if (std::abs(cart.positionFromCenterHalfTravel) >
                config_.balanceControl.maximumBalanceStartPositionFraction) {
                notify("Balance start rejected: cart is outside the allowed center window; run 'home center' again.",
                       true);
                return;
            }
        } catch (const std::exception& error) {
            notify(std::string("Balance start rejected: ") + error.what(), true);
            return;
        }
        if (balanceThread_.joinable()) {
            balanceThread_.join();
        }
        balanceAbort_.store(false);
        balanceMissedDeadlines_.store(0);
        balanceMaxJitterMicroseconds_.store(0.0);
        const auto referenceSample = latestPendulumSample();
        const auto targetCount = referenceSample.positionCounts;
        const auto downToStart = wrappedCounts(
            targetCount - pendulumDownCount_.load(),
            static_cast<std::int64_t>(
                config_.balanceControl.pendulumCountsPerRevolution));
        const auto halfRevolution = static_cast<std::int64_t>(
            config_.balanceControl.pendulumCountsPerRevolution / 2);
        const auto uprightErrorCounts =
            std::llabs(std::llabs(downToStart) - halfRevolution);
        const auto maximumStartErrorCounts = static_cast<std::int64_t>(
            std::ceil(config_.balanceControl.maximumBalanceAngleRadians *
                      static_cast<double>(
                          config_.balanceControl.pendulumCountsPerRevolution) /
                      (2.0 * std::numbers::pi)));
        if (!pendulumZeroCaptured_.load() ||
            uprightErrorCounts > maximumStartErrorCounts) {
            notify("Balance start rejected: manually hold the pendulum within the reference model's +/-30 degree upright region.",
                   true);
            return;
        }
        pendulumUprightCount_.store(targetCount);
        balanceCartReferenceCount_.store(referenceSample.motorPositionCounts);
        balanceRunning_.store(true);
        const auto runId = balanceRunId_.fetch_add(1) + 1;
        balanceThread_ =
            std::jthread([this, runId](std::stop_token token) { balanceLoop(token, runId); });
        notify("Balance loop started: run_id=" + std::to_string(runId) +
               ", frequency_hz=" +
               std::to_string(config_.balanceControl.frequencyHz) +
               ", balance_reference_count=" + std::to_string(targetCount) +
               ", cart_reference_count=" +
               std::to_string(referenceSample.motorPositionCounts) +
               ", controller=Copy_of_LQR_lp1_1 manual-upright branch.");
    }


    void printBalanceGains() const {
        std::ostringstream message;
        message << "reference_lqr_locked=true"
                << ", kx=" << pendulum::control::ReferenceLqrVelocityController::kCartPositionGain
                << ", kv=" << pendulum::control::ReferenceLqrVelocityController::kCartVelocityGain
                << ", ktheta=" << pendulum::control::ReferenceLqrVelocityController::kPendulumAngleGain
                << ", komega=" << pendulum::control::ReferenceLqrVelocityController::kPendulumAngularRateGain
                << ", velocity_pi_p=" << pendulum::control::ReferenceLqrVelocityController::kVelocityProportionalGain
                << ", velocity_pi_i=" << pendulum::control::ReferenceLqrVelocityController::kVelocityIntegralGain;
        notify(message.str());
    }

    void stopBalance(const std::string& reason) noexcept {
        balanceAbort_.store(true);
        pendulumSampleCondition_.notify_all();
        if (balanceThread_.joinable() &&
            balanceThread_.get_id() != std::this_thread::get_id()) {
            balanceThread_.request_stop();
            balanceThread_.join();
        }
        if (balanceRunning_.exchange(false)) {
            stopOutputs(reason);
            record("Balance stopped: " + reason);
            notify("Balance stopped; AO0=0 V, Servo OFF.");
        }
    }

    void balanceLoop(std::stop_token stopToken, std::uint64_t runId) noexcept {
        const auto statisticsStart = std::chrono::steady_clock::now();
        const auto a710AtStart = a710AlarmCount_.load();
        std::uint64_t statisticsSamples = 0;
        double angleErrorSumDegrees = 0.0;
        double absoluteAngleErrorSumDegrees = 0.0;
        double squaredAngleErrorSumDegrees = 0.0;
        double maximumAngleErrorDegrees = 0.0;
        double maximumOutputVoltage = 0.0;
        try {
            using Controller =
                pendulum::control::ReferenceLqrVelocityController;
            const auto& settings = config_.balanceControl;
            const auto period = std::chrono::duration_cast<
                std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(Controller::kControlSampleSeconds));
            Controller controller;
            controller.reset();
            const auto initialOutput = controller.update(0, 0);
            if (initialOutput.outputVoltage != 0.0) {
                throw std::runtime_error(
                    "reference controller did not initialize at zero output");
            }

            {
                std::scoped_lock lock(outputMutex_);
                analogOutput_.writeVoltage(0.0);
                commandedVoltage_.store(0.0);
                ni_.setServoEnabled(true);
                servoOn_.store(true);
            }

            auto sample = latestPendulumSample();
            auto previousSampleSequence = sample.sequence;
            auto nextControlTime = sample.time + period;
            const auto balanceStartTime = sample.time;
            std::uint64_t sampleIndex = 0;
            record("Reference Copy_of_LQR_lp1_1 LQR+ACC2VOL active at 100 Hz; swing-up disabled; velocity mode; exact fixed gains");

            while (!stopToken.stop_requested() && !balanceAbort_.load() &&
                   !faultLatched_.load() && !safety_.stopRequested()) {
                {
                    std::unique_lock lock(pendulumSampleMutex_);
                    pendulumSampleCondition_.wait_for(lock, 20ms, [&] {
                        return (pendulumSample_.sequence != previousSampleSequence &&
                                pendulumSample_.time >= nextControlTime) ||
                               stopToken.stop_requested() || balanceAbort_.load() ||
                               faultLatched_.load() || safety_.stopRequested();
                    });
                    if (stopToken.stop_requested() || balanceAbort_.load() ||
                        faultLatched_.load() || safety_.stopRequested()) {
                        break;
                    }
                    if (pendulumSample_.sequence == previousSampleSequence ||
                        pendulumSample_.time < nextControlTime) {
                        continue;
                    }
                    sample = pendulumSample_;
                }
                previousSampleSequence = sample.sequence;

                const double jitterUs = std::abs(
                    std::chrono::duration<double, std::micro>(
                        sample.time - nextControlTime).count());
                balanceMaxJitterMicroseconds_.store(std::max(
                    balanceMaxJitterMicroseconds_.load(), jitterUs));
                while (nextControlTime + period <= sample.time) {
                    nextControlTime += period;
                    balanceMissedDeadlines_.fetch_add(1);
                }
                nextControlTime += period;

                if (leftTriggered_.load() || rightTriggered_.load()) {
                    throw std::runtime_error("limit triggered during balance");
                }
                const auto output = controller.update(
                    sample.positionCounts - pendulumUprightCount_.load(),
                    sample.motorPositionCounts - balanceCartReferenceCount_.load());
                if (std::abs(output.pendulumAngleRadians) >=
                    settings.maximumBalanceAngleRadians) {
                    throw std::runtime_error(
                        "pendulum left the reference LQR region; swing-up is disabled");
                }
                const auto cartSafety = currentCartSessionState(0.0);
                if (std::abs(cartSafety.positionFromCenterHalfTravel) >=
                    settings.maximumBalancePositionFraction) {
                    throw std::runtime_error(
                        "cart exceeded the session-center software travel envelope");
                }

                {
                    std::scoped_lock lock(outputMutex_);
                    if (balanceAbort_.load() || leftTriggered_.load() ||
                        rightTriggered_.load()) {
                        throw std::runtime_error("balance output interrupted");
                    }
                    analogOutput_.writeVoltage(output.outputVoltage);
                    commandedVoltage_.store(output.outputVoltage);
                }

                const double angleDegrees =
                    output.pendulumAngleRadians * 180.0 / std::numbers::pi;
                const double angularRateDegrees =
                    output.pendulumAngularRateRadiansPerSecond * 180.0 /
                    std::numbers::pi;
                balanceAngleDegrees_.store(angleDegrees);
                balanceAngularRateDegrees_.store(angularRateDegrees);
                balanceReferenceAcceleration_.store(
                    output.accelerationCommandMetersPerSecondSquared);
                balanceReferenceVelocity_.store(
                    output.velocityReferenceMetersPerSecond);
                balanceCartPositionMeters_.store(output.cartPositionMeters);
                balanceCartVelocityMetersPerSecond_.store(
                    output.cartVelocityMetersPerSecond);
                balanceVelocityError_.store(output.velocityErrorMetersPerSecond);
                balancePiIntegralVoltage_.store(output.integralVoltage);
                balanceProportionalVoltage_.store(output.proportionalVoltage);

                ++statisticsSamples;
                angleErrorSumDegrees += angleDegrees;
                absoluteAngleErrorSumDegrees += std::abs(angleDegrees);
                squaredAngleErrorSumDegrees += angleDegrees * angleDegrees;
                maximumAngleErrorDegrees = std::max(
                    maximumAngleErrorDegrees, std::abs(angleDegrees));
                maximumOutputVoltage = std::max(
                    maximumOutputVoltage, std::abs(output.outputVoltage));

                if (++sampleIndex % settings.telemetryDivider == 0) {
                    std::ostringstream telemetry;
                    telemetry << "run_id=" << runId
                              << ",controller_mode=reference_lqr_acc2vol_manual_upright"
                              << ",timestamp="
                              << std::chrono::duration<double>(
                                     sample.time - balanceStartTime).count()
                              << ",sample=" << sampleIndex
                              << ",pendulum_count=" << sample.positionCounts
                              << ",motor_count=" << sample.motorPositionCounts
                              << ",theta_rad=" << output.pendulumAngleRadians
                              << ",theta_dot_rad_s="
                              << output.pendulumAngularRateRadiansPerSecond
                              << ",x_m=" << output.cartPositionMeters
                              << ",x_dot_m_s="
                              << output.cartVelocityMetersPerSecond
                              << ",acceleration_cmd_m_s2="
                              << output.accelerationCommandMetersPerSecondSquared
                              << ",velocity_ref_m_s="
                              << output.velocityReferenceMetersPerSecond
                              << ",velocity_error_m_s="
                              << output.velocityErrorMetersPerSecond
                              << ",pi_p_v=" << output.proportionalVoltage
                              << ",pi_i_v=" << output.integralVoltage
                              << ",ao_v=" << output.outputVoltage
                              << ",jitter_us=" << jitterUs;
                    logger_.log(pendulum::logging::Level::Info,
                                "BalanceTelemetry", telemetry.str());
                }
            }
        } catch (const std::exception& error) {
            record(std::string("Balance loop stopped: ") + error.what(),
                   pendulum::logging::Level::Error);
            notify(std::string("Balance stopped: ") + error.what(), true);
        } catch (...) {
            record("Balance loop stopped: unknown error",
                   pendulum::logging::Level::Error);
        }
        stopOutputs("balance loop exit");
        balanceRunning_.store(false);
        const double stableTimeSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - statisticsStart).count();
        const double denominator = statisticsSamples > 0
            ? static_cast<double>(statisticsSamples) : 1.0;
        std::ostringstream statistics;
        statistics << "Balance statistics: mean_angle_error_deg="
                   << angleErrorSumDegrees / denominator
                   << ", mean_abs_angle_error_deg="
                   << absoluteAngleErrorSumDegrees / denominator
                   << ", max_angle_error_deg=" << maximumAngleErrorDegrees
                   << ", rms_angle_error_deg="
                   << std::sqrt(squaredAngleErrorSumDegrees / denominator)
                   << ", stable_time_s=" << stableTimeSeconds
                   << ", max_output_voltage=" << maximumOutputVoltage
                   << ", A710_alarm_count="
                   << (a710AlarmCount_.load() - a710AtStart);
        record(statistics.str());
        notify(statistics.str());
    }


    void printBalanceStatus() const {
        std::ostringstream message;
        message << "balance_running=" << std::boolalpha
                << balanceRunning_.load()
                << ", controller=Copy_of_LQR_lp1_1_LQR_ACC2VOL"
                << ", swing_up=false"
                << ", drive=" << config_.balanceControl.driveModel
                << ", mode=" << config_.balanceControl.driveControlMode
                << ", control_period_s=0.01"
                << ", acc2vol_integrator_ts_s=0.005"
                << ", angle_deg=" << balanceAngleDegrees_.load()
                << ", angular_rate_deg_s="
                << balanceAngularRateDegrees_.load()
                << ", acceleration_cmd_m_s2="
                << balanceReferenceAcceleration_.load()
                << ", velocity_ref_m_s=" << balanceReferenceVelocity_.load()
                << ", cart_position_m=" << balanceCartPositionMeters_.load()
                << ", cart_velocity_m_s="
                << balanceCartVelocityMetersPerSecond_.load()
                << ", velocity_error_m_s=" << balanceVelocityError_.load()
                << ", pi_p_v=" << balanceProportionalVoltage_.load()
                << ", pi_i_v=" << balancePiIntegralVoltage_.load()
                << ", ao_v=" << commandedVoltage_.load()
                << ", max_jitter_us=" << balanceMaxJitterMicroseconds_.load()
                << ", missed_deadlines=" << balanceMissedDeadlines_.load()
                << ", motor_encoder_feedback=true"
                << ", fresh_home_center_required=true";
        notify(message.str());
    }

    void runHomeOperation(bool measureOnly) {
        if (balanceRunning_.load()) {
            notify("Stop balance control before homing.", true);
            return;
        }
        if (!sampleReady_.load() || faultLatched_.load()) {
            notify("Homing cannot start because hardware monitoring is unavailable.", true);
            return;
        }
        if (calibrationRunning_.load() || homingRunning_.exchange(true)) {
            notify("Another calibration or homing operation is already running.", true);
            return;
        }
        calibrationAbort_.store(false);
        homeResultAvailable_.store(false);
        setHomingState("STARTING");

        try {
            stopOutputs("home-center start");
            record(measureOnly
                       ? "Home travel measurement started: using the first limit as the session reference"
                       : "Home-center started: measuring both limits for this session");
            notify(measureOnly
                       ? "Travel measurement started: probing both limits, then releasing inward and stopping."
                       : "Homing started: probing both limits, then returning to this session's center.");

            const bool positiveMovesLeft =
                config_.pci1723.positiveVoltageCartDirection == "LEFT";
            pendulum::calibration::HomeCenterController controller(
                config_.homeCenter,
                positiveMovesLeft,
                [this] {
                    return pendulum::calibration::HomeCenterSample{
                        motorPositionCounts_.load(), leftTriggered_.load(),
                        rightTriggered_.load()};
                },
                [this](
                    double voltage, pendulum::calibration::LimitSide releaseSide) {
                    const bool left = leftTriggered_.load();
                    const bool right = rightTriggered_.load();
                    if (left && right) {
                        throw std::runtime_error("Both limits are active");
                    }
                    if (left || right) {
                        const auto activeSide = left
                            ? pendulum::calibration::LimitSide::Left
                            : pendulum::calibration::LimitSide::Right;
                        if (releaseSide != activeSide) {
                            throw std::runtime_error(
                                "Motion into an active limit was rejected");
                        }
                    }
                    std::scoped_lock lock(outputMutex_);
                    if (!servoOn_.load()) {
                        analogOutput_.writeVoltage(0.0);
                        commandedVoltage_.store(0.0);
                        ni_.setServoEnabled(true);
                        servoOn_.store(true);
                    }
                    analogOutput_.writeVoltage(voltage);
                    commandedVoltage_.store(voltage);
                },
                [this](const std::string& reason) { stopOutputs(reason); },
                [this] {
                    return faultLatched_.load() || safety_.stopRequested();
                },
                [this](const std::string& message) {
                    setHomingState(message);
                    record("Home: " + message);
                    notify("[home] " + message);
                });

            const auto result = measureOnly ? controller.measureTravel()
                                            : controller.run();
            if (!measureOnly) {
                static_cast<void>(captureStablePendulumZero("post-home"));
            }
            if (!measureOnly) {
                homeCalibrationLeftBoundary_.store(result.leftBoundaryCounts);
                homeCalibrationRightBoundary_.store(result.rightBoundaryCounts);
                homeCalibrationCenter_.store(result.centerCounts);
                homeCalibrationTravel_.store(result.travelCounts);
                homeCalibrationError_.store(result.centerErrorCounts);
                homeResultAvailable_.store(true);
            }

            std::ostringstream message;
            message << (measureOnly ? "Travel measurement complete: left="
                                    : "Homing complete: left=")
                    << result.leftBoundaryCounts
                    << ", right=" << result.rightBoundaryCounts
                    << ", travel=" << result.travelCounts
                    << ", forward=" << result.forwardTravelCounts
                    << ", reverse=" << result.reverseTravelCounts
                    << ", disagreement=" << result.travelDisagreementCounts
                    << ", center=" << result.centerCounts
                    << ", final=" << result.finalPositionCounts
                    << ", center_error=" << result.centerErrorCounts
                    << " counts, mode="
                    << (measureOnly ? "MEASURE_ONLY" : "FRESH_TWO_LIMIT_SESSION")
                    << "; AO0=0 V, Servo OFF";
            setHomingState(measureOnly ? "MEASURED" : "COMPLETE");
            record(message.str());
            notify(message.str());
        } catch (const std::exception& error) {
            stopOutputs("home-center failed or aborted");
            record(std::string("Home-center failed: ") + error.what(),
                   pendulum::logging::Level::Error);
            notify(std::string("Home-center failed: ") + error.what() +
                       "; AO0=0 V, Servo OFF",
                   true);
        }
        homingRunning_.store(false);
    }

    void runHomeReturn() {
        if (balanceRunning_.load()) {
            notify("Stop balance control before returning to center.", true);
            return;
        }
        if (!homeResultAvailable_.load()) {
            notify("Home return rejected: run 'home center' once in this process first.", true);
            return;
        }
        if (!sampleReady_.load() || faultLatched_.load() ||
            leftTriggered_.load() || rightTriggered_.load()) {
            notify("Home return rejected: monitoring, fault, or limit state is unsafe.", true);
            return;
        }
        if (calibrationRunning_.load() || homingRunning_.exchange(true)) {
            notify("Another calibration or homing operation is already running.", true);
            return;
        }
        calibrationAbort_.store(false);
        setHomingState("RETURNING_TO_KNOWN_CENTER");

        try {
            stopOutputs("known-center return start");
            const auto center = homeCalibrationCenter_.load();
            const auto travel = homeCalibrationTravel_.load();
            const bool positiveMovesLeft =
                config_.pci1723.positiveVoltageCartDirection == "LEFT";
            const auto positiveVoltageBoundary = positiveMovesLeft
                ? homeCalibrationLeftBoundary_.load()
                : homeCalibrationRightBoundary_.load();
            if (positiveVoltageBoundary == center) {
                throw std::runtime_error("Known-center motor direction is invalid");
            }
            const double aoToEncoderSign =
                positiveVoltageBoundary > center ? 1.0 : -1.0;

            pendulum::calibration::HomeCenterController controller(
                config_.homeCenter, positiveMovesLeft,
                [this] {
                    return pendulum::calibration::HomeCenterSample{
                        motorPositionCounts_.load(), leftTriggered_.load(),
                        rightTriggered_.load()};
                },
                [this](double voltage,
                       pendulum::calibration::LimitSide releaseSide) {
                    if (leftTriggered_.load() || rightTriggered_.load() ||
                        releaseSide != pendulum::calibration::LimitSide::None) {
                        throw std::runtime_error(
                            "Known-center motion rejected by limit state");
                    }
                    std::scoped_lock lock(outputMutex_);
                    if (!servoOn_.load()) {
                        analogOutput_.writeVoltage(0.0);
                        commandedVoltage_.store(0.0);
                        ni_.setServoEnabled(true);
                        servoOn_.store(true);
                    }
                    analogOutput_.writeVoltage(voltage);
                    commandedVoltage_.store(voltage);
                },
                [this](const std::string& reason) { stopOutputs(reason); },
                [this] {
                    return calibrationAbort_.load() || faultLatched_.load() ||
                           safety_.stopRequested();
                },
                [this](const std::string& message) {
                    setHomingState(message);
                    record("HomeReturn: " + message);
                    notify("[home-return] " + message);
                });

            const auto finalPosition = controller.returnToKnownCenter(
                center, travel, aoToEncoderSign);
            homeCalibrationError_.store(finalPosition - center);
            const auto message =
                "Home return complete: center=" + std::to_string(center) +
                ", final=" + std::to_string(finalPosition) +
                ", error=" + std::to_string(finalPosition - center) +
                " counts; AO0=0 V, Servo OFF";
            setHomingState("RETURN_COMPLETE");
            record(message);
            notify(message);
        } catch (const std::exception& error) {
            stopOutputs("known-center return failed");
            const auto message =
                std::string("Home return failed: ") + error.what() +
                "; AO0=0 V, Servo OFF";
            record(message, pendulum::logging::Level::Error);
            notify(message, true);
        }
        homingRunning_.store(false);
    }

    void setHomingState(const std::string& state) {
        std::scoped_lock lock(homingStateMutex_);
        homingState_ = state;
    }

    void printStatus() const {
        std::ostringstream message;
        message << std::boolalpha << "servo_on=" << servoOn_.load()
                << ", ao_voltage=" << commandedVoltage_.load()
                << ", fault=" << faultLatched_.load()
                << ", motor_position=" << motorPositionCounts_.load()
                << ", pendulum_position=" << pendulumPositionCounts_.load()
                << ", home_session_result="
                << (homeResultAvailable_.load() ? "available" : "not_run");
        if (homeResultAvailable_.load()) {
            message << ", home_travel=" << homeCalibrationTravel_.load()
                    << ", home_center=" << homeCalibrationCenter_.load()
                    << ", home_offset=" << homeCalibrationError_.load();
        }
        notify(message.str());
        if (!dashboardMode_) {
            printLimits();
        }
    }

    void printLimits() const {
        std::ostringstream message;
        message << "left: raw=" << (leftRawHigh_.load() ? "HIGH" : "LOW")
                << ", state=" << (leftTriggered_.load() ? "TRIGGERED" : "CLEAR")
                << "; right: raw=" << (rightRawHigh_.load() ? "HIGH" : "LOW")
                << ", state=" << (rightTriggered_.load() ? "TRIGGERED" : "CLEAR");
        notify(message.str());
    }

    void printHelp() {
        if (dashboardMode_) {
            dashboardHelpVisible_.store(true);
            notify("Help open; run any command to return to recent events.");
            return;
        }
        std::cout
            << "Commands:\n"
            << "  status | limits | encoder | log | help | quit\n"
            << "  servo on | servo off\n"
            << "  voltage <volts>                 Set and hold AO voltage\n"
            << "  voltage <volts> <duration_ms>   Timed voltage, then Servo OFF\n"
            << "  home measure                    Measure relative two-limit travel only\n"
            << "  home center                     Measure both limits and center this session\n"
            << "  home return                     Return to the known session center\n"
            << "  calibrate zero                  Calibrate and hold motor Vzero\n"
            << "  balance zero                    Recapture downward count (automatic at startup)\n"
            << "  balance start                   Start locked reference LQR+ACC2VOL\n"
            << "  balance gains                   Show locked reference-model constants\n"
            << "  balance stop | status           Stop or inspect angle stabilization\n";
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
        if (dashboardMode_) {
            notify("CSV log: " + logger_.path().string());
            return;
        }
        std::scoped_lock lock(historyMutex_);
        for (const auto& entry : history_) {
            std::cout << "  " << entry << '\n';
        }
        std::cout << "CSV log: " << logger_.path().string() << '\n';
    }

    pendulum::config::AppConfig config_;
    std::filesystem::path configPath_;
    pendulum::logging::AsyncLogger& logger_;
    pendulum::safety::SafetyManager& safety_;
    pendulum::hardware::NI6602 ni_;
    pendulum::hardware::PCI1723 analogOutput_;
    pendulum::safety::SafetyManager::Registration aoRegistration_;
    pendulum::safety::SafetyManager::Registration servoRegistration_;
    std::jthread monitor_;
    std::jthread dashboard_;
    std::jthread balanceThread_;
    mutable std::mutex outputMutex_;
    mutable std::mutex historyMutex_;
    mutable std::mutex inputMutex_;
    mutable std::mutex messageMutex_;
    mutable std::mutex renderMutex_;
    mutable std::mutex homingStateMutex_;
    mutable std::mutex pendulumSampleMutex_;
    std::condition_variable pendulumSampleCondition_;
    PendulumSample pendulumSample_;
    std::deque<std::string> history_;
    std::deque<std::string> commandHistory_;
    std::size_t commandHistoryCursor_{0};
    std::string commandHistoryDraft_;
    std::string inputBuffer_;
    mutable std::string lastMessage_;
    std::string homingState_{"IDLE"};
    std::atomic<bool> dashboardMode_{false};
    std::atomic<bool> dashboardHelpVisible_{false};
    std::atomic<bool> sampleReady_{false};
    std::atomic<bool> leftRawHigh_{false};
    std::atomic<bool> rightRawHigh_{false};
    std::atomic<bool> leftTriggered_{false};
    std::atomic<bool> rightTriggered_{false};
    std::atomic<bool> faultLatched_{false};
    std::atomic<bool> limitStopActive_{false};
    std::atomic<bool> calibrationRunning_{false};
    std::atomic<bool> homingRunning_{false};
    std::atomic<bool> calibrationAbort_{false};
    std::atomic<bool> balanceRunning_{false};
    std::atomic<bool> balanceAbort_{false};
    std::atomic<std::uint64_t> balanceRunId_{0};
    std::atomic<bool> pendulumZeroCaptured_{false};
    std::atomic<bool> servoOn_{false};
    std::atomic<double> commandedVoltage_{0.0};
    std::atomic<std::uint32_t> encoderRaw_{0};
    std::atomic<std::uint32_t> pendulumEncoderRaw_{0};
    std::atomic<std::int64_t> motorPositionCounts_{0};
    std::atomic<std::int64_t> pendulumPositionCounts_{0};
    std::atomic<double> motorSpeedCountsPerSecond_{0.0};
    std::atomic<double> pendulumSpeedCountsPerSecond_{0.0};
    std::atomic<std::int64_t> pendulumDownCount_{0};
    std::atomic<std::int64_t> pendulumUprightCount_{0};
    std::atomic<double> balanceAngleDegrees_{0.0};
    std::atomic<double> balanceAngularRateDegrees_{0.0};
    std::atomic<double> balanceReferenceAcceleration_{0.0};
    std::atomic<double> balanceReferenceVelocity_{0.0};
    std::atomic<double> balanceCartPositionMeters_{0.0};
    std::atomic<double> balanceCartVelocityMetersPerSecond_{0.0};
    std::atomic<double> balanceVelocityError_{0.0};
    std::atomic<double> balanceProportionalVoltage_{0.0};
    std::atomic<double> balancePiIntegralVoltage_{0.0};
    std::atomic<std::int64_t> balanceCartReferenceCount_{0};
    std::atomic<double> balanceMaxJitterMicroseconds_{0.0};
    std::atomic<std::uint64_t> balanceMissedDeadlines_{0};
    std::atomic<std::uint64_t> a710AlarmCount_{0};
    std::atomic<bool> homeResultAvailable_{false};
    std::atomic<std::int64_t> homeCalibrationLeftBoundary_{0};
    std::atomic<std::int64_t> homeCalibrationRightBoundary_{0};
    std::atomic<std::int64_t> homeCalibrationCenter_{0};
    std::atomic<std::int64_t> homeCalibrationTravel_{0};
    std::atomic<std::int64_t> homeCalibrationError_{0};
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
    ManualConsole console(std::move(config), options.configPath, logger, safety);
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
