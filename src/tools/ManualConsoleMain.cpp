#include "pendulum/calibration/HomeCenterController.h"
#include "pendulum/calibration/MotorEncoderCalibration.h"
#include "pendulum/calibration/MotorZeroCalibrator.h"
#include "pendulum/config/Config.h"
#include "pendulum/control/AnglePdController.h"
#include "pendulum/control/PendulumStateEstimator.h"
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

class ManualConsole final {
public:
    ManualConsole(pendulum::config::AppConfig config,
                  std::filesystem::path configPath,
                  pendulum::logging::AsyncLogger& logger,
                  pendulum::safety::SafetyManager& safety)
        : config_(std::move(config)), configPath_(std::move(configPath)),
          logger_(logger), safety_(safety) {
        balanceAngleGain_.store(config_.balanceControl.angleGainRatedTorquePerRadian);
        balanceAngularRateGain_.store(
            config_.balanceControl.angularRateGainRatedTorquePerRadianPerSecond);
    }

    int run() {
        config_.validateForManualConsole();
        WindowsTimerResolution timerResolution(1);
        initializeHardware();
        monitor_ = std::jthread([this](std::stop_token stopToken) { monitor(stopToken); });
        waitForFirstSample();
        const auto startupPendulumSample = latestPendulumSample();
        pendulumDownCount_.store(startupPendulumSample.positionCounts);
        pendulumZeroCaptured_.store(true);
        record("Pendulum downward zero initialized automatically at count=" +
               std::to_string(startupPendulumSample.positionCounts));

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
        std::chrono::steady_clock::time_point time{};
        std::uint64_t sequence{0};
    };

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
            const int eventRows = showHelp ? 0 : std::clamp(consoleHeight - 18, 0, 7);

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
                motion = "BALANCING / PENDULUM FEEDBACK ONLY";
            } else if (homingRunning_.load()) {
                std::scoped_lock lock(homingStateMutex_);
                motion = "HOMING / " + homingState_;
            } else if (left || right) {
                motion = std::string("LIMIT STOP / ") + (left ? "LEFT" : "RIGHT");
            } else if (calibrationRunning_.load()) {
                motion = "CALIBRATING MOTOR ZERO";
            } else if (!servo) {
                motion = "IDLE / SERVO OFF";
            } else if (std::abs(voltage -
                                config_.balanceControl.analogTorqueZeroVoltage) < 1e-12) {
                motion = "SERVO ON / CALIBRATED TORQUE ZERO";
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
            std::ostringstream balance;
            balance << "BALANCE   "
                    << (balanceRunning_.load() ? "RUNNING" : "STOPPED")
                    << "    down-zero "
                    << (pendulumZeroCaptured_.load() ? "CAPTURED" : "NOT SET")
                    << "    target " << pendulumUprightCount_.load()
                    << "    angle " << std::showpos << std::fixed
                    << std::setprecision(3) << balanceAngleDegrees_.load()
                    << " deg    rate " << balanceAngularRateDegrees_.load()
                    << " deg/s    polarity " << std::showpos
                    << balancePolarity_.load()
                    << "    Kp " << std::noshowpos << balanceAngleGain_.load()
                    << "    Kd " << balanceAngularRateGain_.load();
            screen << row(balance.str(), width) << '\n';

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
                screen << row("balance zero (recapture)    balance start [+|-]", width) << '\n';
                screen << row("balance stop|status    balance gains", width) << '\n';
                screen << row("balance kp <value>    balance kd <value>", width) << '\n';
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
                screen << row("balance start [+|-]    balance zero (recapture)    balance stop|status", width)
                       << '\n';
                screen << row("balance kp <v>    balance kd <v>    balance gains", width)
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
                                        config_.ni6602.pendulumEncoderBTerminal);
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
                    startBalance(std::nullopt);
                } else if (polarity == "+" || polarity == "+1") {
                    startBalance(1);
                } else if (polarity == "-" || polarity == "-1") {
                    startBalance(-1);
                } else {
                    notify("Usage: balance start [+|-]", true);
                }
            } else if (action == "stop") {
                stopBalance("operator balance stop");
            } else if (action == "status") {
                printBalanceStatus();
            } else if (action == "gains") {
                printBalanceGains();
            } else if (action == "kp" || action == "kd") {
                double value = 0.0;
                if (!(input >> value)) {
                    notify("Usage: balance " + action + " <non-negative value>", true);
                } else {
                    setBalanceGain(action, value);
                }
            } else {
                notify("Usage: balance zero|start|stop|status|gains|kp <value>|kd <value>", true);
            }
        } else if (command == "home") {
            std::string target;
            input >> target;
            if (target == "center") {
                runHomeOperation(false);
            } else if (target == "measure") {
                runHomeOperation(true);
            } else {
                notify("Usage: home measure|center", true);
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
            analogOutput_.writeVoltage(
                config_.balanceControl.analogTorqueZeroVoltage);
            commandedVoltage_.store(
                config_.balanceControl.analogTorqueZeroVoltage);
            ni_.setServoEnabled(true);
            servoOn_.store(true);
        }
        record("Operator command: servo on");
        std::ostringstream message;
        message << "Servo ON; AO0 calibrated torque zero="
                << config_.balanceControl.analogTorqueZeroVoltage << " V.";
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
            notify("Stop balance control before capturing the upright zero.", true);
            return;
        }
        if (!sampleReady_.load() || faultLatched_.load()) {
            notify("Downward zero capture requires live encoder samples.", true);
            return;
        }
        const auto count = pendulumPositionCounts_.load();
        pendulumDownCount_.store(count);
        pendulumZeroCaptured_.store(true);
        balanceAngleDegrees_.store(0.0);
        balanceAngularRateDegrees_.store(0.0);
        record("Pendulum downward zero captured at count=" + std::to_string(count));
        notify("Pendulum downward zero captured: count=" +
               std::to_string(count));
    }

    void startBalance(std::optional<int> polarityOverride) {
        if (balanceRunning_.load()) {
            notify("Balance control is already running.", true);
            return;
        }
        if (!pendulumZeroCaptured_.load()) {
            notify("Let the pendulum hang down and run 'balance zero' first.", true);
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
        if (!config_.balanceControl.analogTorqueZeroCalibrated) {
            notify("Warning: analog torque zero is not marked calibrated; run SGD7S Fn009 or Fn00B.",
                   true);
        }
        if (balanceThread_.joinable()) {
            balanceThread_.join();
        }
        balanceAbort_.store(false);
        balanceMissedDeadlines_.store(0);
        balanceMaxJitterMicroseconds_.store(0.0);
        balancePolarity_.store(
            polarityOverride.value_or(config_.balanceControl.defaultPolarity));
        const auto targetSample = latestPendulumSample();
        const auto targetCount = targetSample.positionCounts;
        pendulumUprightCount_.store(targetCount);
        const auto downToTargetCounts =
            pendulum::control::PendulumStateEstimator::wrappedCounts(
                targetCount - pendulumDownCount_.load(),
                config_.balanceControl.pendulumCountsPerRevolution);
        balanceRunning_.store(true);
        balanceThread_ =
            std::jthread([this](std::stop_token token) { balanceLoop(token); });
        notify("Balance loop started at " +
               std::to_string(config_.balanceControl.frequencyHz) +
               " Hz, polarity=" +
               (balancePolarity_.load() > 0 ? "+1" : "-1") +
               ", target_count=" + std::to_string(targetCount) +
               ", down_to_target_counts=" +
               std::to_string(downToTargetCounts) +
               ", Kp=" + std::to_string(balanceAngleGain_.load()) +
               ", Kd=" + std::to_string(balanceAngularRateGain_.load()) + ".");
    }

    void setBalanceGain(const std::string& gain, double value) {
        if (!std::isfinite(value) || value < 0.0) {
            notify("Balance gain must be a finite non-negative value.", true);
            return;
        }
        if (gain == "kp") {
            balanceAngleGain_.store(value);
        } else {
            balanceAngularRateGain_.store(value);
        }
        std::ostringstream message;
        message << "Balance " << gain << " updated immediately: " << value
                << " (runtime only)";
        record(message.str());
        notify(message.str());
    }

    void printBalanceGains() const {
        std::ostringstream message;
        message << "balance_kp=" << balanceAngleGain_.load()
                << ", balance_kd=" << balanceAngularRateGain_.load()
                << ", maximum_rated_torque_percent="
                << config_.balanceControl.maximumAbsoluteRatedTorqueFraction * 100.0;
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

    void balanceLoop(std::stop_token stopToken) noexcept {
        try {
            const auto& settings = config_.balanceControl;
            pendulum::control::PendulumStateEstimator estimator(
                settings.pendulumCountsPerRevolution,
                settings.angularRateFilterAlpha);
            pendulum::control::AnglePdController controller(
                balanceAngleGain_.load(),
                balanceAngularRateGain_.load(),
                settings.maximumAbsoluteRatedTorqueFraction);
            double activeAngleGain = balanceAngleGain_.load();
            double activeAngularRateGain = balanceAngularRateGain_.load();
            const auto period = std::chrono::nanoseconds(
                1'000'000'000LL / static_cast<std::int64_t>(settings.frequencyHz));

            {
                std::scoped_lock lock(outputMutex_);
                analogOutput_.writeVoltage(settings.analogTorqueZeroVoltage);
                commandedVoltage_.store(settings.analogTorqueZeroVoltage);
                ni_.setServoEnabled(true);
                servoOn_.store(true);
            }
            auto sample = latestPendulumSample();
            if (sample.sequence == 0) {
                throw std::runtime_error("no timestamped pendulum sample available");
            }
            estimator.reset(pendulumUprightCount_.load(),
                            sample.positionCounts, sample.time);
            auto previousSampleTime = sample.time;
            auto previousSampleSequence = sample.sequence;
            std::uint64_t sampleIndex = 0;
            record("Balance loop active in SGD7S analog torque mode; control is driven by timestamped pendulum samples; motor encoder excluded from feedback");

            while (!stopToken.stop_requested() && !balanceAbort_.load() &&
                   !faultLatched_.load() && !safety_.stopRequested()) {
                {
                    std::unique_lock lock(pendulumSampleMutex_);
                    pendulumSampleCondition_.wait_for(lock, 20ms, [&] {
                        return pendulumSample_.sequence != previousSampleSequence ||
                               stopToken.stop_requested() || balanceAbort_.load() ||
                               faultLatched_.load() || safety_.stopRequested();
                    });
                    if (stopToken.stop_requested() || balanceAbort_.load() ||
                        faultLatched_.load() || safety_.stopRequested()) {
                        break;
                    }
                    if (pendulumSample_.sequence == previousSampleSequence) {
                        continue;
                    }
                    sample = pendulumSample_;
                }

                const auto sampleInterval = sample.time - previousSampleTime;
                const double sampleIntervalUs =
                    std::chrono::duration<double, std::micro>(sampleInterval).count();
                const double jitterUs = std::abs(
                    std::chrono::duration<double, std::micro>(sampleInterval - period)
                        .count());
                if (jitterUs > balanceMaxJitterMicroseconds_.load()) {
                    balanceMaxJitterMicroseconds_.store(jitterUs);
                }
                const auto elapsedPeriods = sampleInterval / period;
                if (elapsedPeriods > 1) {
                    balanceMissedDeadlines_.fetch_add(
                        static_cast<std::uint64_t>(elapsedPeriods - 1));
                }
                previousSampleTime = sample.time;
                previousSampleSequence = sample.sequence;

                if (leftTriggered_.load() || rightTriggered_.load()) {
                    throw std::runtime_error("limit triggered during balance");
                }
                const auto state = estimator.update(
                    sample.positionCounts, sample.time);
                const double requestedAngleGain = balanceAngleGain_.load();
                const double requestedAngularRateGain = balanceAngularRateGain_.load();
                if (requestedAngleGain != activeAngleGain ||
                    requestedAngularRateGain != activeAngularRateGain) {
                    controller = pendulum::control::AnglePdController(
                        requestedAngleGain, requestedAngularRateGain,
                        settings.maximumAbsoluteRatedTorqueFraction);
                    activeAngleGain = requestedAngleGain;
                    activeAngularRateGain = requestedAngularRateGain;
                }
                balanceAngleDegrees_.store(
                    state.pendulumAngleRadians * 180.0 / std::numbers::pi);
                balanceAngularRateDegrees_.store(
                    state.pendulumAngularRateRadiansPerSecond * 180.0 /
                    std::numbers::pi);
                const double torqueFraction = controller.ratedTorqueFraction(
                    state, balancePolarity_.load());
                const double voltage = std::clamp(
                    settings.analogTorqueZeroVoltage +
                        torqueFraction * settings.ratedTorqueCommandVoltage,
                    config_.pci1723.minimumVoltage,
                    config_.pci1723.maximumVoltage);
                {
                    std::scoped_lock lock(outputMutex_);
                    if (balanceAbort_.load() || leftTriggered_.load() ||
                        rightTriggered_.load()) {
                        throw std::runtime_error("balance output interrupted");
                    }
                    analogOutput_.writeVoltage(voltage);
                    commandedVoltage_.store(voltage);
                }
                balanceTorquePercent_.store(torqueFraction * 100.0);

                if (++sampleIndex % settings.telemetryDivider == 0) {
                    std::ostringstream telemetry;
                    telemetry << "sample=" << sampleIndex
                              << ",pendulum_count="
                              << sample.positionCounts
                              << ",sample_sequence=" << sample.sequence
                              << ",sample_interval_us=" << sampleIntervalUs
                              << ",theta_rad=" << state.pendulumAngleRadians
                              << ",theta_dot_rad_s="
                              << state.pendulumAngularRateRadiansPerSecond
                              << ",kp=" << activeAngleGain
                              << ",kd=" << activeAngularRateGain
                              << ",rated_torque_fraction=" << torqueFraction
                              << ",rated_torque_percent=" << torqueFraction * 100.0
                              << ",polarity=" << balancePolarity_.load()
                              << ",ao_v=" << voltage
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
    }

    void printBalanceStatus() const {
        std::ostringstream message;
        message << "balance_running=" << std::boolalpha
                << balanceRunning_.load()
                << ", downward_zero_captured=" << pendulumZeroCaptured_.load()
                << ", downward_count=" << pendulumDownCount_.load()
                << ", balance_target_count=" << pendulumUprightCount_.load()
                << ", angle_deg=" << balanceAngleDegrees_.load()
                << ", angular_rate_deg_s="
                << balanceAngularRateDegrees_.load()
                << ", rated_torque_percent=" << balanceTorquePercent_.load()
                << ", maximum_rated_torque_percent="
                << config_.balanceControl.maximumAbsoluteRatedTorqueFraction * 100.0
                << ", kp=" << balanceAngleGain_.load()
                << ", kd=" << balanceAngularRateGain_.load()
                << ", polarity=" << std::showpos << balancePolarity_.load()
                << ", max_jitter_us="
                << balanceMaxJitterMicroseconds_.load()
                << ", missed_deadlines="
                << balanceMissedDeadlines_.load()
                << ", sample_driven=true"
                << ", motor_encoder_feedback=false"
                << ", drive=" << config_.balanceControl.driveModel
                << ", mode=" << config_.balanceControl.driveControlMode
                << ", pn400=" << config_.balanceControl.pn400Setting
                << ", rated_torque_voltage="
                << config_.balanceControl.ratedTorqueCommandVoltage
                << ", torque_zero_v="
                << config_.balanceControl.analogTorqueZeroVoltage
                << ", torque_zero_calibrated="
                << config_.balanceControl.analogTorqueZeroCalibrated
                << ", angle_kp_rated_torque_per_rad="
                << balanceAngleGain_.load()
                << ", angle_kd_rated_torque_per_rad_s="
                << balanceAngularRateGain_.load();
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
            homeResultAvailable_.store(true);
            homeCalibrationTravel_.store(result.travelCounts);
            homeCalibrationError_.store(result.centerErrorCounts);

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
            << "  calibrate zero                  Calibrate and hold motor Vzero\n"
            << "  balance zero                    Recapture downward count (automatic at startup)\n"
            << "  balance start [+|-]             Capture current upright target and stabilize\n"
            << "  balance kp <value>              Set angle gain immediately (runtime only)\n"
            << "  balance kd <value>              Set angular-rate gain immediately (runtime only)\n"
            << "  balance gains                   Show active Kp/Kd and torque limit\n"
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
    std::atomic<double> balanceTorquePercent_{0.0};
    std::atomic<double> balanceAngleGain_{0.0};
    std::atomic<double> balanceAngularRateGain_{0.0};
    std::atomic<int> balancePolarity_{1};
    std::atomic<double> balanceMaxJitterMicroseconds_{0.0};
    std::atomic<std::uint64_t> balanceMissedDeadlines_{0};
    std::atomic<bool> homeResultAvailable_{false};
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
