#include "DoubleBalanceConfig.h"
#include "ConsoleCommands.h"
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
    bool validateOnly{false};
    bool preview{false};
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
        } else if (argument == "--preview") {
            options.preview = true;
        } else if (argument == "--validate-only") {
            options.validateOnly = true;
        } else if (argument == "--help" || argument == "-h") {
            std::cout << "Usage: pendulum_console [--config PATH]\n";
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

class PendulumConsole final {
public:
    PendulumConsole(pendulum::config::AppConfig config,
                  std::filesystem::path configPath,
                  pendulum::logging::AsyncLogger& logger,
                  pendulum::safety::SafetyManager& safety, bool useDouble)
        : config_(std::move(config)), configPath_(std::move(configPath)),
          logger_(logger), safety_(safety) {
        if (useDouble) doubleConfig_ = pendulum::tools::loadDoubleConfig(configPath_, 0.0);
    }

    ~PendulumConsole() {
        stopBalance("session closed");
        stopOutputs("session closed");
        monitor_.request_stop();
        if (monitor_.joinable()) monitor_.join();
    }

    int preview() {
        dashboardMode_ = enableDashboardTerminal();
        std::cout << "Preview: output disabled; Q / Esc to return\n";
        if (dashboardMode_) {
            static_cast<void>(readDashboardCommand());
        } else {
            std::string line;
            while (std::getline(std::cin,line)) {
                if (pendulum::tools::parseConsoleCommand(trim(line)) == pendulum::tools::ConsoleCommand::Stop) break;
            }
        }
        return 0;
    }

    int run() {
        config_.validateForManualConsole();
        WindowsTimerResolution timerResolution(1);
        initializeHardware();
        monitor_ = std::jthread([this](std::stop_token token) { monitor(token); });
        waitForFirstSample();
        dashboardMode_ = enableDashboardTerminal();
        std::cout << (doubleConfig_ ? "Double" : "Single")
                  << " control selected; Q / Esc to stop\n";
        // The selected command owns a fixed hardware configuration for this session.
        // Homing and downward-zero capture remain inside the automatic sequence.
        runAutoBalance();
        if (!operatorStop_.load() && !safety_.stopRequested()) {
            if (dashboardMode_) {
                static_cast<void>(readDashboardCommand());
            } else {
                std::string line;
                while (std::getline(std::cin,line) && execute(trim(line))) {}
            }
        }
        stopBalance("operator stop");
        stopOutputs("operator stop");
        monitor_.request_stop();
        if (monitor_.joinable()) monitor_.join();
        return safety_.stopRequested() || faultLatched_.load() ? 1 : 0;
    }

private:
    struct PendulumSample {
        std::int64_t secondPositionCounts{0};
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
                "automatic centering is required for cart feedback");
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

    bool pollOperatorStop() {
        if (operatorStop_.load()) return true;
        if (_isatty(_fileno(stdin)) && _kbhit()) {
            const int key = _getwch();
            if (key == 0 || key == 0xE0) { static_cast<void>(_getwch()); return false; }
            if (key == 'q' || key == 'Q' || key == 27) {
                operatorStop_.store(true);
                calibrationAbort_.store(true);
                balanceAbort_.store(true);
                return true;
            }
        }
        return false;
    }

    std::optional<std::string> readDashboardCommand() {
        while (!safety_.stopRequested()) {
            if (pollOperatorStop()) return std::string("q");
            std::this_thread::sleep_for(10ms);
        }
        return std::nullopt;
    }

    void notify(std::string message, bool error = false) const {
        logger_.log(error ? pendulum::logging::Level::Error : pendulum::logging::Level::Info,
                    "PendulumConsole", message);
        if (error) std::cerr << message << '\n';
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
        if (doubleConfig_) {
            ni_.configureSecondPendulumEncoderRaw(doubleConfig_->secondCounter,
                doubleConfig_->secondATerminal, doubleConfig_->secondBTerminal,
                doubleConfig_->secondFilterSeconds);
        }
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
        std::uint32_t previousPendulum = 0, previousSecond = 0;
        std::int64_t secondPosition = 0;
        std::int64_t motorPosition = 0;
        std::int64_t pendulumPosition = 0;
        auto previousTime = std::chrono::steady_clock::now();
        bool havePreviousEncoderSample = false;
        try {
            while (!stopToken.stop_requested() && !safety_.stopRequested()) {
                const auto limits = ni_.readLimitInputs();
                const auto motorEncoder = ni_.readMotorEncoderRaw();
                const auto pendulumEncoder = ni_.readPendulumEncoderRaw();
                const auto secondEncoder = doubleConfig_ ? ni_.readSecondPendulumEncoderRaw() : 0U;
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
                if (doubleConfig_) {
                    const auto delta = pendulum::calibration::MotorEncoderCalibration::deltaWithRollover(
                        havePreviousEncoderSample ? previousSecond : 0U, secondEncoder);
                    secondPosition += delta;
                    if (havePreviousEncoderSample) {
                        const double dt = std::chrono::duration<double>(sampleTime - previousTime).count();
                        if (dt > 0) secondSpeedCountsPerSecond_.store(
                            0.8 * secondSpeedCountsPerSecond_.load() + 0.2 * static_cast<double>(delta) / dt);
                    }
                    secondPositionCounts_.store(secondPosition);
                    previousSecond = secondEncoder;
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
                    pendulumSample_.secondPositionCounts = secondPosition;
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
        logger_.log(pendulum::logging::Level::Warning, "PendulumConsole",
                    "Outputs stopped: " + reason);
    }

    bool outputsAllowed() const noexcept {
        return !operatorStop_.load() && sampleReady_.load() && !faultLatched_.load() &&
               !leftTriggered_.load() && !rightTriggered_.load();
    }

    bool execute(const std::string& line) {
        if (pendulum::tools::parseConsoleCommand(line) == pendulum::tools::ConsoleCommand::Stop) return false;
        if (!line.empty()) notify("Stop with Q, then select 1 or 2.",true);
        return true;
    }

    bool captureStablePendulumZero(const std::string& reason) {
        constexpr auto samplePeriod = 10ms;
        const auto requiredSamples = std::max<std::size_t>(
            2, static_cast<std::size_t>(std::ceil(
                   (doubleConfig_ ? doubleConfig_->downwardZeroCaptureSeconds : config_.balanceControl.downwardZeroCaptureSeconds) / 0.010)));
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration<double>(
                                  doubleConfig_ ? doubleConfig_->downwardZeroSettleTimeoutSeconds : config_.balanceControl.downwardZeroSettleTimeoutSeconds);
        std::deque<std::int64_t> window, secondWindow;
        pendulumZeroCaptured_.store(false);
        while (std::chrono::steady_clock::now() < deadline &&
               !faultLatched_.load() && !safety_.stopRequested() && !pollOperatorStop()) {
            const auto zeroSample = latestPendulumSample();
            window.push_back(zeroSample.positionCounts);
            secondWindow.push_back(zeroSample.secondPositionCounts);
            if (window.size() > requiredSamples) {
                window.pop_front();
                secondWindow.pop_front();
            }
            if (window.size() == requiredSamples) {
                const std::vector<std::int64_t> samples(window.begin(), window.end());
                try {
                    const auto count = stableRepresentative(
                        samples,
                        doubleConfig_ ? doubleConfig_->firstDownwardMaximumSpanCounts : config_.balanceControl.downwardZeroMaximumSpanCounts);
                    const auto [minimum, maximum] =
                        std::minmax_element(samples.begin(), samples.end());
                    if (doubleConfig_) {
                        secondDownCount_.store(stableRepresentative(
                            std::vector<std::int64_t>(secondWindow.begin(), secondWindow.end()),
                            doubleConfig_->secondDownwardMaximumSpanCounts));
                    }
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

    void runAutoBalance() {
        if (operatorStop_.load() || balanceRunning_.load()) {
            notify("Balance control is stopped or already running.", true);
            return;
        }
        runHomeOperation(false);
        const bool centered = homeResultAvailable_.load();
        if (pollOperatorStop() || !centered || !homeResultAvailable_.load() ||
            !pendulumZeroCaptured_.load() || faultLatched_.load() ||
            safety_.stopRequested() || leftTriggered_.load() ||
            rightTriggered_.load()) {
            notify("Automatic sequence aborted before swing-up because homing or safety validation failed.",
                   true);
            return;
        }
        startBalance(true);
    }

    #include "DoubleConsoleControl.inc"

    void startBalance(bool automaticSwingUp) {
        if (doubleConfig_) { startDoubleBalance(automaticSwingUp); return; }
        if (operatorStop_.load() || balanceRunning_.load()) {
            notify("Balance control is stopped or already running.", true);
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
            notify("Automatic centering did not complete. Stop and select the mode again.",
                   true);
            return;
        }
        try {
            const auto cart = currentCartSessionState(0.0);
            if (std::abs(cart.positionFromCenterHalfTravel) >
                config_.balanceControl.maximumBalanceStartPositionFraction) {
                notify("Cart is outside the allowed center window. Stop and select the mode again.",
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
        balanceSoftwareLimitActive_.store(false);
        balanceMissedDeadlines_.store(0);
        balanceMaxJitterMicroseconds_.store(0.0);
        const auto referenceSample = latestPendulumSample();
        const auto currentCount = referenceSample.positionCounts;
        const auto downToStart = wrappedCounts(
            currentCount - pendulumDownCount_.load(),
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
        if (!pendulumZeroCaptured_.load()) {
            notify("Balance start rejected: no stable downward zero is available.", true);
            return;
        }
        if (automaticSwingUp) {
            if (std::llabs(downToStart) >
                config_.balanceControl.downwardZeroMaximumSpanCounts) {
                notify("Automatic swing-up rejected: pendulum is not stationary at the captured downward position.",
                       true);
                return;
            }
        } else if (uprightErrorCounts > maximumStartErrorCounts) {
            notify("Balance start rejected: manually hold the pendulum within the reference model's +/-30 degree upright region.",
                   true);
            return;
        }
        const auto targetCount = automaticSwingUp
            ? pendulumDownCount_.load() + halfRevolution
            : currentCount;
        pendulumUprightCount_.store(targetCount);
        balanceCartReferenceCount_.store(referenceSample.motorPositionCounts);
        balanceAutoMode_.store(automaticSwingUp);
        balanceRunning_.store(true);
        const auto runId = balanceRunId_.fetch_add(1) + 1;
        balanceThread_ = std::jthread(
            [this, runId, automaticSwingUp](std::stop_token token) {
                balanceLoop(token, runId, automaticSwingUp);
            });
        notify("Balance loop started: run_id=" + std::to_string(runId) +
               ", frequency_hz=" +
               std::to_string(config_.balanceControl.frequencyHz) +
               ", balance_reference_count=" + std::to_string(targetCount) +
               ", cart_reference_count=" +
               std::to_string(referenceSample.motorPositionCounts) +
                ", controller=Copy_of_LQR_lp1_1 " +
                (automaticSwingUp ? "full Swing_up+LQR auto mode."
                                  : "manual-upright LQR mode."));
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

    void balanceLoop(std::stop_token stopToken, std::uint64_t runId,
                     bool automaticSwingUp) noexcept {
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
            auto sample = latestPendulumSample();
            const auto initialOutput = controller.update(
                sample.positionCounts - pendulumUprightCount_.load(),
                sample.motorPositionCounts - balanceCartReferenceCount_.load(),
                automaticSwingUp);
            if (!automaticSwingUp && initialOutput.outputVoltage != 0.0) {
                throw std::runtime_error(
                    "reference controller did not initialize at zero output");
            }

            {
                std::scoped_lock lock(outputMutex_);
                if (balanceAbort_.load() || !outputsAllowed() || safety_.stopRequested()) {
                    throw std::runtime_error("balance stopped before enabling output");
                }
                analogOutput_.writeVoltage(0.0);
                commandedVoltage_.store(0.0);
                ni_.setServoEnabled(true);
                servoOn_.store(true);
                if (balanceAbort_.load() || leftTriggered_.load() ||
                    rightTriggered_.load()) {
                    throw std::runtime_error("balance output interrupted at startup");
                }
                analogOutput_.writeVoltage(initialOutput.outputVoltage);
                commandedVoltage_.store(initialOutput.outputVoltage);
            }

            auto previousSampleSequence = sample.sequence;
            auto nextControlTime = sample.time + period;
            const auto balanceStartTime = sample.time;
            std::uint64_t sampleIndex = 0;
            auto previousSoftwareLimitSide =
                pendulum::control::SoftwareTravelLimitSide::None;
            const bool positiveVoltageMovesRight =
                config_.pci1723.positiveVoltageCartDirection == "RIGHT";
            record(std::string("Reference Copy_of_LQR_lp1_1 ") +
                   (automaticSwingUp ? "Swing_up+LQR" : "LQR") +
                   "+ACC2VOL active at 100 Hz; velocity mode; exact fixed gains");

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
                    sample.motorPositionCounts - balanceCartReferenceCount_.load(),
                    automaticSwingUp);
                if (!automaticSwingUp &&
                    std::abs(output.pendulumAngleRadians) >=
                    settings.maximumBalanceAngleRadians) {
                    throw std::runtime_error(
                        "pendulum left the reference LQR region; swing-up is disabled");
                }
                const auto cartSafety = currentCartSessionState(0.0);
                auto softwareLimit =
                    pendulum::control::SoftwareTravelLimitOutput{
                        output.outputVoltage};
                if (automaticSwingUp) {
                    softwareLimit = Controller::applySoftwareTravelLimit(
                        output.outputVoltage,
                        cartSafety.positionFromCenterHalfTravel,
                        settings.maximumBalancePositionFraction,
                        positiveVoltageMovesRight);
                    balanceSoftwareLimitActive_.store(
                        softwareLimit.side !=
                        pendulum::control::SoftwareTravelLimitSide::None);
                    if (softwareLimit.side != previousSoftwareLimitSide) {
                        if (softwareLimit.side ==
                            pendulum::control::SoftwareTravelLimitSide::None) {
                            record("Swing-up software limit cleared; normal command resumed");
                            notify("Swing-up recovered inside the software travel limit; continuing.");
                        } else {
                            const char* side =
                                softwareLimit.side ==
                                        pendulum::control::SoftwareTravelLimitSide::Left
                                    ? "LEFT"
                                    : "RIGHT";
                            record(std::string("Swing-up software limit reached: ") +
                                   side +
                                   "; outward commands blocked, run remains active",
                                   pendulum::logging::Level::Warning);
                            notify(std::string("Swing-up software limit: ") + side +
                                   "; attempting inward recovery without stopping.",
                                   true);
                        }
                        previousSoftwareLimitSide = softwareLimit.side;
                    }
                    if (softwareLimit.outwardCommandBlocked) {
                        controller.resetCommandIntegrators();
                    }
                } else if (std::abs(cartSafety.positionFromCenterHalfTravel) >=
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
                    analogOutput_.writeVoltage(softwareLimit.outputVoltage);
                    commandedVoltage_.store(softwareLimit.outputVoltage);
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
                balanceSwingUpActive_.store(output.swingUpActive);

                ++statisticsSamples;
                angleErrorSumDegrees += angleDegrees;
                absoluteAngleErrorSumDegrees += std::abs(angleDegrees);
                squaredAngleErrorSumDegrees += angleDegrees * angleDegrees;
                maximumAngleErrorDegrees = std::max(
                    maximumAngleErrorDegrees, std::abs(angleDegrees));
                maximumOutputVoltage = std::max(
                    maximumOutputVoltage,
                    std::abs(softwareLimit.outputVoltage));

                if (++sampleIndex % settings.telemetryDivider == 0) {
                    std::ostringstream telemetry;
                    telemetry << "run_id=" << runId
                              << ",controller_mode="
                              << (automaticSwingUp
                                      ? "reference_swing_up_lqr_acc2vol_auto"
                                      : "reference_lqr_acc2vol_manual_upright")
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
                              << ",lqr_acceleration_m_s2="
                              << output.lqrAccelerationMetersPerSecondSquared
                              << ",swing_acceleration_m_s2="
                              << output.swingUpAccelerationMetersPerSecondSquared
                              << ",swing_up_active=" << std::boolalpha
                              << output.swingUpActive
                              << ",velocity_ref_m_s="
                              << output.velocityReferenceMetersPerSecond
                              << ",velocity_error_m_s="
                              << output.velocityErrorMetersPerSecond
                              << ",pi_p_v=" << output.proportionalVoltage
                              << ",pi_i_v=" << output.integralVoltage
                              << ",controller_ao_v=" << output.outputVoltage
                              << ",ao_v=" << softwareLimit.outputVoltage
                              << ",software_limit_active="
                              << (softwareLimit.side !=
                                  pendulum::control::SoftwareTravelLimitSide::None)
                              << ",software_limit_outward_blocked="
                              << softwareLimit.outwardCommandBlocked
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
        balanceSwingUpActive_.store(false);
        balanceSoftwareLimitActive_.store(false);
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
                    return pollOperatorStop() || faultLatched_.load() || safety_.stopRequested();
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

    void setHomingState(const std::string& state) {
        std::scoped_lock lock(homingStateMutex_);
        homingState_ = state;
    }

    void record(const std::string& message,
                pendulum::logging::Level level = pendulum::logging::Level::Info) noexcept {
        logger_.log(level, "PendulumConsole", message);
    }


    std::optional<pendulum::tools::DoubleBalanceConfig> doubleConfig_;
    std::atomic<std::int64_t> secondPositionCounts_{0}, secondDownCount_{0}, secondReferenceCount_{0};
    std::atomic<double> secondSpeedCountsPerSecond_{0.0};
    pendulum::config::AppConfig config_;
    std::filesystem::path configPath_;
    pendulum::logging::AsyncLogger& logger_;
    pendulum::safety::SafetyManager& safety_;
    pendulum::hardware::NI6602 ni_;
    pendulum::hardware::PCI1723 analogOutput_;
    pendulum::safety::SafetyManager::Registration aoRegistration_;
    pendulum::safety::SafetyManager::Registration servoRegistration_;
    std::jthread monitor_;
    std::jthread balanceThread_;
    mutable std::mutex outputMutex_;
    mutable std::mutex homingStateMutex_;
    mutable std::mutex pendulumSampleMutex_;
    std::condition_variable pendulumSampleCondition_;
    PendulumSample pendulumSample_;
    std::string homingState_{"IDLE"};
    std::atomic<bool> operatorStop_{false};
    std::atomic<bool> dashboardMode_{false};
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
    std::atomic<bool> balanceAutoMode_{false};
    std::atomic<bool> balanceSwingUpActive_{false};
    std::atomic<bool> balanceSoftwareLimitActive_{false};
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
    config.validateForManualConsole();
    if (options.validateOnly) {
        static_cast<void>(pendulum::tools::loadDoubleConfig(options.configPath,0.0));
        std::cout << "Single and double configuration valid\n";
        return 0;
    }
    for (;;) {
        DWORD consoleMode = 0;
        const auto output = GetStdHandle(STD_OUTPUT_HANDLE);
        if (_isatty(_fileno(stdout)) && GetConsoleMode(output,&consoleMode) &&
            SetConsoleMode(output,consoleMode | ENABLE_VIRTUAL_TERMINAL_PROCESSING)) {
            std::cout << "\x1b[2J\x1b[H";
        }
        std::cout << "\nPendulumLab\n\n"
                  << "1  Single: swing-up + stabilize\n"
                  << "2  Double: swing-up + stabilize\n\n"
                  << "Q: exit\n> " << std::flush;
        std::string line;
        if (!std::getline(std::cin,line)) return 0;
        const auto command = pendulum::tools::parseConsoleCommand(trim(line));
        if (command == pendulum::tools::ConsoleCommand::Stop) return 0;
        if (command == pendulum::tools::ConsoleCommand::Invalid) continue;
        const bool useDouble = command == pendulum::tools::ConsoleCommand::Double;
        pendulum::logging::AsyncLogger logger(config.logging.directory,config.logging.queueCapacity);
        pendulum::safety::SafetyManager safety([&logger](const std::string& reason) {
            logger.log(pendulum::logging::Level::Critical,"Safety",reason);
        });
        pendulum::safety::ProcessSafetyHooks processHooks(safety);
        pendulum::safety::SafetyGuard guard(safety);
        PendulumConsole console(config,options.configPath,logger,safety,useDouble);
        const int result = options.preview ? console.preview() : console.run();
        if (result != 0) return result;
        // Destructor releases the selected device tasks before another mode is selected.
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
