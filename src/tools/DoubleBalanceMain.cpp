#include "pendulum/calibration/HomeCenterController.h"
#include "pendulum/calibration/MotorEncoderCalibration.h"
#include "pendulum/config/Config.h"
#include "pendulum/control/DoublePendulumLqrController.h"
#include "pendulum/hardware/NI6602.h"
#include "pendulum/hardware/PCI1723.h"
#include "pendulum/safety/ProcessSafety.h"
#include "pendulum/safety/SafetyManager.h"

#include <Windows.h>
#include <mmsystem.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <conio.h>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

namespace {

using Clock = std::chrono::steady_clock;
using json = nlohmann::json;

struct Options {
    std::filesystem::path configPath{"config/config.json"};
    std::optional<double> durationSeconds;
    bool validateOnly{false};
};

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
};

Options parseOptions(int argc, char* argv[]) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--config" && ++index < argc) {
            options.configPath = argv[index];
        } else if (argument == "--duration" && ++index < argc) {
            options.durationSeconds = std::stod(argv[index]);
        } else if (argument == "--validate-only") {
            options.validateOnly = true;
        } else if (argument == "--help" || argument == "-h") {
            std::cout << "Usage: pendulum_double_balance [--config PATH] "
                         "[--duration SECONDS] [--validate-only]\n"
                         "  --duration 0 runs until Q/Esc or a safety stop.\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("Unknown or incomplete argument: " + argument);
        }
    }
    return options;
}

json loadJson(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Cannot open configuration: " + path.string());
    }
    json value;
    input >> value;
    return value;
}

DoubleBalanceConfig loadDoubleConfig(const std::filesystem::path& path,
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
        config.secondJumpCounts <= 0) {
        throw std::runtime_error("Invalid or unsafe double_balance_control settings");
    }
    // Constructing validates all controller fields and gains.
    static_cast<void>(pendulum::control::DoublePendulumLqrController(config.controller));
    return config;
}

std::string timeStamp() {
    const auto now = std::chrono::system_clock::now();
    const auto value = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
    localtime_s(&local, &value);
    std::ostringstream stream;
    stream << std::put_time(&local, "%Y%m%d_%H%M%S");
    return stream.str();
}

class CounterAccumulator {
public:
    std::int64_t update(std::uint32_t raw) {
        lastDelta_ = 0;
        if (initialized_) {
            lastDelta_ = pendulum::calibration::MotorEncoderCalibration::deltaWithRollover(
                previousRaw_, raw);
            value_ += lastDelta_;
        }
        previousRaw_ = raw;
        initialized_ = true;
        return value_;
    }
    std::int64_t value() const noexcept { return value_; }
    std::int64_t lastDelta() const noexcept { return lastDelta_; }

private:
    bool initialized_{false};
    std::uint32_t previousRaw_{0};
    std::int64_t value_{0};
    std::int64_t lastDelta_{0};
};

struct HardwareSample {
    Clock::time_point time;
    std::uint32_t rawCart{0};
    std::uint32_t rawFirst{0};
    std::uint32_t rawSecond{0};
    std::int64_t cartCounts{0};
    std::int64_t firstCounts{0};
    std::int64_t secondCounts{0};
    std::int64_t cartDelta{0};
    std::int64_t firstDelta{0};
    std::int64_t secondDelta{0};
    bool leftLimit{false};
    bool rightLimit{false};
};

class HardwareSession {
public:
    HardwareSession(const pendulum::config::AppConfig& app,
                    const DoubleBalanceConfig& control,
                    pendulum::safety::SafetyManager& safety)
        : app_(app), control_(control) {
        ao_.open(app_.pci1723.deviceDescription, app_.pci1723.aoChannel,
                 app_.pci1723.minimumVoltage, app_.pci1723.maximumVoltage);
        aoRegistration_ = safety.registerAction(0, "AO0 zero", [this] { ao_.forceZeroVolts(); });
        ni_.configureServoOutput(app_.ni6602.servoEnableLine, app_.ni6602.servoActiveHigh);
        servoRegistration_ = safety.registerAction(10, "Servo OFF", [this] { ni_.forceServoOff(); });
        ao_.writeVoltage(0.0);
        ni_.setServoEnabled(false);
        ni_.configureLimitInputs(app_.ni6602.leftLimitLine, app_.ni6602.leftLimitActiveHigh,
                                 app_.ni6602.rightLimitLine, app_.ni6602.rightLimitActiveHigh);
        ni_.configureMotorEncoder(app_.ni6602.motorCounter,
                                  app_.ni6602.motorEncoderATerminal,
                                  app_.ni6602.motorEncoderBTerminal,
                                  app_.ni6602.motorEncoderPulsesPerRevolution,
                                  app_.ni6602.motorEncoderFilterMinPulseWidthMicroseconds * 1e-6);
        ni_.configurePendulumEncoderRaw(
            app_.ni6602.pendulumCounter, app_.ni6602.pendulumEncoderATerminal,
            app_.ni6602.pendulumEncoderBTerminal,
            app_.ni6602.pendulumEncoderFilterMinPulseWidthMicroseconds * 1e-6);
        ni_.configureSecondPendulumEncoderRaw(
            control_.secondCounter, control_.secondATerminal, control_.secondBTerminal,
            control_.secondFilterSeconds);
        sample();
    }

    HardwareSample sample() {
        HardwareSample value;
        const auto limits = ni_.readLimitInputs();
        value.rawCart = ni_.readMotorEncoderRaw();
        value.rawFirst = ni_.readPendulumEncoderRaw();
        value.rawSecond = ni_.readSecondPendulumEncoderRaw();
        value.time = Clock::now();
        value.cartCounts = cart_.update(value.rawCart);
        value.firstCounts = first_.update(value.rawFirst);
        value.secondCounts = second_.update(value.rawSecond);
        value.cartDelta = cart_.lastDelta();
        value.firstDelta = first_.lastDelta();
        value.secondDelta = second_.lastDelta();
        value.leftLimit = limits.leftTriggered;
        value.rightLimit = limits.rightTriggered;
        last_ = value;
        return value;
    }

    void command(double voltage,
                 pendulum::calibration::LimitSide releaseSide =
                     pendulum::calibration::LimitSide::None,
                 bool enforceInstantLimit = true) {
        if (enforceInstantLimit) {
            const auto limits = ni_.readLimitInputs();
            if (limits.leftTriggered || limits.rightTriggered) {
                const auto active = limits.leftTriggered
                    ? pendulum::calibration::LimitSide::Left
                    : pendulum::calibration::LimitSide::Right;
                if (limits.leftTriggered && limits.rightTriggered) {
                    throw std::runtime_error("Both physical limits are active");
                }
                if (releaseSide != active) {
                    throw std::runtime_error("Motion into an active physical limit was rejected");
                }
            }
        }
        if (!servoEnabled_) {
            ao_.writeVoltage(0.0);
            ni_.setServoEnabled(true);
            servoEnabled_ = true;
        }
        ao_.writeVoltage(voltage);
    }

    void stop() noexcept {
        ao_.forceZeroVolts();
        ni_.forceServoOff();
        servoEnabled_ = false;
    }

private:
    const pendulum::config::AppConfig& app_;
    const DoubleBalanceConfig& control_;
    pendulum::hardware::NI6602 ni_;
    pendulum::hardware::PCI1723 ao_;
    pendulum::safety::SafetyManager::Registration aoRegistration_;
    pendulum::safety::SafetyManager::Registration servoRegistration_;
    CounterAccumulator cart_;
    CounterAccumulator first_;
    CounterAccumulator second_;
    HardwareSample last_;
    bool servoEnabled_{false};
};

struct Metrics {
    double sumTheta1Squared{0.0};
    double sumTheta2Squared{0.0};
    double maxX{0.0};
    double maxTheta1{0.0};
    double maxTheta2{0.0};
    double maxVoltage{0.0};
    std::uint64_t samples{0};
    std::uint64_t saturatedSamples{0};
};

void writeMetrics(const std::filesystem::path& path, const Metrics& metrics,
                  double duration, const std::string& result,
                  const std::string& reason) {
    json value{
        {"result", result}, {"termination_reason", reason},
        {"duration_seconds", duration}, {"samples", metrics.samples},
        {"rms_theta1_degrees", metrics.samples == 0 ? 0.0 :
            std::sqrt(metrics.sumTheta1Squared / metrics.samples) * 180.0 / std::numbers::pi},
        {"rms_theta2_degrees", metrics.samples == 0 ? 0.0 :
            std::sqrt(metrics.sumTheta2Squared / metrics.samples) * 180.0 / std::numbers::pi},
        {"max_abs_x_meters", metrics.maxX},
        {"max_abs_theta1_degrees", metrics.maxTheta1 * 180.0 / std::numbers::pi},
        {"max_abs_theta2_degrees", metrics.maxTheta2 * 180.0 / std::numbers::pi},
        {"max_abs_voltage", metrics.maxVoltage},
        {"voltage_saturation_fraction", metrics.samples == 0 ? 0.0 :
            static_cast<double>(metrics.saturatedSamples) / metrics.samples}};
    std::ofstream output(path);
    output << value.dump(2) << '\n';
}

int run(const Options& options) {
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    const auto app = pendulum::config::AppConfig::load(options.configPath);
    app.validateForManualConsole();
    const auto config = loadDoubleConfig(options.configPath, options.durationSeconds);
    if (options.validateOnly) {
        std::cout << "Double-pendulum configuration valid: 200 Hz, K=[";
        for (std::size_t index = 0; index < config.controller.gain.size(); ++index) {
            if (index != 0) std::cout << ", ";
            std::cout << config.controller.gain[index];
        }
        std::cout << "], duration=";
        if (config.durationSeconds == 0.0) {
            std::cout << "unlimited (Q/Esc to stop)\n";
        } else {
            std::cout << config.durationSeconds << " s\n";
        }
        return 0;
    }

    timeBeginPeriod(1);
    struct TimerGuard { ~TimerGuard() { timeEndPeriod(1); } } timerGuard;
    const auto runDirectory = std::filesystem::path("experiments") /
                              "double_balance" / ("run_" + timeStamp());
    std::filesystem::create_directories(runDirectory);
    std::filesystem::copy_file(options.configPath, runDirectory / "config_snapshot.json");
    std::ofstream telemetry(runDirectory / "telemetry.csv");
    telemetry << "time_s,raw_cart,raw_theta1,raw_theta2,cart_counts,theta1_counts,"
                 "theta2_relative_counts,x_m,theta1_rad,theta2_rad,xdot_m_s,"
                 "theta1dot_rad_s,theta2dot_rad_s,accel_m_s2,velocity_ref_m_s,"
                 "voltage,left_limit,right_limit,warning\n";

    pendulum::safety::SafetyManager safety;
    pendulum::safety::ProcessSafetyHooks processHooks(safety);
    pendulum::safety::SafetyGuard safetyGuard(safety);
    HardwareSession hardware(app, config, safety);

    std::cout << "硬件已启动，正在自动寻找两端限位并返回中点。\n";
    const bool positiveMovesLeft = app.pci1723.positiveVoltageCartDirection == "LEFT";
    pendulum::calibration::HomeCenterController home(
        app.homeCenter, positiveMovesLeft,
        [&hardware] {
            const auto s = hardware.sample();
            return pendulum::calibration::HomeCenterSample{
                s.cartCounts, s.leftLimit, s.rightLimit};
        },
        [&hardware](double voltage, pendulum::calibration::LimitSide side) {
            hardware.command(voltage, side);
        },
        [&hardware](const std::string&) { hardware.stop(); },
        [&safety] { return safety.stopRequested(); },
        [](const std::string& message) { std::cout << "[回中] " << message << '\n'; });
    const auto homeResult = home.run();
    hardware.stop();
    {
        const json homeRecord{
            {"left_boundary_counts", homeResult.leftBoundaryCounts},
            {"right_boundary_counts", homeResult.rightBoundaryCounts},
            {"travel_counts", homeResult.travelCounts},
            {"forward_travel_counts", homeResult.forwardTravelCounts},
            {"reverse_travel_counts", homeResult.reverseTravelCounts},
            {"travel_disagreement_counts", homeResult.travelDisagreementCounts},
            {"center_counts", homeResult.centerCounts},
            {"final_position_counts", homeResult.finalPositionCounts},
            {"center_error_counts", homeResult.centerErrorCounts}};
        std::ofstream homeOutput(runDirectory / "home.json");
        homeOutput << homeRecord.dump(2) << '\n';
    }
    std::cout << "回中完成：行程=" << homeResult.travelCounts
              << " counts，中点误差=" << homeResult.centerErrorCounts
              << " counts。\n";
    std::cout << "请扶正两级摆，扶稳后按回车开始 LQR：" << std::flush;
    std::string line;
    std::getline(std::cin, line);

    const auto reference = hardware.sample();
    if (reference.leftLimit || reference.rightLimit) {
        throw std::runtime_error("Cannot arm while a physical limit is active");
    }
    pendulum::control::DoublePendulumLqrController controller(config.controller);
    controller.reset();
    hardware.command(0.0);

    if (config.durationSeconds == 0.0) {
        std::cout << "无限运行模式：按 Q 或 Esc 正常结束并保存日志；Ctrl+C 为紧急停机。\n";
    }

    Metrics metrics;
    const auto start = Clock::now();
    auto previousTime = start;
    auto next = start;
    std::string result = "completed";
    std::string reason = "duration reached";
    bool positionWarningPrinted = false;
    std::uint32_t leftLimitCount = 0;
    std::uint32_t rightLimitCount = 0;
    const auto limitDebounceSamples = app.manualConsole.limitDebounceSamples;
    std::cout << "物理限位防抖：连续 " << limitDebounceSamples
              << " 个采样触发后停机。\n";
    try {
        while (!safety.stopRequested()) {
            next += std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(config.controller.sampleSeconds));
            const auto sample = hardware.sample();
            const double elapsed = std::chrono::duration<double>(sample.time - start).count();
            const double sampleElapsed =
                std::chrono::duration<double>(sample.time - previousTime).count();
            previousTime = sample.time;
            if (config.durationSeconds > 0.0 && elapsed >= config.durationSeconds) break;
            if (_kbhit()) {
                const int key = _getch();
                if (key == 'q' || key == 'Q' || key == 27) {
                    reason = "operator requested stop";
                    break;
                }
            }
            if (sampleElapsed > config.sampleTimeoutSeconds) {
                throw std::runtime_error("sample timeout");
            }
            leftLimitCount = sample.leftLimit ? leftLimitCount + 1 : 0;
            rightLimitCount = sample.rightLimit ? rightLimitCount + 1 : 0;
            if (leftLimitCount >= limitDebounceSamples) {
                throw std::runtime_error("LEFT physical limit stable after debounce");
            }
            if (rightLimitCount >= limitDebounceSamples) {
                throw std::runtime_error("RIGHT physical limit stable after debounce");
            }
            if (std::llabs(sample.cartDelta) > config.cartJumpCounts ||
                std::llabs(sample.firstDelta) > config.firstJumpCounts ||
                std::llabs(sample.secondDelta) > config.secondJumpCounts) {
                throw std::runtime_error("encoder jump detected");
            }

            const auto output = controller.update(
                sample.cartCounts - homeResult.centerCounts,
                sample.firstCounts - reference.firstCounts,
                sample.secondCounts - reference.secondCounts);
            if (std::abs(output.firstAngleRadians) >= config.stopAngleRadians ||
                std::abs(output.secondAngleRadians) >= config.stopAngleRadians) {
                throw std::runtime_error("pendulum reached 10 degree stop angle");
            }
            if (std::abs(output.cartPositionMeters) >= config.positionStopMeters) {
                throw std::runtime_error("cart reached 0.35 m software stop");
            }
            const bool warning =
                std::abs(output.cartPositionMeters) >= config.positionWarningMeters;
            if (warning && !positionWarningPrinted) {
                std::cout << "警告：小车位置达到 0.30 m。\n";
                positionWarningPrinted = true;
            }
            // Limit safety is enforced above after the same consecutive-sample
            // debounce used by the proven single-pendulum monitor.
            hardware.command(output.outputVoltage,
                             pendulum::calibration::LimitSide::None, false);

            telemetry << std::setprecision(17) << elapsed << ',' << sample.rawCart << ','
                      << sample.rawFirst << ',' << sample.rawSecond << ','
                      << sample.cartCounts << ',' << sample.firstCounts << ','
                      << sample.secondCounts << ',' << output.cartPositionMeters << ','
                      << output.firstAngleRadians << ',' << output.secondAngleRadians << ','
                      << output.cartVelocityMetersPerSecond << ','
                      << output.firstAngularRateRadiansPerSecond << ','
                      << output.secondAngularRateRadiansPerSecond << ','
                      << output.accelerationCommandMetersPerSecondSquared << ','
                      << output.velocityReferenceMetersPerSecond << ','
                      << output.outputVoltage << ',' << sample.leftLimit << ','
                      << sample.rightLimit << ',' << warning << '\n';
            ++metrics.samples;
            metrics.sumTheta1Squared += output.firstAngleRadians * output.firstAngleRadians;
            metrics.sumTheta2Squared += output.secondAngleRadians * output.secondAngleRadians;
            metrics.maxX = std::max(metrics.maxX, std::abs(output.cartPositionMeters));
            metrics.maxTheta1 = std::max(metrics.maxTheta1,
                                         std::abs(output.firstAngleRadians));
            metrics.maxTheta2 = std::max(metrics.maxTheta2,
                                         std::abs(output.secondAngleRadians));
            metrics.maxVoltage = std::max(metrics.maxVoltage,
                                           std::abs(output.outputVoltage));
            metrics.saturatedSamples += output.voltageSaturated ? 1U : 0U;
            if (metrics.samples % 100 == 0) {
                std::cout << '\r' << std::fixed << std::setprecision(2)
                          << "LQR " << elapsed << " s  x=" << output.cartPositionMeters
                          << " m  θ1=" << output.firstAngleRadians * 180.0 / std::numbers::pi
                          << "°  θ2=" << output.secondAngleRadians * 180.0 / std::numbers::pi
                          << "°  V=" << output.outputVoltage << "   " << std::flush;
            }
            std::this_thread::sleep_until(next);
        }
    } catch (const std::exception& error) {
        result = "safety_abort";
        reason = error.what();
    }
    hardware.stop();
    const double actualDuration = std::chrono::duration<double>(Clock::now() - start).count();
    telemetry.flush();
    writeMetrics(runDirectory / "metrics.json", metrics, actualDuration, result, reason);
    std::cout << "\n测试结束：" << result << "，原因：" << reason
              << "\n结果目录：" << runDirectory.string() << '\n';
    return result == "completed" ? 0 : 2;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        return run(parseOptions(argc, argv));
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    } catch (...) {
        std::cerr << "FAILED: unknown exception\n";
        return 2;
    }
}
