#include "pendulum/calibration/MotorZeroCalibrator.h"

#include "pendulum/calibration/MotorEncoderCalibration.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace pendulum::calibration {
namespace {

std::string measurementText(double voltage, double speed) {
    std::ostringstream stream;
    stream << std::showpos << std::fixed << std::setprecision(7) << voltage
           << " V, speed=" << std::setprecision(3) << speed << " counts/s";
    return stream.str();
}

std::vector<double> inclusiveRange(double start, double end, double step) {
    std::vector<double> result;
    const auto count = static_cast<std::size_t>(
        std::floor((end - start) / step + 0.5));
    result.reserve(count + 1);
    for (std::size_t index = 0; index <= count; ++index) {
        result.push_back(start + static_cast<double>(index) * step);
    }
    if (result.empty() || result.back() < end - step * 0.25) {
        result.push_back(end);
    }
    return result;
}

}  // namespace

MotorZeroCalibrator::MotorZeroCalibrator(config::MotorZeroCalibrationConfig settings,
                                         double hardwareMinimumVoltage,
                                         double hardwareMaximumVoltage,
                                         WriteVoltage writeVoltage,
                                         ReadCount readCount,
                                         AbortRequested abortRequested,
                                         Progress progress)
    : settings_(std::move(settings)),
      dacLsbVoltage_((hardwareMaximumVoltage - hardwareMinimumVoltage) /
                     std::ldexp(1.0, static_cast<int>(settings_.dacBits))),
      writeVoltage_(std::move(writeVoltage)),
      readCount_(std::move(readCount)),
      abortRequested_(std::move(abortRequested)),
      progress_(std::move(progress)) {
    if (!writeVoltage_ || !readCount_ || !abortRequested_ ||
        !std::isfinite(dacLsbVoltage_) || dacLsbVoltage_ <= 0.0) {
        throw std::invalid_argument("Invalid motor zero calibrator callbacks or DAC range");
    }
}

double MotorZeroCalibrator::median(std::vector<double> values) {
    if (values.empty()) {
        throw std::invalid_argument("Cannot calculate median of an empty sequence");
    }
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    if (values.size() % 2 != 0) {
        return *middle;
    }
    const double upper = *middle;
    const double lower = *std::max_element(values.begin(), middle);
    return (lower + upper) / 2.0;
}

double MotorZeroCalibrator::findZeroCrossing(const std::vector<double>& voltages,
                                             const std::vector<double>& speeds) {
    if (voltages.empty() || voltages.size() != speeds.size()) {
        throw std::invalid_argument("Zero-crossing inputs must be nonempty and equal length");
    }
    for (std::size_t index = 0; index + 1 < voltages.size(); ++index) {
        if (speeds[index] == 0.0) {
            return voltages[index];
        }
        if ((speeds[index] < 0.0 && speeds[index + 1] > 0.0) ||
            (speeds[index] > 0.0 && speeds[index + 1] < 0.0)) {
            return voltages[index] + (-speeds[index]) *
                   (voltages[index + 1] - voltages[index]) /
                   (speeds[index + 1] - speeds[index]);
        }
    }
    const auto best = std::min_element(
        speeds.begin(), speeds.end(),
        [](double left, double right) { return std::abs(left) < std::abs(right); });
    return voltages[static_cast<std::size_t>(std::distance(speeds.begin(), best))];
}

VoltageInterval MotorZeroCalibrator::findLongestDeadband(
    const std::vector<double>& voltages, const std::vector<double>& speeds,
    double threshold) {
    if (voltages.empty() || voltages.size() != speeds.size() || threshold < 0.0) {
        throw std::invalid_argument("Invalid deadband inputs");
    }
    VoltageInterval result;
    std::size_t bestLength = 0;
    std::size_t start = voltages.size();
    for (std::size_t index = 0; index <= speeds.size(); ++index) {
        const bool inside = index < speeds.size() && std::abs(speeds[index]) <= threshold;
        if (inside && start == voltages.size()) {
            start = index;
        } else if (!inside && start != voltages.size()) {
            const auto length = index - start;
            if (length > bestLength) {
                bestLength = length;
                result = VoltageInterval{voltages[start], voltages[index - 1], true};
            }
            start = voltages.size();
        }
    }
    return result;
}

MotorZeroCalibrationResult MotorZeroCalibrator::run() {
    report("Servo settling at AO0=0 V");
    wait(settings_.servoSettleSeconds);

    report("Coarse scan starting");
    const auto coarseVoltages = inclusiveRange(settings_.coarseStartVoltage,
                                               settings_.coarseEndVoltage,
                                               settings_.coarseStepVoltage);
    std::vector<double> coarseSpeeds;
    coarseSpeeds.reserve(coarseVoltages.size());
    for (const double voltage : coarseVoltages) {
        const double speed = measureAtVoltage(voltage, settings_.coarseSettleSeconds,
                                              settings_.coarseSampleSeconds,
                                              settings_.coarseRepeats);
        coarseSpeeds.push_back(speed);
        report("coarse: " + measurementText(voltage, speed));
    }
    const double candidate = findZeroCrossing(coarseVoltages, coarseSpeeds);
    report("Coarse zero candidate: " + measurementText(candidate, 0.0));

    const auto lowCode = static_cast<std::int64_t>(
        std::floor((candidate - settings_.fineHalfRangeVoltage) / dacLsbVoltage_));
    const auto highCode = static_cast<std::int64_t>(
        std::ceil((candidate + settings_.fineHalfRangeVoltage) / dacLsbVoltage_));
    std::vector<double> fineVoltages;
    fineVoltages.reserve(static_cast<std::size_t>(highCode - lowCode + 1));
    for (auto code = lowCode; code <= highCode; ++code) {
        fineVoltages.push_back(static_cast<double>(code) * dacLsbVoltage_);
    }

    report("Forward DAC-code scan starting");
    writeBounded(fineVoltages.front() -
                 settings_.finePreconditionCodes * dacLsbVoltage_);
    wait(settings_.finePreconditionSeconds);
    std::vector<double> forwardSpeeds;
    forwardSpeeds.reserve(fineVoltages.size());
    for (const double voltage : fineVoltages) {
        const double speed = measureAtVoltage(voltage, settings_.fineSettleSeconds,
                                              settings_.fineSampleSeconds,
                                              settings_.fineRepeats);
        forwardSpeeds.push_back(speed);
        report("forward: " + measurementText(voltage, speed));
    }

    report("Reverse DAC-code scan starting");
    writeBounded(fineVoltages.back() +
                 settings_.finePreconditionCodes * dacLsbVoltage_);
    wait(settings_.finePreconditionSeconds);
    std::vector<double> reverseSpeeds(fineVoltages.size());
    for (std::size_t index = fineVoltages.size(); index-- > 0;) {
        reverseSpeeds[index] = measureAtVoltage(
            fineVoltages[index], settings_.fineSettleSeconds,
            settings_.fineSampleSeconds, settings_.fineRepeats);
        report("reverse: " + measurementText(fineVoltages[index], reverseSpeeds[index]));
    }

    const auto forwardBand = findLongestDeadband(
        fineVoltages, forwardSpeeds, settings_.targetSpeedCountsPerSecond);
    const auto reverseBand = findLongestDeadband(
        fineVoltages, reverseSpeeds, settings_.targetSpeedCountsPerSecond);
    double rawZero = 0.0;
    if (forwardBand.found && reverseBand.found) {
        const double overlapLow = std::max(forwardBand.low, reverseBand.low);
        const double overlapHigh = std::min(forwardBand.high, reverseBand.high);
        rawZero = overlapLow <= overlapHigh
                      ? (overlapLow + overlapHigh) / 2.0
                      : ((forwardBand.low + forwardBand.high) / 2.0 +
                         (reverseBand.low + reverseBand.high) / 2.0) /
                            2.0;
    } else if (forwardBand.found) {
        rawZero = (forwardBand.low + forwardBand.high) / 2.0;
    } else if (reverseBand.found) {
        rawZero = (reverseBand.low + reverseBand.high) / 2.0;
    } else {
        double bestScore = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < fineVoltages.size(); ++index) {
            const double score = std::max(std::abs(forwardSpeeds[index]),
                                          std::abs(reverseSpeeds[index]));
            if (score < bestScore) {
                bestScore = score;
                rawZero = fineVoltages[index];
            }
        }
    }
    double zero = std::round(rawZero / dacLsbVoltage_) * dacLsbVoltage_;
    report("Initial DAC-aligned zero: " + measurementText(zero, 0.0));

    report("Final three-code refinement starting");
    const std::vector<double> testVoltages{zero - dacLsbVoltage_, zero,
                                           zero + dacLsbVoltage_};
    std::vector<double> scores;
    scores.reserve(testVoltages.size());
    for (const double voltage : testVoltages) {
        writeBounded(voltage - settings_.finalPreconditionCodes * dacLsbVoltage_);
        wait(settings_.finalPreconditionSeconds);
        writeBounded(voltage);
        wait(settings_.finalSettleSeconds);
        std::vector<double> speeds;
        speeds.reserve(settings_.finalRepeats);
        for (std::uint32_t repeat = 0; repeat < settings_.finalRepeats; ++repeat) {
            speeds.push_back(measureSignedSpeed(settings_.finalSampleSeconds));
        }
        std::vector<double> absoluteSpeeds = speeds;
        std::transform(absoluteSpeeds.begin(), absoluteSpeeds.end(),
                       absoluteSpeeds.begin(), [](double value) { return std::abs(value); });
        scores.push_back(median(std::move(absoluteSpeeds)));
        report("refine: " + measurementText(voltage, median(std::move(speeds))) +
               ", score=" + std::to_string(scores.back()));
    }
    const auto best = std::min_element(scores.begin(), scores.end());
    zero = testVoltages[static_cast<std::size_t>(std::distance(scores.begin(), best))];

    writeBounded(zero);
    wait(settings_.verificationSettleSeconds);
    report("Long verification starting");
    std::vector<double> verification;
    verification.reserve(settings_.verificationRepeats);
    for (std::uint32_t repeat = 0; repeat < settings_.verificationRepeats; ++repeat) {
        const double speed = measureSignedSpeed(settings_.verificationSampleSeconds);
        verification.push_back(speed);
        report("verify " + std::to_string(repeat + 1) + "/" +
               std::to_string(settings_.verificationRepeats) + ": " +
               measurementText(zero, speed));
    }
    const double signedSpeed = median(verification);
    std::transform(verification.begin(), verification.end(), verification.begin(),
                   [](double value) { return std::abs(value); });
    const double absoluteSpeed = median(std::move(verification));
    const bool accepted = absoluteSpeed <=
                          settings_.maximumAcceptedSpeedCountsPerSecond;
    if (!accepted) {
        writeBounded(0.0);
    }
    return MotorZeroCalibrationResult{
        zero, dacLsbVoltage_, signedSpeed, absoluteSpeed, accepted,
        absoluteSpeed <= settings_.targetSpeedCountsPerSecond};
}

double MotorZeroCalibrator::measureSignedSpeed(double sampleSeconds) {
    checkAbort();
    const auto startCount = readCount_();
    const auto start = std::chrono::steady_clock::now();
    wait(sampleSeconds);
    const auto endCount = readCount_();
    const auto end = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(end - start).count();
    const auto delta = MotorEncoderCalibration::deltaWithRollover(startCount, endCount);
    return static_cast<double>(delta) / elapsed;
}

double MotorZeroCalibrator::measureAtVoltage(double voltage, double settleSeconds,
                                             double sampleSeconds,
                                             std::uint32_t repeats) {
    writeBounded(voltage);
    wait(settleSeconds);
    std::vector<double> speeds;
    speeds.reserve(repeats);
    for (std::uint32_t repeat = 0; repeat < repeats; ++repeat) {
        speeds.push_back(measureSignedSpeed(sampleSeconds));
    }
    return median(std::move(speeds));
}

void MotorZeroCalibrator::writeBounded(double voltage) {
    checkAbort();
    writeVoltage_(std::clamp(voltage, -settings_.voltageLimit,
                             settings_.voltageLimit));
}

void MotorZeroCalibrator::wait(double seconds) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < deadline) {
        checkAbort();
        const std::chrono::duration<double> remaining =
            deadline - std::chrono::steady_clock::now();
        const std::chrono::duration<double> pollInterval =
            std::chrono::milliseconds(5);
        std::this_thread::sleep_for(std::min(remaining, pollInterval));
    }
    checkAbort();
}

void MotorZeroCalibrator::checkAbort() const {
    if (abortRequested_()) {
        throw std::runtime_error("Motor zero calibration aborted");
    }
}

void MotorZeroCalibrator::report(const std::string& message) const {
    if (progress_) {
        progress_(message);
    }
}

}  // namespace pendulum::calibration
