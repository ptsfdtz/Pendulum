#include "pendulum/calibration/HomeCenterController.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace pendulum::calibration {
namespace {

const char* sideName(LimitSide side) {
    if (side == LimitSide::Left) {
        return "LEFT";
    }
    return side == LimitSide::Right ? "RIGHT" : "ANY";
}

bool selected(LimitSide side, const HomeCenterSample& sample) {
    return side == LimitSide::Left ? sample.leftLimit : sample.rightLimit;
}

LimitSide opposite(LimitSide side) {
    return side == LimitSide::Left ? LimitSide::Right : LimitSide::Left;
}

}  // namespace

HomeCenterController::HomeCenterController(config::HomeCenterConfig settings,
                                           bool positiveVoltageMovesLeft,
                                           ReadSample readSample,
                                           CommandMotion commandMotion,
                                           StopMotion stopMotion,
                                           AbortRequested abortRequested,
                                           Progress progress)
    : settings_(settings), positiveVoltageMovesLeft_(positiveVoltageMovesLeft),
      readSample_(std::move(readSample)), commandMotion_(std::move(commandMotion)),
      stopMotion_(std::move(stopMotion)), abortRequested_(std::move(abortRequested)),
      progress_(std::move(progress)) {}

std::int64_t HomeCenterController::midpoint(std::int64_t first, std::int64_t second) {
    const auto low = std::min(first, second);
    const auto high = std::max(first, second);
    return low + (high - low) / 2;
}

HomeCenterResult HomeCenterController::run() {
    return execute(true, true);
}

HomeCenterResult HomeCenterController::measureTravel() {
    return execute(false, false);
}

HomeCenterResult HomeCenterController::execute(bool returnToCenter,
                                               bool enforceMinimumTravel) {
    auto sample = readSample_();
    if (sample.leftLimit && sample.rightLimit) {
        throw std::runtime_error("Both limits are active");
    }

    const auto startPosition = sample.positionCounts;
    LimitSide firstSide = LimitSide::None;
    if (sample.leftLimit) {
        firstSide = LimitSide::Left;
        discoverDirectionFromActiveLimit(firstSide);
    } else if (sample.rightLimit) {
        firstSide = LimitSide::Right;
        discoverDirectionFromActiveLimit(firstSide);
    } else {
        const double initialVoltage =
            (positiveVoltageMovesLeft_ ? 1.0 : -1.0) * settings_.searchVoltage;
        const auto firstEdge = seek(initialVoltage, LimitSide::None);
        firstSide = firstEdge.side;
        learnDirection(firstSide, initialVoltage);
    }

    const auto firstBoundary = refine(firstSide);
    report(std::string("Refined ") + sideName(firstSide) + " boundary=" +
           std::to_string(firstBoundary));
    static_cast<void>(backOff(firstSide));

    const auto secondSide = opposite(firstSide);
    report(std::string("Measuring full travel toward ") + sideName(secondSide) +
           " at travel voltage");
    const auto secondEdge = seek(towardSign(secondSide) * settings_.travelVoltage,
                                 secondSide);
    static_cast<void>(secondEdge);
    const auto secondBoundary = refine(secondSide);
    report(std::string("Refined ") + sideName(secondSide) + " boundary=" +
           std::to_string(secondBoundary));
    static_cast<void>(backOff(secondSide));

    const auto signedForwardTravel = secondBoundary - firstBoundary;
    const auto forwardTravel = std::llabs(signedForwardTravel);
    report("Forward measurement: start=" + std::to_string(startPosition) +
           ", first_boundary=" + std::to_string(firstBoundary) +
           ", second_boundary=" + std::to_string(secondBoundary) +
           ", relative_travel=" + std::to_string(forwardTravel));
    if (enforceMinimumTravel && forwardTravel < settings_.minimumTravelCounts) {
        throw std::runtime_error(
            "Measured forward travel " + std::to_string(forwardTravel) +
            " counts is below configured minimum " +
            std::to_string(settings_.minimumTravelCounts));
    }

    report(std::string("Verifying reverse travel toward ") + sideName(firstSide) +
           " at travel voltage");
    static_cast<void>(seek(towardSign(firstSide) * settings_.travelVoltage,
                           firstSide));
    const auto verifiedFirstBoundary = refine(firstSide);
    report(std::string("Verified ") + sideName(firstSide) + " boundary=" +
           std::to_string(verifiedFirstBoundary));
    static_cast<void>(backOff(firstSide));

    const auto signedReverseTravel = verifiedFirstBoundary - secondBoundary;
    const auto reverseTravel = std::llabs(signedReverseTravel);
    const auto disagreement = std::llabs(forwardTravel - reverseTravel);
    const bool oppositeDirections =
        signedForwardTravel != 0 && signedReverseTravel != 0 &&
        ((signedForwardTravel > 0) != (signedReverseTravel > 0));
    const auto allowedDisagreement = static_cast<std::int64_t>(std::ceil(
        static_cast<double>(std::max(forwardTravel, reverseTravel)) *
        settings_.maximumTravelDisagreementFraction));
    report("Travel verification: forward=" + std::to_string(forwardTravel) +
           ", reverse=" + std::to_string(reverseTravel) +
           ", disagreement=" + std::to_string(disagreement) +
           ", allowed=" + std::to_string(allowedDisagreement));
    if (!oppositeDirections) {
        throw std::runtime_error(
            "Encoder direction was not opposite on the return traversal");
    }
    if (enforceMinimumTravel && reverseTravel < settings_.minimumTravelCounts) {
        throw std::runtime_error(
            "Measured reverse travel " + std::to_string(reverseTravel) +
            " counts is below configured minimum " +
            std::to_string(settings_.minimumTravelCounts));
    }
    if (disagreement > allowedDisagreement) {
        throw std::runtime_error(
            "Forward/reverse travel disagreement " + std::to_string(disagreement) +
            " counts exceeds allowed " + std::to_string(allowedDisagreement));
    }

    const auto travel = midpoint(forwardTravel, reverseTravel);
    const auto center = midpoint(verifiedFirstBoundary, secondBoundary);
    report("Accepted travel=" + std::to_string(travel) + ", center=" +
           std::to_string(center));

    const double firstAwaySign = -towardSign(firstSide);
    const double aoToEncoderSign =
        (signedForwardTravel > 0 ? 1.0 : -1.0) * firstAwaySign;
    std::int64_t finalPosition = 0;
    if (returnToCenter) {
        finalPosition = moveToCenter(center, travel, aoToEncoderSign);
        stopMotion_("homing complete");
    } else {
        stopMotion_("travel measurement complete");
        finalPosition = readSample_().positionCounts;
    }

    HomeCenterResult result;
    result.leftBoundaryCounts =
        firstSide == LimitSide::Left ? verifiedFirstBoundary : secondBoundary;
    result.rightBoundaryCounts =
        firstSide == LimitSide::Right ? verifiedFirstBoundary : secondBoundary;
    result.centerCounts = center;
    result.travelCounts = travel;
    result.forwardTravelCounts = forwardTravel;
    result.reverseTravelCounts = reverseTravel;
    result.travelDisagreementCounts = disagreement;
    result.finalPositionCounts = finalPosition;
    result.centerErrorCounts = finalPosition - center;
    return result;
}

HomeCenterController::Edge HomeCenterController::seek(double voltage,
                                                       LimitSide expected) {
    report(std::string("Seeking ") + sideName(expected) + " at " +
           std::to_string(voltage) + " V");
    commandMotion_(voltage, LimitSide::None);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(settings_.searchTimeoutSeconds);
    while (true) {
        checkAbort();
        const auto sample = readSample_();
        if (sample.leftLimit || sample.rightLimit) {
            stopMotion_("limit reached during homing search");
            if (sample.leftLimit && sample.rightLimit) {
                throw std::runtime_error("Both limits are active during search");
            }
            const auto actual = sample.leftLimit ? LimitSide::Left : LimitSide::Right;
            if (expected != LimitSide::None && actual != expected) {
                throw std::runtime_error(std::string("Expected ") + sideName(expected) +
                                         " limit, reached " + sideName(actual));
            }
            return Edge{actual, sample.positionCounts};
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            throw std::runtime_error(std::string("Timed out seeking ") + sideName(expected));
        }
        waitPoll();
    }
}

void HomeCenterController::learnDirection(LimitSide reachedSide,
                                          double towardVoltage) {
    if (reachedSide == LimitSide::None || towardVoltage == 0.0) {
        throw std::runtime_error("Cannot learn motor direction from this limit event");
    }
    const double reachedSign = towardVoltage > 0.0 ? 1.0 : -1.0;
    leftTowardSign_ =
        reachedSide == LimitSide::Left ? reachedSign : -reachedSign;
    report(std::string("Learned direction: ") +
           (leftTowardSign_ > 0.0 ? "+V -> LEFT, -V -> RIGHT"
                                  : "+V -> RIGHT, -V -> LEFT"));
}

void HomeCenterController::discoverDirectionFromActiveLimit(LimitSide side) {
    const double configuredToward =
        ((side == LimitSide::Left) == positiveVoltageMovesLeft_) ? 1.0 : -1.0;
    const double preferredAway = -configuredToward;
    for (const double awaySign : {preferredAway, -preferredAway}) {
        report(std::string("Testing release direction from ") + sideName(side) +
               " at " + std::to_string(awaySign * settings_.escapeVoltage) +
               " V");
        commandMotion_(awaySign * settings_.escapeVoltage, side);
        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(settings_.awayDirectionTestMilliseconds);
        while (std::chrono::steady_clock::now() < deadline) {
            checkAbort();
            const auto sample = readSample_();
            if (sample.leftLimit && sample.rightLimit) {
                throw std::runtime_error("Both limits are active during direction test");
            }
            if (!selected(side, sample)) {
                stopMotion_("active-limit direction learned");
                learnDirection(side, -awaySign);
                return;
            }
            waitPoll();
        }
        stopMotion_("release direction test did not clear limit");
    }
    throw std::runtime_error(std::string("Could not release active ") +
                             sideName(side) + " limit");
}

std::int64_t HomeCenterController::refine(LimitSide side) {
    backOff(side);
    return seek(towardSign(side) * settings_.fineVoltage, side).position;
}

std::int64_t HomeCenterController::backOff(LimitSide side) {
    auto sample = readSample_();
    bool released = !selected(side, sample);
    std::int64_t releasePosition = sample.positionCounts;
    const double voltage = -towardSign(side) * settings_.escapeVoltage;
    commandMotion_(voltage, side);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(settings_.backoffTimeoutSeconds);
    while (true) {
        checkAbort();
        sample = readSample_();
        if (sample.leftLimit && sample.rightLimit) {
            throw std::runtime_error("Both limits are active during release");
        }
        if (!released && !selected(side, sample)) {
            released = true;
            releasePosition = sample.positionCounts;
        }
        if (released && std::llabs(sample.positionCounts - releasePosition) >=
                            settings_.escapeCounts) {
            stopMotion_("limit release complete");
            report(std::string("Released ") + sideName(side) + " by " +
                   std::to_string(std::llabs(sample.positionCounts - releasePosition)) +
                   " counts");
            return sample.positionCounts;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            throw std::runtime_error(std::string("Timed out releasing ") + sideName(side));
        }
        waitPoll();
    }
}

std::int64_t HomeCenterController::moveToCenter(std::int64_t target,
                                                std::int64_t travel,
                                                double aoToEncoderSign) {
    const auto tolerance = std::max(
        settings_.minimumCenterToleranceCounts,
        static_cast<std::int64_t>(std::llround(
            static_cast<double>(travel) * settings_.centerToleranceFraction)));
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(settings_.centerTimeoutSeconds);
    while (true) {
        checkAbort();
        const auto sample = readSample_();
        if (sample.leftLimit || sample.rightLimit) {
            throw std::runtime_error("Limit triggered while moving to center");
        }
        const auto error = target - sample.positionCounts;
        const auto distance = std::llabs(error);
        if (distance <= tolerance) {
            stopMotion_("center reached");
            std::this_thread::sleep_for(
                std::chrono::milliseconds(settings_.centerSettleMilliseconds));
            checkAbort();
            const auto settled = readSample_();
            if (settled.leftLimit || settled.rightLimit) {
                throw std::runtime_error("Limit triggered while settling at center");
            }
            return settled.positionCounts;
        }
        const double magnitude = distance > travel * 15 / 100
                                     ? settings_.centerFastVoltage
                                     : distance > travel * 3 / 100
                                           ? settings_.centerMidVoltage
                                           : settings_.centerSlowVoltage;
        const double encoderDirection = error > 0 ? 1.0 : -1.0;
        commandMotion_(encoderDirection * aoToEncoderSign * magnitude,
                       LimitSide::None);
        if (std::chrono::steady_clock::now() >= deadline) {
            throw std::runtime_error("Timed out moving to center");
        }
        waitPoll();
    }
}

double HomeCenterController::towardSign(LimitSide side) const {
    if (leftTowardSign_ == 0.0 || side == LimitSide::None) {
        throw std::runtime_error("Motor direction has not been learned");
    }
    return side == LimitSide::Left ? leftTowardSign_ : -leftTowardSign_;
}

void HomeCenterController::waitPoll() const {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(settings_.pollPeriodMilliseconds));
}

void HomeCenterController::checkAbort() const {
    if (abortRequested_()) {
        throw std::runtime_error("Homing aborted");
    }
}

void HomeCenterController::report(const std::string& message) const {
    if (progress_) {
        progress_(message);
    }
}

}  // namespace pendulum::calibration
