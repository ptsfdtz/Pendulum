#pragma once

#include "pendulum/config/Config.h"

#include <cstdint>
#include <functional>
#include <string>

namespace pendulum::calibration {

enum class LimitSide { None, Left, Right };

struct HomeCenterSample {
    std::int64_t positionCounts{0};
    bool leftLimit{false};
    bool rightLimit{false};
};

struct HomeCenterResult {
    std::int64_t leftBoundaryCounts{0};
    std::int64_t rightBoundaryCounts{0};
    std::int64_t centerCounts{0};
    std::int64_t travelCounts{0};
    std::int64_t finalPositionCounts{0};
    std::int64_t centerErrorCounts{0};
};

class HomeCenterController final {
public:
    using ReadSample = std::function<HomeCenterSample()>;
    using CommandMotion = std::function<void(double, LimitSide)>;
    using StopMotion = std::function<void(const std::string&)>;
    using AbortRequested = std::function<bool()>;
    using Progress = std::function<void(const std::string&)>;

    HomeCenterController(config::HomeCenterConfig settings,
                         bool positiveVoltageMovesLeft,
                         ReadSample readSample,
                         CommandMotion commandMotion,
                         StopMotion stopMotion,
                         AbortRequested abortRequested,
                         Progress progress = {});

    HomeCenterResult run();
    HomeCenterResult runUsingStoredTravel(std::int64_t travelCounts);
    static std::int64_t midpoint(std::int64_t first, std::int64_t second);

private:
    struct Edge {
        LimitSide side{LimitSide::None};
        std::int64_t position{0};
    };

    Edge seek(double voltage, LimitSide expected);
    void learnDirection(LimitSide reachedSide, double towardVoltage);
    void discoverDirectionFromActiveLimit(LimitSide side);
    std::int64_t refine(LimitSide side);
    std::int64_t backOff(LimitSide side);
    std::int64_t moveToCenter(std::int64_t target, std::int64_t travel,
                              double aoToEncoderSign);
    double towardSign(LimitSide side) const;
    void waitPoll() const;
    void checkAbort() const;
    void report(const std::string& message) const;

    config::HomeCenterConfig settings_;
    bool positiveVoltageMovesLeft_;
    double leftTowardSign_{0.0};
    ReadSample readSample_;
    CommandMotion commandMotion_;
    StopMotion stopMotion_;
    AbortRequested abortRequested_;
    Progress progress_;
};

}  // namespace pendulum::calibration
