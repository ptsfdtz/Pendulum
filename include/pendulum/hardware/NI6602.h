#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace pendulum::hardware {

struct NiDeviceInfo {
    std::string name;
    std::string productType;
};

struct LimitInputState {
    bool leftRawHigh{false};
    bool rightRawHigh{false};
    bool leftTriggered{false};
    bool rightTriggered{false};
};

class NI6602 {
public:
    NI6602();
    ~NI6602();

    NI6602(const NI6602&) = delete;
    NI6602& operator=(const NI6602&) = delete;
    NI6602(NI6602&&) = delete;
    NI6602& operator=(NI6602&&) = delete;

    static std::vector<NiDeviceInfo> enumerateDevices();
    static bool readDigitalLine(const std::string& physicalLine);

    void configureMotorEncoder(const std::string& counter,
                               const std::string& phaseATerminal,
                               const std::string& phaseBTerminal,
                               std::uint32_t pulsesPerRevolution,
                               double filterMinPulseWidthSeconds);
    std::uint32_t readMotorEncoderRaw();
    bool motorEncoderConfigured() const noexcept;

    void configurePendulumEncoderRaw(const std::string& counter,
                                     const std::string& phaseATerminal,
                                     const std::string& phaseBTerminal,
                                     double filterMinPulseWidthSeconds);
    std::uint32_t readPendulumEncoderRaw();
    bool pendulumEncoderConfigured() const noexcept;

    void configureSecondPendulumEncoderRaw(
        const std::string& counter, const std::string& phaseATerminal,
        const std::string& phaseBTerminal,
        double filterMinPulseWidthSeconds);
    std::uint32_t readSecondPendulumEncoderRaw();
    bool secondPendulumEncoderConfigured() const noexcept;

    void configureLimitInputs(const std::string& leftLine,
                              bool leftActiveHigh,
                              const std::string& rightLine,
                              bool rightActiveHigh);
    LimitInputState readLimitInputs();
    bool limitInputsConfigured() const noexcept;

    void configureServoOutput(const std::string& physicalLine, bool activeHigh);
    void setServoEnabled(bool enabled);
    void forceServoOff() noexcept;
    bool servoOutputConfigured() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace pendulum::hardware
