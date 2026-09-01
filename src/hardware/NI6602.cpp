#include "pendulum/hardware/NI6602.h"

#include <NIDAQmx.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

namespace pendulum::hardware {
namespace {

std::string trim(std::string value) {
    const auto notSpace = [](unsigned char character) { return !std::isspace(character); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
    value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
    return value;
}

std::string daqError(int32 status) {
    std::array<char, 4096> buffer{};
    if (DAQmxGetErrorString(status, buffer.data(), static_cast<uInt32>(buffer.size())) < 0) {
        return "Unknown NI-DAQmx error " + std::to_string(status);
    }
    return std::string(buffer.data()) + " (status " + std::to_string(status) + ')';
}

void checkDaq(int32 status, const char* operation) {
    if (DAQmxFailed(status)) {
        throw std::runtime_error(std::string(operation) + ": " + daqError(status));
    }
}

class Task final {
public:
    ~Task() { clear(); }

    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;
    Task() = default;

    TaskHandle* address() { return &handle_; }
    TaskHandle get() const { return handle_; }
    bool valid() const { return handle_ != nullptr; }

    void clear() noexcept {
        if (handle_ != nullptr) {
            DAQmxStopTask(handle_);
            DAQmxClearTask(handle_);
            handle_ = nullptr;
        }
    }

private:
    TaskHandle handle_{nullptr};
};

}  // namespace

struct NI6602::Impl {
    Task motorEncoderTask;
    Task pendulumEncoderTask;
    Task leftLimitTask;
    Task rightLimitTask;
    Task servoTask;
    bool leftLimitActiveHigh{true};
    bool rightLimitActiveHigh{true};
    bool servoActiveHigh{true};
};

NI6602::NI6602() : impl_(std::make_unique<Impl>()) {}

NI6602::~NI6602() {
    forceServoOff();
}

std::vector<NiDeviceInfo> NI6602::enumerateDevices() {
    std::array<char, 4096> deviceNames{};
    checkDaq(DAQmxGetSysDevNames(deviceNames.data(), static_cast<uInt32>(deviceNames.size())),
             "DAQmxGetSysDevNames");

    std::vector<NiDeviceInfo> devices;
    std::string remaining(deviceNames.data());
    std::size_t offset = 0;
    while (offset <= remaining.size()) {
        const auto comma = remaining.find(',', offset);
        auto name = trim(remaining.substr(offset, comma - offset));
        if (!name.empty()) {
            std::array<char, 1024> product{};
            checkDaq(DAQmxGetDevProductType(name.c_str(), product.data(),
                                            static_cast<uInt32>(product.size())),
                     "DAQmxGetDevProductType");
            devices.push_back(NiDeviceInfo{std::move(name), product.data()});
        }
        if (comma == std::string::npos) {
            break;
        }
        offset = comma + 1;
    }
    return devices;
}

bool NI6602::readDigitalLine(const std::string& physicalLine) {
    if (physicalLine.empty()) {
        throw std::invalid_argument("NI digital input line must not be empty");
    }
    Task task;
    checkDaq(DAQmxCreateTask("PendulumLabDigitalInput", task.address()), "DAQmxCreateTask(DI)");
    checkDaq(DAQmxCreateDIChan(task.get(), physicalLine.c_str(), "", DAQmx_Val_ChanPerLine),
             "DAQmxCreateDIChan");
    std::array<uInt8, 1> data{};
    int32 samplesRead = 0;
    int32 bytesPerSample = 0;
    checkDaq(DAQmxReadDigitalLines(task.get(), 1, 1.0, DAQmx_Val_GroupByChannel, data.data(),
                                   static_cast<uInt32>(data.size()), &samplesRead,
                                   &bytesPerSample, nullptr),
             "DAQmxReadDigitalLines");
    if (samplesRead != 1 || bytesPerSample < 1) {
        throw std::runtime_error("NI digital input returned an unexpected sample count");
    }
    return data[0] != 0;
}

void NI6602::configureMotorEncoder(const std::string& counter,
                                   const std::string& phaseATerminal,
                                   const std::string& phaseBTerminal,
                                   std::uint32_t pulsesPerRevolution,
                                   double filterMinPulseWidthSeconds) {
    if (counter.empty() || counter == "UNCONFIRMED" || phaseATerminal.empty() ||
        phaseATerminal == "UNCONFIRMED" || phaseBTerminal.empty() ||
        phaseBTerminal == "UNCONFIRMED" || pulsesPerRevolution == 0 ||
        !std::isfinite(filterMinPulseWidthSeconds) || filterMinPulseWidthSeconds < 0.0) {
        throw std::invalid_argument("Invalid or unconfirmed motor encoder configuration");
    }
    const bool useDefaultRouting = phaseATerminal == "DEFAULT" && phaseBTerminal == "DEFAULT";
    const bool useExplicitRouting = phaseATerminal != "DEFAULT" &&
                                    phaseBTerminal != "DEFAULT" &&
                                    phaseATerminal != phaseBTerminal;
    if (!useDefaultRouting && !useExplicitRouting) {
        throw std::invalid_argument("Encoder A/B routing must both be DEFAULT or distinct terminals");
    }

    impl_->motorEncoderTask.clear();
    checkDaq(DAQmxCreateTask("PendulumLabMotorEncoder", impl_->motorEncoderTask.address()),
             "DAQmxCreateTask(motor encoder)");
    try {
        checkDaq(DAQmxCreateCIAngEncoderChan(
                     impl_->motorEncoderTask.get(), counter.c_str(), "", DAQmx_Val_X4, false,
                     0.0, DAQmx_Val_AHighBHigh, DAQmx_Val_Ticks, pulsesPerRevolution, 0.0,
                     nullptr),
                 "DAQmxCreateCIAngEncoderChan");
        if (!useDefaultRouting) {
            checkDaq(DAQmxSetCIEncoderAInputTerm(impl_->motorEncoderTask.get(), "",
                                                 phaseATerminal.c_str()),
                     "DAQmxSetCIEncoderAInputTerm");
            checkDaq(DAQmxSetCIEncoderBInputTerm(impl_->motorEncoderTask.get(), "",
                                                 phaseBTerminal.c_str()),
                     "DAQmxSetCIEncoderBInputTerm");
        }
        if (filterMinPulseWidthSeconds > 0.0) {
            checkDaq(DAQmxSetCIEncoderAInputDigFltrMinPulseWidth(
                         impl_->motorEncoderTask.get(), "", filterMinPulseWidthSeconds),
                     "DAQmxSetCIEncoderAInputDigFltrMinPulseWidth");
            checkDaq(DAQmxSetCIEncoderBInputDigFltrMinPulseWidth(
                         impl_->motorEncoderTask.get(), "", filterMinPulseWidthSeconds),
                     "DAQmxSetCIEncoderBInputDigFltrMinPulseWidth");
            checkDaq(DAQmxSetCIEncoderAInputDigFltrEnable(
                         impl_->motorEncoderTask.get(), "", true),
                     "DAQmxSetCIEncoderAInputDigFltrEnable");
            checkDaq(DAQmxSetCIEncoderBInputDigFltrEnable(
                         impl_->motorEncoderTask.get(), "", true),
                     "DAQmxSetCIEncoderBInputDigFltrEnable");
        }
        checkDaq(DAQmxStartTask(impl_->motorEncoderTask.get()),
                 "DAQmxStartTask(motor encoder)");
    } catch (...) {
        impl_->motorEncoderTask.clear();
        throw;
    }
}

std::uint32_t NI6602::readMotorEncoderRaw() {
    if (!impl_->motorEncoderTask.valid()) {
        throw std::logic_error("Motor encoder task has not been configured");
    }
    uInt32 value = 0;
    checkDaq(DAQmxReadCounterScalarU32(impl_->motorEncoderTask.get(), 1.0, &value, nullptr),
             "DAQmxReadCounterScalarU32(motor encoder)");
    return value;
}

bool NI6602::motorEncoderConfigured() const noexcept {
    return impl_ && impl_->motorEncoderTask.valid();
}

void NI6602::configurePendulumEncoderRaw(const std::string& counter,
                                         const std::string& phaseATerminal,
                                         const std::string& phaseBTerminal) {
    if (counter.empty() || counter == "UNCONFIRMED" || phaseATerminal.empty() ||
        phaseATerminal == "UNCONFIRMED" || phaseBTerminal.empty() ||
        phaseBTerminal == "UNCONFIRMED") {
        throw std::invalid_argument("Invalid or unconfirmed pendulum encoder configuration");
    }
    const bool useDefaultRouting = phaseATerminal == "DEFAULT" && phaseBTerminal == "DEFAULT";
    const bool useExplicitRouting = phaseATerminal != "DEFAULT" &&
                                    phaseBTerminal != "DEFAULT" &&
                                    phaseATerminal != phaseBTerminal;
    if (!useDefaultRouting && !useExplicitRouting) {
        throw std::invalid_argument(
            "Pendulum encoder A/B routing must both be DEFAULT or distinct terminals");
    }

    impl_->pendulumEncoderTask.clear();
    checkDaq(DAQmxCreateTask("PendulumLabPendulumEncoder",
                             impl_->pendulumEncoderTask.address()),
             "DAQmxCreateTask(pendulum encoder)");
    try {
        // pulsesPerRev does not scale raw DAQmx_Val_Ticks reads, but DAQmx requires it nonzero.
        checkDaq(DAQmxCreateCIAngEncoderChan(
                     impl_->pendulumEncoderTask.get(), counter.c_str(), "", DAQmx_Val_X4,
                     false, 0.0, DAQmx_Val_AHighBHigh, DAQmx_Val_Ticks, 1, 0.0, nullptr),
                 "DAQmxCreateCIAngEncoderChan(pendulum encoder)");
        if (!useDefaultRouting) {
            checkDaq(DAQmxSetCIEncoderAInputTerm(impl_->pendulumEncoderTask.get(), "",
                                                 phaseATerminal.c_str()),
                     "DAQmxSetCIEncoderAInputTerm(pendulum encoder)");
            checkDaq(DAQmxSetCIEncoderBInputTerm(impl_->pendulumEncoderTask.get(), "",
                                                 phaseBTerminal.c_str()),
                     "DAQmxSetCIEncoderBInputTerm(pendulum encoder)");
        }
        checkDaq(DAQmxStartTask(impl_->pendulumEncoderTask.get()),
                 "DAQmxStartTask(pendulum encoder)");
    } catch (...) {
        impl_->pendulumEncoderTask.clear();
        throw;
    }
}

std::uint32_t NI6602::readPendulumEncoderRaw() {
    if (!impl_->pendulumEncoderTask.valid()) {
        throw std::logic_error("Pendulum encoder task has not been configured");
    }
    uInt32 value = 0;
    checkDaq(DAQmxReadCounterScalarU32(impl_->pendulumEncoderTask.get(), 1.0, &value, nullptr),
             "DAQmxReadCounterScalarU32(pendulum encoder)");
    return value;
}

bool NI6602::pendulumEncoderConfigured() const noexcept {
    return impl_ && impl_->pendulumEncoderTask.valid();
}

void NI6602::configureLimitInputs(const std::string& leftLine,
                                  bool leftActiveHigh,
                                  const std::string& rightLine,
                                  bool rightActiveHigh) {
    if (leftLine.empty() || leftLine == "UNCONFIRMED" || rightLine.empty() ||
        rightLine == "UNCONFIRMED" || leftLine == rightLine) {
        throw std::invalid_argument("Limit input lines must be distinct and confirmed");
    }

    impl_->leftLimitTask.clear();
    impl_->rightLimitTask.clear();
    impl_->leftLimitActiveHigh = leftActiveHigh;
    impl_->rightLimitActiveHigh = rightActiveHigh;
    try {
        checkDaq(DAQmxCreateTask("PendulumLabLeftLimit", impl_->leftLimitTask.address()),
                 "DAQmxCreateTask(left limit)");
        checkDaq(DAQmxCreateDIChan(impl_->leftLimitTask.get(), leftLine.c_str(), "",
                                   DAQmx_Val_ChanPerLine),
                 "DAQmxCreateDIChan(left limit)");
        checkDaq(DAQmxStartTask(impl_->leftLimitTask.get()),
                 "DAQmxStartTask(left limit)");

        checkDaq(DAQmxCreateTask("PendulumLabRightLimit", impl_->rightLimitTask.address()),
                 "DAQmxCreateTask(right limit)");
        checkDaq(DAQmxCreateDIChan(impl_->rightLimitTask.get(), rightLine.c_str(), "",
                                   DAQmx_Val_ChanPerLine),
                 "DAQmxCreateDIChan(right limit)");
        checkDaq(DAQmxStartTask(impl_->rightLimitTask.get()),
                 "DAQmxStartTask(right limit)");
    } catch (...) {
        impl_->leftLimitTask.clear();
        impl_->rightLimitTask.clear();
        throw;
    }
}

LimitInputState NI6602::readLimitInputs() {
    if (!limitInputsConfigured()) {
        throw std::logic_error("Limit input tasks have not been configured");
    }

    const auto readLine = [](TaskHandle task, const char* operation) {
        uInt8 value = 0;
        int32 samplesRead = 0;
        int32 bytesPerSample = 0;
        checkDaq(DAQmxReadDigitalLines(task, 1, 0.1, DAQmx_Val_GroupByChannel, &value, 1,
                                       &samplesRead, &bytesPerSample, nullptr),
                 operation);
        if (samplesRead != 1 || bytesPerSample < 1) {
            throw std::runtime_error(std::string(operation) +
                                     " returned an unexpected sample count");
        }
        return value != 0;
    };

    LimitInputState state;
    state.leftRawHigh = readLine(impl_->leftLimitTask.get(),
                                 "DAQmxReadDigitalLines(left limit)");
    state.rightRawHigh = readLine(impl_->rightLimitTask.get(),
                                  "DAQmxReadDigitalLines(right limit)");
    state.leftTriggered = state.leftRawHigh == impl_->leftLimitActiveHigh;
    state.rightTriggered = state.rightRawHigh == impl_->rightLimitActiveHigh;
    return state;
}

bool NI6602::limitInputsConfigured() const noexcept {
    return impl_ && impl_->leftLimitTask.valid() && impl_->rightLimitTask.valid();
}

void NI6602::configureServoOutput(const std::string& physicalLine, bool activeHigh) {
    if (physicalLine.empty() || physicalLine == "UNCONFIRMED") {
        throw std::invalid_argument("Refusing to configure an unconfirmed Servo output line");
    }
    impl_->servoTask.clear();
    impl_->servoActiveHigh = activeHigh;
    checkDaq(DAQmxCreateTask("PendulumLabServoEnable", impl_->servoTask.address()),
             "DAQmxCreateTask(DO)");
    try {
        checkDaq(DAQmxCreateDOChan(impl_->servoTask.get(), physicalLine.c_str(), "",
                                   DAQmx_Val_ChanPerLine),
                 "DAQmxCreateDOChan");
        setServoEnabled(false);
    } catch (...) {
        impl_->servoTask.clear();
        throw;
    }
}

void NI6602::setServoEnabled(bool enabled) {
    if (!impl_->servoTask.valid()) {
        throw std::logic_error("Servo output task has not been configured");
    }
    const bool physicalHigh = enabled ? impl_->servoActiveHigh : !impl_->servoActiveHigh;
    const uInt8 value = physicalHigh ? 1U : 0U;
    int32 written = 0;
    checkDaq(DAQmxWriteDigitalLines(impl_->servoTask.get(), 1, true, 1.0,
                                    DAQmx_Val_GroupByChannel, &value, &written, nullptr),
             "DAQmxWriteDigitalLines");
    if (written != 1) {
        throw std::runtime_error("NI Servo output write did not report one sample written");
    }
}

void NI6602::forceServoOff() noexcept {
    if (!impl_ || !impl_->servoTask.valid()) {
        return;
    }
    try {
        setServoEnabled(false);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "CRITICAL: Servo OFF failed: %s\n", error.what());
    } catch (...) {
        std::fputs("CRITICAL: Servo OFF failed with an unknown error\n", stderr);
    }
}

bool NI6602::servoOutputConfigured() const noexcept {
    return impl_ && impl_->servoTask.valid();
}

}  // namespace pendulum::hardware
