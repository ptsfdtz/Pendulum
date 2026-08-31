#include "pendulum/hardware/PCI1723.h"

#include <bdaqctrl.h>
#include <Windows.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

namespace pendulum::hardware {
namespace {

using namespace Automation::BDaq;

std::wstring toWide(const std::string& value) {
    if (value.empty()) {
        return {};
    }
    const int count = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, value.data(),
                                          static_cast<int>(value.size()), nullptr, 0);
    if (count <= 0) {
        throw std::runtime_error("Cannot convert Advantech device description to UTF-16");
    }
    std::wstring result(static_cast<std::size_t>(count), L'\0');
    MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, value.data(),
                        static_cast<int>(value.size()), result.data(), count);
    return result;
}

std::string toUtf8(const wchar_t* value) {
    if (value == nullptr || *value == L'\0') {
        return {};
    }
    const int count = WideCharToMultiByte(CP_UTF8, 0, value, -1, nullptr, 0, nullptr, nullptr);
    if (count <= 1) {
        return {};
    }
    std::string result(static_cast<std::size_t>(count), '\0');
    WideCharToMultiByte(CP_UTF8, 0, value, -1, result.data(), count, nullptr, nullptr);
    result.pop_back();
    return result;
}

std::string bdaqError(ErrorCode status) {
    std::array<wchar_t, 256> description{};
    AdxEnumToString(L"ErrorCode", static_cast<int32>(status),
                    static_cast<int32>(description.size()), description.data());
    return toUtf8(description.data()) + " (status " +
           std::to_string(static_cast<int32>(status)) + ')';
}

void checkBdaq(ErrorCode status, const char* operation) {
    if (BioFailed(status)) {
        throw std::runtime_error(std::string(operation) + ": " + bdaqError(status));
    }
}

}  // namespace

struct PCI1723::Impl {
    InstantAoCtrl* controller{nullptr};
    int channel{0};
    double minimumVoltage{-10.0};
    double maximumVoltage{10.0};
    bool opened{false};

    ~Impl() {
        if (controller != nullptr) {
            controller->Dispose();
            controller = nullptr;
        }
    }
};

PCI1723::PCI1723() : impl_(std::make_unique<Impl>()) {}

PCI1723::~PCI1723() {
    forceZeroVolts();
}

std::vector<std::string> PCI1723::enumerateDevices() {
    InstantAoCtrl* controller = InstantAoCtrl::Create();
    if (controller == nullptr) {
        throw std::runtime_error("InstantAoCtrl::Create returned null");
    }
    try {
        std::vector<std::string> result;
        auto* devices = controller->getSupportedDevices();
        if (devices == nullptr) {
            throw std::runtime_error("DAQNavi returned a null supported-device list");
        }
        for (int32 index = 0; index < devices->getLength(); ++index) {
            result.push_back(toUtf8(devices->getItem(index).Description));
        }
        controller->Dispose();
        return result;
    } catch (...) {
        controller->Dispose();
        throw;
    }
}

void PCI1723::open(const std::string& deviceDescription, int channel, double minimumVoltage,
                   double maximumVoltage) {
    if (impl_->controller != nullptr) {
        throw std::logic_error("PCI-1723 is already open");
    }
    if (deviceDescription.empty() || channel < 0 || !std::isfinite(minimumVoltage) ||
        !std::isfinite(maximumVoltage) || minimumVoltage >= maximumVoltage) {
        throw std::invalid_argument("Invalid PCI-1723 open parameters");
    }

    impl_->controller = InstantAoCtrl::Create();
    if (impl_->controller == nullptr) {
        throw std::runtime_error("InstantAoCtrl::Create returned null");
    }
    try {
        const auto description = toWide(deviceDescription);
        const DeviceInformation device(description.c_str(), ModeWrite);
        checkBdaq(impl_->controller->setSelectedDevice(device),
                  "InstantAoCtrl::setSelectedDevice");
        const int32 channelCount = impl_->controller->getFeatures()->getChannelCountMax();
        if (channel >= channelCount) {
            throw std::runtime_error("Configured AO channel exceeds PCI-1723 channel count");
        }
        impl_->channel = channel;
        impl_->minimumVoltage = minimumVoltage;
        impl_->maximumVoltage = maximumVoltage;
        impl_->opened = true;
    } catch (...) {
        impl_->controller->Dispose();
        impl_->controller = nullptr;
        throw;
    }
}

void PCI1723::writeVoltage(double voltage) {
    if (!impl_->opened || impl_->controller == nullptr) {
        throw std::logic_error("PCI-1723 is not open");
    }
    if (!std::isfinite(voltage) || voltage < impl_->minimumVoltage ||
        voltage > impl_->maximumVoltage) {
        throw std::out_of_range("Requested AO voltage is invalid or outside configured limits");
    }
    checkBdaq(impl_->controller->Write(impl_->channel, voltage), "InstantAoCtrl::Write");
}

void PCI1723::forceZeroVolts() noexcept {
    if (!impl_ || !impl_->opened || impl_->controller == nullptr) {
        return;
    }
    try {
        writeVoltage(0.0);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "CRITICAL: AO0 = 0 V failed: %s\n", error.what());
    } catch (...) {
        std::fputs("CRITICAL: AO0 = 0 V failed with an unknown error\n", stderr);
    }
}

bool PCI1723::isOpen() const noexcept {
    return impl_ && impl_->opened && impl_->controller != nullptr;
}

}  // namespace pendulum::hardware
