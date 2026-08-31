#pragma once

#include <memory>
#include <string>
#include <vector>

namespace pendulum::hardware {

class PCI1723 {
public:
    PCI1723();
    ~PCI1723();

    PCI1723(const PCI1723&) = delete;
    PCI1723& operator=(const PCI1723&) = delete;
    PCI1723(PCI1723&&) = delete;
    PCI1723& operator=(PCI1723&&) = delete;

    static std::vector<std::string> enumerateDevices();

    void open(const std::string& deviceDescription, int channel, double minimumVoltage,
              double maximumVoltage);
    void writeVoltage(double voltage);
    void forceZeroVolts() noexcept;
    bool isOpen() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace pendulum::hardware
