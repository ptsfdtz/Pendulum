#pragma once

#include <exception>

namespace pendulum::safety {

class SafetyManager;

class ProcessSafetyHooks {
public:
    explicit ProcessSafetyHooks(SafetyManager& manager);
    ~ProcessSafetyHooks();

    ProcessSafetyHooks(const ProcessSafetyHooks&) = delete;
    ProcessSafetyHooks& operator=(const ProcessSafetyHooks&) = delete;

private:
    std::terminate_handler previousTerminate_{nullptr};
};

}  // namespace pendulum::safety

