#include "pendulum/safety/ProcessSafety.h"

#include "pendulum/safety/SafetyManager.h"

#include <Windows.h>

#include <cstdlib>
#include <stdexcept>

namespace pendulum::safety {
namespace {

BOOL WINAPI consoleHandler(DWORD event) {
    switch (event) {
        case CTRL_C_EVENT:
        case CTRL_BREAK_EVENT:
        case CTRL_CLOSE_EVENT:
        case CTRL_LOGOFF_EVENT:
        case CTRL_SHUTDOWN_EVENT:
            EmergencyStop("Windows console control event " + std::to_string(event));
            ExitProcess(130);
        default:
            return FALSE;
    }
}

[[noreturn]] void terminateHandler() noexcept {
    EmergencyStop("Unhandled exception or std::terminate");
    std::abort();
}

}  // namespace

ProcessSafetyHooks::ProcessSafetyHooks(SafetyManager& manager) {
    setGlobalSafetyManager(&manager);
    previousTerminate_ = std::set_terminate(terminateHandler);
    if (!SetConsoleCtrlHandler(consoleHandler, TRUE)) {
        setGlobalSafetyManager(nullptr);
        std::set_terminate(previousTerminate_);
        throw std::runtime_error("SetConsoleCtrlHandler failed");
    }
}

ProcessSafetyHooks::~ProcessSafetyHooks() {
    SetConsoleCtrlHandler(consoleHandler, FALSE);
    std::set_terminate(previousTerminate_);
    setGlobalSafetyManager(nullptr);
}

}  // namespace pendulum::safety
