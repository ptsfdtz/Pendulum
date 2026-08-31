#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace pendulum::safety {

class SafetyManager {
public:
    using Action = std::function<void()>;
    using LogSink = std::function<void(const std::string&)>;

    class Registration {
    public:
        Registration() = default;
        ~Registration();
        Registration(const Registration&) = delete;
        Registration& operator=(const Registration&) = delete;
        Registration(Registration&& other) noexcept;
        Registration& operator=(Registration&& other) noexcept;

    private:
        friend class SafetyManager;
        Registration(SafetyManager* owner, std::uint64_t id);
        void reset() noexcept;

        SafetyManager* owner_{nullptr};
        std::uint64_t id_{0};
    };

    explicit SafetyManager(LogSink sink = {});
    ~SafetyManager();

    SafetyManager(const SafetyManager&) = delete;
    SafetyManager& operator=(const SafetyManager&) = delete;

    Registration registerAction(int priority, std::string name, Action action);
    void safeShutdown(const std::string& reason) noexcept;
    void emergencyStop(const std::string& reason) noexcept;
    bool stopRequested() const noexcept;

private:
    struct Entry {
        int priority;
        std::uint64_t id;
        std::string name;
        Action action;
    };

    void unregister(std::uint64_t id) noexcept;
    void executeActions() noexcept;

    mutable std::mutex mutex_;
    std::vector<Entry> actions_;
    LogSink sink_;
    std::atomic<bool> stopRequested_{false};
    std::uint64_t nextId_{1};
};

class SafetyGuard {
public:
    explicit SafetyGuard(SafetyManager& manager) noexcept;
    ~SafetyGuard();
    SafetyGuard(const SafetyGuard&) = delete;
    SafetyGuard& operator=(const SafetyGuard&) = delete;

private:
    SafetyManager& manager_;
    int uncaughtExceptionsOnEntry_{0};
};

void setGlobalSafetyManager(SafetyManager* manager) noexcept;
void EmergencyStop(const std::string& reason) noexcept;

}  // namespace pendulum::safety
