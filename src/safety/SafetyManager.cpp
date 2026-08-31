#include "pendulum/safety/SafetyManager.h"

#include <algorithm>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <utility>

namespace pendulum::safety {
namespace {

std::atomic<SafetyManager*> globalManager{nullptr};

}  // namespace

SafetyManager::Registration::Registration(SafetyManager* owner, std::uint64_t id)
    : owner_(owner), id_(id) {}

SafetyManager::Registration::~Registration() {
    reset();
}

SafetyManager::Registration::Registration(Registration&& other) noexcept
    : owner_(std::exchange(other.owner_, nullptr)), id_(std::exchange(other.id_, 0)) {}

SafetyManager::Registration& SafetyManager::Registration::operator=(Registration&& other) noexcept {
    if (this != &other) {
        reset();
        owner_ = std::exchange(other.owner_, nullptr);
        id_ = std::exchange(other.id_, 0);
    }
    return *this;
}

void SafetyManager::Registration::reset() noexcept {
    if (owner_ != nullptr) {
        owner_->unregister(id_);
        owner_ = nullptr;
        id_ = 0;
    }
}

SafetyManager::SafetyManager(LogSink sink) : sink_(std::move(sink)) {}

SafetyManager::~SafetyManager() {
    SafetyManager* expected = this;
    globalManager.compare_exchange_strong(expected, nullptr);
}

SafetyManager::Registration SafetyManager::registerAction(int priority, std::string name,
                                                          Action action) {
    if (!action) {
        throw std::invalid_argument("Safety action must be callable");
    }
    std::scoped_lock lock(mutex_);
    const auto id = nextId_++;
    actions_.push_back(Entry{priority, id, std::move(name), std::move(action)});
    std::stable_sort(actions_.begin(), actions_.end(),
                     [](const Entry& left, const Entry& right) {
                         return left.priority < right.priority;
                     });
    return Registration(this, id);
}

void SafetyManager::emergencyStop(const std::string& reason) noexcept {
    stopRequested_.store(true, std::memory_order_release);
    std::fprintf(stderr, "EMERGENCY STOP: %s\n", reason.c_str());
    if (sink_) {
        try {
            sink_(reason);
        } catch (...) {
        }
    }

    executeActions();
}

void SafetyManager::safeShutdown(const std::string& reason) noexcept {
    std::fprintf(stderr, "SAFE SHUTDOWN: %s\n", reason.c_str());
    executeActions();
}

void SafetyManager::executeActions() noexcept {
    std::scoped_lock lock(mutex_);
    for (const auto& entry : actions_) {
        try {
            entry.action();
        } catch (const std::exception& error) {
            std::fprintf(stderr, "CRITICAL: Safety action '%s' failed: %s\n",
                         entry.name.c_str(), error.what());
        } catch (...) {
            std::fprintf(stderr, "CRITICAL: Safety action '%s' failed\n", entry.name.c_str());
        }
    }
}

bool SafetyManager::stopRequested() const noexcept {
    return stopRequested_.load(std::memory_order_acquire);
}

void SafetyManager::unregister(std::uint64_t id) noexcept {
    std::scoped_lock lock(mutex_);
    std::erase_if(actions_, [id](const Entry& entry) { return entry.id == id; });
}

SafetyGuard::SafetyGuard(SafetyManager& manager) noexcept
    : manager_(manager), uncaughtExceptionsOnEntry_(std::uncaught_exceptions()) {}

SafetyGuard::~SafetyGuard() {
    if (std::uncaught_exceptions() > uncaughtExceptionsOnEntry_) {
        manager_.emergencyStop("Exception unwinding through RAII safety guard");
    } else {
        manager_.safeShutdown("RAII safety guard scope exit");
    }
}

void setGlobalSafetyManager(SafetyManager* manager) noexcept {
    globalManager.store(manager, std::memory_order_release);
}

void EmergencyStop(const std::string& reason) noexcept {
    if (auto* manager = globalManager.load(std::memory_order_acquire); manager != nullptr) {
        manager->emergencyStop(reason);
    } else {
        std::fprintf(stderr, "EMERGENCY STOP requested without active SafetyManager: %s\n",
                     reason.c_str());
    }
}

}  // namespace pendulum::safety
