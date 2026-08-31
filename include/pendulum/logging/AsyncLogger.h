#pragma once

#include <condition_variable>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace pendulum::logging {

enum class Level { Info, Warning, Error, Critical };

class AsyncLogger {
public:
    AsyncLogger(const std::filesystem::path& directory, std::size_t capacity);
    ~AsyncLogger();

    AsyncLogger(const AsyncLogger&) = delete;
    AsyncLogger& operator=(const AsyncLogger&) = delete;

    void log(Level level, std::string component, std::string message) noexcept;
    const std::filesystem::path& path() const noexcept;
    std::size_t droppedMessages() const noexcept;

private:
    struct Entry {
        std::string timestamp;
        Level level;
        std::string component;
        std::string message;
    };

    void run() noexcept;
    void write(const Entry& entry) noexcept;

    std::filesystem::path path_;
    std::ofstream output_;
    std::size_t capacity_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::queue<Entry> queue_;
    std::thread worker_;
    bool stopping_{false};
    std::size_t dropped_{0};
};

}  // namespace pendulum::logging

