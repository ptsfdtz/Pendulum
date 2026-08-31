#include "pendulum/logging/AsyncLogger.h"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace pendulum::logging
{
    namespace
    {

        std::string timestamp(bool fileSafe = false)
        {
            const auto now = std::chrono::system_clock::now();
            const auto value = std::chrono::system_clock::to_time_t(now);
            std::tm local{};
            localtime_s(&local, &value);
            std::ostringstream stream;
            stream << std::put_time(&local, fileSafe ? "%Y%m%d_%H%M%S" : "%Y-%m-%dT%H:%M:%S");
            const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          now.time_since_epoch()) %
                                      1000;
            if (!fileSafe)
            {
                stream << '.' << std::setw(3) << std::setfill('0') << milliseconds.count();
            }
            else
            {
                stream << '_' << std::setw(3) << std::setfill('0') << milliseconds.count();
            }
            return stream.str();
        }

        const char *levelName(Level level)
        {
            switch (level)
            {
            case Level::Info:
                return "INFO";
            case Level::Warning:
                return "WARNING";
            case Level::Error:
                return "ERROR";
            case Level::Critical:
                return "CRITICAL";
            }
            return "UNKNOWN";
        }

        std::string csvEscape(const std::string &value)
        {
            std::string result = "\"";
            for (const char character : value)
            {
                if (character == '"')
                {
                    result += "\"\"";
                }
                else if (character != '\r' && character != '\n')
                {
                    result += character;
                }
                else
                {
                    result += ' ';
                }
            }
            result += '"';
            return result;
        }

    } // namespace

    AsyncLogger::AsyncLogger(const std::filesystem::path &directory, std::size_t capacity)
        : capacity_(capacity)
    {
        std::filesystem::create_directories(directory);
        const auto stem = "phase1_" + timestamp(true);
        path_ = directory / (stem + ".csv");
        for (std::size_t suffix = 1; std::filesystem::exists(path_); ++suffix)
        {
            path_ = directory / (stem + "_" + std::to_string(suffix) + ".csv");
        }
        output_.open(path_, std::ios::out | std::ios::trunc);
        if (!output_)
        {
            throw std::runtime_error("Cannot create log file: " + path_.string());
        }
        output_ << "timestamp,level,component,message\n";
        worker_ = std::thread(&AsyncLogger::run, this);
    }

    AsyncLogger::~AsyncLogger()
    {
        {
            std::scoped_lock lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_all();
        if (worker_.joinable())
        {
            worker_.join();
        }
        output_.flush();
    }

    void AsyncLogger::log(Level level, std::string component, std::string message) noexcept
    {
        try
        {
            std::scoped_lock lock(mutex_);
            if (stopping_)
            {
                return;
            }
            if (queue_.size() >= capacity_)
            {
                ++dropped_;
                return;
            }
            queue_.push(Entry{timestamp(), level, std::move(component), std::move(message)});
            condition_.notify_one();
        }
        catch (...)
        {
            ++dropped_;
        }
    }

    const std::filesystem::path &AsyncLogger::path() const noexcept
    {
        return path_;
    }

    std::size_t AsyncLogger::droppedMessages() const noexcept
    {
        std::scoped_lock lock(mutex_);
        return dropped_;
    }

    void AsyncLogger::run() noexcept
    {
        for (;;)
        {
            Entry entry;
            {
                std::unique_lock lock(mutex_);
                condition_.wait(lock, [this]
                                { return stopping_ || !queue_.empty(); });
                if (queue_.empty() && stopping_)
                {
                    break;
                }
                entry = std::move(queue_.front());
                queue_.pop();
            }
            write(entry);
        }
    }

    void AsyncLogger::write(const Entry &entry) noexcept
    {
        try
        {
            output_ << entry.timestamp << ',' << levelName(entry.level) << ','
                    << csvEscape(entry.component) << ',' << csvEscape(entry.message) << '\n';
        }
        catch (...)
        {
            std::scoped_lock lock(mutex_);
            ++dropped_;
        }
    }

} // namespace pendulum::logging
