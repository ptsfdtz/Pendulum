#pragma once
#include <string_view>

namespace pendulum::tools {
enum class ConsoleCommand { Single, Double, Stop, Invalid };

constexpr ConsoleCommand parseConsoleCommand(std::string_view command) noexcept {
    if (command == "1") return ConsoleCommand::Single;
    if (command == "2") return ConsoleCommand::Double;
    if (command == "q" || command == "Q" || command == "\x1b") return ConsoleCommand::Stop;
    return ConsoleCommand::Invalid;
}
} // namespace pendulum::tools
