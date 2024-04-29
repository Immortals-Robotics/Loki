#pragma once

#include <source_location>

#include <spdlog/spdlog.h>

namespace Tyr::Common
{
class Logger
{
public:
    template <typename... Args>
    inline void log(std::source_location source, spdlog::level::level_enum level,
                    spdlog::format_string_t<Args...> format, Args &&...args)
    {
        spdlog::source_loc source_loc{source.file_name(), (int) source.line(), source.function_name()};
        m_logger->log(source_loc, level, format, std::forward<Args>(args)...);
    }

protected:
    Logger();
    ~Logger() = default;

    friend struct Services;

private:
    std::unique_ptr<spdlog::logger> m_logger;
};
} // namespace Tyr::Common
