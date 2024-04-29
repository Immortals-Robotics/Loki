#pragma once

#include <source_location>

namespace Tyr::Common
{

#define LOG_MACRO(fn, lvl)                                                                                             \
    template <typename... Args>                                                                                        \
    struct fn                                                                                                          \
    {                                                                                                                  \
        fn(spdlog::format_string_t<Args...> format, Args &&...args,                                                    \
           std::source_location             source = std::source_location::current())                                  \
        {                                                                                                              \
            logger().log(source, spdlog::level::lvl, format, std::forward<Args>(args)...);                             \
        }                                                                                                              \
    };                                                                                                                 \
    template <typename... Args>                                                                                        \
    fn(spdlog::format_string_t<Args...> format, Args &&...args) -> fn<Args...>;

LOG_MACRO(logTrace, trace);
LOG_MACRO(logDebug, debug);
LOG_MACRO(logInfo, info);
LOG_MACRO(logWarning, warn);
LOG_MACRO(logError, err);
LOG_MACRO(logCritical, critical);

} // namespace Tyr::Common