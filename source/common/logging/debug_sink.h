#pragma once

#include "../debugging/hub.h"
#include "../services.h"

namespace Tyr::Common
{
template <typename Mutex>
class DebugSink final : public spdlog::sinks::base_sink<Mutex>
{
protected:
    void sink_it_(const spdlog::details::log_msg &msg) override
    {
        debug().log(msg);
    }
    void flush_() override
    {}
};

using DebugSinkMt = DebugSink<std::mutex>;
using DebugSinkSt = DebugSink<spdlog::details::null_mutex>;

} // namespace Tyr::Common
