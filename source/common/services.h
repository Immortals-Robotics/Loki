#pragma once

#include "debug/debug.h"
#include "logging/logging.h"
#include "setting.h"
#include "timer/timer.h"

namespace Tyr::Common
{
struct Services
{
    static void initialize()
    {
        s_logger = new Logger();

        const ConfigReader config("config.toml");
        s_setting = new Setting();
        s_setting->load(config.getRoot());

        s_debug = new Debug(s_setting->debug_address, s_setting->enable_debug);

        s_global_timer = new Timer();
        s_global_timer->start();
    }

    static void shutdown()
    {
        delete s_setting;
        delete s_debug;
        delete s_global_timer;
        delete s_logger;
    }

    static const Setting &setting()
    {
        return *s_setting;
    }

    static Debug &debug()
    {
        return *s_debug;
    }

    static Logger &logger()
    {
        return *s_logger;
    }

    static Timer &global_timer()
    {
        return *s_global_timer;
    }

private:
    static inline Setting *s_setting;
    static inline Debug   *s_debug;
    static inline Logger  *s_logger;
    static inline Timer   *s_global_timer;
};

static const Setting &setting()
{
    return Services::setting();
}

static Debug &debug()
{
    return Services::debug();
}

static Logger &logger()
{
    return Services::logger();
}

static Timer &global_timer()
{
    return Services::global_timer();
}
} // namespace Tyr::Common

#include "logging/macros.h"
