#ifndef AUTO_AIM_LOGGER_HPP_
#define AUTO_AIM_LOGGER_HPP_

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <memory>
#include <string>

namespace autoaim {

inline std::shared_ptr<spdlog::logger> getLogger(const std::string &name = "autoaim") {
    static auto logger = [] {
        auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        auto file = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/autoaim.log", true);
        std::vector<spdlog::sink_ptr> sinks{console, file};
        auto l = std::make_shared<spdlog::logger>("autoaim", sinks.begin(), sinks.end());
        l->set_level(spdlog::level::info);
        l->flush_on(spdlog::level::info);
        return l;
    }();
    return logger;
}

inline void setLogLevel(const std::string &level) {
    auto l = getLogger();
    if (level == "trace")    l->set_level(spdlog::level::trace);
    else if (level == "debug") l->set_level(spdlog::level::debug);
    else if (level == "warn")  l->set_level(spdlog::level::warn);
    else if (level == "error") l->set_level(spdlog::level::err);
    else                       l->set_level(spdlog::level::info);
}

}  // namespace autoaim

#endif  // AUTO_AIM_LOGGER_HPP_
