#pragma once
#include <spdlog/spdlog.h>

namespace aimlog {

    struct spdlog_advancer {
        static void be_pretty() {
            spdlog::set_pattern("[%@] [%!] %+");
        }
    };//in-complete, suggest to not use it before complete
    //usage: spdlog_advancer::be_pretty();
    //       spdlog::info("Hello, {}!", "World");

    #ifdef _MSC_VER
        #define INFO(str, ...) spdlog::info("[{}] " str, __FUNCSIG__ __VA_OPT__(,) __VA_ARGS__)
        #define WARN(str, ...) spdlog::warn("[{}] " str, __FUNCSIG__ __VA_OPT__(,) __VA_ARGS__)
        #define ERROR(str, ...) spdlog::error("[{}] " str, __FUNCSIG__ __VA_OPT__(,) __VA_ARGS__)
        #define DEBUG(str, ...) spdlog::debug("[{}] " str, __FUNCSIG__ __VA_OPT__(,) __VA_ARGS__)
        #define TRACE(str, ...) spdlog::trace("[{}] " str, __FUNCSIG__ __VA_OPT__(,) __VA_ARGS__)
    #else
        #define INFO(str, ...) spdlog::info("[{}] " str, __PRETTY_FUNCTION__ __VA_OPT__(,) __VA_ARGS__)
        #define WARN(str, ...) spdlog::warn("[{}] " str, __PRETTY_FUNCTION__ __VA_OPT__(,) __VA_ARGS__)
        #define ERROR(str, ...) spdlog::error("[{}] " str, __PRETTY_FUNCTION__ __VA_OPT__(,) __VA_ARGS__)
        #define DEBUG(str, ...) spdlog::debug("[{}] " str, __PRETTY_FUNCTION__ __VA_OPT__(,) __VA_ARGS__)
        #define TRACE(str, ...) spdlog::trace("[{}] " str, __PRETTY_FUNCTION__ __VA_OPT__(,) __VA_ARGS__)
    #endif
}