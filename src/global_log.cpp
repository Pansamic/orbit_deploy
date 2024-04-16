#include <chrono>
#include <sys/time.h>
#include "global_log.h"
std::string GetTimeStampStr()
{
    char buffer[128] = {0};
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm timeinfo;
    localtime_r(&tv.tv_sec, &timeinfo);
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    std::string strTime(buffer);
    return strTime;
}

void InitLogger()
{
    static int log_init = 0;
    if(log_init)
    {
        spdlog::warn("Logger already initialized");
        return;
    }

    std::string log_path(__FILE__);
    log_path = log_path.substr(0, log_path.find_last_of('/'));
    log_path = log_path.substr(0, log_path.find_last_of('/'));
    log_path = log_path.substr(0, log_path.find_last_of('/'));
    log_path += "/logs/";
    log_path += GetTimeStampStr();
    log_path += ".txt";

    std::vector<spdlog::sink_ptr> sinks;

    auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_st>();
    console_sink->set_level(spdlog::level::warn);
    sinks.push_back(console_sink);

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_path);
    file_sink->set_level(spdlog::level::debug);
    sinks.push_back(file_sink);
    auto logger = std::make_shared<spdlog::logger>("global_log", sinks.begin(), sinks.end());
    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);
    spdlog::set_level(spdlog::level::debug);
    log_init = 1;
}
