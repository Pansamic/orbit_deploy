#ifndef GLOBAL_LOG_H
#define GLOBAL_LOG_H
#include <string>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

std::string GetTimeStampStr();
void InitLogger();


#endif // GLOBAL_LOG_H