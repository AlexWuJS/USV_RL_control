#include "logger/logger.h"

#include <vector>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstdlib>    // system()
#include <sys/stat.h> // mkdir

// !!! 必须包含 ROS 包管理头文件 !!!
#include <ros/package.h> 

// spdlog 头文件
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"
#include "spdlog/async.h"

namespace logger {

Logger& Logger::GetInstance() {
  static Logger instance;
  return instance;
}

Logger::Logger() {
  try {
    // 1. 初始化线程池
    spdlog::init_thread_pool(8192, 1);
    
    // 2. 设置自动刷新 (防止崩溃丢失数据)
    spdlog::flush_every(std::chrono::seconds(3));

    // 3. 获取日期字符串
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d");
    std::string date_str = ss.str();

    std::string pkg_path = ros::package::getPath("logger");
    
    std::string base_path;
    if (!pkg_path.empty()) {
        // 成功获取：路径指向 .../src/logger/logs/
        base_path = pkg_path + "/logs/";
    } else {
        // 获取失败（通常是因为包名写错了），回退到 /tmp/
        fprintf(stderr, "\033[1;31m[Logger] Error: Cannot find package 'logger'. Saving to /tmp/ instead.\033[0m\n");
        base_path = "/tmp/logger_logs/";
    }

    std::string log_dir = base_path + date_str + "/";

    // 4. 自动创建文件夹 (mkdir -p)
    std::string cmd = "mkdir -p " + log_dir; 
    system(cmd.c_str());

    std::string log_filename = log_dir + "daily.log";
    std::string sensor_filename = log_dir + "sensor.log";
    
    // 打印路径，方便确认
    printf("\n------------------------------------------------\n");
    printf("[Logger] Log File Path: %s\n", log_filename.c_str());
    printf("------------------------------------------------\n\n");

    // 5. 创建 Sinks (保持不变)
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::trace);

    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      log_filename, 1024 * 1024 * 10, 3);

    auto sensor_sink  = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(sensor_filename, 1024 * 1024 * 10, 3);

    file_sink->set_level(spdlog::level::trace);

    std::vector<spdlog::sink_ptr> sinks_both = {console_sink, file_sink};
    std::vector<spdlog::sink_ptr> sinks_console = {console_sink};
    std::vector<spdlog::sink_ptr> sinks_file = {file_sink};
    std::vector<spdlog::sink_ptr> sinks_sensor = {sensor_sink};

    logger_both_ = std::make_shared<spdlog::logger>("both", sinks_both.begin(), sinks_both.end());
    logger_console_ = std::make_shared<spdlog::logger>("console", sinks_console.begin(), sinks_console.end());
    logger_file_ = std::make_shared<spdlog::logger>("file", sinks_file.begin(), sinks_file.end());

    logger_sensor_ = std::make_shared<spdlog::async_logger>(
      "sensor", 
      sinks_sensor.begin(), 
      sinks_sensor.end(),
      spdlog::thread_pool(),
      spdlog::async_overflow_policy::block
    );  

    spdlog::register_logger(logger_both_);
    spdlog::set_default_logger(logger_both_);

    std::string pattern = "%^[%Y-%m-%d %H:%M:%S.%e] [thread %t] <%s>%!:#%#,%$ %v";
    logger_both_->set_pattern(pattern);
    logger_console_->set_pattern(pattern);
    logger_file_->set_pattern(pattern);
    logger_sensor_->set_pattern(pattern);

    logger_both_->set_level(spdlog::level::trace);
    logger_console_->set_level(spdlog::level::trace);
    logger_file_->set_level(spdlog::level::trace);
    logger_sensor_->set_level(spdlog::level::trace);

    logger_both_->flush_on(spdlog::level::info);
    logger_console_->flush_on(spdlog::level::info);
    logger_file_->flush_on(spdlog::level::info);
    logger_sensor_->flush_on(spdlog::level::info);
    
    spdlog::flush_on(spdlog::level::info);

  } catch (const spdlog::spdlog_ex& ex) {
    fprintf(stderr, "Log initialization failed: %s\n", ex.what());
  }
}

Logger::~Logger() {
  if (logger_both_) logger_both_->flush();
  if (logger_console_) logger_console_->flush();
  if (logger_file_) logger_file_->flush();
  if (logger_sensor_) logger_sensor_->flush();
  spdlog::shutdown();
}

}  // namespace logger