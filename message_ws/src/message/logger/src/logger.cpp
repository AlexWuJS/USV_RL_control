#include "logger/logger.h" // 对应我们修改后的头文件

#include <vector>
#include <chrono>   // 新增：用于获取时间
#include <iomanip>  // 新增：用于格式化时间字符串
#include <sstream>  // 新增：用于拼接字符串

// 包含 spdlog 的具体实现头文件
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"
#include "spdlog/async.h"


namespace logger {

// GetInstance 的实现
Logger& Logger::GetInstance() {
  // C++11 保证了静态局部变量的初始化是线程安全的。
  static Logger instance;
  return instance;
}

// Init 函数的实现
Logger::Logger() {
  try {
    // 1. 初始化线程池 (必须！)
    spdlog::init_thread_pool(8192, 1);
    // 1. 获取当前日期字符串 (例如: "2023-10-27")
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    // 使用 std::put_time 格式化时间
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d");
    std::string date_str = ss.str();

    std::string log_filename = "logs/" + date_str + "/daily.log";
    std::string sensor_filename = "logs/" + date_str + "/sensor.log";
    // 1. 创建 sinks：一个用于控制台，一个用于文件
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::trace);

    // 10MB
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      log_filename, 1024 * 1024 * 10, 3);

    auto sensor_sink  = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(sensor_filename, 1024 * 1024 * 10, 3);
    // 每天一个文件，保留30天
    // auto file_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(
    //   log_filename, 2, 30);
    file_sink->set_level(spdlog::level::trace);

    // 2. 准备 sink 列表
    std::vector<spdlog::sink_ptr> sinks_both;
    sinks_both.push_back(console_sink);
    sinks_both.push_back(file_sink);

    std::vector<spdlog::sink_ptr> sinks_console;
    sinks_console.push_back(console_sink);

    std::vector<spdlog::sink_ptr> sinks_file;
    sinks_file.push_back(file_sink);

    std::vector<spdlog::sink_ptr> sinks_sensor;
    sinks_sensor.push_back(sensor_sink);
    // 3. 初始化三个 Logger
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
    // 4. 注册默认 logger (防止 spdlog 内部报错)
    spdlog::register_logger(logger_both_);
    spdlog::set_default_logger(logger_both_);

    // 5. 统一设置格式 (对所有 logger 生效)
    // 注意：spdlog::set_pattern 是设置全局默认值，但为了保险，最好对每个 logger 单独设置
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
    // 7. 设置 flush 策略
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
