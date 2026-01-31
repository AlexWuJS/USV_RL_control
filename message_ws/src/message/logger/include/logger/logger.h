#ifndef LOGGER_SRC_LOGGER_
#define LOGGER_SRC_LOGGER_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
// **新增：为 spdlog::source_loc 提供支持**
#include "spdlog/spdlog.h" 

// **修改：重写所有宏以传递源码位置信息**
#define LOG_TRACE(fmt, ...) logger::Logger::GetInstance().get_logger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::trace, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  logger::Logger::GetInstance().get_logger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::info, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  logger::Logger::GetInstance().get_logger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::warn, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) logger::Logger::GetInstance().get_logger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::err, fmt, ##__VA_ARGS__)
#define LOG_CRITICAL(fmt, ...) logger::Logger::GetInstance().get_logger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::critical, fmt, ##__VA_ARGS__)

#define LOG_TRACE_CONSOLE(fmt, ...) logger::Logger::GetInstance().get_console_logger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::trace, fmt, ##__VA_ARGS__)
#define LOG_INFO_CONSOLE(fmt, ...)  logger::Logger::GetInstance().get_console_logger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::info, fmt, ##__VA_ARGS__)
#define LOG_WARN_CONSOLE(fmt, ...)  logger::Logger::GetInstance().get_console_logger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::warn, fmt, ##__VA_ARGS__)
#define LOG_ERROR_CONSOLE(fmt, ...) logger::Logger::GetInstance().get_console_logger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::err, fmt, ##__VA_ARGS__)

#define LOG_TRACE_FILE(fmt, ...) logger::Logger::GetInstance().get_file_logger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::trace, fmt, ##__VA_ARGS__)
#define LOG_INFO_FILE(fmt, ...)  logger::Logger::GetInstance().get_file_logger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::info, fmt, ##__VA_ARGS__)
#define LOG_WARN_FILE(fmt, ...)  logger::Logger::GetInstance().get_file_logger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::warn, fmt, ##__VA_ARGS__)
#define LOG_ERROR_FILE(fmt, ...) logger::Logger::GetInstance().get_file_logger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::err, fmt, ##__VA_ARGS__)

#define LOG_SENSOR_INFO(...)  logger::Logger::GetInstance().GetSensorLogger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::info, __VA_ARGS__)
#define LOG_SENSOR_DEBUG(...) logger::Logger::GetInstance().GetSensorLogger()->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::debug, __VA_ARGS__)

// Routed logs (multi-logger factory): write to a user-defined subfolder/file under logs/YYYY-MM-DD/
// Example: LOG_INFO_TO("sensors/imu", "ax={}, ay={}", ax, ay);
#define LOG_TRACE_TO(route, fmt, ...) logger::Logger::GetInstance().GetRoutedLogger(route)->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::trace, fmt, ##__VA_ARGS__)
#define LOG_DEBUG_TO(route, fmt, ...) logger::Logger::GetInstance().GetRoutedLogger(route)->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::debug, fmt, ##__VA_ARGS__)
#define LOG_INFO_TO(route, fmt, ...)  logger::Logger::GetInstance().GetRoutedLogger(route)->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::info, fmt, ##__VA_ARGS__)
#define LOG_WARN_TO(route, fmt, ...)  logger::Logger::GetInstance().GetRoutedLogger(route)->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::warn, fmt, ##__VA_ARGS__)
#define LOG_ERROR_TO(route, fmt, ...) logger::Logger::GetInstance().GetRoutedLogger(route)->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::err, fmt, ##__VA_ARGS__)
#define LOG_CRITICAL_TO(route, fmt, ...) logger::Logger::GetInstance().GetRoutedLogger(route)->log(spdlog::source_loc{__FILE__, __LINE__, __FUNCTION__}, spdlog::level::critical, fmt, ##__VA_ARGS__)

// Predefined sensor/category macros (file-only, one folder per category).
#define LOG_IMU_DEBUG(fmt, ...)  LOG_DEBUG_TO("sensors/imu", fmt, ##__VA_ARGS__)
#define LOG_IMU_INFO(fmt, ...)   LOG_INFO_TO("sensors/imu", fmt, ##__VA_ARGS__)
#define LOG_IMU_WARN(fmt, ...)   LOG_WARN_TO("sensors/imu", fmt, ##__VA_ARGS__)
#define LOG_IMU_ERROR(fmt, ...)  LOG_ERROR_TO("sensors/imu", fmt, ##__VA_ARGS__)

#define LOG_RTK_DEBUG(fmt, ...)  LOG_DEBUG_TO("sensors/rtk", fmt, ##__VA_ARGS__)
#define LOG_RTK_INFO(fmt, ...)   LOG_INFO_TO("sensors/rtk", fmt, ##__VA_ARGS__)
#define LOG_RTK_WARN(fmt, ...)   LOG_WARN_TO("sensors/rtk", fmt, ##__VA_ARGS__)
#define LOG_RTK_ERROR(fmt, ...)  LOG_ERROR_TO("sensors/rtk", fmt, ##__VA_ARGS__)

#define LOG_LIDAR_DEBUG(fmt, ...)  LOG_DEBUG_TO("sensors/lidar", fmt, ##__VA_ARGS__)
#define LOG_LIDAR_INFO(fmt, ...)   LOG_INFO_TO("sensors/lidar", fmt, ##__VA_ARGS__)
#define LOG_LIDAR_WARN(fmt, ...)   LOG_WARN_TO("sensors/lidar", fmt, ##__VA_ARGS__)
#define LOG_LIDAR_ERROR(fmt, ...)  LOG_ERROR_TO("sensors/lidar", fmt, ##__VA_ARGS__)

#define LOG_CAMERA_DEBUG(fmt, ...)  LOG_DEBUG_TO("sensors/camera", fmt, ##__VA_ARGS__)
#define LOG_CAMERA_INFO(fmt, ...)   LOG_INFO_TO("sensors/camera", fmt, ##__VA_ARGS__)
#define LOG_CAMERA_WARN(fmt, ...)   LOG_WARN_TO("sensors/camera", fmt, ##__VA_ARGS__)
#define LOG_CAMERA_ERROR(fmt, ...)  LOG_ERROR_TO("sensors/camera", fmt, ##__VA_ARGS__)

// TCP communication logs (writes to logs/YYYY-MM-DD/tcp.log)
#define LOG_TCP_DEBUG(fmt, ...)  LOG_DEBUG_TO("tcp", fmt, ##__VA_ARGS__)
#define LOG_TCP_INFO(fmt, ...)   LOG_INFO_TO("tcp", fmt, ##__VA_ARGS__)
#define LOG_TCP_WARN(fmt, ...)   LOG_WARN_TO("tcp", fmt, ##__VA_ARGS__)
#define LOG_TCP_ERROR(fmt, ...)  LOG_ERROR_TO("tcp", fmt, ##__VA_ARGS__)

namespace logger {

class Logger {
 public:
  // 禁止拷贝构造和赋值操作
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  // 获取 Logger 的单例实例。
  static Logger& GetInstance();

  // **新增：提供一个获取底层 spdlog::logger 的公共方法**
  std::shared_ptr<spdlog::logger> get_logger() { return logger_both_; }
  std::shared_ptr<spdlog::logger> get_console_logger() { return logger_console_; }
  std::shared_ptr<spdlog::logger> get_file_logger() { return logger_file_; }
  std::shared_ptr<spdlog::logger> GetSensorLogger() { return logger_sensor_; }

  // Returns a cached async logger.
  // - Top-level route "tcp" -> logs/YYYY-MM-DD/tcp.log
  // - Nested route "sensors/imu" -> logs/YYYY-MM-DD/sensors/imu/imu.log
  std::shared_ptr<spdlog::logger> GetRoutedLogger(const std::string& route);

  // Convenience: one folder per sensor name under logs/YYYY-MM-DD/sensors/<name>/<name>.log
  std::shared_ptr<spdlog::logger> GetSensorLogger(const std::string& sensor_name);
  // **移除：不再需要 Trace, Info, Warn 等模板函数**

 private:
  // 构造函数现在是初始化的入口
  Logger();
  ~Logger();

  // 指向 spdlog 日志记录器的共享指针。
  std::shared_ptr<spdlog::logger> logger_both_;    // 终端 + 文件
  std::shared_ptr<spdlog::logger> logger_console_; // 仅终端
  std::shared_ptr<spdlog::logger> logger_file_;    // 仅文件
  std::shared_ptr<spdlog::logger> logger_sensor_;  // 传感器

  std::string log_root_dir_;
  std::string log_pattern_;
  std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> routed_loggers_;
  std::mutex routed_loggers_mutex_;
};

}  // namespace logger

#endif  // LOGGER_SRC_LOGGER_
