#include "logger/logger.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <vector>
#include <chrono>
#include <iomanip>
#include <sstream>
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

namespace {

constexpr size_t kMaxLogFileSizeBytes = 1024 * 1024 * 10; // 10MB
constexpr size_t kMaxLogFiles = 3;

static bool EnsureDirectory(const std::string& dir_path)
{
  if (dir_path.empty()) return false;

  std::string path = dir_path;
  if (!path.empty() && path.back() == '/') path.pop_back();
  if (path.empty()) return false;

  for (size_t i = 1; i < path.size(); ++i)
  {
    if (path[i] != '/') continue;
    const std::string subdir = path.substr(0, i);
    if (subdir.empty()) continue;
    if (::mkdir(subdir.c_str(), 0755) != 0 && errno != EEXIST) return false;
  }
  if (::mkdir(path.c_str(), 0755) != 0 && errno != EEXIST) return false;
  return true;
}

static std::string SanitizeSegment(const std::string& segment)
{
  std::string out;
  out.reserve(segment.size());
  for (char c : segment)
  {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc) || c == '_' || c == '-' || c == '.') out.push_back(c);
    else out.push_back('_');
  }
  if (out.empty()) out = "misc";
  return out;
}

static std::string NormalizeRoute(const std::string& route)
{
  // Normalize "a//b/../c" -> "a/b/c" (reject ".."), sanitize characters.
  std::string cleaned;
  cleaned.reserve(route.size());
  for (char c : route)
  {
    cleaned.push_back((c == '\\') ? '/' : c);
  }

  std::vector<std::string> parts;
  size_t start = 0;
  while (start < cleaned.size())
  {
    while (start < cleaned.size() && cleaned[start] == '/') ++start;
    if (start >= cleaned.size()) break;
    const size_t end = cleaned.find('/', start);
    const size_t len = (end == std::string::npos) ? (cleaned.size() - start) : (end - start);
    std::string part = cleaned.substr(start, len);
    start = (end == std::string::npos) ? cleaned.size() : end;

    if (part.empty() || part == ".") continue;
    if (part == "..") continue; // prevent path traversal

    parts.emplace_back(SanitizeSegment(part));
  }

  if (parts.empty()) return "misc";

  std::string normalized;
  for (size_t i = 0; i < parts.size(); ++i)
  {
    if (i) normalized.push_back('/');
    normalized.append(parts[i]);
  }
  return normalized;
}

static std::string RouteLeafName(const std::string& normalized_route)
{
  const size_t pos = normalized_route.find_last_of('/');
  if (pos == std::string::npos) return normalized_route;
  const std::string leaf = normalized_route.substr(pos + 1);
  return leaf.empty() ? std::string("misc") : leaf;
}

}  // namespace

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

    log_root_dir_ = base_path + date_str + "/";
    if (!EnsureDirectory(log_root_dir_))
    {
      fprintf(stderr,
              "\033[1;31m[Logger] Error: Cannot create log dir: %s. Saving to /tmp/ instead.\033[0m\n",
              log_root_dir_.c_str());
      log_root_dir_ = std::string("/tmp/logger_logs/") + date_str + "/";
      EnsureDirectory(log_root_dir_);
    }

    std::string log_filename = log_root_dir_ + "daily.log";
    std::string sensor_filename = log_root_dir_ + "sensor.log";
    
    // 打印路径，方便确认
    printf("\n------------------------------------------------\n");
    printf("[Logger] Log File Path: %s\n", log_filename.c_str());
    printf("------------------------------------------------\n\n");

    // 5. 创建 Sinks (保持不变)
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::trace);

    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      log_filename, kMaxLogFileSizeBytes, kMaxLogFiles);

    auto sensor_sink  = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(sensor_filename, kMaxLogFileSizeBytes, kMaxLogFiles);

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

    log_pattern_ = "%^[%Y-%m-%d %H:%M:%S.%e] [thread %t] <%s>%!:#%#,%$ %v";
    logger_both_->set_pattern(log_pattern_);
    logger_console_->set_pattern(log_pattern_);
    logger_file_->set_pattern(log_pattern_);
    logger_sensor_->set_pattern(log_pattern_);

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

std::shared_ptr<spdlog::logger> Logger::GetRoutedLogger(const std::string& route)
{
  const std::string normalized = NormalizeRoute(route);

  {
    std::lock_guard<std::mutex> lock(routed_loggers_mutex_);
    auto it = routed_loggers_.find(normalized);
    if (it != routed_loggers_.end()) return it->second;
  }

  const std::string leaf = RouteLeafName(normalized);
  std::string filename;
  const bool is_top_level = (normalized.find('/') == std::string::npos);
  const bool is_reserved_root = (normalized == "daily" || normalized == "sensor");

  if (is_top_level && !is_reserved_root)
  {
    filename = log_root_dir_ + leaf + ".log";
  }
  else
  {
    const std::string route_dir = log_root_dir_ + normalized + "/";
    if (!EnsureDirectory(route_dir))
    {
      // Fall back to the root log dir to avoid crashing the caller.
      return logger_file_ ? logger_file_ : logger_both_;
    }
    filename = route_dir + leaf + ".log";
  }

  try
  {
    auto route_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      filename, kMaxLogFileSizeBytes, kMaxLogFiles);
    route_sink->set_level(spdlog::level::trace);

    std::vector<spdlog::sink_ptr> sinks_route = {route_sink};

    const std::string logger_name = "route:" + normalized;
    auto routed_logger = std::make_shared<spdlog::async_logger>(
      logger_name,
      sinks_route.begin(),
      sinks_route.end(),
      spdlog::thread_pool(),
      spdlog::async_overflow_policy::block);

    routed_logger->set_pattern(log_pattern_);
    routed_logger->set_level(spdlog::level::trace);
    routed_logger->flush_on(spdlog::level::info);

    try
    {
      spdlog::register_logger(routed_logger);
    }
    catch (const spdlog::spdlog_ex&)
    {
      // Ignore duplicate registration; we'll still cache and return our instance.
    }

    std::lock_guard<std::mutex> lock(routed_loggers_mutex_);
    auto it = routed_loggers_.find(normalized);
    if (it != routed_loggers_.end()) return it->second;
    routed_loggers_.emplace(normalized, routed_logger);
    return routed_logger;
  }
  catch (const spdlog::spdlog_ex& ex)
  {
    fprintf(stderr, "[Logger] Failed to create routed logger '%s': %s\n", normalized.c_str(), ex.what());
    return logger_file_ ? logger_file_ : logger_both_;
  }
}

std::shared_ptr<spdlog::logger> Logger::GetSensorLogger(const std::string& sensor_name)
{
  return GetRoutedLogger(std::string("sensors/") + sensor_name);
}

Logger::~Logger() {
  if (logger_both_) logger_both_->flush();
  if (logger_console_) logger_console_->flush();
  if (logger_file_) logger_file_->flush();
  if (logger_sensor_) logger_sensor_->flush();
  {
    std::lock_guard<std::mutex> lock(routed_loggers_mutex_);
    for (auto& kv : routed_loggers_)
    {
      if (kv.second) kv.second->flush();
    }
  }
  spdlog::shutdown();
}

}  // namespace logger
