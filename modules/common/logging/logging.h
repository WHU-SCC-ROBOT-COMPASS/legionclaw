/**
 * @file    logging.h
 * @author  jiangchengjie
 * @date    2019-04-24
 * @version 1.0.0
 * @par     Copyright(c)
 *          hy
 */
#pragma once

#include <iostream>
#include <sstream>

#if GLOG_ENABLE
#include "glog/logging.h"
#include "glog/raw_logging.h"
#include "glog/log_severity.h"
#include "modules/common/binary/binary.h"
#elif MDCLOG_ENABLE
#include <ara/log/logging.h>
using namespace ara::log;
#endif

using namespace std;

/**
 * @namespace legionclaw::common
 * @brief legionclaw::common
 */
namespace legionclaw {
namespace common {
typedef enum {
  _INFO = 0,
  _WARNING = 1,
  _ERROR = 2,
  _FATAL = 3,
  _OFF = 6,
} LogLevel;

typedef struct {
  std::string app_name;
  std::string description;
  std::string file_path;
  LogLevel log_level;
  bool logging_data_enable;
} LoggingConf;

#if MDCLOG_ENABLE
enum HafValueType {
  HAF_VALUE_ZERO = 0,
  HAF_VALUE_ONE = 1,
  HAF_VALUE_TWO = 2,
  HAF_VALUE_THREE = 3,
  HAF_VALUE_FOUR = 4,
  HAF_VALUE_FIVE = 5,
  HAF_VALUE_SIX = 6
};

static ara::log::LogMode ParseLogMode(const int32_t logMode) {
  ara::log::LogMode mode;
  const auto type = static_cast<HafValueType>(logMode);
  switch (type) {
  case HAF_VALUE_ZERO:
    mode = ara::log::LogMode::kRemote;
    break;
  case HAF_VALUE_ONE:
    mode = ara::log::LogMode::kConsole;
    break;
  case HAF_VALUE_TWO:
    mode = ara::log::LogMode::kFile;
    break;
  case HAF_VALUE_THREE:
    mode = ara::log::LogMode::kRemote | ara::log::LogMode::kConsole;
    break;
  case HAF_VALUE_FOUR:
    mode = ara::log::LogMode::kRemote | ara::log::LogMode::kFile;
    break;
  case HAF_VALUE_FIVE:
    mode = ara::log::LogMode::kConsole | ara::log::LogMode::kFile;
    break;
  case HAF_VALUE_SIX:
    mode = ara::log::LogMode::kRemote | ara::log::LogMode::kConsole |
           ara::log::LogMode::kFile;
    break;
  default:
    mode = ara::log::LogMode::kRemote;
    break;
  }
  return mode;
}

static ara::log::LogLevel ParseLogLevel(const int32_t logLevel) {
  ara::log::LogLevel level;
  const auto type = static_cast<HafValueType>(logLevel);
  switch (type) {
  case HAF_VALUE_ZERO:
    level = ara::log::LogLevel::kVerbose;
    break;
  case HAF_VALUE_ONE:
    level = ara::log::LogLevel::kDebug;
    break;
  case HAF_VALUE_TWO:
    level = ara::log::LogLevel::kInfo;
    break;
  case HAF_VALUE_THREE:
    level = ara::log::LogLevel::kWarn;
    break;
  case HAF_VALUE_FOUR:
    level = ara::log::LogLevel::kError;
    break;
  case HAF_VALUE_FIVE:
    level = ara::log::LogLevel::kFatal;
    break;
  case HAF_VALUE_SIX:
    level = ara::log::LogLevel::kOff;
    break;
  default:
    level = ara::log::LogLevel::kError;
    break;
  }
  return level;
}
#endif

/**
 * @class Logging
 * @brief  日志类.
 */
class Logging {
public:
  Logging() = default;
  ~Logging() = default;

  typedef enum {
    DISPLAY_ONLY = 1,
    DISPLAY_AND_STORE = 2,
  } LogMode;

  /**
   * @brief     google log 初始化确定是否需要日志功能,以及设置log存储路径.
   * @param[in] log_enable true=日志记录 false=不记录.
   * @return    void.
   */
  static void Init(LoggingConf logging_conf) {
#if GLOG_ENABLE
    //级别高于ERROR的才输出到屏幕上
    // google::SetStderrLogging(google::ERROR);
    //设置输出到屏幕的日志显示相应颜色
    legionclaw::common::binary::SetName(logging_conf.app_name);
    FLAGS_colorlogtostderr = true;
    //设置 google::INFO 级别的日志存储路径和文件名前缀
    std::string file_path = logging_conf.file_path + "/INFO_";
    google::SetLogDestination(google::INFO, file_path.c_str());
    //设置 google::WARNING 级别的日志存储路径和文件名前缀
    file_path = logging_conf.file_path + "/WARNING_";
    google::SetLogDestination(google::WARNING, file_path.c_str());
    //设置 google::ERROR 级别的日志存储路径和文件名前缀
    file_path = logging_conf.file_path + "/ERROR_";
    google::SetLogDestination(google::ERROR, file_path.c_str());
    FLAGS_minloglevel = logging_conf.log_level;
    //缓冲日志输出，默认为30秒，此处改为立即输出
    FLAGS_logbufsecs = 0;
    //最大日志大小为 10MB
    FLAGS_max_log_size = 10;
    //自定义VLOG(m)时，m值小于此处设置值的语句才有输出
    FLAGS_v = 5;
    //当磁盘被写满时，停止日志输出
    FLAGS_stop_logging_if_full_disk = true;
    if (logging_conf.logging_data_enable)
      FLAGS_logtostderr = false;
    else
      FLAGS_logtostderr = true;
    //捕捉 core dumped
    google::InstallFailureSignalHandler();
#endif

#if MDCLOG_ENABLE
    //参数1  程序名
    //参数2  描述
    //参数3  log等级 0：verbose, 1：debug, 2：info, 3：warn, 4：error, 5：fatal,
    // 6：off 参数4 log模式 {0, 1, 2, 3, 4, 5, 6}, 0: 固定文件存储，异步日志 1:
    //打屏，同步日志，适合调试， 2: 指定文件存储，同步日志，适合调试，
    // 3：固定+打屏， 4：固定+指定， 5：打屏+指定， 6：固定+打屏+指定
    //参数5 日志存储目录
    Logging::GetInstance().InitLogging(
        logging_conf.app_name, logging_conf.> description,
        (int32_t)logging_conf.log_level, 6, logging_conf.file_path);
    //参数3为 log等级
    Logging::GetInstance().CreateLogger(logging_conf.app_name,
                                        logging_conf.> description,
                                        (int32_t)logging_conf.log_level);
#endif
  }

#if MDCLOG_ENABLE
  static Logging& GetInstance() {
    static Logging instance;
    return instance;
  }

  void InitLogging(const std::string appId, const std::string appDescription,
                   const int32_t logLevel, const int32_t logMode,
                   const std::string directoryPath) {
    const ara::log::LogLevel level = ParseLogLevel(logLevel);
    const ara::log::LogMode mode = ParseLogMode(logMode);
    ara::log::InitLogging(appId, appDescription, level, mode, directoryPath);
  }

  void CreateLogger(const std::string ctxId, const std::string ctxDescription,
                    const int32_t ctxLogLevel) {
    const ara::log::LogLevel level = ParseLogLevel(ctxLogLevel);
    ara::core::StringView ctxIdView(ctxId.c_str());
    ara::core::StringView ctxDescView(ctxDescription.c_str());
    ara::log::Logger& logger =
        ara::log::CreateLogger(ctxIdView, ctxDescView, level);
    m_logger = &logger;
  }

  ara::log::Logger* AdsfiGetLogger() const { return m_logger; }

public:
  ara::log::Logger* m_logger;
#endif
};
#if MDCLOG_ENABLE
inline ara::log::Logger* GetAdsfiLogger() {
  return Logging::GetInstance().AdsfiGetLogger();
}
#endif

#if GLOG_ENABLE
#define LEFT_BRACKET "["
#define RIGHT_BRACKET "]"
#ifndef MODULE_NAME
#define MODULE_NAME legionclaw::common::binary::GetName().c_str()
#endif
#define ADEBUG_MODULE(module)                                                  \
  VLOG(4) << LEFT_BRACKET << module << RIGHT_BRACKET << "[DEBUG] "
#define ADEBUG ADEBUG_MODULE(MODULE_NAME)
#define AINFO ALOG_MODULE(MODULE_NAME, INFO)
#define AWARN ALOG_MODULE(MODULE_NAME, WARN)
#define AERROR ALOG_MODULE(MODULE_NAME, ERROR)
#define AFATAL ALOG_MODULE(MODULE_NAME, FATAL)

#ifndef ALOG_MODULE_STREAM
#define ALOG_MODULE_STREAM(log_severity) ALOG_MODULE_STREAM_##log_severity
#endif

#ifndef ALOG_MODULE
#define ALOG_MODULE(module, log_severity)                                      \
  ALOG_MODULE_STREAM(log_severity)(module)
#endif
#define ALOG_MODULE_STREAM_INFO(module)                                        \
  google::LogMessage(__FILE__, __LINE__, google::INFO).stream()                \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_WARN(module)                                        \
  google::LogMessage(__FILE__, __LINE__, google::WARNING).stream()             \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_ERROR(module)                                       \
  google::LogMessage(__FILE__, __LINE__, google::ERROR).stream()               \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_FATAL(module)                                       \
  google::LogMessage(__FILE__, __LINE__, google::FATAL).stream()               \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define AINFO_IF(cond) ALOG_IF(INFO, cond, MODULE_NAME)
#define AWARN_IF(cond) ALOG_IF(WARN, cond, MODULE_NAME)
#define AERROR_IF(cond) ALOG_IF(ERROR, cond, MODULE_NAME)
#define AFATAL_IF(cond) ALOG_IF(FATAL, cond, MODULE_NAME)

#define ALOG_IF(severity, cond, module)                                        \
  !(cond) ? (void)0                                                            \
          : google::LogMessageVoidify() & ALOG_MODULE(module, severity)

#define ACHECK(cond) CHECK(cond) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#define AINFO_EVERY(freq)                                                      \
  LOG_EVERY_N(INFO, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AWARN_EVERY(freq)                                                      \
  LOG_EVERY_N(WARNING, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AERROR_EVERY(freq)                                                     \
  LOG_EVERY_N(ERROR, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#if !defined(RETURN_IF_NULL)
#define RETURN_IF_NULL(ptr)                                                    \
  if (ptr == nullptr) {                                                        \
    AWARN << #ptr << " is nullptr.";                                           \
    return;                                                                    \
  }
#endif

#if !defined(RETURN_VAL_IF_NULL)
#define RETURN_VAL_IF_NULL(ptr, val)                                           \
  if (ptr == nullptr) {                                                        \
    AWARN << #ptr << " is nullptr.";                                           \
    return val;                                                                \
  }
#endif

#if !defined(RETURN_IF)
#define RETURN_IF(condition)                                                   \
  if (condition) {                                                             \
    AWARN << #condition << " is met.";                                         \
    return;                                                                    \
  }
#endif

#if !defined(RETURN_VAL_IF)
#define RETURN_VAL_IF(condition, val)                                          \
  if (condition) {                                                             \
    AWARN << #condition << " is met.";                                         \
    return val;                                                                \
  }
#endif

#if !defined(_RETURN_VAL_IF_NULL2__)
#define _RETURN_VAL_IF_NULL2__
#define RETURN_VAL_IF_NULL2(ptr, val)                                          \
  if (ptr == nullptr) {                                                        \
    return (val);                                                              \
  }
#endif

#if !defined(_RETURN_VAL_IF2__)
#define _RETURN_VAL_IF2__
#define RETURN_VAL_IF2(condition, val)                                         \
  if (condition) {                                                             \
    return (val);                                                              \
  }
#endif

#if !defined(_RETURN_IF2__)
#define _RETURN_IF2__
#define RETURN_IF2(condition)                                                  \
  if (condition) {                                                             \
    return;                                                                    \
  }
#endif
#elif MDCLOG_ENABLE
#define HAF_LOG_HEAD __FUNCTION__ << "() | "
#define AFATAL (GetAdsfiLogger()->LogFatal() << HAF_LOG_HEAD)
#define AERROR (GetAdsfiLogger()->LogError() << HAF_LOG_HEAD)
#define AWARN (GetAdsfiLogger()->LogWarn() << HAF_LOG_HEAD)
#define AINFO (GetAdsfiLogger()->LogInfo() << HAF_LOG_HEAD)
#define ADEBUG (GetAdsfiLogger()->LogDebug() << HAF_LOG_HEAD)
#define AVERBOSE (GetAdsfiLogger()->LogVerbose() << HAF_LOG_HEAD)
#endif
} // namespace common
} // namespace legionclaw
