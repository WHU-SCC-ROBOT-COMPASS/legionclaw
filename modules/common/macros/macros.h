

#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>

#include "modules/common/base/macros.h"

DEFINE_TYPE_TRAIT(HasShutdown, Shutdown)

template <typename T>
typename std::enable_if<HasShutdown<T>::value>::type CallShutdown(T* instance) {
  instance->Shutdown();
}

template <typename T>
typename std::enable_if<!HasShutdown<T>::value>::type CallShutdown(
    T* instance) {
  (void)instance;
}

// There must be many copy-paste versions of these macros which are same
// things, undefine them to avoid conflict.
#undef UNUSED
#undef DISALLOW_COPY_AND_ASSIGN

#define UNUSED(param) (void)param

#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname&) = delete;     \
  classname& operator=(const classname&) = delete;

#define LOGGING_INIT(conf_ptr)                                              \
  LoggingConf logging_conf;                                                 \
  logging_conf.app_name = conf_ptr->app_name();                             \
  logging_conf.description = conf_ptr->description();                       \
  logging_conf.file_path = conf_ptr->log_file_path();                       \
  logging_conf.log_level = (legionclaw::common::LogLevel)conf_ptr->log_level(); \
  logging_conf.logging_data_enable = conf_ptr->logging_data_enable();       \
  Logging::Init(logging_conf);

//感知模块使用的双参数版本
#define LOGGING_INIT2(conf_ptr, json_ptr)                                     \
  LoggingConf logging_conf;                                                 \
  logging_conf.app_name = json_ptr["app_name"];                             \
  logging_conf.description = json_ptr["description"];                       \
  logging_conf.file_path = json_ptr["log_file_path"];                       \
  logging_conf.log_level = (legionclaw::common::LogLevel)json_ptr["log_level"]; \
  logging_conf.logging_data_enable = json_ptr["logging_data_enable"];       \
  Logging::Init(logging_conf);

#define MESSAGE_INIT(conf_ptr)                                         \
  auto message_info = conf_ptr->messages().message_info();             \
  auto messages_map_ptr =                                              \
      conf_ptr->mutable_messages()->mutable_messages_map();            \
  std::for_each(                                                       \
      message_info.begin(), message_info.end(),                        \
      [=](MessageInfo& message_info) {                                 \
        messages_map_ptr->insert({message_info.name(), message_info}); \
      });                                                              \
  MessagesInit();

//感知模块使用的双参数版本
#define MESSAGE_INIT2(conf_ptr, json_ptr)                               \
  MessagesInit();

#define MODULE(k) fault_json["modules"][k]
#define FAULTTYPE(k, i) MODULE(k)["faultcode"][i]
#define FAULT(k, i, j) FAULTTYPE(k, i)["list"][j]
#ifdef SAFEGUARD_MODE
#define THIS_FAULTCODESET legionclaw::interface::SafeFaultCodeSet
#define DO_SAFEMODE_ACTION(faultset)                                         \
  for (int k = 0; k < module_list_size; k++) {                               \
    std::string target = MODULE(k)["target"];                                  \
    std::string timeout_fault = MODULE(k)["timeout_fault"];                  \
    std::string interrupt_fault = MODULE(k)["interrupt_fault"];              \
    int target_id = MODULE(k)["target_id"];                                  \
    std::string topicname(MODULE(k)["topic_name"]);                          \
    int topicid = MODULE(k)["topic_id"];                                     \
    if (topicname.length() > 0 && timeout_fault.length()) {                  \
      if (faultset->get_watch().count(timeout_fault) > 0) {                  \
        faultset->add_topicnametotimeout_map(                                \
            topicname, faultset->get_watch()[timeout_fault]);                \
      }                                                                      \
      if (faultset->get_watch().count(interrupt_fault) > 0) {                \
        faultset->add_topicnametointerrupt_map(                              \
            topicname, faultset->get_watch()[interrupt_fault]);              \
      }                                                                      \
      faultset->add_channel_watch_list(topicname);                           \
    }                                                                        \
    if (topicid > 0 && timeout_fault.length()) {                             \
      faultset->add_topicidtotimeout_map(                                    \
          topicid, faultset->get_watch()[timeout_fault]);                    \
      faultset->add_topicidtointerrupt_map(                                  \
          topicid, faultset->get_watch()[interrupt_fault]);                  \
    }                                                                        \
    uint32_t fault_code_list_size = MODULE(k)["faultcode"].size();           \
    for (int i = 0; i < fault_code_list_size; i++) {                         \
      uint32_t type_list_size = FAULTTYPE(k, i)["list"].size();              \
      for (int j = 0; j < type_list_size; j++) {                             \
        int enable = FAULT(k, i, j)["enable"];                            \
        if (enable == -1) {                                                  \
          std::cout << "FAULT(k, i, j)[code]; == " << FAULT(k, i, j)["code"] \
                    << endl;                                                 \
          continue;                                                          \
        }                                                                    \
        legionclaw::interface::FaultProperty faulttype;                          \
        uint32_t code = FAULT(k, i, j)["code"];                              \
        faulttype.set_code(code);                                            \
        faulttype.set_enable(enable);                                        \
        faulttype.set_condition_0(FAULT(k, i, j)["condition_0"]);            \
        faulttype.set_condition_f(FAULT(k, i, j)["condition_f"]);            \
        faultset->add_faulttype_hashmap(code, faulttype);                    \
      }                                                                     \
    }                                                                        \
  }
#else
#define THIS_FAULTCODESET legionclaw::interface::FaultCodeSet
#define DO_SAFEMODE_ACTION(faultset) printf("NOT SAFEGURAD\n");
#endif
#define FAULTCODE_INIT(json_path, module, faultset, send_callback,          \
                       fault_callback)                                      \
  json fault_json;                                                          \
  std::ifstream in_3(json_path);                                            \
  AINFO << "fault_json_path: " << json_path << std::endl;                  \
  in_3 >> fault_json;                                                       \
  int32_t module_list_size = fault_json["modules"].size();                  \
  for (int k = 0; k < module_list_size; k++) {                              \
    std::string target = MODULE(k)["target"];                               \
    bool match = (0 == target.compare(module));                             \
    if (false == match) {                                                   \
      continue;                                                             \
    }                                                                       \
    uint32_t target_id = MODULE(k)["target_id"];                            \
    bool fault_enable = MODULE(k)["fault_enable"];                          \
    bool heart_enable = MODULE(k)["heart_enable"];                          \
    int heart_period = MODULE(k)["heart_period"];                           \
    faultset = new THIS_FAULTCODESET(                                        \
        target_id, (heart_enable == true) ? send_callback : nullptr,        \
        heart_period);                                                      \
    if (fault_enable == true) {                                             \
      uint32_t fault_code_list_size = MODULE(k)["faultcode"].size();        \
      for (uint32_t i = 0; i < fault_code_list_size; i++) {                 \
        int type = FAULTTYPE(k, i)["fault_type"];                           \
        uint32_t type_list_size = FAULTTYPE(k, i)["list"].size();            \
        for (uint32_t j = 0; j < type_list_size; j++) {                     \
          int enable = FAULT(k, i, j)["enable"];                            \
          std::string black_or_white;                                       \
          if (enable == -1) {                                               \
            continue;                                                       \
          }                                                                 \
          legionclaw::interface::FaultProperty faulttype;                          \
          faulttype.set_code(FAULT(k, i, j)["code"]);                      \
          faulttype.set_name(FAULT(k, i, j)["name"]);                        \
          faulttype.set_enable(enable);                                     \
          switch (type) {                                                    \
            case legionclaw::interface::FAULT_TYPE::DATA_TIMEOUT:               \
            case legionclaw::interface::FAULT_TYPE::DATA_INTERRUPT:             \
            case legionclaw::interface::FAULT_TYPE::TIME_ERROR:                 \
            case legionclaw::interface::FAULT_TYPE::FEEDBACK_ABNORMAL:          \
            case legionclaw::interface::FAULT_TYPE::CALCULATE_TIMEOUT:          \
              faulttype.set_timeout(FAULT(k, i, j)["timeout"]);             \
              break;                                                        \
            case legionclaw::interface::FAULT_TYPE::DATA_LOST:                  \
              faulttype.set_max_count(FAULT(k, i, j)["max_count"]);         \
              break;                                                        \
            case legionclaw::interface::FAULT_TYPE::DATA_ABNORMAL:              \
              black_or_white = FAULT(k, i, j)["black_or_white"];            \
              faulttype.set_black_or_white(black_or_white);                 \
              if (0 != black_or_white.compare("white_p8") &&                \
                  0 != black_or_white.compare("black_p8")) {                \
                faulttype.set_upper(FAULT(k, i, j)["upper"]);               \
              }                                                             \
              if (0 != black_or_white.compare("white_n8") &&                \
                  0 != black_or_white.compare("black_n8")) {                \
                faulttype.set_lower(FAULT(k, i, j)["lower"]);               \
              }                                                             \
              break;                                                        \
          }                                                                 \
          auto fault_object =                                               \
              legionclaw::interface::FaultClientFactory::RegisterFaultClients(  \
                  type, fault_callback, faulttype.name(), faulttype.code(), \
                  faulttype.timeout(), faulttype.max_count(),               \
                  faulttype.black_or_white(), faulttype.lower(),             \
                  faulttype.upper());                                       \
          fault_object->set_enable(enable);                                 \
          faultset->add_watch_map(faulttype.name(), fault_object);          \
        }                                                                   \
      }                                                                     \
    }                                                                       \
    if (true == match) {                                                    \
      break;                                                                \
    }                                                                       \
  }                                                                         \
  DO_SAFEMODE_ACTION(faultset)

#define ENABLE_SWITCH(str, bool)                       \
  if (faultcodeset_->get_watch().count(str) > 0) {     \
    faultcodeset_->get_watch()[str]->set_enable(bool); \
  }

#define CHECK_FAULT(str, value)                               \
  if (faultcodeset_->get_watch().count(str) > 0) {            \
    if (faultcodeset_->get_watch()[str]->enable() == false) { \
      faultcodeset_->get_watch()[str]->set_enable(true);      \
    }                                                         \
    faultcodeset_->get_watch()[str]->set_check_value(value);  \
  }

#define MESSAGE_HEADER_PARSER(obj)                 \
  legionclaw::interface::Header header;                \
  uint32_t seq = (uint32_t)msg->header.seq;        \
  header.set_seq(seq);                             \
  legionclaw::interface::Time interface_time;          \
  interface_time.set_sec(msg->header.stamp.sec);   \
  interface_time.set_nsec(msg->header.stamp.nsec); \
  header.set_stamp(interface_time);                \
  header.set_frame_id(msg->header.frame_id);       \
  obj.set_header(header);

#define MESSAGE_HEADER_ROS2_PARSER(obj)               \
  static uint32_t obj##_seq = 0;                      \
  legionclaw::interface::Header header;                   \
  uint32_t seq = obj##_seq++;                         \
  header.set_seq(seq);                                \
  legionclaw::interface::Time interface_time;             \
  interface_time.set_sec(msg->header.stamp.sec);      \
  interface_time.set_nsec(msg->header.stamp.nanosec); \
  header.set_stamp(interface_time);                   \
  header.set_frame_id(msg->header.frame_id);          \
  obj.set_header(header);

#define MESSAGE_DDS_HEADER_PARSER(obj)                   \
  legionclaw::interface::Header header;                      \
  uint32_t seq = (uint32_t)msg->header().seq();          \
  header.set_seq(seq);                                   \
  legionclaw::interface::Time interface_time;                \
  interface_time.set_sec(msg->header().stamp().sec());   \
  interface_time.set_nsec(msg->header().stamp().nsec()); \
  header.set_stamp(interface_time);                      \
  header.set_frame_id(msg->header().frame_id());         \
  obj.set_header(header);

#define MESSAGE_HEADER_ASSIGN(param, obj)          \
  param::Header header;                            \
  header.stamp.sec = msg.header().stamp().sec();   \
  header.stamp.nsec = msg.header().stamp().nsec(); \
  header.seq = msg.header().seq();                 \
  header.frame_id = msg.header().frame_id();       \
  obj.header = header;

#define MESSAGE_HEADER_ROS2_ASSIGN(param, obj)        \
  param::Header header;                               \
  header.stamp.sec = msg.header().stamp().sec();      \
  header.stamp.nanosec = msg.header().stamp().nsec(); \
  header.frame_id = msg.header().frame_id();          \
  obj.header = header;

#define MESSAGE_DDS_HEADER_ASSIGN(param, obj)          \
  param::Header header;                                \
  header.stamp().sec() = msg.header().stamp().sec();   \
  header.stamp().nsec() = msg.header().stamp().nsec(); \
  header.seq() = msg.header().seq();                   \
  header.frame_id() = msg.header().frame_id();         \
  obj.header() = header;

#define DDS_FAULTS_PARSER(param, obj)                        \
  std::vector<param##_interface::msg::Fault> param##_faults; \
  std::vector<legionclaw::interface::Fault> interface_faults;    \
  msg.faults(interface_faults);                              \
  for (auto it : interface_faults) {                         \
    param##_interface::msg::Fault fault;                     \
    fault.timestamp().sec() = it.timestamp().sec();          \
    fault.timestamp().nsec() = it.timestamp().nsec();        \
    fault.code() = it.code();                                \
    fault.reason() = it.reason();                            \
    param##_faults.emplace_back(fault);                      \
  }                                                          \
  obj.faults() = param##_faults;

#define FAULTS_PARSER(param, obj)                         \
  std::vector<param##_interface::Fault> param##_faults;   \
  std::vector<legionclaw::interface::Fault> interface_faults; \
  msg.faults(interface_faults);                           \
  for (auto it : interface_faults) {                     \
    param##_interface::Fault fault;                       \
    fault.timestamp.sec = it.timestamp().sec();           \
    fault.timestamp.nsec = it.timestamp().nsec();        \
    fault.code = it.code();                               \
    fault.reason = it.reason();                           \
    param##_faults.emplace_back(fault);                  \
  }                                                       \
  obj.faults = param##_faults;
// ros2
#define FAULTS_PARSER_ROS2(param, obj)                       \
  std::vector<param##_interface::msg::Fault> param##_faults; \
  std::vector<legionclaw::interface::Fault> interface_faults;    \
  msg.faults(interface_faults);                              \
  for (auto it : interface_faults) {                         \
    param##_interface::msg::Fault fault;                     \
    fault.timestamp.sec = it.timestamp().sec();              \
    fault.timestamp.nsec = it.timestamp().nsec();        \
    fault.code = it.code();                                \
    fault.reason = it.reason();                            \
    param##_faults.emplace_back(fault);                      \
  }                                                          \
  obj.faults = param##_faults;

#define FAULTS_RECEIVE(obj)                          \
  legionclaw::interface::Fault fault_temp;               \
  std::vector<legionclaw::interface::Fault> faults_temp; \
  for (int i = 0; i < msg->faults_size; i++) {       \
    fault_temp.set_code(msg->faults[i].code);        \
    faults_temp.push_back(fault_temp);               \
  }                                                  \
  obj.set_faults(&faults_temp);

#define MSG_PARSER_POINT3D(param, object) \
  param.set_x(msg->param.x);              \
  param.set_y(msg->param.y);              \
  param.set_z(msg->param.z);              \
  object.set_##param(param);

#define MSG_PARSER_POINTLLH(param, object) \
  param.set_lon(msg->param.lon);           \
  param.set_lat(msg->param.lat);           \
  param.set_height(msg->param.height);     \
  object.set_##param(param);

#define MSG_PARSER_POINTENU(param, object) \
  param.set_x(msg->param.x);              \
  param.set_y(msg->param.y);              \
  param.set_z(msg->param.z);              \
  object.set_##param(param);

#define INTERFACE_HEADER_ASSIGN(param)         \
  static uint32_t param##_seq = 0;             \
  header.set_seq(param##_seq++);               \
  header.set_stamp(TimeTool::Now2TmeStruct()); \
  param.set_header(header);

#define DECLARE_SINGLETON(classname)                                      \
 public:                                                                  \
  static classname* Instance(bool create_if_needed = true) {              \
    static classname* instance = nullptr;                                 \
    if (!instance && create_if_needed) {                                  \
      static std::once_flag flag;                                        \
      std::call_once(flag,                                               \
                     [&] { instance = new (std::nothrow) classname(); }); \
    }                                                                     \
    return instance;                                                      \
  }                                                                       \
                                                                           \
  static void CleanUp() {                                                 \
    auto instance = Instance(false);                                      \
    if (instance != nullptr) {                                            \
      CallShutdown(instance);                                            \
    }                                                                     \
  }                                                                       \
                                                                           \
 private:                                                                 \
  classname();                                                            \
  DISALLOW_COPY_AND_ASSIGN(classname)
