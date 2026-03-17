/**
 * @file              fault_handle.hpp
 * @author       jiangchengjie
 * @brief
 * @version     1.0.0
 * @date           2021-10-15 08:02:38
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */

#pragma once

#include <vector>
#include <iostream>
#include <mutex>
#include <iostream>
#include <memory>
#include <stdlib.h>
#include <thread>
#include <algorithm>
#include <map>
#include <functional>
#include <unordered_map>

#include "../time/time_tool.h"
#include "modules/common/interface/faults.hpp"

namespace legionclaw {
namespace interface {

using namespace legionclaw::common;
using namespace std;

/*
设备类型
*/
enum FAULT_TYPE : uint8_t {
  DATA_TIMEOUT = 1,
  DATA_INTERRUPT = 2,
  TIME_ERROR = 3,
  DATA_LOST = 4,
  DATA_ERROR = 5,
  DATA_ABNORMAL = 6,
  FEEDBACK_ABNORMAL = 7,
  INIT_FAILED = 8,
  CALCULATE_FAILED = 9,
  CALCULATE_TIMEOUT = 10,
};

enum BLACK_WHITE_TYPE : uint8_t {
  WHITE = 0,    //白名单
  WHITE_n8 = 1, //白负无穷
  WHITE_p8 = 2, //白正无穷
  BLACK = 3,    //黑名单
  BLACK_n8 = 4, //黑负无穷
  BLACK_p8 = 5, //黑正无穷
};

typedef std::function<void()> FaultCodeCallback; //故障相关回调函数通用类型定义

class FaultExt : public Fault {
public:
  FaultExt() = default;
  ~FaultExt() = default;

  //重载==
  bool operator==(const FaultExt& objstruct) const {
    return objstruct.code_ == code_;
  }
  //重载<
  bool operator<(const FaultExt& objstruct) const {
    return code_ < objstruct.code_;
  }
  //重载>
  bool operator>(const FaultExt& objstuct) const {
    return code_ > objstuct.code_;
  }
};

class FaultProperty : public Fault {
public:
  FaultProperty() = default;
  ~FaultProperty() = default;

  void set_ch_name(std::string ch_name) { ch_name_ = ch_name; }
  std::string ch_name() const { return ch_name_; }

  void set_name(std::string name) { name_ = name; }
  std::string name() const { return name_; }

  void set_enable(int enable) { enable_ = enable; }
  int enable() const { return enable_; }

  void set_description(std::string description) { description_ = description; }
  std::string description() const { return description_; }

  void set_timeout(uint16_t timeout) { timeout_ = timeout; }
  uint16_t timeout() const { return timeout_; }

  void set_black_or_white(std::string black_or_white) {
    black_or_white_ = black_or_white;
  }
  std::string black_or_white() const { return black_or_white_; }

  void set_lower(double lower) { lower_ = lower; }
  double lower() const { return lower_; }

  void set_upper(double upper) { upper_ = upper; }
  double upper() const { return upper_; }

  void set_max_count(uint16_t max_count) { max_count_ = max_count; }
  uint16_t max_count() const { return max_count_; }

  void set_condition_0(uint16_t condition_0) { condition_0_ = condition_0; }
  uint16_t condition_0() const { return condition_0_; }

  void set_condition_f(uint16_t condition_f) { condition_f_ = condition_f; }
  uint16_t condition_f() const { return condition_f_; }

protected:
  std::string ch_name_;
  std::string name_;
  bool enable_;
  std::string description_;
  uint16_t timeout_;
  std::string black_or_white_;
  double lower_;
  double upper_;
  uint16_t max_count_;
  uint16_t condition_0_;
  uint16_t condition_f_;
};

class FaultsExt : public Faults {
public:
  FaultsExt() : Faults() {}
  ~FaultsExt() = default;

  inline void copy_faults(legionclaw::interface::Faults* faults) {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    std::vector<legionclaw::interface::Fault> faults_tmp;
    for (auto val : faults_) {
      legionclaw::interface::Fault f;
      f.CopyFrom(val);
      faults_tmp.push_back(f);
    }
    faults->set_faults(&faults_tmp);
  }

  inline void insert_faults(legionclaw::interface::Faults& faults) {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    FaultExt code_ext;
    for (auto val : faults.faults()) {
      code_ext.CopyFrom(val);
      faults_.push_back(code_ext);
    }
    sort(faults_.begin(), faults_.end());
    faults_.erase(unique(faults_.begin(), faults_.end()), faults_.end());
  }

  inline void insert_fault_code(legionclaw::interface::Fault& code) {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    FaultExt code_ext;
    code_ext.CopyFrom(code);
    auto it = find(faults_.begin(), faults_.end(), code_ext);
    if (it != faults_.end()) {
      ///保留已有的故障码，不更新
      return;
    }
    code_ext.set_timestamp(TimeTool::Now2TmeStruct());
    faults_.push_back(code_ext);
    sort(faults_.begin(), faults_.end());
    // faults_.erase(unique(faults_.begin(), faults_.end()), faults_.end());
  }

  inline void remove_fault_code(legionclaw::interface::Fault& code) {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    FaultExt code_ext;
    code_ext.CopyFrom(code);
    auto it = find(faults_.begin(), faults_.end(), code_ext);
    if (it != faults_.end()) {
      faults_.erase(it);
    }
  }

  inline void clear_faults() {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    faults_.clear();
  }

  inline int32_t get_size() {
    std::lock_guard<std::mutex> lock(*faults_mutex_);
    return faults_.size();
  }

protected:
  std::vector<legionclaw::interface::FaultExt> faults_;
};

///通用故障控制类，用于各模块监控节点的故障码发送，安全中心故障消息接收处理
class FaultCodeCmd {
public:
  /*
  故障控制类构造函数
  sendheart_callback_func: IN，发送故障报文的回调函数
  heart_interval： IN，心跳/故障报文间隔
  */
  FaultCodeCmd(FaultCodeCallback sendheart_callback_func,
               int32_t heart_interval) {
    sendheart_callback_func_ = sendheart_callback_func;
    heart_interval_ = heart_interval;
    if (sendheart_callback_func_ != nullptr && heart_interval_ > 0) {
      heart_thread_.reset(
          new std::thread([this] { publish_faultcodemsg_thread_func(); }));
      if (heart_thread_ == nullptr) {
        std::cout << "Unable to create heart publish thread." << std::endl;
      }

    } else {
      heart_thread_ = nullptr;
    }
  }

  ///获取发生的故障列表
  void get_fault_code_list(legionclaw::interface::Faults* faults_in) {
    faultcode_list_.copy_faults(faults_in);
  }

  ///在故障列表中插入故障码
  void insert_faults(legionclaw::interface::Faults& faults) {
    faultcode_list_.insert_faults(faults);
  }
  ///在故障列表中插入故障码
  void insert_fault_code(legionclaw::interface::Fault& code) {
    faultcode_list_.insert_fault_code(code);
  }
  ///在故障列表中删除故障码
  void remove_fault_code(legionclaw::interface::Fault& code) {
    faultcode_list_.remove_fault_code(code);
  }

  /*
  发布心跳/故障码报文函数
  */
  void publish_faultcodemsg_thread_func() {
    while (1) {
      sendheart_callback_func_();
      std::this_thread::sleep_for(std::chrono::milliseconds(heart_interval_));
    }
  }

  /*
  获取故障码列表长度函数
  */
  int32_t get_fault_code_list_size() const {
    return faultcode_list_.get_size();
  }

  /*
  清空故障码列表
  */
  void clear_faults() { faultcode_list_.clear_faults(); }

protected:
  ///故障码列表
  legionclaw::interface::FaultsExt faultcode_list_;
  ///心跳间隔ms
  int32_t heart_interval_;

private:
  ///发送心跳及故障消息的回调函数指针
  FaultCodeCallback sendheart_callback_func_;
  ///发送心跳的线程
  std::unique_ptr<std::thread> heart_thread_;
};

///通用故障监控类，用于各模块（包括安全中心）的监控节点对已知故障的监控
class FaultCodeWatch {
public:
  /*
  通用故障监控类的构造函数
  faultcode: IN，绑定的故障码值
  fault_callback_func:
  IN，故障发生时的故障回调函数，用于模块的故障预处理（如先于安全中心提前制动）
  */
  FaultCodeWatch(std::string faultname, int32_t faultcode,
                 FaultCodeCallback fault_callback_func) {
    binding_faultcode_.set_reason(faultname);
    binding_faultcode_.set_code(faultcode);
    fault_callback_func_ = fault_callback_func;
    error_happens_ = false;
    enable_ = false;
    faultprocessor_ = nullptr;
  }

  /*
  故障检测函数，根据异常计数判断是否发生故障
  time_to_now: IN，当前系统时间（ms，unix time）
  返回值: true，发生故障，false，未发生故障
  */

  virtual void set_check_value(double value) {
    if (enable_ == false) {
      return;
    }
    error_happens_ = (bool)value;
  }

  virtual bool check_error() {
    if (enable_ == false) {
      return false;
    }
    bool check_state = false;
    if (error_happens_ == true) {
      if (faultprocessor_ != nullptr) {
        faultprocessor_->insert_fault_code(binding_faultcode_);
      }
      check_state = true;
      if (fault_callback_func_ != nullptr) {
        fault_callback_func_();
      }
    } else if (faultprocessor_ != nullptr) {
      faultprocessor_->remove_fault_code(binding_faultcode_);
    }
    return check_state;
  }

  /*
  故障监控使能设置函数
  enable: IN，true表示使能，false表示去使能
  */
  void set_enable(bool enable) {
    if (enable_ == true && enable == false && faultprocessor_ != nullptr) {
      faultprocessor_->remove_fault_code(binding_faultcode_);
    }
    enable_ = enable;
  }

  bool enable() { return enable_; }

  /*
  获取绑定的故障码
  */
  legionclaw::interface::Fault fault() { return binding_faultcode_; }

  virtual void set_start_time(int64_t start_time) {}

  virtual void set_value(double value) {}

  virtual void set_count(uint32_t set_count) {}

  void set_processor(FaultCodeCmd* faultprocessor) {
    faultprocessor_ = faultprocessor;
  }

protected:
  /// watch使能
  bool enable_;
  ///绑定的故障码
  legionclaw::interface::Fault binding_faultcode_;

private:
  ///故障控制对象指针
  FaultCodeCmd* faultprocessor_;
  ///用于记录故障是否发生
  bool error_happens_;
  ///故障发生后的回调函数，用于业务处理，例如紧急刹车等
  FaultCodeCallback fault_callback_func_;
};

///通信超时故障监控类
class TimeoutFaultCodeWatch : public FaultCodeWatch {
public:
  /*
  通信超时故障监控对象构造函数
  timeout: IN，超时时间（ms）
  其他参数同基类
  */
  TimeoutFaultCodeWatch(std::string faultname, int32_t faultcode,
                        FaultCodeCallback fault_callback_func, uint32_t timeout)
      : FaultCodeWatch(faultname, faultcode, fault_callback_func) {
    timeout_ = timeout;
    start_time_ = 0;
  }

  /*
  通信超时故障检测函数，根据超时时间判断是否发生故障
  返回值: 同基类
  */
  bool check_error() override {
    if (enable_ == false) {
      start_time_ = 0;
      return false;
    }
    int64_t time_to_now = TimeTool::Now2Ms();
    if (start_time_ == 0) {
      start_time_ = time_to_now;
    }
    FaultCodeWatch::set_check_value(time_to_now - start_time_ >= timeout_);
    return FaultCodeWatch::check_error();
  }

  void set_check_value(double value) override {
    if (0 == value) {
      start_time_ = TimeTool::Now2Ms();
    } else {
      start_time_ = value;
    }
  }

protected:
  ///设置起始时间(unix time，ms)
  int64_t start_time_;
  ///超时的阈值(ms)
  uint32_t timeout_;
};

///黑白名单故障监控类
class TbwFaultCodeWatch : public FaultCodeWatch {
public:
  /*
通信超时故障监控对象构造函数
black_or_white: IN，黑白名单类型
l：IN，下限
u：IN，上限
其他参数同基类
*/
  TbwFaultCodeWatch(std::string faultname, int32_t faultcode,
                    FaultCodeCallback fault_callback_func,
                    std::string black_or_white, double l, double u,
                    uint32_t timeout)
      : FaultCodeWatch(faultname, faultcode, fault_callback_func) {
    lower_ = l;
    upper_ = u;
    timeout_ = timeout;
    if (black_or_white.compare("white") == 0) {
      black_or_white_ = BLACK_WHITE_TYPE::WHITE; ///白名单
    } else if (black_or_white.compare("white_n8") == 0) {
      black_or_white_ = BLACK_WHITE_TYPE::WHITE_n8; ///白名单负无穷n8
    } else if (black_or_white.compare("white_p8") == 0) {
      black_or_white_ = BLACK_WHITE_TYPE::WHITE_p8; ///白名单正无穷p8
    } else if (black_or_white.compare("black") == 0) {
      black_or_white_ = BLACK_WHITE_TYPE::BLACK; ///黑名单
    } else if (black_or_white.compare("black_n8") == 0) {
      black_or_white_ = BLACK_WHITE_TYPE::BLACK_n8; ///黑名单n8
    } else if (black_or_white.compare("black_p8") == 0) {
      black_or_white_ = BLACK_WHITE_TYPE::BLACK_p8; ///黑名单p8
    }
    value_ = 0.0;
  }

  /*
故障检测函数，根据数据区间判断是否发生故障
返回值: 同基类
*/
  bool check_error() override {
    bool temp_error = false;
    if (enable_ == false || binding_faultcode_.timestamp().sec() == 0) {
      return false;
    }

    switch (black_or_white_) {
    case BLACK_WHITE_TYPE::WHITE:
      temp_error = (value_ > upper_ || value_ < lower_); ///白名单
      break;

    case BLACK_WHITE_TYPE::WHITE_n8:
      temp_error = (value_ > upper_); ///白名单负无穷n8
      break;

    case BLACK_WHITE_TYPE::WHITE_p8:
      temp_error = (value_ < lower_); ///白名单正无穷p8
      break;

    case BLACK_WHITE_TYPE::BLACK:
      temp_error = (value_ <= upper_ && value_ >= lower_); ///黑名单
      break;

    case BLACK_WHITE_TYPE::BLACK_n8:
      temp_error = (value_ <= upper_); ///黑名单n8
      break;

    case BLACK_WHITE_TYPE::BLACK_p8:
      temp_error = (value_ >= lower_); ///黑名单p8
      break;

    default:
      break;
    }
    if (temp_error && timeout_ > 0 &&
        binding_faultcode_.timestamp().sec() > 0 &&
        TimeTool::GetTimeDiffNow(binding_faultcode_.timestamp()) * 1000 >
            timeout_) {
      temp_error = false;
    }
    // if (!temp_error && binding_faultcode_.timestamp().sec() > 0)
    // {
    //   binding_faultcode_.set_timestamp(legionclaw::interface::Time());
    // }
    FaultCodeWatch::set_check_value(temp_error);
    return FaultCodeWatch::check_error();
  }

  void set_check_value(double value) override {
    value_ = value;
    binding_faultcode_.set_timestamp(TimeTool::Now2TmeStruct());
  }

protected:
  ///下边界
  double lower_;
  ///上边界
  double upper_;
  ///黑白类型
  BLACK_WHITE_TYPE black_or_white_;
  ///实际值
  double value_;
  ///故障无更新后自动清除的超时时间(ms)，0表示无超时机制（持续上报直至更新）
  uint32_t timeout_;
};

///计数故障监控类
class CountFaultCodeWatch : public FaultCodeWatch {
public:
  /*
  计数故障监控对象构造函数
  max_count: IN，最大计数值
  其他参数同基类
  */
  CountFaultCodeWatch(std::string faultname, int32_t faultcode,
                      FaultCodeCallback fault_callback_func, uint32_t max_count)
      : FaultCodeWatch(faultname, faultcode, fault_callback_func) {
    max_count_ = max_count;
  }

  /*
  故障检测函数，根据计数是否连续判断是否发生故障
  返回值: 同基类
  */
  bool check_error() override {
    static int64_t last_count = -1;
    if (enable_ == false) {
      return false;
    }
    FaultCodeWatch::set_check_value(
        reality_count_ > max_count_ ||
        (last_count != -1 && abs(reality_count_ - last_count) > 1));
    return FaultCodeWatch::check_error();
  }

  void set_check_value(double value) override { reality_count_ = value; }

protected:
  uint32_t max_count_;
  uint32_t reality_count_;
};

///故障处理集合基类，将FaultCodeCmd和FaultCodeWatch打包起来，方便统一检查各个Watch是否已产生故障
class FaultCodeSet {
public:
  ///构造函数，若check_interval大于0,则在类内部开线程检查faultcodewatch_list_中各监控对象，否则在主模块代码中开线程来检查
  FaultCodeSet(uint8_t target_id, FaultCodeCallback sendheart_callback_func,
               int32_t heart_interval) {
    send_faultcodecmd_ =
        new FaultCodeCmd(sendheart_callback_func, heart_interval);
    target_id_ = target_id;
    check_interval_ = heart_interval;
    if (check_interval_ > 0) {
      check_thread_.reset(
          new std::thread([this] { check_faultcode_watch_thread_func(); }));
      if (check_thread_ == nullptr) {
        // Logging::LogError("Unable to create check watch thread.");
      }
    } else {
      check_thread_ = nullptr;
    }
  }
  ~FaultCodeSet() {
    ///删除清理send_faultcodecmd_, faultcodewatch_list_中各指针
    if (send_faultcodecmd_ != nullptr) {
      delete send_faultcodecmd_;
    }
    for (auto it : faultcodewatch_list_) {
      FaultCodeWatch* ptr = it;
      if (ptr != nullptr) {
        delete ptr;
      }
    }
  }

  ///设置和读取检查线程的周期，一般不用，构建时设置即可
  void set_check_interval(int32_t check_interval) {
    check_interval_ = check_interval;
  }
  int32_t get_check_interval() const { return check_interval_; }

  ///增加监控类对象指针到faultcodewatch_list_
  bool add_faultcodewatch_list(FaultCodeWatch* ptr) {
    if (ptr != nullptr) {
      faultcodewatch_list_.push_back(ptr);
      return true;
    }
    return false;
  }

  void add_watch_map(std::string name, FaultCodeWatch* ptr) {
    //  std::cout<<"name === "<<name<<endl;
    if (ptr != nullptr) {
      watch_map_.insert(pair<std::string, FaultCodeWatch*>(name, ptr));
      add_faultcodewatch_list(ptr);
      ptr->set_processor(send_faultcodecmd_);
    }
  }

  std::map<std::string, FaultCodeWatch*> get_watch() { return watch_map_; }

  Fault current_fault() { return current_fault_; }

  void set_current_fault(Fault code) {
    code.set_timestamp(TimeTool::Now2TmeStruct());
    current_fault_ = code;
  }

  void check_all_watches() {
    for (auto it : faultcodewatch_list_) {
      FaultCodeWatch* ptr = it;
      if (ptr != nullptr) {
        set_current_fault(ptr->fault());
        ptr->check_error();
      }
    }
  }

  ///使能所有FaultCodeWatch
  void enable_all_watches(bool enable) {
    for (auto it : faultcodewatch_list_) {
      FaultCodeWatch* ptr = it;
      if (ptr != nullptr) {
        ptr->set_enable(true);
      }
    }
  }

  ///异常监测线程执行函数．
  void check_faultcode_watch_thread_func() {
    int64_t time_to_now;
    while (1) {
      check_all_watches();
      std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_));
      // for(map<std::string,FaultCodeWatch*>::iterator
      // it=watch_map_.begin();it!=watch_map_.end();it++)
      // {
      //     std::cout<<it->first<<endl;
      // }
      // std::cout<<"faultcodewatch_list_.size() ===
      // "<<faultcodewatch_list_.size()<<endl; std::cout<<"watch_map_.size() ===
      // "<<watch_map_.size()<<endl;
    }
  }

  uint8_t get_target_id() { return target_id_; }

  void get_fault_code_list(legionclaw::interface::Faults* faults_in) {
    send_faultcodecmd_->get_fault_code_list(faults_in);
  }

  void clear_faults() { send_faultcodecmd_->clear_faults(); }

  void remove_fault_code(legionclaw::interface::Fault& code) {
    send_faultcodecmd_->remove_fault_code(code);
  }

  virtual void add_topicnametotimeout_map(std::string topic_name,
                                          std::string timeout_fault) {}

  virtual void add_topicnametointerrupt_map(std::string topic_name,
                                            std::string interrupt_fault) {}

  virtual void add_faulttype_hashmap(uint32_t code, FaultProperty type) {}

  virtual void add_channel_watch_list(std::string topic_name) {}

  virtual void add_module_watch_list(std::string module_name) {}

protected:
  ///用于模块收集故障和发送心跳/故障消息的类对象
  FaultCodeCmd* send_faultcodecmd_;
  ///故障名到故障监控对象的映射
  std::map<std::string, FaultCodeWatch*> watch_map_;
  ///模块ID
  uint8_t target_id_;
  ///检查间隔ms
  int32_t check_interval_;
  ///检查故障的线程
  std::unique_ptr<std::thread> check_thread_;
  ///监控对象指针列表
  std::vector<FaultCodeWatch*> faultcodewatch_list_;
  ///当前故障
  Fault current_fault_;
};

class SafeFaultCodeSet : public FaultCodeSet {
public:
  SafeFaultCodeSet(uint8_t target_id, FaultCodeCallback sendheart_callback_func,
                   int32_t heart_interval)
      : FaultCodeSet(target_id, sendheart_callback_func, heart_interval) {}
  ~SafeFaultCodeSet() {}

  void add_topicnametotimeout_map(std::string topic_name,
                                  std::string timeout_fault) override {
    if (topic_name.length() != 0) {
      topicnametotimeout_map_.insert(
          pair<std::string, std::string>(topic_name, timeout_fault));
    }
  }

  void add_topicnametointerrupt_map(std::string topic_name,
                                    std::string interrupt_fault) override {
    if (topic_name.length() != 0) {
      topicnametointerrupt_map_.insert(
          pair<std::string, std::string>(topic_name, interrupt_fault));
    }
  }

  void add_faulttype_hashmap(uint32_t code, FaultProperty type) override {
    if (code > 0) {
      faulttype_hashmap_.insert(pair<uint32_t, FaultProperty>(code, type));
    }
  }

  void add_channel_watch_list(std::string topic_name) override {
    if (topic_name.length() != 0) {
      channel_watch_list_.push_back(topic_name);
    }
  }

  void add_module_watch_list(std::string module_name) override {
    if (module_name.length() != 0) {
      module_watch_list_.push_back(module_name);
    }
  }

  /*
  插入故障码函数
  code： IN，故障码对象
  */
  std::vector<string> get_channel_watch_list() { return channel_watch_list_; }

  std::vector<string> get_module_watch_list() { return module_watch_list_; }

  // void insert_fault_code(legionclaw::interface::Fault code) {
  //   return send_faultcodecmd_->insert_fault_code(code);
  // }

  void insert_faults(legionclaw::interface::Faults faults) {
    return send_faultcodecmd_->insert_faults(faults);
  }

  std::map<std::string, std::string> get_target_by_name_timeout() {
    return topicnametotimeout_map_;
  }

  std::map<std::string, std::string> get_target_by_name_interrupt() {
    return topicnametointerrupt_map_;
  }

  unordered_map<uint32_t, FaultProperty> get_faulttype() {
    return faulttype_hashmap_;
  }

private:
  /// topic通道名到超时故障的映射
  std::map<std::string, std::string> topicnametotimeout_map_;
  /// topic通道名到中断故障的映射
  std::map<std::string, std::string> topicnametointerrupt_map_;
  ///故障码到故障配置的哈希映射
  unordered_map<uint32_t, legionclaw::interface::FaultProperty> faulttype_hashmap_;
  //接收通道名列表
  std::vector<string> channel_watch_list_;
  //模块名列表
  std::vector<string> module_watch_list_;
};

} // namespace interface
} // namespace legionclaw
