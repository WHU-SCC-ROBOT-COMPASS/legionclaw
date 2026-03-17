/**
 * @file         timer_manager.h
 * @author jiangchengjie
 * @date       2020-03-26
 * @version 1.0.0
 * @par          Copyright(c)
 * @license   GNU General Public License (GPL)
 */

#pragma once

#include <sys/time.h>
#include <time.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "modules/common/interface/time.hpp"

namespace legionclaw {
namespace common {

class TimeTool {
public:
  static uint32_t ElapseSeconds(std::chrono::system_clock::time_point& start) {
    return std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::system_clock::now() - start)
        .count();
  }

  static int64_t ElapseMs(std::chrono::system_clock::time_point& start) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now() - start)
        .count();
  }

  static int64_t ElapseUs(std::chrono::system_clock::time_point& start) {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::system_clock::now() - start)
        .count();
  }

  static double NowToSeconds() {
    int64_t time_now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
    return (double)time_now / 1000000000.0;
  }

  static uint32_t Now2Seconds() {
    return std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }

  static int64_t Now2Ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }

  static int64_t Now2Us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }

  static int64_t Now2Ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }

  static int64_t ToSeconds(const std::chrono::system_clock::time_point& time) {
    return std::chrono::duration_cast<std::chrono::seconds>(
               time.time_since_epoch())
        .count();
  }

  static int64_t ToMs(const std::chrono::system_clock::time_point& time) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               time.time_since_epoch())
        .count();
  }

  static int64_t ToUs(const std::chrono::system_clock::time_point& time) {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               time.time_since_epoch())
        .count();
  }

  static legionclaw::interface::Time Now2TmeStruct() {
    legionclaw::interface::Time time;
    time.set_sec(Now2Seconds());
    time.set_nsec((uint32_t)(int64_t)(Now2Ns() - time.sec() * 1000000000));
    return time;
  }

  static int64_t TimeStruct2Ms(legionclaw::interface::Time time) {
    int64_t ms;
    ms = time.sec();
    ms *= 1000;
    ms += time.nsec() / 1000000;
    return ms;
  }

  static int64_t TimeStruct2Us(legionclaw::interface::Time time) {
    int64_t us;
    us = time.sec();
    us *= 1000000;
    us += time.nsec() / 1000;
    return us;
  }

  static int64_t TimeStruct2Ns(legionclaw::interface::Time time) {
    int64_t us;
    us = time.sec();
    us *= 1000000000;
    us += time.nsec();
    return us;
  }

  static double TimeStruct2S(legionclaw::interface::Time time) {
    double s;
    s = (double)time.sec();
    s += (double)time.nsec() / 1000000000.0;
    return s;
  }

  static double GetTimeDiffNow(legionclaw::interface::Time time) {
    return NowToSeconds() - TimeStruct2S(time);
  }

  static double GetTimeDiffNow(double time) { return NowToSeconds() - time; }

  static double GetTimeDiff(legionclaw::interface::Time s,
                            legionclaw::interface::Time e) {
    return (TimeStruct2S(e) - TimeStruct2S(s));
  }

  static double GetTimeDiff(double s, double e) { return (e - s); }

  static legionclaw::interface::Time Second2Time(double s) {
    legionclaw::interface::Time t;
    int int_sec = (int)s;
    uint32_t sec = (uint32_t)int_sec;
    t.set_sec(sec);
    double decimal_sec = s - (double)int_sec;
    uint32_t nsec = (uint32_t)(decimal_sec * 1000000000);
    t.set_nsec(nsec);
    return t;
  }

  static std::string ToStr(const std::chrono::system_clock::time_point& time) {
    uint64_t mill = std::chrono::duration_cast<std::chrono::milliseconds>(
                        time.time_since_epoch())
                        .count() -
                    std::chrono::duration_cast<std::chrono::seconds>(
                        time.time_since_epoch())
                            .count() *
                        1000;

    std::stringstream _time;
    time_t tt = std::chrono::system_clock::to_time_t(time);
    struct tm local_time;
    localtime_r(&tt, &local_time);

    _time << local_time.tm_year + 1900 << "-" << std::setfill('0')
          << std::setw(2) << local_time.tm_mon + 1 << "-" << std::setfill('0')
          << std::setw(2) << local_time.tm_mday << " " << std::setfill('0')
          << std::setw(2) << local_time.tm_hour << ":" << std::setfill('0')
          << std::setw(2) << local_time.tm_min << ":" << std::setfill('0')
          << std::setw(2) << local_time.tm_sec << "." << std::setfill('0')
          << std::setw(3) << mill;

    return _time.str();
  }

  static int64_t Now2MsEx() {
    struct timeval tv;
    ::gettimeofday(&tv, 0);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
  }

  static time_t ToTime(std::string str) {
    time_t t_;
    tm tm_;
    strptime(str.c_str(), "%Y-%m-%d %H:%M:%S", &tm_); //将字符串转换为tm时间
    t_ = mktime(&tm_); //将tm时间转换为秒时间

    return t_;
  }

  static std::chrono::system_clock::time_point ToTimeEx(std::string str) {
    int pos = str.length() - 3;

    time_t t_;
    tm tm_;
    strptime(str.substr(0, pos).c_str(), "%Y-%m-%d %H:%M:%S",
             &tm_);    //将字符串转换为tm时间
    t_ = mktime(&tm_); //将tm时间转换为秒时间

    int milli = std::stoi(str.substr(pos));

    return std::chrono::system_clock::time_point(
        std::chrono::milliseconds(t_ * 1000 + milli));
  }

  static std::string ToStr(uint64_t ms) {
    uint64_t mill = ms % 1000;

    time_t tt = ms / 1000;
    struct tm* local_time = localtime(&tt);

    std::stringstream _time;

    _time << local_time->tm_year + 1900 << "-" << std::setfill('0')
          << std::setw(2) << local_time->tm_mon + 1 << "-" << std::setfill('0')
          << std::setw(2) << local_time->tm_mday << " " << std::setfill('0')
          << std::setw(2) << local_time->tm_hour << ":" << std::setfill('0')
          << std::setw(2) << local_time->tm_min << ":" << std::setfill('0')
          << std::setw(2) << local_time->tm_sec << "." << std::setfill('0')
          << std::setw(3) << mill;

    return _time.str();
  }

  static uint32_t ElapseSeconds(time_t& start) {
    time_t now;
    time(&now);
    return static_cast<uint32_t>(std::difftime(now, start));
  }

  static std::string ToStr(const std::time_t& time) {
    char _time[25] = {0};
    struct tm local_time;
    localtime_r(&time, &local_time);
    strftime(_time, 22, "%Y-%m-%d %H:%M:%S", &local_time);

    return std::string(_time);
  }
};
} // namespace common
} // namespace legionclaw
