/**
 * @file         ad_timer_manager.h
 * @author jiangchengjie
 * @date       2020-03-26
 * @version 1.0.0
 * @par          Copyright(c)
 * @license   GNU General Public License (GPL)
 */

#pragma once

#include <list>
#include <vector>
#include <memory>
#include <sys/time.h>

/**
 * @namespace legionclaw::common
 * @brief legionclaw::common
 */
namespace legionclaw {
namespace common {
#define TVN_BITS 6
#define TVR_BITS 8
#define TVN_SIZE (1 << TVN_BITS)
#define TVR_SIZE (1 << TVR_BITS)
#define TVN_MASK (TVN_SIZE - 1)
#define TVR_MASK (TVR_SIZE - 1)
#define OFFSET(N) (TVR_SIZE + (N)*TVN_SIZE)
#define INDEX(V, N) ((V >> (TVR_BITS + (N)*TVN_BITS)) & TVN_MASK)

template <typename Object, typename Param> class ADTimerManager;

/**
 * @class WheelTimer
 *
 * @brief  WheelTimer.
 */
template <typename Object, typename Param> class WheelTimer {
public:
  typedef void (Object::*CbFun)(Param*);

  enum TimerType { ONCE, CIRCLE };

  WheelTimer(std::shared_ptr<ADTimerManager<Object, Param>> manager);
  ~WheelTimer();

  void AddTimer(int duration, CbFun fun, Object* instance);
  void Stop();

private:
  void OnTimer(uint64_t now, Param*);

private:
  CbFun cb_fun_;   //回调函数指针
  Object* object_; //调用对象

private:
  friend class ADTimerManager<Object, Param>;

  std::shared_ptr<ADTimerManager<Object, Param>> manager_;
  TimerType timerType_;
  unsigned interval_;
  uint64_t expires_;

  int vecIndex_;

  typename std::list<WheelTimer<Object, Param>*>::iterator itr_;
};

/**
 * @class ADTimerManager
 *
 * @brief  ADTimerManager.
 */
template <typename Object, typename Param> class ADTimerManager {
public:
  ADTimerManager();

  static uint64_t GetCurrentMillisecs();
  void DetectTimers(Param* par);

private:
  friend class WheelTimer<Object, Param>;
  void AddTimer(WheelTimer<Object, Param>* timer);
  void RemoveTimer(WheelTimer<Object, Param>* timer);

  int Cascade(int offset, int index);

private:
  typedef std::list<WheelTimer<Object, Param>*> TimeList;
  std::vector<TimeList> tvec_;
  uint64_t checkTime_;
};
template <typename Object, typename Param>
inline void WheelTimer<Object, Param>::AddTimer(int duration, CbFun fun,
                                                Object* instance) {
  Stop();
  object_ = instance;
  interval_ = duration;
  cb_fun_ = fun;
  timerType_ = WheelTimer::CIRCLE;
  expires_ = interval_ + ADTimerManager<Object, Param>::GetCurrentMillisecs();
  manager_->AddTimer(this);
}

template <typename Object, typename Param>
WheelTimer<Object, Param>::WheelTimer(
    std::shared_ptr<ADTimerManager<Object, Param>> manager)
    : manager_(manager), vecIndex_(-1) {}

template <typename Object, typename Param>
WheelTimer<Object, Param>::~WheelTimer() {
  Stop();
}

template <typename Object, typename Param>
void WheelTimer<Object, Param>::Stop() {
  if (vecIndex_ != -1) {
    manager_->RemoveTimer(this);
    vecIndex_ = -1;
  }
}

template <typename Object, typename Param>
void WheelTimer<Object, Param>::OnTimer(uint64_t now, Param* par) {
  if (timerType_ == WheelTimer::CIRCLE) {
    expires_ = interval_ + now;
    manager_->AddTimer(this);
  } else {
    vecIndex_ = -1;
  }

  (object_->*cb_fun_)(par);
}

template <typename Object, typename Param>
ADTimerManager<Object, Param>::ADTimerManager() {
  tvec_.resize(TVR_SIZE + 4 * TVN_SIZE);
  checkTime_ = GetCurrentMillisecs();
}

template <typename Object, typename Param>
void ADTimerManager<Object, Param>::AddTimer(WheelTimer<Object, Param>* timer) {
  uint64_t expires = timer->expires_;
  uint64_t idx = expires - checkTime_;

  if (idx < TVR_SIZE) {
    timer->vecIndex_ = expires & TVR_MASK;
  } else if (idx < 1 << (TVR_BITS + TVN_BITS)) {
    timer->vecIndex_ = OFFSET(0) + INDEX(expires, 0);
  } else if (idx < 1 << (TVR_BITS + 2 * TVN_BITS)) {
    timer->vecIndex_ = OFFSET(1) + INDEX(expires, 1);
  } else if (idx < 1 << (TVR_BITS + 3 * TVN_BITS)) {
    timer->vecIndex_ = OFFSET(2) + INDEX(expires, 2);
  } else if ((int64_t)idx < 0) {
    timer->vecIndex_ = checkTime_ & TVR_MASK;
  } else {
    if (idx > 0xffffffffUL) {
      idx = 0xffffffffUL;
      expires = idx + checkTime_;
    }
    timer->vecIndex_ = OFFSET(3) + INDEX(expires, 3);
  }

  TimeList& tlist = tvec_[timer->vecIndex_];
  tlist.push_back(timer);
  timer->itr_ = tlist.end();
  --timer->itr_;
}

template <typename Object, typename Param>
void ADTimerManager<Object, Param>::RemoveTimer(
    WheelTimer<Object, Param>* timer) {
  TimeList& tlist = tvec_[timer->vecIndex_];
  tlist.erase(timer->itr_);
}

template <typename Object, typename Param>
void ADTimerManager<Object, Param>::DetectTimers(Param* par) {
  uint64_t now = GetCurrentMillisecs();
  while (checkTime_ <= now) {
    int index = checkTime_ & TVR_MASK;
    if (!index && !Cascade(OFFSET(0), INDEX(checkTime_, 0)) &&
        !Cascade(OFFSET(1), INDEX(checkTime_, 1)) &&
        !Cascade(OFFSET(2), INDEX(checkTime_, 2))) {
      Cascade(OFFSET(3), INDEX(checkTime_, 3));
    }
    ++checkTime_;

    TimeList& tlist = tvec_[index];
    TimeList temp;
    temp.splice(temp.end(), tlist);
    for (typename TimeList::iterator itr = temp.begin(); itr != temp.end();
         ++itr) {
      (*itr)->OnTimer(now, par);
    }
  }
}

template <typename Object, typename Param>
int ADTimerManager<Object, Param>::Cascade(int offset, int index) {
  TimeList& tlist = tvec_[offset + index];
  TimeList temp;
  temp.splice(temp.end(), tlist);

  for (typename TimeList::iterator itr = temp.begin(); itr != temp.end();
       ++itr) {
    AddTimer(*itr);
  }

  return index;
}

template <typename Object, typename Param>
uint64_t ADTimerManager<Object, Param>::GetCurrentMillisecs() {
  timeval tv;
  ::gettimeofday(&tv, 0);
  uint64_t ret = tv.tv_sec;
  return ret * 1000 + tv.tv_usec / 1000;
}
} // namespace common
} // namespace legionclaw
