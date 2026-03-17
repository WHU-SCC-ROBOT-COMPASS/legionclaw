/**
 * @file              state.hpp
 * @author       jiangchengjie
 * @brief
 * @version     1.0.0
 * @date           2021-10-29 02:16:55
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */
#ifndef __STATE_HPP__
#define __STATE_HPP__

#include <cassert>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include "common/logging/logging.h"
#include "state_flags.hpp"

using namespace legionclaw::common;
namespace legionclaw {
namespace common {
namespace state_machine {
class State {
private:
  std::shared_ptr<State> child_state_;

  std::string state_name_;
  uint64_t state_id_;

  std::shared_ptr<State> parent_state_;

  std::function<void(const std::string&)> CallbackUpdateFunc = nullptr;
  std::function<void(const std::string&)> CallbackEntryFunc = nullptr;
  std::function<void(const std::string&)> CallbackExitFunc = nullptr;

  std::map<std::string, uint64_t> transition_map_;

  std::string entered_key_;

public:
  State(std::string _state_name, uint64_t _state_id,
        std::shared_ptr<State> _parent_state = NULL)
      : child_state_(NULL), state_name_(_state_name), state_id_(_state_id),
        parent_state_(_parent_state), entered_key_("") {
#if 0
    CallbackUpdateFunc =
        [&](const std::string &arg) { /*DEBUG_PRINT("[%s]:%s is not registered\n", state_name_.c_str(), "update");*/ };
    CallbackEntryFunc =
        [&](const std::string &arg) { /*DEBUG_PRINT("[%s]:%s is not registered\n", state_name_.c_str(), "entry");*/ };
    CallbackExitFunc =
        [&](const std::string &arg) { /*DEBUG_PRINT("[%s]:%s is not registered\n", state_name_.c_str(), "exit");*/ };
#endif
  }
  ~State() {}

  void OnUpdate(void) {
    if (child_state_) {
      child_state_->OnUpdate();
    }
    if (CallbackUpdateFunc != nullptr) {
      CallbackUpdateFunc(state_name_);
    }
  }

  void OnEntry(void) {
    // ADEBUG << "[" << state_name_ << ":Entry]";
    if (CallbackEntryFunc)
      CallbackEntryFunc(state_name_);
  }
  void OnExit(void) {
    // ADEBUG << "[" << state_name_ << ":Exit]";
    if (child_state_)
      child_state_->OnExit();

    SetChild(nullptr);

    if (CallbackExitFunc)
      CallbackExitFunc(state_name_);
  }
  void SetCallbackUpdate(std::function<void(const std::string&)> _f) {
    CallbackUpdateFunc = _f;
  }
  void SetCallbackEntry(std::function<void(const std::string&)> _f) {
    CallbackEntryFunc = _f;
  }

  void SetCallbackExit(std::function<void(const std::string&)> _f) {
    CallbackExitFunc = _f;
  }

  void ShowStateName(void) { std::cout << state_name_ << "-"; }
  void SetParent(std::shared_ptr<State> _parent) {
    assert(this != _parent.get());

    parent_state_ = _parent;
  }

  void SetChild(std::shared_ptr<State> _child) {
    assert(this != _child.get());
    child_state_ = _child;
  }

  std::shared_ptr<State> GetParent() { return parent_state_; }
  std::shared_ptr<State> GetChild() { return child_state_; }
  std::string GetStateName(void) { return std::string(state_name_); }

  void AddTransition(const std::string key, const uint64_t val) {
    transition_map_[key] = val;
  }

  uint64_t GetTansitionVal(std::string key) const {
    return transition_map_.at(key);
  }

  const std::map<std::string, uint64_t>& GetTransitionMap(void) {
    return transition_map_;
  }

  uint64_t GetStateID(void) { return state_id_; }
  void SetEnteredKey(const std::string& key) { entered_key_ = key; }
  std::string GetEnteredKey(void) { return entered_key_; }
};
} // namespace state_machine
} // namespace common
} // namespace legionclaw
#endif
