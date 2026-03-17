/**
 * @file    stories.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>
#include <string>

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class Stories {
public:
  Stories() { note_.clear(); }
  ~Stories() = default;

  inline void set_note(const std::string &note) {
    note_ = note;
    note_ptr_ = &note_;
  }

  inline const std::string &note() const { return note_; }

  inline std::string *mutable_note() { return &note_; }

  inline bool has_note() { return (note_ptr_ != nullptr); }

  void operator=(const Stories &stories) { CopyFrom(stories); }

  void CopyFrom(const Stories &stories) { note_ = stories.note(); }

protected:
  std::string note_;
  std::string *note_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
