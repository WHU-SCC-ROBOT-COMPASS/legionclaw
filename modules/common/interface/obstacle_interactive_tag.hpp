/**
 * @file    obstacle_interactive_tag.hpp
 * @author  zdhy
 * @date    2024-02-21
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

#include <iostream>
#include <stdint.h>

/**
 * @namespace legionclaw::interface
 * @brief legionclaw::interface
 */
namespace legionclaw {
namespace interface {
class ObstacleInteractiveTag {
public:
  ObstacleInteractiveTag() {
    interactive_tag_ =
        legionclaw::interface::ObstacleInteractiveTag::InteractiveTag::INTERACTION;
  }
  ~ObstacleInteractiveTag() = default;

  enum InteractiveTag {
    INTERACTION = 1,
    NONINTERACTION = 2,
  };

  inline void set_interactive_tag(
      const legionclaw::interface::ObstacleInteractiveTag::InteractiveTag
          &interactive_tag) {
    interactive_tag_ = interactive_tag;
    interactive_tag_ptr_ = &interactive_tag_;
  }

  inline const legionclaw::interface::ObstacleInteractiveTag::InteractiveTag &
  interactive_tag() const {
    return interactive_tag_;
  }

  inline legionclaw::interface::ObstacleInteractiveTag::InteractiveTag *
  mutable_interactive_tag() {
    return &interactive_tag_;
  }

  inline bool has_interactive_tag() {
    return (interactive_tag_ptr_ != nullptr);
  }

  void operator=(const ObstacleInteractiveTag &obstacle_interactive_tag) {
    CopyFrom(obstacle_interactive_tag);
  }

  void CopyFrom(const ObstacleInteractiveTag &obstacle_interactive_tag) {
    interactive_tag_ = obstacle_interactive_tag.interactive_tag();
  }

protected:
  legionclaw::interface::ObstacleInteractiveTag::InteractiveTag interactive_tag_;
  legionclaw::interface::ObstacleInteractiveTag::InteractiveTag
      *interactive_tag_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
