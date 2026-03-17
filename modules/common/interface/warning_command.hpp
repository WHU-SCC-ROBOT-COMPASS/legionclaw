/**
 * @file    warning_command.hpp
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
class WarningCommand {
public:
  WarningCommand() {
    sound_alarm_ = legionclaw::interface::WarningCommand::SoundAlarm::NO_SOUND;
    light_alarm_ = legionclaw::interface::WarningCommand::LightAlarm::NO_LIGHT;
    media_alarm_ = legionclaw::interface::WarningCommand::MediaAlarm::NO_MEDIA;
    motion_alarm_ = legionclaw::interface::WarningCommand::MotionAlarm::NO_MOTION;
  }
  ~WarningCommand() = default;

  enum SoundAlarm {
    NO_SOUND = 0,
    SOUND_NORMAL = 1,
    SOUND_HIGH = 2,
    SOUND_URGENT = 3,
  };

  enum MotionAlarm {
    NO_MOTION = 0,
    MOTION_NORMAL = 1,
    MOTION_URGENT = 2,
  };

  enum LightAlarm {
    NO_LIGHT = 0,
    LIGHT_NORMAL = 1,
    LIGHT_HIGH = 2,
    LIGHT_URGENT = 3,
  };

  enum MediaAlarm {
    NO_MEDIA = 0,
    MEDIA_PROMPT = 1,
    MEDIA_NORMAL = 2,
    MEDIA_HIGH = 3,
    MEDIA_URGENT = 4,
  };

  inline void set_sound_alarm(
      const legionclaw::interface::WarningCommand::SoundAlarm &sound_alarm) {
    sound_alarm_ = sound_alarm;
    sound_alarm_ptr_ = &sound_alarm_;
  }

  inline const legionclaw::interface::WarningCommand::SoundAlarm &
  sound_alarm() const {
    return sound_alarm_;
  }

  inline legionclaw::interface::WarningCommand::SoundAlarm *mutable_sound_alarm() {
    return &sound_alarm_;
  }

  inline bool has_sound_alarm() { return (sound_alarm_ptr_ != nullptr); }

  inline void set_light_alarm(
      const legionclaw::interface::WarningCommand::LightAlarm &light_alarm) {
    light_alarm_ = light_alarm;
    light_alarm_ptr_ = &light_alarm_;
  }

  inline const legionclaw::interface::WarningCommand::LightAlarm &
  light_alarm() const {
    return light_alarm_;
  }

  inline legionclaw::interface::WarningCommand::LightAlarm *mutable_light_alarm() {
    return &light_alarm_;
  }

  inline bool has_light_alarm() { return (light_alarm_ptr_ != nullptr); }

  inline void set_media_alarm(
      const legionclaw::interface::WarningCommand::MediaAlarm &media_alarm) {
    media_alarm_ = media_alarm;
    media_alarm_ptr_ = &media_alarm_;
  }

  inline const legionclaw::interface::WarningCommand::MediaAlarm &
  media_alarm() const {
    return media_alarm_;
  }

  inline legionclaw::interface::WarningCommand::MediaAlarm *mutable_media_alarm() {
    return &media_alarm_;
  }

  inline bool has_media_alarm() { return (media_alarm_ptr_ != nullptr); }

  inline void set_motion_alarm(
      const legionclaw::interface::WarningCommand::MotionAlarm &motion_alarm) {
    motion_alarm_ = motion_alarm;
    motion_alarm_ptr_ = &motion_alarm_;
  }

  inline const legionclaw::interface::WarningCommand::MotionAlarm &
  motion_alarm() const {
    return motion_alarm_;
  }

  inline legionclaw::interface::WarningCommand::MotionAlarm *mutable_motion_alarm() {
    return &motion_alarm_;
  }

  inline bool has_motion_alarm() { return (motion_alarm_ptr_ != nullptr); }

  void operator=(const WarningCommand &warning_command) {
    CopyFrom(warning_command);
  }

  void CopyFrom(const WarningCommand &warning_command) {
    sound_alarm_ = warning_command.sound_alarm();
    light_alarm_ = warning_command.light_alarm();
    media_alarm_ = warning_command.media_alarm();
    motion_alarm_ = warning_command.motion_alarm();
  }

protected:
  //声音告警
  legionclaw::interface::WarningCommand::SoundAlarm sound_alarm_;
  legionclaw::interface::WarningCommand::SoundAlarm *sound_alarm_ptr_ = nullptr;
  //车灯告警
  legionclaw::interface::WarningCommand::LightAlarm light_alarm_;
  legionclaw::interface::WarningCommand::LightAlarm *light_alarm_ptr_ = nullptr;
  //媒体告警
  legionclaw::interface::WarningCommand::MediaAlarm media_alarm_;
  legionclaw::interface::WarningCommand::MediaAlarm *media_alarm_ptr_ = nullptr;
  //震动告警
  legionclaw::interface::WarningCommand::MotionAlarm motion_alarm_;
  legionclaw::interface::WarningCommand::MotionAlarm *motion_alarm_ptr_ = nullptr;
};
} // namespace interface
} // namespace legionclaw
