/**
 * @file              state_flags.hpp
 * @author       jiangchengjie
 * @brief
 * @version     1.0.0
 * @date           2021-10-29 02:16:49
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */
#ifndef __STATE_FLAGS_HPP__
#define __STATE_FLAGS_HPP__
namespace legionclaw {
namespace common {
namespace state_machine {
enum class CallbackType { UPDATE, ENTRY, EXIT };

enum TrafficLightColors {
  E_RED = 0,
  E_YELLOW = 0,
  E_GREEN = 1,
  E_COLOR_ERROR = 2
};
} // namespace state_machine
} // namespace common
} // namespace legionclaw
#endif
