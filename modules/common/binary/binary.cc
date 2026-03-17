
/**
 * @file              binary.cc
 * @author       jiangchengjie
 * @brief
 * @version     1.0.0
 * @date           2021-12-29 11:19:46
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */
#include "binary.h"

#include <mutex>
#include <string>

namespace {
std::mutex m;
std::string binary_name;  // NOLINT
}  // namespace

namespace legionclaw {
namespace common {
namespace binary {

std::string GetName() {
  std::lock_guard<std::mutex> lock(m);
  return binary_name;
}
void SetName(const std::string& name) {
  std::lock_guard<std::mutex> lock(m);
  binary_name = name;
}

}  // namespace binary
}  // namespace common
}  // namespace legionclaw
