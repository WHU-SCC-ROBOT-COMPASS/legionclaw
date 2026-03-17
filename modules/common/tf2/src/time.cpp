/*
 * @Author: duanchengwen
 * @Date: 2021-06-08 16:50:02
 * @LastEditors: duanchengwen
 * @LastEditTime: 2021-06-08 17:20:37
 * @Description: file content
 */
#include "../include/tf2/time.h"

namespace tf2 {
double time_to_sec(Time t) { return static_cast<double>(t) / 1e9; }
} // namespace tf2
