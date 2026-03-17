/**
 * @file              fault_client.hpp
 * @author       jiangchengjie
 * @brief
 * @version     1.0.0
 * @date           2021-10-15 08:02:45
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */

/**
 * @file    can_client_factory.h
 * @author  jiangchengjie
 * @date    2019-04-24
 * @version 1.0.0
 * @par     Copyright(c)
 *          hy
 *          thank you legion
 */
/**
 * @file
 * @brief Defines the FaultClientFactory class.
 */

#pragma once

#include <memory>
#include <unordered_map>

#include "fault_handle.hpp"
#include "../util/factory.h"
#include "../logging/logging.h"

namespace legionclaw {
namespace interface {
using namespace legionclaw::common::util;
using namespace legionclaw::common;

/**
 * @class FaultClientFactory
 * @brief FaultClientFactory inherites legionclaw::common::util::Facotory.
 */
class FaultClientFactory {
public:
  /**
   * @brief Register the Fault clients of all brands. This function call the
   *        Function legionclaw::common::util::Factory::Register() for all of the
   *        Fault clients.
   */
  FaultClientFactory(){};
  ~FaultClientFactory(){};
  static FaultCodeWatch* RegisterFaultClients(
      int parameter, legionclaw::interface::FaultCodeCallback fault_callback_func,
      std::string faultname, int32_t faultcode, uint32_t timeout,
      uint16_t max_count, std::string black_or_white, double l, double u) {
    switch (parameter) {
    case DATA_TIMEOUT:
    case DATA_INTERRUPT:
    case TIME_ERROR:
    case FEEDBACK_ABNORMAL:
    case CALCULATE_TIMEOUT:
      return new TimeoutFaultCodeWatch(faultname, faultcode,
                                       fault_callback_func, timeout);
      break;
    case DATA_LOST:
      return new CountFaultCodeWatch(faultname, faultcode, fault_callback_func,
                                     max_count);
      break;
    case DATA_ERROR:
    case INIT_FAILED:
    case CALCULATE_FAILED:
    case DATA_ABNORMAL:
      return new TbwFaultCodeWatch(faultname, faultcode, fault_callback_func,
                                   black_or_white, l, u, timeout);
      break;
    default:
      break;
    }
    return nullptr;
  }
};

} // namespace interface
} // namespace legionclaw
