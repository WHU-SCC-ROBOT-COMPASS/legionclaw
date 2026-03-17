/******************************************************************************
 * Copyright 2017 The Legion Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 */

#pragma once

#include <string>

/**
 * @namespace legionclaw::common
 * @brief legionclaw::common
 */
namespace legionclaw {
namespace common {

/**
 * @class Status
 *
 * @brief A general class to denote the return status of an API call. It
 * can either be an OK status for success, or a failure status with error
 * message and error code enum.
 */
class Status {
public:
  enum ErrorCode {
    // No error, returns on success.
    OK = 0,

    // Control module error codes start from here.
    CONTROL_ERROR = 1000,
    CONTROL_INIT_ERROR = 1001,
    CONTROL_COMPUTE_ERROR = 1002,

    // Canbus module error codes start from here.
    CANBUS_ERROR = 2000,
    CAN_CLIENT_ERROR_BASE = 2100,
    CAN_CLIENT_ERROR_OPEN_DEVICE_FAILED = 2101,
    CAN_CLIENT_ERROR_FRAME_NUM = 2102,
    CAN_CLIENT_ERROR_SEND_FAILED = 2103,
    CAN_CLIENT_ERROR_RECV_FAILED = 2104,
    CAN_CLIENT_ERROR_SEND_PERIOD = 2105,

    // Localization module error codes start from here.
    LOCALIZATION_ERROR = 3000,
    LOCALIZATION_ERROR_MSG = 3100,
    LOCALIZATION_ERROR_LIDAR = 3200,
    LOCALIZATION_ERROR_INTEG = 3300,
    LOCALIZATION_ERROR_GNSS = 3400,

    // Perception module error codes start from here.
    PERCEPTION_ERROR = 4000,
    PERCEPTION_ERROR_TF = 4001,
    PERCEPTION_ERROR_PROCESS = 4002,
    PERCEPTION_FATAL = 4003,
    PERCEPTION_ERROR_NONE = 4004,
    PERCEPTION_ERROR_UNKNOWN = 4005,

    // Control module error codes start from here.
    PREDICTION_ERROR = 5000,

    // Planning module error codes start from here
    PLANNING_ERROR = 6000,
    PLANNING_ERROR_NOT_READY = 6001,

    // HDMap module error codes start from here
    HDMAP_DATA_ERROR = 7000,

    // Routing module error codes
    ROUTING_ERROR = 8000,
    ROUTING_ERROR_REQUEST = 8001,
    ROUTING_ERROR_RESPONSE = 8002,
    ROUTING_ERROR_NOT_READY = 8003,

    // Indicates an input has been exhausted.
    END_OF_INPUT = 9000,

    // Relative Map error codes.
    RELATIVE_MAP_ERROR = 11000, // general relative map error code
    RELATIVE_MAP_NOT_READY = 11001,

    // Driver error codes.
    DRIVER_ERROR_GNSS = 12000,
    DRIVER_ERROR_VELODYNE = 13000,
  };
  /**
   * @brief Create a success status.
   */
  Status() : code_(ErrorCode::OK), msg_() {}
  ~Status() = default;

  /**
   * @brief Create a status with the specified error code and msg as a
   * human-readable string containing more detailed information.
   * @param code the error code.
   * @param msg the message associated with the error.
   */
  Status(ErrorCode code, const std::string& msg) : code_(code), msg_(msg) {}

  /**
   * @brief Create a status with the specified error code and empty msg
   * @param code the error code.
   */
  explicit Status(ErrorCode code) : code_(code), msg_("") {}

  /**
   * @brief generate a success status.
   * @returns a success status
   */
  static Status Ok() { return Status(); }

  /**
   * @brief check whether the return status is OK.
   * @returns true if the code is ErrorCode::OK
   *          false otherwise
   */
  bool ok() const { return code_ == ErrorCode::OK; }

  /**
   * @brief get the error code
   * @returns the error code
   */
  ErrorCode code() const { return code_; }

  /**
   * @brief defines the logic of testing if two Status are equal
   */
  bool operator==(const Status& rh) const {
    return (this->code_ == rh.code_) && (this->msg_ == rh.msg_);
  }

  /**
   * @brief defines the logic of testing if two Status are unequal
   */
  bool operator!=(const Status& rh) const { return !(*this == rh); }

  /**
   * @brief returns the error message of the status, empty if the status is OK.
   * @returns the error message
   */
  const std::string& error_message() const { return msg_; }

  /**
   * @brief returns a string representation in a readable format.
   * @returns the string "OK" if success.
   *          the internal error message otherwise.
   */
  std::string ToString() const {
    if (ok()) {
      return "OK";
    }
    // return ErrorCode_Name(code_) + ": " + msg_;
    return msg_;
  }

  //   /**
  //    * @brief save the error_code and error message to protobuf
  //    * @param the Status protobuf that will store the message.
  //    */
  //   void Save(StatusPb *status_pb) {
  //     if (!status_pb) {
  //       return;
  //     }
  //     status_pb->set_error_code(code_);
  //     if (!msg_.empty()) {
  //       status_pb->set_msg(msg_);
  //     }
  //   }

private:
  ErrorCode code_;
  std::string msg_;
};

inline std::ostream& operator<<(std::ostream& os, const Status& s) {
  os << s.ToString();
  return os;
}

} // namespace common
} // namespace legionclaw
