// Copyright 2026 KUKA Hungaria Kft.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KUKA_DRIVERS_CORE__STOP_REASON_HPP_
#define KUKA_DRIVERS_CORE__STOP_REASON_HPP_

#include <cstdint>

namespace kuka_drivers_core
{
/**
 * @brief Why the hardware component last left the active state, exported as state/stop_reason
 */
enum class StopReason : std::uint8_t
{
  NONE = 0,
  RSI_RECEIVE_FAILED = 1,
  RSI_SEND_FAILED = 2
};

inline const char * StopReasonName(StopReason reason)
{
  switch (reason)
  {
    case StopReason::NONE:
      return "none";
    case StopReason::RSI_RECEIVE_FAILED:
      return "RSI receive failed";
    case StopReason::RSI_SEND_FAILED:
      return "RSI send failed";
  }
  return "unknown";
}

}  // namespace kuka_drivers_core

#endif  // KUKA_DRIVERS_CORE__STOP_REASON_HPP_
