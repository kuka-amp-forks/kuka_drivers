// Copyright 2026 KUKA
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

#ifndef KUKA_RSI_DRIVER__TEST_PLUGIN__HARDWARE_INTERFACE_MXA_STATUS_FAKE_HPP_
#define KUKA_RSI_DRIVER__TEST_PLUGIN__HARDWARE_INTERFACE_MXA_STATUS_FAKE_HPP_

#include <cstdint>

#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_rsi_driver/hardware_interface_mxa_rsi.hpp"

namespace kuka_rsi_driver
{

// The mxA interface with MxaStatusFake in place of kss::mxa::Robot. Test-only: built under
// BUILD_TESTING, never installed, selected by naming it in the <plugin> of a robot description.
class KukaMxaRsiStatusFakeHardwareInterface : public KukaMxaRsiHardwareInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

protected:
  void CreateRobotInstance(const kuka::external::control::kss::Configuration &) override;

private:
  uint16_t fake_port_ = 0;
};

}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__TEST_PLUGIN__HARDWARE_INTERFACE_MXA_STATUS_FAKE_HPP_
