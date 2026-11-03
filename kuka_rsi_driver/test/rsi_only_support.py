# Copyright 2026 KUKA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Robot description for launch tests that run the rsi_only component on the RSI simulator."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory

COMPONENT = "kr6_r700_2"
RSI_ONLY_PLUGIN = "<plugin>kuka_rsi_driver::KukaRSIHardwareInterface</plugin>"

# The simulator cycles at 40 ms; the driver's default read timeout of 12 ms assumes 4 ms.
SIMULATOR_READ_TIMEOUT_MS = 100


def hardware_params(**values):
    """Render extra <param> elements for the <hardware> block."""
    return "".join(f'<param name="{name}">{value}</param>' for name, value in values.items())


def rsi_only_description(read_timeout_ms=SIMULATOR_READ_TIMEOUT_MS, extra_params=""):
    """Build the rsi_only description with the simulator's read timeout in the hardware block."""
    urdf = xacro.process_file(
        os.path.join(
            get_package_share_directory("kuka_agilus_support"), "urdf", f"{COMPONENT}.urdf.xacro"
        ),
        mappings={
            "driver_version": "rsi_only",
            "client_ip": "0.0.0.0",
            "client_port": "59152",
            "roundtrip_time": "4000",
        },
    ).toxml()
    assert RSI_ONLY_PLUGIN in urdf
    return urdf.replace(
        RSI_ONLY_PLUGIN,
        RSI_ONLY_PLUGIN + hardware_params(read_timeout_ms=read_timeout_ms) + extra_params,
    )


__all__ = [
    "COMPONENT",
    "SIMULATOR_READ_TIMEOUT_MS",
    "hardware_params",
    "rsi_only_description",
]
