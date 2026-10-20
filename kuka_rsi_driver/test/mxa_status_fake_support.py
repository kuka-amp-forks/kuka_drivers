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

"""Shared pieces for launch tests that run the mxA component on the test-only status fake."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory

COMPONENT = "kr6_r700_2"
FAKE_PORT = 59153
OP_MODE_T1 = 1
OP_MODE_EXT = 4

PRODUCTION_PLUGIN = "<plugin>kuka_rsi_driver::KukaMxaRsiHardwareInterface</plugin>"
FAKE_PLUGIN = (
    "<plugin>kuka_rsi_driver::KukaMxaRsiStatusFakeHardwareInterface</plugin>"
    f'<param name="mxa_status_fake_port">{FAKE_PORT}</param>'
)


def robot_description():
    """Build the mxa_rsi description with the test-only plugin in place of the shipped one."""
    urdf = xacro.process_file(
        os.path.join(
            get_package_share_directory("kuka_agilus_support"), "urdf", f"{COMPONENT}.urdf.xacro"
        ),
        mappings={
            "driver_version": "mxa_rsi",
            "client_ip": "0.0.0.0",
            "client_port": "59152",
            "controller_ip": "127.0.0.1",
            "verify_robot_model": "true",
            "roundtrip_time": "4000",
        },
    ).toxml()
    assert PRODUCTION_PLUGIN in urdf
    return urdf.replace(PRODUCTION_PLUGIN, FAKE_PLUGIN)
