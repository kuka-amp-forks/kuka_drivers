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

"""Shared CPU-spin measurement, so every loop-timing test reads the same threshold."""

import psutil

# A sleeping or blocking loop stays far below one core; a spinning loop pegs one.
# Wide margin on both sides so CI noise cannot flip the verdict.
SPIN_THRESHOLD_PERCENT = 50.0
MEASURE_SECONDS = 1.5


def measure_cpu_percent(pid, seconds=MEASURE_SECONDS):
    """Block for `seconds`; percent is relative to a single core (100 == one core busy)."""
    return psutil.Process(pid).cpu_percent(interval=seconds)


__all__ = [
    "MEASURE_SECONDS",
    "SPIN_THRESHOLD_PERCENT",
    "measure_cpu_percent",
]
