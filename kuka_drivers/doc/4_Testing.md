# Testing the RSI driver

`kuka_rsi_driver`'s tests are launch tests (`launch_testing`) that run the real hardware component inside a controller manager, against `kuka_rsi_simulator` in place of the robot controller. Each one checks a single behaviour of the hardware lifecycle: what a transition does, what a fault does, and what the component reports. Each was written to fail without the change it covers. The [Lyrical CI](../../.github/workflows/lyrical_ci.yml) job runs all of them on every pull request.

## Running them

```bash
colcon build --packages-up-to kuka_rsi_driver kuka_rsi_simulator
source install/setup.bash
# With rmw_zenoh_cpp (as in CI), start a router first, or nothing discovers anything.
ros2 run rmw_zenoh_cpp rmw_zenohd &

colcon test --packages-select kuka_rsi_driver kuka_rsi_simulator --executor sequential
colcon test-result --verbose

# One test, by its ctest name
colcon test --packages-select kuka_rsi_driver --ctest-args -R test_mxa_loss
```

Run single tests through `colcon test`/`ctest`, not `launch_test` directly: tests that use the mxA status fake need the test-only plugin prefix that `CMakeLists.txt` adds to `AMENT_PREFIX_PATH`. Run the suite sequentially; the tests share UDP ports.

## Two seams make faults reachable without a robot

- **RSI: `kuka_rsi_simulator` fault injection.** The simulator answers the driver's RSI packets like a controller. ROS parameters inject faults while the driver is active: `fault_late_packet_cycles`, `fault_dropped_packet_count` and `fault_repeated_ipoc_count`. They are documented in [1_RSI.md](1_RSI.md#fault-injection) and are all off by default.
- **mxA: the status fake.** The test-only plugin `kuka_rsi_driver::KukaMxaRsiStatusFakeHardwareInterface` ([`test_plugin/`](../../kuka_rsi_driver/test_plugin)) runs the `mxa_rsi` component with the simulator's real RSI stream, but takes the controller status from text datagrams on a local UDP port. A test sends, for example, `emergency_stop=1`, `operation_mode=1`, `link_down`, `stall_receive=200`, `error=STOPMESS+active` or `send_fail=1`; the full command list is at the top of `mxa_status_fake.hpp`. The plugin is built into a separate prefix under the build directory and is never installed. `mxa_status_fake_support.py` holds the launch description and the send helper.

The component also has an opt-in `fault_injection` hardware parameter (`gpio/stop_request`, `gpio/rsi_mute_ms`) that drives the same stop paths on a real controller. It is described in [1_RSI.md](1_RSI.md#hardware-parameters).

## What is covered

| Area | Tests (`kuka_rsi_driver/test/`) |
|---|---|
| Bring-up (from upstream) | `test_driver_startup_*`, `test_driver_activation*` |
| Runs without the KUKA controllers | `test_activates_without_kroshu_controllers`, `test_streams_without_event_broadcaster`, `test_exports_no_runtime_config`, `test_refuses_unsupported_mode_switch` |
| Bounded configure and activate | `test_configure_times_out_without_mxa`, `test_activate_bounded_without_first_packet`, `test_activate_refused_while_stop_pending`, `test_on_activate_refuses_when_not_ready`, `test_on_activate_sees_fresh_status`, `test_first_activation_receives_first_packet`, `test_drives_on_after_normal_stop`, `test_activation_fails_on_estop_during_drives_on` |
| Non-blocking deactivate | `test_deactivate_returns_within_one_cycle`, `test_deactivate_returns_with_mxa_link_down` |
| A fault stops the component | `test_deactivates_on_receive_failure`, `test_deactivates_on_send_failure`, `test_faults_on_read_timeout`, `test_no_command_after_latch`, `test_mxa_status_deactivates`, `test_sdk_events_deactivate`, `test_fault_injection` |
| mxA link loss, and what is not a loss | `test_mxa_loss`, `test_receive_stall_does_not_error`, `test_rejects_staleness_bound_at_sdk_timeout` |
| Reactivation from `inactive` | `test_reactivates_after_deactivate`, `test_reactivation_discards_queued_packets`, `test_activation_refused_while_packets_arrive`, `test_activation_waits_for_fresh_status` |
| Reported status and stop causes | `test_exports_status_state_interfaces`, `test_exports_brake_test_status`, `test_exports_inhibitor_slots`, `test_inhibitor_slots`, `test_inhibitor_latched_until_activate`, `test_status_latches_while_inactive` |
| The control loop never busy-spins | `test_loop_timing`, `test_loop_spins_when_errored_to_unconfigured` |
| The seams themselves | `test_mxa_status_fake`, `test_mxa_status_fake_active`, `kuka_rsi_simulator/test/test_fault_injection.py` |

## Writing a new one

- Start from the closest existing test and its support module: `rsi_only_support.py` (the `rsi_only` component on the simulator), `mxa_status_fake_support.py` (the `mxa_rsi` component on the status fake) or `rsi_fault_support.py` (inject a fault while active, then read the controllers and inhibitor slots).
- Drive the lifecycle the way a client would: `controller_manager` services (`set_hardware_component_state`, `switch_controller`), not `robot_manager`.
- Assert the discriminating value, such as the inhibitor slot that latched or the lifecycle state reached, not just that something failed. Read `state/*` and `inhibitor/*` with the state-only GPIO controller in `test/config/inhibitor_broadcaster.yaml`, since `joint_state_broadcaster` does not publish them.
- Count RSI timing in IPOC cycles, not wall time: the Python simulator cycles at 40 ms with timer jitter, so tests set `read_timeout_ms` to 100 (`SIMULATOR_READ_TIMEOUT_MS`). The mxA staleness bound is wall-clock by design, and the status fake keeps a wall-clock telegram clock for it.
- Register the file with `add_launch_test()` in `kuka_rsi_driver/CMakeLists.txt`, adding `${TEST_PLUGIN_ENV}` if it uses the status fake. CI fails if a registered test is missing from the run.
- Show the test failing without your change, and say so in the pull request.
