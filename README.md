# ROS2 KUKA Drivers

This repository contains ROS2 drivers for all KUKA operating systems.

> [!IMPORTANT]
> **This is a fork.** The `amp/lyrical` branch carries [kuka-ros/kuka_drivers](https://github.com/kuka-ros/kuka_drivers) ported to ROS 2 Lyrical, with `kuka_rsi_driver` hardened for unattended operation on iiQKA.OS2 (`mxa_rsi`). It builds against the forks of the [SDK](https://github.com/kuka-amp-forks/kuka_external_control_sdk) and the [robot descriptions](https://github.com/kuka-amp-forks/kuka_robot_descriptions) at the commits pinned in [`lyrical_ci.yml`](.github/workflows/lyrical_ci.yml). Only the RSI driver, the RSI simulator, `kuka_drivers_core` and the KUKA controllers are carried. What differs from upstream:
>
> - Faults and recovery go through the ros2_control hardware lifecycle. A stop takes the component from `active` to `inactive` (`write()` returns `DEACTIVATE`); a lost mxA link takes it to `unconfigured`. Recovery is `set_hardware_component_state` then `switch_controllers`, with no `robot_manager` required.
> - The component no longer depends on the KUKA controllers: the `runtime_config/*` command interfaces are gone, the RSI cycle is fixed at 4 ms and joint position is the only command mode.
> - Controller status and stop causes are exported as `state/*` interfaces and six `inhibitor/*` slots, and every wait in a lifecycle callback is bounded by a hardware parameter. See [KSS and iiQKA.OS2 drivers (RSI)](kuka_drivers/doc/1_RSI.md).
> - Every behaviour is covered by a launch test that runs in CI without hardware; see [Testing](#testing).
>
> **Hardware validation covers one configuration:** an LBR iisy 15 R930 on iiQKA.OS2 9.3.1 with `driver_version:=mxa_rsi`. It is also the only model whose description the fork changes (collision geometry on the wrist and link 3). Other robots, `rsi_only` and `eki_rsi` build and pass the simulator tests, which use `kr6_r700_2`, but have not been run on a controller with these changes.

ROS2 Distro | Branch | Github CI | SonarCloud
------------ | -------------- | -------------- | --------------
**Jazzy** | [`master`](https://github.com/kuka-ros/kuka_drivers/tree/master) | [![Build Status](https://github.com/kuka-ros//kuka_drivers/actions/workflows/industrial_ci_jazzy.yml/badge.svg?branch=master)](https://github.com/kuka-ros/kuka_drivers/actions/workflows/industrial_ci_jazzy.yml?branch=master) | [![Quality gate status](https://sonarcloud.io/api/project_badges/measure?project=kuka-ros_kuka_drivers&metric=alert_status&token=7043f08818e18d9f33876189039e663e8ba13f09&branch=master)](https://sonarcloud.io/summary/new_code?id=kuka-ros_kuka_drivers)
**Humble** | [`humble`](https://github.com/kuka-ros/kuka_drivers/tree/humble) | [![Build Status](https://github.com/kuka-ros//kuka_drivers/actions/workflows/industrial_ci_humble.yml/badge.svg?branch=humble)](https://github.com/kuka-ros/kuka_drivers/actions/workflows/industrial_ci_humble.yml) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=kuka-ros_kuka_drivers&metric=alert_status&token=7043f08818e18d9f33876189039e663e8ba13f09&branch=humble)](https://sonarcloud.io/dashboard?id=kuka-ros_kuka_drivers)

## Requirements

The drivers require a system with ROS installed. It is recommended to use Ubuntu 24.04 with ROS Jazzy.

Additionally, there exists a ROS Humble version of the drivers, and its corresponding configuration can be found under the `humble` branch.

It is also recommended to use a client machine with a real-time kernel, as all three drivers require cyclic, real-time communication. Due to the real-time requirement, Windows systems are not recommended and covered in the documentation.

## Installation

### Installation as binary package

The driver is also available as a binary package. Installing the `kuka_drivers` metapackage will only install the packages strictly necessary for using the drivers. To install all available robot models, the `kuka_robot_descriptions` package should be also installed.

```bash
sudo apt install ros-jazzy-kuka-drivers
sudo apt install ros-jazzy-kuka-robot-descriptions
```

If due to lack of resources this is not intended, it is also possible to install support packages only for a single robot family.
```bash
sudo apt install ros-jazzy-kuka-drivers
sudo apt install ros-jazzy-kuka-agilus-support
```

> [!NOTE]
> As the ROS2 packages are not immediately available via apt after the release, it is possible that the installed version lacks some features already available on the development branch.

### Installation from source

The driver can be also built from source. The main advantage of this is to get features before they are released and available for installation. All configuration options should also be available if using the released binary packages.

Create ROS2 workspace (if not already created).

```bash
mkdir -p ~/ros2_ws/src
```

Clone KUKA ROS2 repositories.

```bash
cd ~/ros2_ws/src
git clone https://github.com/kuka-ros/kuka_drivers.git
vcs import < kuka_drivers/upstream.repos
```

Install and initialize rosdep (if not already done)

```bash
sudo apt install python3-rosdep
sudo rosdep init
```

Install dependencies using `rosdep`.

```bash
cd ~/ros2_ws
rosdep update
sudo apt upgrade
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Build all packages in workspace.

```bash
cd ~/ros2_ws
colcon build
```

Source workspace.

```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source ~/ros2_ws/install/setup.bash
```

> [!NOTE]
> As the `kuka_external_control_sdk` package is designed to work also outside of the ROS2 ecosystem, and is simply wrapped with a `package.xml` to simplify the setup, the following warnings are expected and can be ignored after sourcing:
> ```
> not found: "<WS>/install/kuka_external_control_sdk_common/share/kuka_external_control_sdk_common/local_setup.bash"
> not found: "<WS>/install/kuka_external_control_sdk_iiqka2/share/> kuka_external_control_sdk_iiqka2/local_setup.bash"
> ```

## Getting Started

Documentation of this project is available in the package's [`doc`](kuka_drivers/doc) directory:

- [Project overview](kuka_drivers/doc/0_Overview.md)
- [KSS and iiQKA.OS2 drivers (RSI)](kuka_drivers/doc/1_RSI.md)
- [KUKA-specific controllers](kuka_drivers/doc/2_Controllers.md)
- [Real-time patch setup](kuka_drivers/doc/3_Realtime.md)

## Testing

`kuka_rsi_driver`'s tests are launch tests (`launch_testing`) that run the real hardware component inside a controller manager, against `kuka_rsi_simulator` in place of the robot controller. Each one checks a single behaviour of the hardware lifecycle: what a transition does, what a fault does, and what the component reports. Each was written to fail without the change it covers. The [Lyrical CI](.github/workflows/lyrical_ci.yml) job runs all of them on every pull request.

### Running them

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

### Two seams make faults reachable without a robot

- **RSI: `kuka_rsi_simulator` fault injection.** The simulator answers the driver's RSI packets like a controller. ROS parameters inject faults while the driver is active: `fault_late_packet_cycles`, `fault_dropped_packet_count` and `fault_repeated_ipoc_count`. They are documented in [1_RSI.md](kuka_drivers/doc/1_RSI.md#fault-injection) and are all off by default.
- **mxA: the status fake.** The test-only plugin `kuka_rsi_driver::KukaMxaRsiStatusFakeHardwareInterface` ([`test_plugin/`](kuka_rsi_driver/test_plugin)) runs the `mxa_rsi` component with the simulator's real RSI stream, but takes the controller status from text datagrams on a local UDP port. A test sends, for example, `emergency_stop=1`, `operation_mode=1`, `link_down`, `stall_receive=200`, `error=STOPMESS+active` or `send_fail=1`; the full command list is at the top of `mxa_status_fake.hpp`. The plugin is built into a separate prefix under the build directory and is never installed. `mxa_status_fake_support.py` holds the launch description and the send helper.

The component also has an opt-in `fault_injection` hardware parameter (`gpio/stop_request`, `gpio/rsi_mute_ms`) that drives the same stop paths on a real controller. It is described in [1_RSI.md](kuka_drivers/doc/1_RSI.md#hardware-parameters).

### What is covered

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

### Writing a new one

- Start from the closest existing test and its support module: `rsi_only_support.py` (the `rsi_only` component on the simulator), `mxa_status_fake_support.py` (the `mxa_rsi` component on the status fake) or `rsi_fault_support.py` (inject a fault while active, then read the controllers and inhibitor slots).
- Drive the lifecycle the way a client would: `controller_manager` services (`set_hardware_component_state`, `switch_controller`), not `robot_manager`.
- Assert the discriminating value, such as the inhibitor slot that latched or the lifecycle state reached, not just that something failed. Read `state/*` and `inhibitor/*` with the state-only GPIO controller in `test/config/inhibitor_broadcaster.yaml`, since `joint_state_broadcaster` does not publish them.
- Count RSI timing in IPOC cycles, not wall time: the Python simulator cycles at 40 ms with timer jitter, so tests set `read_timeout_ms` to 100 (`SIMULATOR_READ_TIMEOUT_MS`). The mxA staleness bound is wall-clock by design, and the status fake keeps a wall-clock telegram clock for it.
- Register the file with `add_launch_test()` in `kuka_rsi_driver/CMakeLists.txt`, adding `${TEST_PLUGIN_ENV}` if it uses the status fake. CI fails if a registered test is missing from the run.
- Show the test failing without your change, and say so in the pull request.

If you find something confusing, not working, or would like to contribute, please read our [contributing guide](CONTRIBUTING.md) before opening an issue or creating a pull request.
