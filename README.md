# ROS2 KUKA Drivers

This repository contains ROS2 drivers for all KUKA operating systems.

> [!IMPORTANT]
> **This is a fork.** The `amp/lyrical` branch carries [kuka-ros/kuka_drivers](https://github.com/kuka-ros/kuka_drivers) ported to ROS 2 Lyrical, with `kuka_rsi_driver` hardened for unattended operation on iiQKA.OS2 (`mxa_rsi`). It builds against the forks of the [SDK](https://github.com/kuka-amp-forks/kuka_external_control_sdk) and the [robot descriptions](https://github.com/kuka-amp-forks/kuka_robot_descriptions) at the commits pinned in [`lyrical_ci.yml`](.github/workflows/lyrical_ci.yml). Only the RSI driver, the RSI simulator, `kuka_drivers_core` and the KUKA controllers are carried. What differs from upstream:
>
> - Faults and recovery go through the ros2_control hardware lifecycle. A stop takes the component from `active` to `inactive` (`write()` returns `DEACTIVATE`); a lost mxA link takes it to `unconfigured`. Recovery is `set_hardware_component_state` then `switch_controllers`, with no `robot_manager` required.
> - The component no longer depends on the KUKA controllers: the `runtime_config/*` command interfaces are gone, the RSI cycle is fixed at 4 ms and joint position is the only command mode.
> - Controller status and stop causes are exported as `state/*` interfaces and six `inhibitor/*` slots, and every wait in a lifecycle callback is bounded by a hardware parameter. See [KSS and iiQKA.OS2 drivers (RSI)](kuka_drivers/doc/1_RSI.md).
> - Every behaviour is covered by a launch test that runs in CI without hardware; see [Testing the RSI driver](kuka_drivers/doc/4_Testing.md).
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
- [Testing the RSI driver](kuka_drivers/doc/4_Testing.md)

If you find something confusing, not working, or would like to contribute, please read our [contributing guide](CONTRIBUTING.md) before opening an issue or creating a pull request.
