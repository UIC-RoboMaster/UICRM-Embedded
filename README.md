<div align="center">

# BNBU-UIC RoboMaster Embedded

![arm](https://github.com/UIC-RoboMaster/UICRM-Embedded/workflows/arm%20build/badge.svg)
![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)

[English](README.md) | [简体中文](README.zh-CN.md)

</div>

**UICRM-Embedded** is the STM32 embedded firmware repository for the BNBU-UIC RoboMaster team.  It contains general board-level drivers, algorithms and components, hardware examples, and robot programs. The project is built using C/C++, CMake, and the GNU Arm Embedded Toolchain.

[Architecture](#architecture) · [User Guide](#user-guide) · [Developer Guide](#developer-guide) · [Contributing](#contributing)

---

## Architecture

```
uicrm/
├── boards/                  Shared libraries
│   ├── base/                Board support packages (STM32CubeMX HAL) for 6 MCU boards
│   ├── platform/            RTOS & HAL abstraction (stm32f1 / stm32f4 / stm32h7)
│   ├── algorithm/           Control algorithms — PID, AHRS, Quaternion EKF, CRC, power limiting...
│   ├── drivers/             Peripheral drivers — IMU, motors, DBUS/SBUS, OLED, RGB LED, Supercap...
│   ├── components/          Robot subsystems — gimbal, chassis, shooter, referee UI
│   └── third_party/         External libraries — MahonyAHRS, QuaternionEKF, SEGGER RTT...
├── cmake/                   CMake modules — toolchain, build helpers, clang-format, Doxygen
├── examples/                Standalone peripheral examples
├── openocd/                 OpenOCD configs (stm32f1 / stm32f4 / stm32h7)
├── programs/                Complete Robot firmware
└── scripts/                 Utility scripts (launch.json generation, RTT viewer, formatting)
```

### Supported Hardware

| MCU Family | Core | Boards |
|---|---|---|
| STM32F1 | Cortex-M3 | `F103_Nano_general`, `BulletExchanger_F103` |
| STM32F4 | Cortex-M4 | `DJI_Board_TypeC_general` (F407), `DJI_Board_TypeA_general` (F427), `DM_MC01_general` (F446) |
| STM32H7 | Cortex-M7 | `DM_MC02_general` (H723) |

---

## User Guide

You can follow the instructions below to set up the necessary environments for building the source code and flashing the embedded chips.

### 1. Requirements

| Tool | Required | Note |
|---|---|---|
| Arm GNU Toolchain | ✅ | Cross-compiler for STM32 (GCC 10.3+) |
| CMake (≥3.8) | ✅ | Build system; bundled with CLion |
| Ninja | ✅ | Build backend on Windows |
| OpenOCD | ✅ | Debug & flash via CMSIS-DAP / ST-LINK |
| CLion | ⭐ Recommended | IDE with integrated build, flash & debug |


#### Install Arm GNU Toolchain

- **macOS**: `brew install --cask gcc-arm-embedded`
- **Linux / Windows**: download from the [Arm GNU downloads page](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads).

  Extract the archive and note the path to the `bin` folder — for example:

  ```
  /Users/yourname/gcc-arm-none-eabi-10.3-2021.10/bin
  ```

  Add the `bin` directory to your `PATH` (add to `~/.bashrc` or `~/.zshrc`):

  ```sh
  export PATH=<path-to-bin>:$PATH
  ```

#### Install CMake

> Skip this step if you are using **CLion** — it bundles CMake.

- **macOS**: `brew install cmake`
- **Linux**: `sudo apt install cmake` (Ubuntu) / `sudo pacman -S cmake` (Arch)
- **Windows**: download from [cmake.org/download](https://cmake.org/download/)

#### Install Ninja

> Skip this step if you are using **CLion** — it bundles Ninja.

- **macOS**: `brew install ninja`
- **Linux**: `sudo apt install ninja-build` (Ubuntu) / `sudo pacman -S ninja` (Arch)
- **Windows**: download from [ninja-build.org](https://ninja-build.org) and place it on your `PATH`

#### Install OpenOCD

- **macOS**: `brew install open-ocd`
- **Linux / Windows**: download from [gnutoolchains.com/arm-eabi/openocd](https://gnutoolchains.com/arm-eabi/openocd/).

  Extract the archive and note the path to the `bin` folder — for example:

  ```
  /Users/yourname/openocd-0.12.0/bin
  ```

  Add the `bin` directory to your `PATH` (same procedure as the toolchain above).

**Verify your setup by running these commands in a terminal:**

```sh
arm-none-eabi-gcc --version
cmake --version
openocd --version
```

### 2. Building the Project

#### Option A — CLion (Recommended)

1. Open the project root in CLion.
2. Go to **Settings → Build, Execution, Deployment → CMake** and set the Arm GNU Toolchain path.
3. On **Windows**, also set **Generator** to `Ninja`.
4. Select a build target from the toolbar and click **Build**.

#### Option B — Command Line

```sh
cd uicrm-embedded
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

> **Windows**: Use the `Ninja` generator:
> ```sh
> cmake -DCMAKE_BUILD_TYPE=Release .. -G "Ninja"
> ninja -j
> ```

Use `Debug` or `RelWithDebInfo` instead of `Release` when you need GDB debugging.
Note that `Debug` builds run significantly slower due to disabled optimizations.

### 3. Flashing Firmware

#### Option A — CLion (Recommended)

Select your target and click the **Run** button (or **Debug** for step-through debugging).

The default configuration assumes a **CMSIS-DAP** debugger. If you are using **ST-LINK**, change the debug probe in the CLion run configuration.

#### Option B — Command Line (OpenOCD)

The repository provides OpenOCD configuration files in `openocd/` for each MCU family. 
For example, to flash a DJI_Board_TypeC (STM32F4):

```sh
openocd -f openocd/stm32f4/daplink.cfg
```

See [OpenOCD Flash Commands](https://openocd.org/doc/html/Flash-Commands.html) for details.


### 4. Generating Documentation

Install [Doxygen](https://www.doxygen.nl/index.html):

- **macOS**: `brew install doxygen`
- **Ubuntu**: `sudo apt install doxygen`
- **Arch**: `sudo pacman -S doxygen`

Then build the docs:

```sh
cd build
make doc
# or: ninja doc (Windows)
```

Open `docs/html/index.html` in your browser to view the result.

---

## Developer Guide

Follow the guidelines below when contributing to this repository.

### Editing the Code

You can use any editor, but we recommend [CLion](https://www.jetbrains.com/clion/).

### Formatting Code

The continuous integration system will check the source code against a specific coding style. If the code does not follow the style, the formatting check will fail and the code will not be merged.
All codes are required to be formatted correctly before merging. There are several integrated build commands that can help you automatically format your changes.

**Prerequisite**: install `clang-format` **18.1.8**. CMake will not create the format target if `clang-format` is missing.

* For Linux users:

  * Prefer the pinned LLVM binary:
    [x86_64](https://github.com/llvm/llvm-project/releases/download/llvmorg-18.1.8/clang+llvm-18.1.8-x86_64-linux-gnu-ubuntu-18.04.tar.xz)
    [aarch64](https://github.com/llvm/llvm-project/releases/download/llvmorg-18.1.8/clang+llvm-18.1.8-aarch64-linux-gnu.tar.xz)
    ```bash
    tar -xf clang+llvm-18.1.8-*.tar.xz
    cp clang+llvm-18.1.8-*/bin/clang-format /usr/local/bin/
    clang-format --version
    ```
  * Or: `pip install clang-format==18.1.8`
  * Avoid `apt install clang-format-18` on Ubuntu 24.04 — that package is **18.1.3**, not 18.1.8.
  
* For Mac users:

  * Recommend: `brew install llvm@18` then ensure `clang-format` 18.1.8 is on `PATH`
  * Or official package:
    [Apple Silicon](https://github.com/llvm/llvm-project/releases/download/llvmorg-18.1.8/clang+llvm-18.1.8-arm64-apple-macos11.tar.xz)
    ```bash
    tar -xf clang+llvm-18.1.8-arm64-apple-macos11.tar.xz
    cp clang+llvm-18.1.8-arm64-apple-macos11/bin/clang-format /usr/local/bin/
    clang-format --version
    ```
  * Or: `pip install clang-format==18.1.8`
* For Windows users:

  * [Official Installer](https://github.com/llvm/llvm-project/releases/download/llvmorg-18.1.8/LLVM-18.1.8-win64.exe) which files are going to locate `C:\Program Files\LLVM\bin\clang-format.exe` after installation.


**Formatting with CLion**

Select the formatting CMake target and build it. CLion will automatically format the code.
1. `check-format`: Check `diff` between current source and formatted source (without modifying any source file)
2. `format`: Format all source files (**Modifies** file in place)

**Formatting Manually**

You can run the following commands inside `build/` to format your changes.

1. `make check-format`: Check `diff` between current source and formatted source (without modifying any source file)
2. `make format`: Format all source files (**Modifies** file in place)

### Debugging with GDB

Debugging an embedded target requires a remote GDB server. There are two options:

- **CLion Debugger** — The easiest approach. Select the target and click the **Debug** button in CLion.

- **OpenOCD** — Although directly using OpenOCD is possible, it is only recommended for advanced users.

---

## Contributing

The main branch is protected. You need to create a new branch and make a pull request to merge your changes. You need to
<u>pass the CI check (formatting check and build check)</u> before merging.

You should write a meaningful commit message like  
`feat: add a new module to control the gimbal`.

The type must be one of the following:

- **feat** for a new feature for the user, not a new feature for build script.
- **fix** for a bug fix for the user, not a fix to a build script.
- **perf** for performance improvements.
- **docs** for changes to the documentation.
- **style** for formatting changes, missing semicolons, etc.
- **refactor** for refactoring production code, e.g. renaming a variable.
- **test** for adding missing tests, refactoring tests; no production code change.
- **build** for updating build configuration, development tools or other changes irrelevant to the user.
