<div align="center">

# BNBU-UIC RoboMaster Embedded

![arm](https://github.com/UIC-RoboMaster/UICRM-Embedded/workflows/arm%20build/badge.svg)
![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)

[English](README.md) | [简体中文](README.zh-CN.md)

</div>

**UICRM-Embedded** 是 BNBU-UIC RoboMaster 战队的 STM32 嵌入式固件仓库，包含通用板级驱动、算法与组件、硬件示例以及各机器人程序。项目使用 C/C++ 开发，CMake 构建，基于 GNU Arm Embedded Toolchain 编译。

[项目架构](#项目架构) · [使用指南](#使用指南) · [开发指南](#开发指南) · [贡献指南](#贡献指南)

---

## 项目架构

```
uicrm/
├── boards/                  共享库
│   ├── base/                板级支持包（STM32CubeMX HAL），覆盖 6 款 MCU 板卡
│   ├── platform/            RTOS 与 HAL 抽象层（stm32f1 / stm32f4 / stm32h7）
│   ├── algorithm/           控制算法 — PID、AHRS、四元数 EKF、CRC、功率限制
│   ├── drivers/             外设驱动 — IMU、电机、DBUS/SBUS、OLED、RGB LED、超级电容
│   ├── components/          机器人子系统 — 云台、底盘、发射机构、裁判系统 UI
│   └── third_party/         第三方库 — MahonyAHRS、QuaternionEKF、SEGGER RTT
├── cmake/                   CMake 模块 — 工具链、构建辅助、clang-format、Doxygen
├── examples/                独立外设示例（60+）
├── openocd/                 OpenOCD 配置文件（stm32f1 / stm32f4 / stm32h7）
├── programs/                完整机器人固件（12 款机器人）
└── scripts/                 工具脚本（launch.json 生成、RTT 查看器、格式化）
```

### 支持的硬件

| MCU 系列 | 内核 | 板卡 |
|---|---|---|
| STM32F1 | Cortex-M3 | `F103_Nano_general`、`BulletExchanger_F103` |
| STM32F4 | Cortex-M4 | `DJI_Board_TypeC_general` (F407)、`DJI_Board_TypeA_general` (F427)、`DM_MC01_general` (F446) |
| STM32H7 | Cortex-M7 | `DM_MC02_general` (H723) |

---

## 使用指南

按照以下步骤搭建开发环境、构建固件并烧录到开发板。

### 1. 环境要求

| 工具 | 是否必需 | 说明 |
|---|---|---|
| Arm GNU Toolchain | ✅ | STM32 交叉编译器（GCC 10.3+） |
| CMake (≥3.8) | ✅ | 构建系统；CLion 已内置 |
| Ninja | ✅ | Windows 下的构建后端 |
| OpenOCD | ✅ | 通过 CMSIS-DAP / ST-LINK 调试与烧录 |
| CLion | ⭐ 推荐 | 集成构建、烧录、调试的 IDE |


#### 安装 Arm GNU Toolchain

- **macOS**：`brew install --cask gcc-arm-embedded`
- **Linux / Windows**：从 [Arm GNU 下载页面](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) 下载。

  解压后记下 `bin` 目录的路径，例如：

  ```
  /Users/yourname/gcc-arm-none-eabi-10.3-2021.10/bin
  ```

  将 `bin` 目录加入 `PATH`（添加到 `~/.bashrc` 或 `~/.zshrc`）：

  ```sh
  export PATH=<path-to-bin>:$PATH
  ```

#### 安装 CMake

> 如果使用 **CLion**，可跳过此步骤——CLion 已内置 CMake。

- **macOS**：`brew install cmake`
- **Linux**：`sudo apt install cmake`（Ubuntu）/ `sudo pacman -S cmake`（Arch）
- **Windows**：从 [cmake.org/download](https://cmake.org/download/) 下载安装

#### 安装 Ninja

> 如果使用 **CLion**，可跳过此步骤——CLion 已内置 Ninja。

- **macOS**：`brew install ninja`
- **Linux**：`sudo apt install ninja-build`（Ubuntu）/ `sudo pacman -S ninja`（Arch）
- **Windows**：从 [ninja-build.org](https://ninja-build.org) 下载并加入 `PATH`

#### 安装 OpenOCD

- **macOS**：`brew install open-ocd`
- **Linux / Windows**：从 [gnutoolchains.com/arm-eabi/openocd](https://gnutoolchains.com/arm-eabi/openocd/) 下载。

  解压后记下 `bin` 目录的路径，例如：

  ```
  /Users/yourname/openocd-0.12.0/bin
  ```

  将 `bin` 目录加入 `PATH`（与上述工具链配置方式相同）。

**在终端中运行以下命令验证环境是否就绪：**

```sh
arm-none-eabi-gcc --version
cmake --version
openocd --version
```

### 2. 构建项目

#### 方式 A — CLion（推荐）

1. 在 CLion 中打开项目根目录。
2. 进入 **Settings → Build, Execution, Deployment → CMake**，设置 Arm GNU Toolchain 路径。
3. **Windows** 用户还需将 **Generator** 设为 `Ninja`。
4. 在工具栏中选择构建目标，点击 **Build**。

#### 方式 B — 命令行

```sh
cd uicrm-embedded
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

> **Windows** 用户请使用 `Ninja` 生成器：
> ```sh
> cmake -DCMAKE_BUILD_TYPE=Release .. -G "Ninja"
> ninja -j
> ```

如需 GDB 调试，请将构建类型改为 `Debug` 或 `RelWithDebInfo`。
注意 `Debug` 构建因关闭编译器优化，运行速度会明显变慢。

### 3. 烧录固件

#### 方式 A — CLion（推荐）

选择目标，点击 **Run** 按钮（或点击 **Debug** 进行单步调试）。

默认配置使用 **CMSIS-DAP** 调试器。如果使用 **ST-LINK**，请在 CLion 运行配置中切换调试探针。

#### 方式 B — 命令行（OpenOCD）

仓库在 `openocd/` 目录下提供了各 MCU 系列的 OpenOCD 配置文件。
例如，烧录 DJI_Board_TypeC (STM32F4)：

```sh
openocd -f openocd/stm32f4/daplink.cfg
```

详见 [OpenOCD Flash Commands](https://openocd.org/doc/html/Flash-Commands.html)。


### 4. 生成文档

安装 [Doxygen](https://www.doxygen.nl/index.html)：

- **macOS**：`brew install doxygen`
- **Ubuntu**：`sudo apt install doxygen`
- **Arch**：`sudo pacman -S doxygen`

然后构建文档：

```sh
cd build
make doc
# 或（Windows）：ninja doc
```

在浏览器中打开 `docs/html/index.html` 即可查看。

---

## 开发指南

参与本仓库开发的注意事项。

### 编辑代码

任意编辑器均可，推荐使用 [CLion](https://www.jetbrains.com/clion/)。

### 代码格式化

CI 系统会对所有源码进行代码风格检查。不符合规范的代码将无法通过格式化检查，不能合并入仓库。
所有代码在合并前必须正确格式化。项目提供了集成构建命令帮助你自动格式化代码。

**前置条件**：安装 `clang-format` **18.1.8**。若未安装，CMake 将不会创建格式化构建目标。

* Linux 用户：

  * 推荐使用固定版本的 LLVM 二进制：
    [x86_64](https://github.com/llvm/llvm-project/releases/download/llvmorg-18.1.8/clang+llvm-18.1.8-x86_64-linux-gnu-ubuntu-18.04.tar.xz)
    [aarch64](https://github.com/llvm/llvm-project/releases/download/llvmorg-18.1.8/clang+llvm-18.1.8-aarch64-linux-gnu.tar.xz)
    ```bash
    tar -xf clang+llvm-18.1.8-*.tar.xz
    cp clang+llvm-18.1.8-*/bin/clang-format /usr/local/bin/
    clang-format --version
    ```
  * 或：`pip install clang-format==18.1.8`
  * 避免在 Ubuntu 24.04 上使用 `apt install clang-format-18`——该软件包版本为 **18.1.3**，非 18.1.8。

* macOS 用户：

  * 推荐：`brew install llvm@18` 并确保 `clang-format` 18.1.8 在 `PATH` 中
  * 或使用官方包（Apple Silicon）：
    [Apple Silicon](https://github.com/llvm/llvm-project/releases/download/llvmorg-18.1.8/clang+llvm-18.1.8-arm64-apple-macos11.tar.xz)
    ```bash
    tar -xf clang+llvm-18.1.8-arm64-apple-macos11.tar.xz
    cp clang+llvm-18.1.8-arm64-apple-macos11/bin/clang-format /usr/local/bin/
    clang-format --version
    ```
  * 或：`pip install clang-format==18.1.8`

* Windows 用户：

  * [官方安装包](https://github.com/llvm/llvm-project/releases/download/llvmorg-18.1.8/LLVM-18.1.8-win64.exe)，安装后 `clang-format.exe` 位于 `C:\Program Files\LLVM\bin\`。


**在 CLion 中格式化**

选择格式化 CMake 构建目标并编译，CLion 会自动格式化代码。
1. `check-format`：显示当前源码与格式化后源码的 `diff`（不修改文件）
2. `format`：原地格式化所有源文件

**命令行格式化**

在 `build/` 目录下运行以下命令：

1. `make check-format`（或 `ninja check-format`）：预览格式化差异
2. `make format`（或 `ninja format`）：执行原地格式化

### GDB 调试

调试嵌入式目标需要远程 GDB 服务器，有两种选择：

- **CLion Debugger** — 最便捷的方式。选择目标，在 CLion 中点击 **Debug** 按钮即可。

- **OpenOCD** — 虽然可以直接使用 OpenOCD，但仅推荐高级用户使用。

---

## 贡献指南

`main` 分支受保护。请创建新分支并发起 Pull Request 来合并你的修改。
合并前<u>必须通过 CI 检查（格式化检查和构建检查）</u>。

请编写有意义的提交信息，例如：
`feat: 新增云台控制模块`。

提交类型必须是以下之一：

- **feat**：面向用户的新功能，非构建脚本的新功能
- **fix**：面向用户的 bug 修复，非构建脚本的修复
- **perf**：性能优化
- **docs**：文档修改
- **style**：格式化修改、缺失分号等
- **refactor**：生产代码重构，如变量重命名
- **test**：添加或重构测试；无生产代码修改
- **build**：构建配置、开发工具等与用户无关的修改
