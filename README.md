# UIC RoboMaster Embedded

![arm](https://github.com/UIC-RoboMaster/UICRM-Embedded/workflows/arm%20build/badge.svg)

**UICRM-Embedded** is the STM32 embedded firmware repository for the BNBU-UIC RoboMaster team.  It contains general board-level drivers, algorithms and components, hardware examples, and robot programs. The project is built using C/C++, CMake, and the GNU Arm Embedded Toolchain.

---

## User Guide

You can follow the instructions below to set up the necessary environments for
building the source code and flashing the embedded chips.

### Setting Up the Environment

**Install Arm GNU Toolchain**

1. Go to the [official download page](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) for the Arm GNU Toolchain.
2. Download the pre-built toolchain for your operating system.
3. Extract it to a directory of your choice and note the path to the `bin` folder.

    For example: `/Users/yry0008/gcc-arm-none-eabi-10.3-2021.10/bin`.

4. Add the `bin` directory to your `PATH` environment variable:

    - **Windows**: Add `<path>` to the system `PATH` environment variable.
    - **Linux / macOS**: Add the following line to `~/.bashrc` (bash) or `~/.zshrc` (zsh):

    ```sh
    export PATH=<path>:$PATH
    ```

**Install OpenOCD**

1. Go to the [official download page](https://gnutoolchains.com/arm-eabi/openocd/) for OpenOCD.
2. Download the pre-built binary for your operating system.
3. Extract it to a directory of your choice and note the path to the `bin` folder.

    For example: `/Users/yry0008/openocd-0.11.0-2021.10/bin`.

4. Add the `bin` directory to your `PATH` environment variable:

    - **Windows**: Add `<path>` to the system `PATH` environment variable.
    - **Linux / macOS**: Add the following line to `~/.bashrc` (bash) or `~/.zshrc` (zsh):

    ```sh
    export PATH=<path>:$PATH
    ```

**Install CMake**

1. Download CMake from the [official download page](https://cmake.org/download/).

> If you are using CLion, CMake is bundled — you can skip this step.

**Install Ninja (Windows only)**

1. Download Ninja from the [official website](https://ninja-build.org).

### Compile Project

**With CLion (Recommended)**

You can open the project directly in CLion and build it.
Set the path of the Arm GNU Toolchain in **Settings → Build, Execution, Deployment → CMake**.

> **Windows users**: Go to **Settings → Build, Execution, Deployment → CMake** and set **Generator** to `Ninja`.

**Building Manually**

1. Open a terminal in the project root directory.
2. Run the following commands to configure and build:

    ```sh
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j
    ```

> **Windows users**: Use the `Ninja` generator:
> ```sh
> cmake -DCMAKE_BUILD_TYPE=Release .. -G "Ninja"
> ninja -j
> ```

Use `Debug` or `RelWithDebInfo` build types for GDB debugging. Note that `Debug` builds may be significantly slower due to disabled compiler optimizations.

### Flashing Firmware

**Flashing with CLion**

Select the target you want to flash and click the **Run** button.

The default configuration uses a CMSIS-DAP debugger. If you are using ST-LINK, update the debugger settings in the CLion run configuration.

**Flashing with OpenOCD**

You can also flash manually using OpenOCD. The repository includes OpenOCD
configuration files in the `openocd/` directory for each MCU family.
Refer to the [OpenOCD documentation](https://openocd.org/doc/html/Flash-Commands.html) for details.

### Generating Documentation

You will need [Doxygen](https://www.doxygen.nl/index.html).

- **macOS**: `brew install doxygen`
- **Ubuntu**: `sudo apt install doxygen`
- **Arch**: `sudo pacman -S doxygen`
- **Other Linux**: Use prebuilt binaries or build from source following the [compile manual](https://www.doxygen.nl/manual/install.html).

To generate documentation after building the project:

- Run `make doc` in the `build/` directory
- On Windows, run `ninja doc` in the `build/` directory

To view the generated documentation:

- Run `firefox docs/html/index.html`, or
- Open `docs/html/index.html` in your browser.

## Developer Guide

Follow the guidelines below when contributing to this repository.

### Editing the Code

You can use any editor, but we recommend [CLion](https://www.jetbrains.com/clion/).

### Formatting Code

The continuous integration system will check the source code against a specific coding style. If the code does not follow the style, the formatting check will fail and the code will not be merged.
All codes are required to be formatted correctly before merging. There are several integrated build commands that can help you automatically format your changes.

**Prerequisite**: install `clang-format` **18.1.8**. CMake will not create the format target if `clang-format` is missing.

* For Linux users:

  * Prefer the pinned LLVM binary (matches CI / macOS Homebrew `18.1.8`):
    [x86_64](https://github.com/llvm/llvm-project/releases/download/llvmorg-18.1.8/clang+llvm-18.1.8-x86_64-linux-gnu-ubuntu-18.04.tar.xz)
    [aarch64](https://github.com/llvm/llvm-project/releases/download/llvmorg-18.1.8/clang+llvm-18.1.8-aarch64-linux-gnu.tar.xz)
    ```bash
    tar -xf clang+llvm-18.1.8-*.tar.xz
    export PATH=$PWD/clang+llvm-18.1.8-*/bin:$PATH
    ```
  * Or: `pip install clang-format==18.1.8`
  * Avoid `apt install clang-format-18` on Ubuntu 24.04 — that package is **18.1.3**, not 18.1.8.
  
* For Mac users:

  * Recommend: `brew install llvm@18` then ensure `clang-format` 18.1.8 is on `PATH`
  * Or official package:
    [Apple Silicon](https://github.com/llvm/llvm-project/releases/download/llvmorg-18.1.8/clang+llvm-18.1.8-arm64-apple-macos11.tar.xz)
    ```bash
    tar -xf clang+llvm-18.1.8-arm64-apple-macos11.tar.xz
    export PATH=$PWD/clang+llvm-18.1.8-arm64-apple-macos11/bin:$PATH
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

### Contributing

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
