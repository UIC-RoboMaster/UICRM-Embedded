# UIC RoboMaster Embedded

![arm](https://github.com/UIC-RoboMaster/UICRM-Embedded/workflows/arm%20build/badge.svg)

Embedded system development @ BNU-HKBU UIC RoboMaster

## User Guide

You can follow the instructions below to set up the necessary environments for
building the source code and flashing the embedded chips.

### Set Up Environment

**Install ARM Toolchain (manual)**

1. Go to the [official download page](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) for ARM Toolchain.
2. Download the pre-built toolchain according to your operating system.
3. Decompress it to some directory and find an absolute path to the `bin` directory.

    In my case: `/Users/yry0008/gcc-arm-none-eabi-10.3-2021.10/bin`.

4. For Windows users, add the following line (replace `<path>` with the actual binary path found in step 3) to `PATH` environment variable.

    For Linux / Mac users, add the following line (replace `<path>` with the actual binary path found in step 3) to `~/.bashrc` for bash users or `~/.zshrc` for zsh users.

    ```sh
    export PATH=<path>:$PATH
    ```

**Install OpenOCD (manual)**
1. Go to the [official download page](https://gnutoolchains.com/arm-eabi/openocd/) for OpenOCD.
2. Download the pre-built toolchain according to your operating system.
3. Decompress it to some directory and find an absolute path to the `bin` directory.

    In my case: `/Users/yry0008/openocd-0.11.0-2021.10/bin`.
4. For Windows users, add the following line (replace `<path>` with the actual binary path found in step 3) to `PATH` environment variable. For Linux / Mac users, add the following line (replace `<path>` with the actual binary path found in step 3) to `~/.bashrc` for bash users or `~/.zshrc` for zsh users.

    ```sh
    export PATH=<path>:$PATH
    ```

**Install CMake**
1. Go to the [official download page](https://cmake.org/download/) for CMake.

> If you are using Clion, this step is not required.
   
**Install Ninja (Windows only)**
1. Go to the [official download page](https://ninja-build.org)

### Compile Project

**With CLion (Recommended)**

You can directly open the project in CLion and build it.
You need to set the path of the embedded toolchain in the CLion settings.

    In Windows, you should open `Settings`, `Build, Execution, Deployment`, `CMake`, then set the `Generator` to Ninja.

**Compile manually**

1. Go to your project root directory in a terminal.
2. Run the following command to build the entire project.

    ```sh
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j
    ```
    In Windows, you should add the option to let cmake use ninja to build.
    ```sh
    cmake -DCMAKE_BUILD_TYPE=Release ... -G "Ninja"
    ```
    Using ninja to build.
    ```sh
    ninja -j
    ```
   
    Change build type to `Debug` or `RelWithDebInfo` in order to debug with `gdb`. Note that `Debug` build could be much slower than the other two due to lack of compiler optimizations.

### Flash Binary to Chip

**Flash using CLion**

Choose the target you want to flash and click the `Run` button.

The default configuration is for CMSIS-DAP debugger. If you are using ST-LINK,
you need to change the configuration in the CLion settings.

**Flash using OpenOCD**
TODO

### Generate document

You will need [Doxygen](https://www.doxygen.nl/index.html).

1. For Mac users, `brew install doxygen` could be a shortcut.
2. For Ubuntu users, `sudo apt install doxygen` could be a shortcut.
3. For Arch users, `sudo pacman -S doxygen` could be a shortcut.
4. For Linux users, either use prebuilt binaries, or build from source following their compile manual.

To generate documentations after compiling the project.

- Run `make doc` in the `build/` directory
- In windows, you need to run `ninja doc` in the `build/` directory

To view the generated document:

- Run `firefox docs/html/index.html`, or
- Open `docs/html/index.html` in your browser.

## Developer Guide

Use the following guide when making contributions to this repo.

### Edit the code
You can use any editor you like, but we recommend using [CLion](https://www.jetbrains.com/clion/).

### Format Code

The continuous integration system will check the source code against
a specific coding style. If the code does not follow the style, the
formatting check will fail and the code will not be merged.
All codes are required to be formatted correctly before merging. There are several
integrated build commands that can help you automatically format your changes.

**Prerequisite**: install `clang-format`. (otherwise CMake will not create the format target)

* Linux's users can simply install it using `sudo apt install clang-format-10`.
* Mac and Windows users need to download prebuilt binaries from [here](https://releases.llvm.org/download.html).

**Format using CLion**

Choose the CMake target and compile it. CLion will automatically format the code for you.
1. `check-format`: Check `diff` between current source and formatted source (without modifying any source file)
2. `format`: Format all source files (**Modifies** file in place)

**Format manually**

You can run the following commands inside `build/` to format your changes.

1. `make check-format`: Check `diff` between current source and formatted source (without modifying any source file)
2. `make format`: Format all source files (**Modifies** file in place)

### Debug with `gdb`

To debug embedded systems on a host machine, we would need a remote gdb server.
There are 2 choices for such server, with tradeoffs of their own.

* **`Clion Debugger`**
    
This is the easiest way to debug. Choose the target and Directly click the `Debug` button in CLion.

* **`OpenOCD`**

Thought directly using `openocd` is possible, but it is only recommended for advanced users.

### Contribute to this repo

The main branch is protected. You need to create a new branch and make a pull request to merge your changes. You need to
<u>pass the CI check (formatting check and build check)</u> before merging.