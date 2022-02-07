# C2000 openinverter

[![Build status](../../actions/workflows/CI-build.yml/badge.svg)](../../actions/workflows/CI-build.yml)

A port of the Huebner inverter project to the [TI C2000](https://www.ti.com/microcontrollers-mcus-processors/microcontrollers/c2000-real-time-control-mcus/overview.html) family of micro-controllers. Specifically this targets the MCU found in the Tesla Model 3 / Y inverter allowing. Eventually this will allow a completely Open Source solution to running Tesla Model 3 / Y drive units in non-Tesla vehicles. For other Tesla drive units, inverters from other manufacturers and DIY inverters have a look at the [openinverter project](https://openinverter.org).

## Goals

The main goal of this firmware is to create a usable and safe control system for Tesla Model 3 / Y drive units. A secondary goal is to create a more portable implementation of the openinverter algorithms that can be tested and/or simulated outside of embedded hardware platforms.

## Non-Goals

This firmware does not aim to replace existing openinverter firmware for STM32 platforms.

The firmware build process does target the STM32F1 processor used in existing openinverter designs. This is to allow validation of changes and refactoring. Using the STM32 builds on a real motor is at the users own risk!

## Hardware Platforms

The current hardware targets are:

* Tesla Model 3 / Y rear drive units 1120980-00-G, 1120990-00-G - using a [JTAG cable](docs/Tesla-M3-JTAG-cable.md)
* Texas Instruments LAUNCHXL-F28379D development board
* All existing STM32F103 based openinverter boards (for verification only)
* x86_64 Linux (unit tests, code coverage and utilities)

## Status

* [x] Port core libopeninv code to work on x86_64 and C2000 with unit tests
* [x] Hardware independent, statically virtualised Field Oriented Control / Sine PWM generation
* [x] Port and update existing unit tests and extend to verify FOC PWM generation
* [x] Fix integer overflows affecting C2000 in PWM generation, SineCore and libopeninv
* [x] Support operation on Tesla M3 rear drive unit hardware
* [x] Tesla M3 gate driver integration
* [ ] Tesla M3 safety PSU/PMIC integration
* [ ] Functional PWM generation on Tesla M3 hardware
* [ ] Cross-platform resolver to digital conversion and Tesla M3 integration
* [ ] Analogue sampling of phase currents for FOC
* [ ] Analogue capture of other Tesla M3 signals
* [ ] Tesla M3 inverter temperature monitoring
* [ ] Over-current and over/under-voltage detection
* [ ] Basic vehicle and throttle integration
* [ ] openinverter compatible CAN support
* [ ] CAN control of openinverter serial parameters and commands
* [ ] CAN firmware upgrade
* [ ] Storing system parameters in Flash
* [ ] High Voltage InterLock support

## Compiling

The build process currently assumes you are running Linux. It may be possible to support Windows and MacOS but not at this time.

### Install Build Tools

You will need to install a Texas Instruments C2000 compiler. This may be the standalone [C2000-CGT](https://www.ti.com/tool/C2000-CGT) or the [Code Composer Studio IDE](https://www.ti.com/tool/CCSTUDIO). The code assumes version 21.6.0.LTS of the compiler is installed and **on the system $PATH**.

The project has a number of build dependencies to install these:
On Fedora run

```
    sudo dnf install git arm-none-eabi-gcc-cs-c++ cmake ninja-build lcov
```

On Ubuntu run

```
    sudo apt-get install git gcc-arm-none-eabi cmake ninja-build lcov
```

### Build Process

The build process uses CMake 3.20 or later and assumes the host compiler is GCC 10.x or later.

Before building any platform the git submodules need to be downloaded and openinverter libopencm3 build for STM32 compiled. To do this:
`make get-deps`

To build for Linux:

```
    mkdir -p build/host
    cd build/host
    cmake --preset default ../..
    cmake --build .
```

To build for C2000:

```
    mkdir -p build/c2000
    cd build/c2000
    cmake --preset c2000 ../..
    cmake --build .
```

To build for STM32F1:

```
    make get-deps # To build libopencm3 dependency
    mkdir -p build/stm32f1
    cd build/stm32f1
    cmake --preset stm32f1 ../..
    cmake --build .
```

## Visual Studio Code Integration

The build process automatically integrates with Visual Studio Code. It is possible to pick the CMake presets above from the UI provided the CMake Visual Studio Code extension is installed. A `launch.json` has been provided with targets for Linux and STM32 executables. A `tasks.json` has been provided to build the STM32 build using the legacy Makefile.

## Code Composer Studio Integration

For now the firmware for the C2000 platform is configured to run out of RAM only. To use this the TI Code Composer Studio IDE is required to load the firmware onto the target device and run it in the supplied debugger. To do this a new project needs to be set up within Code Composer Studio.

At a Linux terminal:
```
    mkdir ~/my-c2000-build-project
    cd ~/my-c2000-build-project
    cmake -G "Eclipse CDT4 - Ninja" -DPLATFORM=c2000 -DCMAKE_BUILD_TYPE=Release <my_git_clone_location>
```

Open Code Composer Studio and select `File | Open Projects From Filesystem...` and enter `~/my-c2000-build-project` as the source path. Once the project import is complete you should have a project called `OpenInverter-Release@my-c2000-build-project` in your workspace. You should be able to build this by right-click the project and selecting `Build Project`.

To debug the inverter a new configuration must be created. To do this:

* Select `Run | Debug Configurations...` menu
* Press the `New Launch Configuration` toolbar
* Change the `Name` field to `c2000-sine inverter`
* Check the `Use default target configuration`
* Select the `Program` tab
* For the `Device` field pick `Texas Instruments XDS100v2 USB Debug Probe_0/C28xx_CPU1`
* For the `Target Configuration` field press `Workspace...` and pick `c2000-sine`
* For the `Program` field enter `${workspace_loc:/c2000-sine/platform/c2000/inverter/inverter}`
* Select the `Target` tab
* Select the `Flash Settings` section
* Ensure the `Download Settings` option is set to `Load RAM only`.  **Failure to do this will wipe any Flash memory on the device rendering it inoperable**
* Press `Apply` and `Debug` to start the debug session

This process needs to be repeated for each program to be debugged changing the Program path for each new configuration.

## Legacy Build Process

For now the legacy `Makefile` build process used by openinverter is retained to enable easier merges.

Now you can compile stm32-sine by typing

`make`

or

`CONTROL=FOC make`

to build the FOC version for synchronous motors.

And upload it to your board using a JTAG/SWD adapter, the [updater.py](https://github.com/jsphuebner/tumanako-inverter-fw-bootloader/blob/master/updater.py) script or the esp8266 web interface.

## License

This software is licensed with the GPL v3 as detailed in [LICENSE](LICENSE).

Additionally some C2000 specific code in `platform/c2000/driverlib` and `platform/c2000/device_support` has this additional license:

```
Copyright (C) 2013-2021 Texas Instruments Incorporated - http://www.ti.com/
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the
  distribution.

  Neither the name of Texas Instruments Incorporated nor the names of
  its contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
