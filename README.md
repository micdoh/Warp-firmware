# Bicycle Computer (based on [Warp](https://github.com/physical-computation/Warp-hardware))

**Name:** Michael Doherty

**College:** N/A

**CRSid:** md934
![Alt text](https://github.com/micdoh/Warp-firmware/blob/Bicycle-computer/doc/letsRide.JPG)
![Alt text](https://github.com/micdoh/Warp-firmware/blob/Bicycle-computer/doc/RPM.JPG)

## Building the application

The below instructions are drawn from the [Warp firmware](https://github.com/physical-computation/Warp-hardware):

**Prerequisites:** You need an arm cross-compiler such as `arm-none-eabi-gcc` installed as well as a working `cmake` (installed, e.g., via `apt-get` on Linux or via [MacPorts](https://www.macports.org) or [Homebrew](https://brew.sh) on macOS). You will also need an installed copy of the SEGGER [JLink commander](https://www.segger.com/downloads/jlink/), `JlinkExe`, which is available for Linux, macOS, and Windows (here are direct links for downloading it for [macOS](https://www.segger.com/downloads/jlink/JLink_MacOSX.pkg), and [Linux tgz 64-bit](https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.tgz)).

### 1. Compiling the application

1. Update `build/ksdk1.1/build.sh` with full path to `bin` directory of arm cross-compiler. Also set environment variable to ARMGCC_DIR to same value on your computer:

    export ARMGCC_DIR=<full-path-to-arm-cross-compiler>

2. Update `tools/scripts/jlink.commands` to directory of executable:

    loadfile <full-path-to-application>/build/ksdk1.1/work/demos/Warp/armgcc/Warp/release/Warp.srec

The application can then be compiled wih the command `make warp` and loaded to your evaluation board with `make load-warp` (commands specified in Makefile in root directory).

### 2. Pin configuration
The evaluation board used is a FRDM KL03Z. Peripherals are MMA8451Q accelerometer (integrated with baord), L3GD20H gyroscope and a SSD1331 display. 
Optionally, an INA219 current sensor is also supported if `WARP_BUILD_BOOT_INA219` is enabled in `config.h`.

OLED display:
```
BOARD		DISPLAY
3.3V	->	VCC
GND	->	GND
PTA8	->	MOSI
PTA9	->	SCK
PTB11	->	OCS
PTB1	->	D/C
PTB0	->	RST
```

MMA8451Q:
```
BOARD		ACCELEROMETER
3.3V	->	VCC
GND	->	GND
PTB3	->	I2C SCL
PTB4	->	I2C SDA
PTA12	->	INT2
```

L3GD20H:
```
BOARD		GYROSCOPE
3.3V	->	VCC
GND	->	GND
PTB3	->	I2C SCL
PTB4	->	I2C SDA
```

INA219 (optional):
```
BOARD		CURRENT SENSOR
3.3V	->	VCC
GND	->	GND
PTB3	->	I2C SCL
PTB4	->	I2C SDA
```

## Using the Bicycle Computer
Mount securely to your handlebars, so you can glance at the screen while you ride.

Remain stationary and on relatively flat surface until boot screen clears after powering on device, to ensure accurate x, y, z acceleration calibration. Cadence/RPM measurements are unaffected.

Tap the board to cycle through display options.

## Source File Descriptions
The section below briefly describes all the source files in the `src/boot/ksdk1.1.0/` directory.
The main file of the application is `src/boot/ksdk1.1.0/boot.c`. 
The drivers for the display are in `devSSD1331.c` and for the sensors in `devMMA8451Q.c`, `devL3GD20H.c`, snd `devINA219.c` (optional), respectively.

##### `boot.c`
Initialise pins and configure sensors. Calculate cadence and/or acceleration in g's on x, y, or z axes. 
By default, screen is updated every 2 seconds with new information. Screen display mode is cycled through by tapping on board.

##### `config.h`
Configuration register value definitions. Also allows optional compilation modes to stream data from the sensors for evaluation.

##### `devSSD1331.*`
Driver for the SSD1331 OLED display.

##### `devMMA8451Q.*`
Driver for the MMA8451Q accelerometer, built-in to the FRDMKL03Z evaluation board.

##### `L3GD20H.*`
Driver for the L3GD20H gyroscope.

##### `devINA219.*`
Driver for the INA219 current sensor.

##### `CMakeLists.txt`
This is the CMake configuration file. Edit this to change the default size of the stack and heap.

##### `SEGGER_RTT.*`
This is the implementation of the SEGGER Real-Time Terminal interface. Do not modify.

##### `SEGGER_RTT_Conf.h`
Configuration file for SEGGER Real-Time Terminal interface. You can increase the size of `BUFFER_SIZE_UP` to reduce text in the menu being trimmed.

##### `SEGGER_RTT_printf.c`
Implementation of the SEGGER Real-Time Terminal interface formatted I/O routines. Do not modify.

##### `gpio_pins.c`
Definition of I/O pin configurations using the KSDK `gpio_output_pin_user_config_t` structure.

##### `gpio_pins.h`
Definition of I/O pin mappings and aliases for different I/O pins to symbolic names relevant to the Warp hardware design, via `GPIO_MAKE_PIN()`.

##### `startup_MKL03Z4.S`
Initialization assembler.

##### `warp.h`
Constant and data structure definitions.

## Acknowledgements
This project was completed as part of the 4B25 Embedded Systems course at the University of Cambridge, taught by Phillip Stanley-Marbell.
#### Reference:
Phillip Stanley-Marbell and Martin Rinard. “A Hardware Platform for Efficient Multi-Modal Sensing with Adaptive Approximation”. ArXiv e-prints (2018). arXiv:1804.09241.
