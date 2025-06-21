# Raspberry Pi Pico spdif_recorder

![PCB Front](doc/rec_oled.jpg)

![Display Annotated](doc/Final_annotated.png)

## Overview
* Hi-Res recorder from S/PDIF input to WAV files on microSD card
* Bit resolution: 16bit or 24bit (2ch)
* Sampling frequency: 44.1 KHz, 48.0 KHz, 88.2 KHz, 96.0 KHz (, 176.4 KHz, 192 KHz)
* Recording only, no monitoring or playback functions
* 64bit wave file support (RF64 or Sony Wawe64) with only limited by the SD card's space
* Date-time filename suffix if wireless ntp or RTC module available
* 128x32 OLED display support with timer, VU meter and status indicators
 

## Supported Board and Peripheral Devices
* Raspberry Pi Pico or Raspberry Pi Pico W (rp2040)
* Cheap 3rd party Pi Pico board YD-RP2040
* S/PDIF Coaxial or TOSLINK Rx module (DLR1160 or equivalent)
* tiny RTC I2C module (be aware to modify if used battery is not rechareable)
* microSD cards (recommend SD-XC, V30 cards)
* SSD1306 128x32 OLED display

## RTC module fix for nonrechareable battery
The "tiny RTC I2C module" designed to use LIR/ML2032 and NOT the regular CR2032
Altough the clock IC uses very low current to run so a regular battery should be enough for years, there is a charging cirtuit in the board.
The following modification suggested to use with non rechargeable battery (remove components crossed with red, short the one indicated with green):

![tiny RTC battery mod](doc/tiny_rtc_no_charge.png)


## Pin Assignment & Connection

![Circuit Diagram](doc/pico_record_oled.png)

### S/PDIF Rx
| Pico Pin # | GPIO | Function | Connection |
----|----|----|----
| 35 | GP29 | DATA | from S/PDIF data |

### microSD card

| Pico Pin # | Pin Name | Function | microSD Pin Name | microSD Pin # |
----|----|----|----|----
|  4 | GP2 | SPI0_SCK | CLK | 5 |
|  5 | GP3 | SPI0_TX | CMD | 3 |
|  6 | GP4 | SPI0_RX | DAT0 | 7 |
|  7 | GP5 | SPI0_CSn | CD/DAT3 | 2 |
|  8 | GND | GND | VSS | 6 |
| 36 | 3V3(OUT) | 3.3V | VDD | 4 |

Note:
* As for the wire length between Pico and SD card, short wiring as possible is desired, otherwise errors such as Mount error and Write fail will occur.

### Button/Switch (Optional)
| Pico Pin # | GPIO | Function | Connection |
----|----|----|----
| 9 | GP6 | Input | 16bit/24bit switch |
| - | GP24 | Input | Start/Stop button |
| - | GP23 | Output | Neopixel |

Note
* The 16/24bit switch default to 24bit if not connected (preprocessor can change)
* Start/Stop used with the board USRKEY
* To use the neopixel, you have to bridge the jumper on the YDRP board

### Tiny RTC I2C module
| Pico Pin # | GPIO | Function | Connection | Module pin |
----|----|----|----|----
| 16 | GP12 | I2C0 SDA | SDA | 3 |
| 17 | GP13 | I2C0 SCL | SCL | 2 |
| 40 | Vout | 5V | VCC | 4 |
| 18 | GND | GND | GND | 5 |

### SSD1306 128x32 Display
| Pico Pin # | GPIO | Function | Connection |
----|----|----|----
| 31 | GP26 | I2C1 SDA | SDA |
| 32 | GP27 | I2C1 SCL | SCL |
| 36 | 3V3(OUT) | 3.3V | VCC |
| 18 | GND | GND | GND |



## How to build
* See ["Getting started with Raspberry Pi Pico"](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
* Put "pico-sdk", "pico-examples" and "pico-extras" on the same level with this project folder.
* Set environmental variables for PICO_SDK_PATH, PICO_EXTRAS_PATH and PICO_EXAMPLES_PATH
* Build is confirmed in Developer Command Prompt for VS 2022 and Visual Studio Code on Windows enviroment
* Confirmed with Pico SDK 1.5.1, cmake-3.27.2-windows-x86_64 and gcc-arm-none-eabi-10.3-2021.10-win32
```
> git clone -b 1.5.1 https://github.com/raspberrypi/pico-sdk.git
> cd pico-sdk
> git submodule update -i
> cd ..
> git clone -b sdk-1.5.1 https://github.com/raspberrypi/pico-examples.git
>
> git clone -b sdk-1.5.1 https://github.com/raspberrypi/pico-extras.git
> 
> git clone -b main https://github.com/elehobica/pico_spdif_recorder.git
> cd pico_spdif_recorder
> git submodule update -i
> cd ..
```
* Lanuch "Developer Command Prompt for VS 2022"
```
> cd pico_spdif_recorder
> mkdir build && cd build
> cmake -G "NMake Makefiles" ..
> nmake
```
* Put "pico_spdif_recorder.uf2" on RPI-RP2 drive

## Serial command interface
* Serial interface is available from USB port of Raspberry Pi Pico.

| Key | Function |
----|----
| Space | Recording Start Standby / Recording Stop |
| r | Resolution change 16bit/24bit (default: 24bit)* |
| s | Manual split |
| b | Auto blank split on/off (default: on) |
| v | Verbose for monitoring messages |
| c | Clear WAV file suffix to 1 |
| w | Configure Wi-Fi (for Raspberry Pi Pico W only) |
| t | Set RTC time (and start - if RTC module found) |
| d | Set RTC date (if RTC module found) |
| 6 | Toggle 64bit wave support** |
| q | Query current datetime (if available) |
| h | Help |

Note
* Stops current recoring
** If legacy wave file needed

### Recording start/stop/split features
* Standby recording start while silence and auto start when sound detected
* Auto stop and standby restart when long blank detected
* Auto split of WAV file when short blank detected
* Manual immediate split of WAV file without gap*
* If not using exFAT and 64bit wave support (force classic wave file) there will be splits at 4000MB

Note
* Some(?) SDcard may experience increased workload during closing file (finalizing header descriptor) and starting a new file that causes buffer overflows and missing recordings

### LED indicator
* Slow blink: Recording is on-going.
* Fast blink: Background file proceses to close previous file and prepare next file are on-going. During this term, no command requests can be accepted.
* Repeated Slow blink and Fast blink: indicating errors (see log on serial console.)

### Neopixel indivator
* Blue: Stop mode
* Green: Prepared for recoring, starts when singal arrives
* Red pulsing: Recoring in prograss
* Solid Red: processing (closing) the file - please don't reboot/power off the device just yet

## microSD card
### Card recommendation for Hi-Res recording
* Due to the limitation of single bit SPI interface driven by Raspberry Pi Pico, even with highest class microSD cards (as of 2024), recording in 24bit 176.4 KHz or 192.0 KHz is challenging. It will sometimes causes the drops of audio sampling data. The bandwidth status can be monitored in Verbose mode.
* The most severe situation for bandwidth is to start suceeeding track recording when previous long-time track more than 20 minutes has to be closed.
* With following recommended microSD cards, recording in 24bit 96.0 KHz will be stable as far as experimentally confirmed.
* Format micorSD card in exFAT with [official SD Card Formatter](https://www.sdcard.org/downloads/formatter/) before usage. 

| # | Vendor | Product Name | Part Number | Comment |
----|----|----|----|----
| 1 | Samsung | PRO Plus 256GB | MB-MD256SA | 24bit/192KHz is worth trying for non-long-time tracks. 24bit/96KHz is stable. |
| 2 | SanDisk | Extreme PRO 256GB | SDSQXCD-256G-GN6MA | 24bit/192KHz is too challenging. 24bit/96KHz is stable. |

<img src="doc/samsung-pro-plus-256gb.jpg" width="80" />  <img src="doc/sandisk-extreme-pro-256gb.jpg" width="80" />

### File timestamp
* File timestamp is synchronized to NTP if using Raspberry Pi Pico W and connected to Wi-Fi.
* Or if you use the tiny_RTC module and have been started.
* If valid RTC available (working RTC clock or ntp succeeded) the filename suffix is also the starting date-time.
* Otherwise, timestamp starts from 0:00 a.m. Jan. 1st, 2024 (UTC+0) and the suffix is just a counter from 001.
* exFAT format (thus, > 64GB SD-XC cards) is recommended to reflect timezone.
* In case of FAT32 format, timestamp cannot reflect timezone, therefore, it's always shown as UTC+0 time.

### Card trouble shooting
* In case of card mount error or fundamental access errors, please confirm with [FatFs test](lib/pico_fatfs_customized/test)
