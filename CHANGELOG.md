# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [alternate fork by CodeKill3r]

## [1.1.2] - 2025-06-12
### Added
* Support for OLED module to display information and status
* Real-time VU-meter
* Adaptve font-size recording timer

## [1.1.1] - 2025-06-07
### Added
* Support for "tiny RTC I2C module" 
((be careful if using with non recharheable battery -- remove components R4/R5/D1 and short R6))
* Menu items to set and check date/time
* Date-time entering on terminal (value range clipped after entering)
### Changed
* If working RTC available use datetime in filename (not just attribute)
### Fixed
* delay 24Bit switch reading to let the pullup stabilize


## [1.1.0] - 2025-06-05
### Added
* Support for YDRP2040 board
* Use YDRP's neopixel LED to indicate status
### Changed
* Module interfaces updated to the most recent version (pico_fatfs / pico_spdif_rx / pico_audio_i2s_32b )
* Updated to SDK 2.1.1
* old RTC lib repaced by aon_timer
* Default bitdepth set to 24bit (easy to change in define)


## [original changes]

## [1.0.1] - 2024-03-30
### Added
* Support Raspberry Pi Pico W to reflect timestamp from NTP
* Store Wi-Fi configuration in user flash (Pico W only)
* Show errors by LED blink
* Have a little blank time before emerging sound at start (PRE_START_SEC)

### Changed
* Introduce FatFs R0.15 (previously R0.14b)
* Confirm bandwidth situation and revise card recommendation (Samsung PRO Plus 256GB is the best currently)

### Fixed
* Fix bug for seek position in case of large size file when closing file

## [1.0.0] - 2024-02-12
* Initial release
