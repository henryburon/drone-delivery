## Arduino Nano with LoRa module (RFM95)

The different parts of the system (droid, winch/drone, base station) will communicate via LoRa. Flash each Arduino Nano with the corresponding file in `src`.  

Requires the [RadioHead](https://www.airspayce.com/mikem/arduino/RadioHead/) (v. 1.120) library by Mike McCauley.

* Set up the Platform.io project with the Arduino Nano ATmega328 (New Bootloader) board.
* Only files in `src`, not its subdirectories, will be compiled and flashed by Platform.io.

