# Introduction

- Forthink ESP32 UWB contains an ESP32 and a NCJ29D5 chip now, we are also actively striving to be compatible with chips on the market, and we hope that we can be compatible with other UWB chips in the near future.

- Ultra-wideband (UWB) is a short-range wireless communication protocol that operates via radio waves. It provides secure and reliable ranging and precision sensing, creating a new spatial context for wireless devices.

- The Forthink UWB module, based on the NXP NCJ29D5 chip, has gained significant popularity among Makers. The NCJ29D5 is an automotive-grade chip widely used in numerous UWB digital key systems.

- As an automotive-grade UWB chip, the NCJ29D5 offers features suitable for the automotive environment, such as resistance to interference and high-temperature tolerance. It ensures fast and reliable vehicle recognition, allowing only authorized digital keys to unlock and start the vehicle. The widespread use of this technology has improved vehicle security and user convenience.

# Advantages:


| Advantage          |  Description    | 
| :----------------: | :-------------: |
|Interoperability with Apple U1 chip | Enables compatibility with the Apple system|
|Alignment with FiRa™ PHY, MAC, and certification development | Makes it more suitable for further applications|
|Compliance with the Car Connectivity Consortium (CCC) | Ensures interoperability with the car access ecosystem|
|Support for UWB channels 5 (6.5 GHz) and 9 (8 GHz) | DWM1000 does not support Channel 9|



# Contact 

- Email：dksupports@everhigh.com.cn

- Telegram：@crazepony


# Hardware requirements
- Arduino board base on esp32, such as board from Heltec.
- UWB module with standard uci base on FiRa.

# Examples 

| Example          |  Protocol support   | Library require    | Minimum uint need     | 
| :--------------- | :-----------------: | :----------------: | :-------------------: |
|range-FiRa-apple | FiRa|BLEServer, Adafruit_GFX, Adafruit_ST7789| 1
|range-FiRa-gui|FiRa| Adafruit_GFX, Adafruit_ST7789 | 2
|range-FiRa-Serial|FiRa| - |2
|rtls-ccc-gui|CCC|Adafruit_GFX, Adafruit_ST7789, EspMQTTClient, ArduinoJson|4
|rtls-FiRa-gui|FiRa|Adafruit_GFX, Adafruit_ST7789, EspMQTTClient, ArduinoJson|4

## Install Library
- Clone this library to your Arduino document path:

```
git clone git@github.com:forthink-xyz/esp32-uwb-release.git
```
- Install the library requirments depends on the recommendation from Arduino IDE




# Release log
- (2024.06.04) - v0.2.101
Features:
  - Nearby interaction support
  - Range base FiRa
  - Range base CCC
  - Examples merge and code clean 



