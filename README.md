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

| Telegram           | Email    |
| :--------------- | :-----------------: |
|@crazepony|dksupports@everhigh.com.cn |

# Hardware requirements
- Arduino board base on esp32, such as board from ```Heltec```, ESP32-UWB zero.
- UWB module with standard uci base on FiRa.

# Install Library
- Clone this library to your Arduino document path:

```
git clone git@github.com:forthink-xyz/esp32-uwb-zero.git
```
- Install the library requirments depends on the recommendation from Arduino IDE

# Terms involve

- ```CCC```： Car Connectivity Consortium, a cross-industry organization advancing global technologies for smartphone-to-car connectivity solutions.
- ```FiRa```： Fine Ranging, is a protocol designed for precise positioning and secure distance measurement in UWB technology.
- ```SS-TWR```： Single-Sided Two-Way Ranging, is a ranging method in UWB systems that requires only one device to be clock-synchronized, offering lower hardware requirements.
- ```DS-TWR```： Double-Sided Two-Way Ranging , is a high-precision ranging method in UWB systems, requiring clock synchronization between two devices.
- ```Initator```： A device that starts the communication or ranging process, sending the first signal in a sequence.
- ```Responder```：A device that receives the initial signal from the Initiator, and sends a response back for distance measurement.
- ```ESP32-UWB zero```： An Arduino board base on ESP32 release from Heltec.

# Examples 

| Example          |     Protocol support     |     Library require     |     Minimum uint(s) need     |     Description     |
| :--------------- | :----------------------: | :---------------------: | :--------------------------: | :-----------------: |
| range-FiRa-apple |           FiRa           | BLEServer, Adafruit_GFX, Adafruit_ST7789 |             1             | Nearby Interaction demo |
| range-FiRa-gui   |           FiRa           | Adafruit_GFX, Adafruit_ST7789 |             2             | Simple range demo between 2 nodes under FiRa with LCD display, specific board dependencies|
| range-FiRa-Serial|           FiRa           | - |             2             |  Simple range demo between 2 nodes under FiRa with serial ouput, only Arduino base environment dependency, highly portable|
| rtls-ccc-gui     |           CCC            | Adafruit_GFX, Adafruit_ST7789, EspMQTTClient, ArduinoJson |             4             | Simple rtls demo between 1 initator and 3 responders under CCC, SS-TWR, car key simulation, need a mqtt broker and a host client to display the location  |
| rtls-FiRa-gui    |           FiRa           | Adafruit_GFX, Adafruit_ST7789, EspMQTTClient, ArduinoJson |             4             | Simple rtls demo between 1 initator and 3 responders under FiRa, DS-TWR, need a mqtt broker and a host client to display the location|



## Usages
- ### range-FiRa-apple

| Hardware requirments    |  Quantity requirments   | 
| :---------------------- | :-----------------: |
|iPhone 11 with iOS 16.0 or above| 1 unit |
|```ESP32-UWB zero``` with ```range-FiRa-apple``` firmware  | At least 1 unit|


1. Install the App called ```Qorvo Nearby Interaction``` on your iPhone;
   
2. Stay the Bluetooth alive on your iPhone;
   
3. Flush the firmware of ```range-FiRa-apple``` to ```ESP32-UWB zero``` with your Arduino IDE;
   
4. Hopefully you can see an accessory on ```Qorvo Nearby Interaction``` named ```uwb zero (d05efbbc) ```if every thing goes well.
   
5. Click the 'connect' button, then move around and you may get the distance and angle from iPhone to  ```ESP32-UWB zero```

```Note1:``` Because of some limitations from Apple, we can only detect accessory around iPhone 9 meters maximum.

```Note2:``` Change nothing in source code ```range-FiRa-apple.ino```, just flushing firmware to the board directly.


- ### range-FiRa-gui

| Hardware requirments    |  Quantity requirments   | 
| :---------------------- | :-----------------: |
|```ESP32-UWB zero``` with ```range-FiRa-gui``` firmware  | At least 2 units|

  Assume that there are 2 ```ESP32-UWB zero``` units you have got already. 
  
1.  We have to get the mac address both of them in advance, power them on and record the mac addres (show LCD) on A4 paper or something like this.
  
2. Via step 1, assume that we get mac address ```0xab44``` and ```0xab33```.
   
3. Trun to Arduino IDE, I think you open ```range-FiRa-gui.ino``` already.
   
4. Find the definition as below begin of the ```range-FiRa-gui.ino``` file.

```

static const uint16_t MAC_RESPOR_1 = 0xfbbe;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
static const uint16_t MAC_RESPOR_2 = 0xfbbc;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
static const uint16_t MAC_RESPOR_3 = 0xfbb2;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
static const uint16_t MAC_RESPOR_4 = 0xfbaf;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder

```

5. Modify the definition as below, remove 2 definition and remain only 2 suit for your environment, if you got 4 ```ESP32-UWB zero``` units, just remain 4 definitions here：
```

static const uint16_t MAC_RESPOR_1 = 0xab44;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
static const uint16_t MAC_RESPOR_2 = 0xab33;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder

```

6. Flush the same firmware to board ```0xab44``` and  ```0xab33```.
   
8. Use a jumper to short 3.3v and GPIO35, that means this node is an initator, and let GPIO35 floating means this node is a responder.

9. Finally, Power ```0xab44``` and ``0xab33``` on, hopefully, something you expect display on the LCD.

- ### range-FiRa-Serial

  Same as ```range-FiRa-gui``` tutorial, ```range-FiRa-Serial``` have no LCD and log the range through serial port.

- ### rtls-ccc-gui

| Hardware requirments    |  Quantity requirments   | 
| :---------------------- | :-----------------: |
|```ESP32-UWB zero``` with ```rtls-ccc-gui``` firmware  | At least 4 units|

  Assume that there are 4 ```ESP32-UWB zero``` units you have got already, 1 node as initator and 3 nodes as responder.
  

  1.For initator, we only modify the 1 definition as below：
  
```

static const uint8_t  REPONDER_NUM   = 3;//the number of the responder nodes in the network

```

  2.For responder n (n = 0,1,2), we only modify the 2 definition as below：
  
```

static const uint8_t    REPONDER_NUM   = 3;//the number of the responder nodes in the network
static const uint16_t   REPONDER_INDEX = n;//the index of this responder node in the network

```

```Note1:``` The firmware for each reponder , we have to modify the ```REPONDER_INDEX``` manually， such as n = 0,1,2

```Note2:``` Range data transfer via mqtt broker, you can build a broker by yourself and calculate the position through TOF algorithm.


- ### rtls-FiRa-gui

| Hardware requirments    |  Quantity requirments   | 
| :---------------------- | :-----------------: |
|```ESP32-UWB zero``` with ```rtls-FiRa-gui``` firmware  | At least 4 units|

  Same as ```range-FiRa-gui``` tutorial, ```rtls-FiRa-gui``` require 4 units at least, so you have left 4 definitions there and just modify the mac address to fit your environment.

```Note1:``` Range data transfer via mqtt broker, you can build a broker by yourself and calculate the position through TOF algorithm.

# Release log
## (2024.06.05) - v0.2.102
  
- Features:
  - Disable ANSI logger feature. (origin Arduino IDE console do not support ANSI)
- Fixed:
  - Session app config, tlv node missing items fixed
- Add:
  - Usages for example *.ino
## (2024.06.04) - v0.2.101

- Features:
  - Nearby interaction support
  - Range base FiRa
  - Range base CCC
  - Examples merge and code clean
- Fixed:
  - None 





