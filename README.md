# 1.Introduction

- ESP32 UWB contains an ESP32 and a NCJ29D5 chip, has gained significant popularity among Makers, Wifi and BLE integrated, make Iot connections are easy enough.
  
- The NCJ29D5 is an automotive-grade chip widely used in numerous UWB digital key systems and other automotive environment.

- This significantly lowers the testing threshold for some civil and industrial applications.

- FiRa and CCC compatible.

# 2.Advantages:


| Advantage          |  Description    | 
| :----------------: | :-------------: |
|Interoperability with Apple U1 chip | Enables compatibility with iOS|
|Alignment with FiRa™ PHY, MAC | Makes it more suitable for further applications|
|Car Connectivity Consortium (CCC) compatible| Ensures interoperability with the car access ecosystem|
|Channels 5 (6.5 GHz) and 9 (8 GHz) support | DWM1000 does not support Channel 9|

# 3.Contact 

| Telegram           | Email                   |
| :--------------- | :-----------------:       |
|@crazepony        |dksupports@everhigh.com.cn |

# 4.Hardware requirements
- ESP32-UWB zero, An Arduino board from ```Heltec```
- UWB module with a standard UCI feature under FiRa.

# 5.Install Library
- Clone this library to your Arduino document path:

```
git clone git@github.com:forthink-xyz/esp32-uwb-zero.git
```
- Install other necessary library depends on the recommendation from Arduino IDE

# 6.Terms involve

- **CCC**： Car Connectivity Consortium, a organization advancing global technologies for smartphone-to-car connectivity solutions.
- **FiRa**： Fine Ranging, protocol designed for precise positioning and secure distance measurement in UWB technology.
- **SS-TWR**： Single-Sided Two-Way Ranging, is a ranging method in UWB systems that requires only one device to be clock-synchronized.
- **DS-TWR**： Double-Sided Two-Way Ranging , is a high-precision ranging method, requiring clock synchronization between two devices.
- **Initator**： A device that starts the communication or ranging process, sending the first signal in a sequence.
- **Responder**：A device that receives the initial signal from the Initiator, and sends a response back for distance measurement.
- **ESP32-UWB zero**： An Arduino board base on ESP32 release from Heltec.

# 7.Examples 

| Example          |     Protocol support     |     Library require     |     Minimum uint(s) need     |     Description     |
| :--------------- | :----------------------: | :---------------------: | :--------------------------: | :-----------------: |
| range-FiRa-apple |           FiRa           | BLEServer, Adafruit_GFX, Adafruit_ST7789 |             1             | Nearby Interaction demo |
| range-FiRa-gui   |           FiRa           | Adafruit_GFX, Adafruit_ST7789 |             2             | Simple range demo between 2 nodes under FiRa with LCD display, specific board dependencies|
| range-FiRa-Serial|           FiRa           | - |             2             |  Simple range demo between 2 nodes under FiRa with serial ouput, only Arduino base environment dependency, highly portable|
| rtls-ccc-gui     |           CCC            | Adafruit_GFX, Adafruit_ST7789, EspMQTTClient, ArduinoJson |             4             | Simple rtls demo between 1 initator and 3 responders under CCC, SS-TWR, car key simulation, need a mqtt broker and a host client to display the location  |
| rtls-FiRa-gui    |           FiRa           | Adafruit_GFX, Adafruit_ST7789, EspMQTTClient, ArduinoJson |             4             | Simple rtls demo between 1 initator and 3 responders under FiRa, DS-TWR, need a mqtt broker and a host client to display the location|



# 8.Usages
- ## 8.1 range-FiRa-apple

| Hardware requirments    |  Quantity requirments   | Range mode |
| :---------------------- | :-----------------: | :-----------------: |
|iPhone 11 with iOS 16.0 or above| 1 unit | DS-TWR |
|`ESP32-UWB zero` with `range-FiRa-apple` firmware  | At least 1 unit| DS-TWR |

- #### Assume that an iPhone and a `ESP32-UWB zero` got already.

     - Install a App called ```Qorvo Nearby Interaction``` on your iPhone;
      
     - Stay the Bluetooth alive on your iPhone;
      
     - Flush the firmware of ```range-FiRa-apple``` to ```ESP32-UWB zero``` with your Arduino IDE;
      
     - Hopefully you will see an accessory on ```Qorvo Nearby Interaction``` named ```uwb zero (d05efbbc) ```if every thing goes well.
      
     - Click the 'connect' button, then move around and you may get the distance and angle from iPhone to  ```ESP32-UWB zero```

```Note1:``` Because of some limitations from Apple, we only touch accessory around iPhone 9 meters maximum.

```Note2:``` Change nothing in source code ```range-FiRa-apple.ino```, just flushing firmware to the board directly.


- ## 8.2 range-FiRa-gui

| Hardware requirments    |  Quantity requirments   | Range mode |
| :---------------------- | :-----------------: | :-----------------: |
|```ESP32-UWB zero``` with ```range-FiRa-gui``` firmware  | At least 2 units| DS-TWR |

- #### Assume that there are 2 ```ESP32-UWB zero``` units you have got already. 
  
  - Power them on and record the mac addres on A4 paper or something like this, mac show on LCD.
  
  - Via step 1, assume that we get mac address ```0xab44``` and ```0xab33```.
   
  - Trun to Arduino IDE, I think you open ```range-FiRa-gui.ino``` already.
   
  - Find the definition as below begin of the ```range-FiRa-gui.ino``` file.

   ```
   
   static const uint16_t MAC_RESPOR_1 = 0xfbbe;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
   static const uint16_t MAC_RESPOR_2 = 0xfbbc;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
   static const uint16_t MAC_RESPOR_3 = 0xfbb2;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
   static const uint16_t MAC_RESPOR_4 = 0xfbaf;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
   
   ```

  - Modify 2 of them to fit your real mac, just as below, I change `MAC_RESPOR_1` and `MAC_RESPOR_2` to `0xab44`, `0xab33` here :
   ```
   
   static const uint16_t MAC_RESPOR_1 = 0xab44;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
   static const uint16_t MAC_RESPOR_2 = 0xab33;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
   static const uint16_t MAC_RESPOR_3 = 0xfbb2;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
   static const uint16_t MAC_RESPOR_4 = 0xfbaf;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
   
   ```

  - Flush the same firmware to board ```0xab44``` and  ```0xab33```.
   
  - Assume that  ```0xab44``` as Initator, ```0xab33``` as responder, GPIO35 is a switch for the node role. 
   
  - For board ```0xab44``` , short GPIO35 to 3.3v (in order to make some convenient to debug, initator has a fixed mac as 0xabcd actually).
   
  - For board ```0xab33``` , let GPIO35 floating.

  - Finally, Power board ```0xab44``` and board ```0xab33``` on, something that you expected will display on the LCD hopefully.

- ## 8.3 range-FiRa-Serial

  Same as `range-FiRa-gui` tutorial, `range-FiRa-Serial` have no LCD and log the range information through serial port.

- ## 8.4 rtls-ccc-gui

| Hardware requirments    |  Quantity requirments   | Range mode | 
| :---------------------- | :-----------------: | :-----------------: |
|```ESP32-UWB zero``` with ```rtls-ccc-gui``` firmware  | At least 4 units| SS-TWR |

- #### Assume that there are 4 ```ESP32-UWB zero``` units you have got already, let 1 node as initator while other 3 nodes as responder.
  
  - For initator, we only modify the definition as below：
  
   ```
   
   static const uint8_t  REPONDER_NUM   = 3;//the number of the responder nodes in the network
   
   ```

  - For responder n (n = 0,1,2), we only modify the definitions as below：
  
   ```
   
   static const uint8_t    REPONDER_NUM   = 3;//the number of the responder nodes in the network
   static const uint16_t   REPONDER_INDEX = n;//the index of this responder node in the network
   
   ```
 
   - Additional, for responder role, modify the parameters of Wifi connection and mqtt broker as below：

   ```
   static char*            WIFI_SSID   = "HGKJ";
   static char*            WIFI_PSWD   = "HGKJ2014";
   static String           TopicUp     = "forthink/uwb/up/";
   static String           TopicDown   = "forthink/uwb/down/";
   static const char*      MQTT_BROKER = "esp32.bitpony.xyz";
   static const uint16_t   MQTT_PORT   = 1883;
   ```


```Note1:``` We have to modify the `REPONDER_INDEX` manually for each reponder， such as n = 0,1,2

```Note2:``` Range data transfer via Wifi to a mqtt broker, you can build a broker by yourself and calculate the position through TOF algorithm.


- ## 8.5 rtls-FiRa-gui

| Hardware requirments    |  Quantity requirments   | Range mode |
| :---------------------- | :-----------------: | :-----------------: |
|```ESP32-UWB zero``` with ```rtls-FiRa-gui``` firmware  | At least 4 units| DS-TWR |

- #### Same as ```range-FiRa-gui``` tutorial, ```rtls-FiRa-gui``` require 4 units at least, let 1 node as initator while other 3 nodes as responder.

   - Additional, for responder role, modify the parameters of Wifi connection and mqtt broker as below：

   ```
   static char*            WIFI_SSID   = "HGKJ";
   static char*            WIFI_PSWD   = "HGKJ2014";
   static String           TopicUp     = "forthink/uwb/up/";
   static String           TopicDown   = "forthink/uwb/down/";
   static const char*      MQTT_BROKER = "esp32.bitpony.xyz";
   static const uint16_t   MQTT_PORT   = 1883;
   ```

```Note1:``` Range data transfer via mqtt broker, you can build a broker by yourself and calculate the position through TOF algorithm.

# 9. Release log
## (2024.06.06) - v0.2.103
- Features:
  - None
- Fixed:
  - add tlv tag name echo.
  - improve readme.md
  - known issues fixed.
- Add:
  - None
  
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





