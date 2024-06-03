/**
 * @file range-FiRa-Serial.ino
 * @brief This file contains the code for initializing and using the UWB (Ultra-Wideband) module to perform range measurements.
 * 
 * The code initializes the necessary components such as SPI communication, OLED display, and UWB module.
 * It also provides functions for LED control, serial communication initialization, and role determination.
 * The UWB module is activated using a license key and the range measurements are displayed on the OLED display.
 * 
 * @note This code assumes that the correct license key has been obtained and provided.
 * 
 * @author [bitpony]
 * 
 * @date [2024.04.23]
 */

#include "heltec.h"
#include "forthink.h"
#include "logger.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include "EspMQTTClient.h"
#include <ArduinoJson.h>
#include <SPI.h>
#include "EEPROM.h"


using namespace ftlib;

#define VeX_PIN        5
#define LED_PIN        38
#define ROLE_SW_PIN    35

#define UWB_RST_PIN    8
#define UWB_INT_PIN    14
#define UWB_RDY_PIN    9

#define SPI_MISO       10
#define SPI_MOSI       12
#define SPI_SCLK       11
#define UWB_SPI_SS     13

#define TFT_RST        17
#define TFT_DC         18
#define TFT_SDA        48
#define TFT_SCL        47
#define TFT_CS         21

#define LICENCE_SIZE   128
#define EEPROM_SIZE    LICENCE_SIZE


#define TFT_BACK_COLOR  ST77XX_BLACK



static const uint16_t MAC_RESPOR_1 = 0xfbbe;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
static const uint16_t MAC_RESPOR_2 = 0xfbbc;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
static const uint16_t MAC_RESPOR_3 = 0xfbb2;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
static const uint16_t MAC_RESPOR_4 = 0xfbaf;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder

static const uint16_t MAC_INITOR   = 0xabcd;//set initator mac address to same value for all of the responder nodes, let responder know the mac address of the initator

static uint16_t                    gself_mac  = 0x0000;    // it will be set in the "uwb_init' function, any value is ok except '0x0000', we set it to 'uid' here for test later
static uint64_t                    gsession_id = 0x87654321;//session id for range session, it's would be better to use a random number, here we use a fixed number for test
static NodeRole                    grole       = NodeRole::RESPONDER;//node role, default is responder, it will be set in the role_init function, use the ROLE_SW_PIN to determine the node role
static const int                   spiClk      = 10*1000*1000;

static EspMQTTClient               *mclient    = NULL;//mqtt client object pointer

static UWBHALClass                 *uwb        = NULL;//uwb object pointer
static SPIClass                    *gspi_lcd   = NULL;//spi object pointer for lcd display
static SPIClass                    *gspi_uwb   = NULL;//spi object pointer for uwb module
static Adafruit_ST7789             *tft        = NULL;//lcd object pointer, it's a 240x135 lcd display, Adafruit dependcy

static const char       *gurl         = "https://licenses.forthink.com.cn";
static char             *guid         = NULL;
static char             glicense[LICENCE_SIZE + 1] = {0,};// 128 characters license, the last character is '\0', so 129 bytes in total

static char*            WIFI_SSID     = "HGKJ";
static char*            WIFI_PSWD     = "HGKJ2014";

static String           TopicUp       = "forthink/uwb/up/";
static String           TopicDown     = "forthink/uwb/down/";
static const char*      MQTT_BROKER   = "esp32.bitpony.xyz";
static const uint16_t   MQTT_PORT     = 1883;

static const uint8_t    REPONDER_NUM   = 6;//the number of the responder nodes in the network
static const uint16_t   REPONDER_INDEX = 1;//the index of this responder node in the network



/**
 * Initializes the LED pin.
 */
void led_init(void){
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

/**
 * Function to control the LED using a finite state machine.
 */
void led_fsm(void){
  static uint8_t last_sta = true, cnt = 0;
  if((cnt++)%20 == 0) last_sta = true;
  else last_sta = false;
  digitalWrite(LED_PIN, last_sta);
}

void tft_drawtext(int16_t x, int16_t y, char *text, uint8_t size = 1 ,uint16_t color = ST77XX_WHITE, bool cover = true) {
  tft->setCursor(x, y);
  if(cover) tft->fillRect(x, y, (strlen(text) + 1)*size*6, 8*size, TFT_BACK_COLOR);//Desired text size. 1 is default 6x8, 2 is 12x16, 3 is 18x24, etc
  tft->setTextColor(color);
  tft->setTextSize(size);//Desired text size. 1 is default 6x8, 2 is 12x16, 3 is 18x24, etc
  tft->println(text);
}

/**
 * Initializes the OLED display.
 * Turns on the Vext pin, waits for 100 milliseconds, and then initializes the display.
 */
void lcd_init(void) {
  pinMode(VeX_PIN, OUTPUT);
  digitalWrite(VeX_PIN, true);
  delay(20);

  tft = new Adafruit_ST7789(gspi_lcd, TFT_CS, TFT_DC, TFT_RST);
  tft->init(135, 240); // Init ST7789 240x135
  tft->setRotation(1); 
  tft->fillScreen(TFT_BACK_COLOR);
}

/**
 * Initializes the serial communication and prints the ESP32 Chip ID.
 */
void serial_init(void){
	Serial.begin(115200);
}

/**
 * @brief Initializes the SPI communication for the UWB module.
 * 
 * This function initializes two instances of the SPIClass attached to VSPI respectively.
 * It sets up the SPI pins and configures the slave select pins as outputs.
 */
void spi_init(void){
 //initialise two instances of the SPIClass attached to VSPI respectively
  gspi_uwb = new SPIClass(FSPI);
  gspi_uwb->begin(SPI_SCLK, SPI_MISO, SPI_MOSI, UWB_SPI_SS);
  //set up slave select pins as outputs as the Arduino API
  pinMode(gspi_uwb->pinSS(), OUTPUT); 

  //initialise the spi bus for the lcd display
  gspi_lcd = new SPIClass(HSPI);
  gspi_lcd->begin(TFT_SCL, -1, TFT_SDA, TFT_CS);
  //set up slave select pins as outputs as the Arduino API
  pinMode(gspi_lcd->pinSS(), OUTPUT); 
}

/**
 * Initializes the role of the node based on the state of the role switch pin.
 * If the switch pin is pulled high, the node role is set to INITIATOR.
 * If the switch pin is pulled low or in an undefined state, the node role is set to RESPONDER by default.
 */
void role_init(void){
  //role control pin
  pinMode(ROLE_SW_PIN, INPUT_PULLDOWN); 
  //check pin to determine the node role
  //default low，   role to respondor,
  //if pull high,   set node to initator
  uint8_t state = digitalRead(ROLE_SW_PIN);
  grole = (state == HIGH) ? NodeRole::INITATOR : NodeRole::RESPONDER;
  const char* pinfo = (grole == NodeRole::INITATOR) ? "Initator":"Responder";
  LOG_I("set node role to [%s]", pinfo);
}


/**
 * Waits for a license from the serial port and saves it to EEPROM.
 * 
 * @param plic Pointer to a character array to store the license.
 * @return Returns true if the license was successfully saved, false otherwise.
 */
bool license_wait_and_save_from_serial(char* plic){
  while (true)
    {   
      uint16_t len = Serial.available();
      if (len == sizeof(glicense) - 1) 
      {
          memset(plic, 0, sizeof(glicense));
          Serial.readBytes((uint8_t*)plic, len);
          for(int i = 0 ; i < sizeof(glicense) - 1; i++){
            if(isHexadecimalDigit(plic[i]) == false){
              LOG_E("hexadecimal license character[%d] is needed!, unvalid license character ascii: %02x", i, plic[i]);
              memset(plic, 0, sizeof(glicense));
              return false;
            }
            EEPROM.write(i, (uint8_t)plic[i]);
          }

          if(EEPROM.commit()){
            LOG_I("save license: %s", plic);
            break;
          }else{
            LOG_E("Failed to save license to EEPROM, %s", plic);
          }
      }
      else if (len > 0){
          LOG_E("License length %d not correct, it should be %d hex characters", len, sizeof(glicense) - 1);
      }
      LOG_W("Wait license (%d hex characters) from serial port...", sizeof(glicense) - 1);
      delay(1000);
    }
  return true;
}

/**
 * @brief Try to loads the license from EEPROM , or wait for a new license from serial port.
 * 
 * This function loads the license from the EEPROM memory. If the EEPROM is not initialized, it logs an error message and sets the license to an empty string. 
 * If the 'flush' parameter is set to true, it waits for a new license input and saves it to the EEPROM. 
 * If 'flush' is false, it checks the validity of the license stored in the EEPROM. If the license is not valid, it logs an error message and sets the license to an empty string. 
 * If the license is valid, it logs an information message and returns the loaded license.
 * 
 * @param plic Pointer to the character array where the license will be stored.
 * @param flush Flag indicating whether to flush the license and wait for a new input.
 */
void license_load(OUT char* plic, IN bool flush = false){
  if (!EEPROM.begin(EEPROM_SIZE)){
      LOG_E("Failed to initialise EEPROM");
      memset(plic, 0, sizeof(glicense));
      return;
  }

  if(flush){
      license_wait_and_save_from_serial(plic);
  }
  else{
      bool license_valid = false;
      for (int i = 0; i < sizeof(glicense) - 1; i++){
        if(isHexadecimalDigit(EEPROM.read(i)) == false){
          LOG_E("EEPROM data is not valid, please input a new license");
          memset(plic, 0, sizeof(glicense));
          license_valid = false;
          break;
        }
        license_valid = true;
        plic[i] = (char)EEPROM.read(i);
      }
      // license valid or not here just mean the format of license is correct, not mean the license can activate the uwb device or not
      if(true == license_valid){
        LOG_I("Load license from EEPROM: %s", plic);
        return;
      }
      else
      {
        license_wait_and_save_from_serial(plic);
      }
  }
}

/**
 * @brief Initializes the UWB (Ultra-Wideband) module and performs necessary configurations.
 * 
 * This function initializes the UWB module by creating a UWBDevClass object, setting up the SPI bus,
 * and configuring the necessary pins. It also retrieves the UID (Unique Identifier) of the UWB module
 * and displays it on an external display. Additionally, it activates the UWB device using a license
 * and checks if the activation was successful. If the activation fails, it displays an error message
 * on the display and enters an infinite loop.
 * 
 * @note This function assumes that the necessary libraries and dependencies are already included.
 * 
 * @param None
 * @return None
 */
void uwb_init(void){
  int16_t dx = 1, dy = 1;
  uint8_t font_size = 1;
  uint8_t dy_step = 8 * font_size;

  //get the version of the uwb api
  tft_drawtext(dx , dy, (char*)(String("forthink uwb api version:") + String(ftlib::get_lib_version())).c_str(), font_size); dy += dy_step;
  LOG_I("Forthink uwb api version = %s", ftlib::get_lib_version());

  //instance a uwb object with a specific spi bus object
  if (uwb != NULL) free(uwb);
  uwb = new UWBHALClass(gspi_uwb, SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  uwb->init();
  uwb->begin(UWB_RST_PIN, UWB_SPI_SS, UWB_INT_PIN, UWB_RDY_PIN);
  guid = uwb->dev_get_uid();


  //load the license from EEPROM, if not exist, wait from serial port
  license_load(glicense, false);
  //uwb module may goto sleep mode(default 500ms), so we need to reset it to wake up after license load
  uwb->hardware_reset();


  //try to activate the uwb device with a license provided above
  tft_drawtext(dx , dy, (char*)String("activate uwb api...").c_str(), font_size);dy += dy_step;
  LOG_I("Try to activate the uwb device with license [%s]", glicense);
  if(true == uwb->dev_activate(glicense)){
    tft_drawtext(dx, dy, (char*)(String("uid:") + String(guid)).c_str(), font_size*2, ST77XX_GREEN);dy += dy_step*2;
    tft_drawtext(dx, dy, (char*)String("Activate OK!").c_str(), font_size*2, ST77XX_GREEN); dy += dy_step*2;
    delay(2000);
  }
  //if uwb return any failed result, you can get a new License by following method
  else{
      tft_drawtext(dx , dy, (char*)String("License error!").c_str(), font_size*2, ST77XX_RED);dy += dy_step*2;
      tft_drawtext(dx , dy, (char*)(String("uid => ") + String(guid)).c_str(), font_size*2, ST77XX_RED);dy += dy_step*2;
      tft_drawtext(dx , dy, "Generate a new one:", font_size*2, ST77XX_RED);dy += dy_step*2;
      tft_drawtext(dx , dy, (char*)gurl, font_size, ST77XX_RED);dy += dy_step;
      while (true){
        LOG_E(">> Jump here to generate a new license by yourself: %s", gurl);
        LOG_E("-----------------------------------------------------------------------------------");
        delay(1000);
      }
    }

  //TODO : set node mac and dst mac via other sync method
  //Use the node role to determine the mac address
  //Self mac address is the last two bytes of the self uid while dest mac address is the last two bytes of the dest nodes uid which you know in advance
  //Here we set the dest mac address hard coded manually for test
  String info = (grole == NodeRole::INITATOR) ? "node role:initator" : "node role:responder";
  tft_drawtext(dx , dy, (char*)info.c_str(), font_size);dy += dy_step;

  // get the last two bytes of the uid as the self mac address
  gself_mac = std::stoi(guid + 4, nullptr, 16);
  tft_drawtext(dx , dy, (char*)(String("self mac:" ) + String(gself_mac, 16)).c_str(), font_size);dy += dy_step;
  delay(1000);



  //Sequence of uwb range setup if the api of uwb device is activated successfully
  bool sta = true;
  sta &= uwb->range_set_session_role(grole);
  //a responder role in ccc mode, parameter 'index of responder' have to be configured mauanlly, index range from 0 ot N, where N is the number of the responder nodes
  if(grole == NodeRole::RESPONDER){
    sta &= uwb->range_set_session_index_responder(REPONDER_INDEX); //increment the index '1' for each responder node
  }
  sta &= uwb->range_set_session_num_responder(REPONDER_NUM); //let initator know and responder know how many responder nodes in the network, set it to 3 for test
  sta &= uwb->range_set_session_param_default(SessionType::CCC);
  sta &= uwb->configuration_commit(SessionType::CCC, gsession_id);

  if(false == sta){
    while(true){
      LOG_E("UWB range setup failed!");
      delay(100);
    }
  }

  uwb->range_set_session_start(gsession_id);
}


/**
 * @brief Initializes the MQTT client.
 * 
 * This function initializes the MQTT client with the provided Wi-Fi credentials, MQTT broker server domain,
 * client name, and MQTT port. It also includes optional functionalities such as debugging messages, HTTP web updater,
 * OTA (Over The Air) updates, and last will message.
 * 
 * @note Make sure to set the Wi-Fi SSID, Wi-Fi password, MQTT broker server domain, client name, and MQTT port
 *       before calling this function.
 */
void mqtt_init(void){
    mclient = new EspMQTTClient(
            WIFI_SSID,    // Wifi SSID
            WIFI_PSWD,    // Wifi Password
            MQTT_BROKER,  // MQTT Broker server domain
            guid,         // Client name that uniquely identify your mqtt client
            MQTT_PORT     // The MQTT port, default to 1883. this line can be omitted
    );
    // Optional functionalities of EspMQTTClient
    // mclient->enableDebuggingMessages(); // Enable debugging messages sent to serial output
    //mclient->enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overridded with enableHTTPWebUpdater("user", "password").
    //mclient->enableOTA(); // Enable OTA (Over The Air) updates. Password defaults to MQTTPassword. Port is the default OTA port. Can be overridden with enableOTA("password", port).
    //mclient->enableLastWillMessage("TestClient/lastwill", "I am going offline");  // You can activate the retain flag by setting the third parameter to true

    while (mclient->isConnected() != true)
    {
        static uint16_t cnt = 0;
        LOG_W("Connecting to MQTT [%s:%d] , client name [%s], %ds... ", mclient->getMqttServerIp(), mclient->getMqttServerPort(), guid, cnt++);
        mclient->loop();
        delay(1000);
    }
}

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished()
{
  LOG_I("MQTT Connected!");

  // Subscribe to "mytopic/wildcardtest/#" and display received message to Serial
  mclient->subscribe(TopicDown + "#", [](const String & topic, const String & payload) {
    Serial.println("(From wildcard) topic: " + topic + ", payload: " + payload);
  });
}


/**
 * @brief Initializes the setup for the UWB range-FiRa example.
 * 
 * This function initializes the serial communication, LED, OLED display, SPI, role, and UWB modules.
 * It also sets the node MAC address and destination MAC addresses based on the node role.
 * Finally, it configures the UWB module for range networking and starts the range measurement session.
 */
void setup() {
    serial_init();
    delay(1000);
    role_init();
    spi_init();
    led_init();
    lcd_init();
    uwb_init();
    mqtt_init();
}

void lcd_fsm(std::map<uint64_t, uint16_t> dismap){
  static bool fisrt_entry = true;
  static uint8_t font_size = 2, dy_step = 8 * font_size;
  static int16_t dx = 1, dy = 1;
  if(fisrt_entry){
    tft->fillScreen(TFT_BACK_COLOR);
    tft_drawtext(dx , dy, (char*)String("Example :CCC Car key").c_str(), font_size, ST77XX_BLUE);  dy += dy_step;
    tft_drawtext(dx , dy, (char*)(grole == NodeRole::INITATOR ? "Role    :Initator" : "Role    :Responder"), font_size, ST77XX_BLUE);   dy += dy_step;
    tft_drawtext(dx , dy, (char*)(String("Self mac:" ) + String(gself_mac, 16)).c_str(), font_size, ST77XX_BLUE);   dy += dy_step;
    fisrt_entry = false;
  }

  uint8_t dy_t = 0;
  for (auto dis = dismap.begin(); dis != dismap.end(); dis++){
    uint16_t distance = dis->second;

    String header = (String("Initator-> "));
    tft_drawtext(dx , dy + dy_t, (char*)header.c_str(), font_size, ST77XX_GREEN, false);
    tft_drawtext(header.length()*6*font_size, dy + dy_t, (char*)(String(distance) + String("cm")).c_str(), font_size, ST77XX_GREEN); dy_t += dy_step;
  }
}

/**
 * @brief The main loop function that runs repeatedly in the Arduino sketch.
 * 
 * This function is responsible for executing the main logic of the program.
 * It listens for range data and performs different actions based on the role of the node.
 * If the node is an initiator, it retrieves range data from all responder nodes and stores it in the `dismap` map.
 * If the node is a responder, it retrieves range data from the initiator node and stores it in the `dismap` map.
 * If the signal seems lost (based on the value of `err_cnt`), the function restarts the range session and increments `err_cnt`.
 * If the node role is unknown, an error message is logged.
 * The function also calls the `led_fsm` function and adds a small delay of 1 millisecond.
 */
void loop() {
  static std::map<uint64_t, uint16_t> dismap;
  static uint16_t err_cnt = 1, distance = 0;
  static unsigned long tstart = 0;
  static uint64_t mqtt_pkg_index = 0;

  mclient->loop();

  //listen data from uwb device, including range data, status, etc.
  uwb->ntf_listening();

  //Logic for different node role choice via 'ROLE_SW_PIN' 
  switch (grole)
  {
    case NodeRole::INITATOR:
            //maybe you can do something here...
            //no distance data return, just check the range status in initiator node in CCC mode
      break;
    case NodeRole::RESPONDER:
            if(true == uwb->get_range_status(0x0000)){
              err_cnt = 1;
              if(true == uwb->get_range_distance(0x0000, &distance)){
                dismap[0x0000] = distance;
                if(millis() - tstart> 300)//3Hz
                {
                  static JsonDocument  json;
                  json["uid"] = guid;
                  json["role"] = "respondor";
                  json["dev_mac"] = gself_mac;
                  json["dst_mac"] = 0x0000;
                  json["ranging"] = distance;
                  json["index"]   = ++mqtt_pkg_index;
                  String sjson;
                  serializeJson(json, sjson);
                  mclient->publish(TopicUp + String(guid, strlen(guid)), sjson); 
                  tstart = millis();
                }
              }
            }
            else{
              //maybe you can do something here...
              err_cnt++;
            }
            //If the signal seems lost (err_cnt reached 20), restart the range session every 500 milliseconds until the signal is regained
            //Initiator node will not restart the range session if no range data return, Initator is like a router in the network, responder have to follow the initiator
            //Some powerful and easy sync method will be added in the future
            if(err_cnt % 20 == 0){
              do{
                uwb->range_set_session_restart(SessionType::CCC, gsession_id);
                LOG_W("Signal lost, restart the range session...");
                delay(500);
              }while(false == uwb->get_range_status(0x0000));
              err_cnt++;//avoid the infinite loop, add 1 to err_cnt
            }
      break;
    default:
          LOG_E("Unknown node role!");
          delay(1000);
      break;
  }
  lcd_fsm(dismap);
  led_fsm();
  delay(1);
}
