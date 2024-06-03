/**
 * @file  range-FiRa-Serial.ino
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

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "heltec.h"
#include "forthink.h"
#include "logger.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include "EEPROM.h"


using namespace ftlib;
using namespace std;

#define LED_PIN        38
#define VeX_PIN        5

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

#define NUS_SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define NUS_CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static uint16_t                       gself_mac = 0x0000;// it will be set in the uwb_init function, any value is ok except 0x0000, we set it to uid for test later
static NodeRole                       grole     = NodeRole::RESPONDER;//node role, default is responder, it will be set in the role_init function, use the ROLE_SW_PIN to determine the node role

static const int                      spiClk     = 10*1000*1000;
static UWBHALClass                    *uwb       = NULL;//uwb object pointer
static SPIClass                       *gspi_lcd  = NULL;//spi object pointer for lcd display
static SPIClass                       *gspi_uwb  = NULL;//spi object pointer for uwb module
static Adafruit_ST7789                *tft       = NULL;//lcd object pointer, it's a 240x135 lcd display, Adafruit dependcy

static const char                     *gurl      = "https://licenses.forthink.com.cn";
static char                           *guid      = NULL;

static BLEServer                      *pServer    = NULL;
static BLEService                     *nusService = NULL;

static BLECharacteristic              *pTxCharacteristic;
static BLECharacteristic              *pRxCharacteristic;

static NearByClass                    *nearbyobj = NULL;
static char                           glicense[LICENCE_SIZE + 1] = {0,};// 128 characters license, the last character is '\0', so 129 bytes in total

/**
 * @brief The license key used for uwb module.
 *
 * This variable stores the license key used in the code. It is a character pointer that points to a string representation of the license key.
 */
class BleServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        LOG_I("Device connected! ");
    }
    void onDisconnect(BLEServer* pServer) {
        nearbyobj->set_oob_phone_uwb_stop_phase();
        delay(500); // give the bluetooth stack the chance to get things ready
        LOG_W("Device disconnected! advertising...");
        pServer->startAdvertising(); 
    }
};

/**
 * @brief Callback class for handling UART RX events.
 * 
 * This class is used as a callback for handling UART RX events in the BLECharacteristic.
 * When a write event occurs on the BLECharacteristic, the `onWrite` function of this class is called.
 * It retrieves the received value and passes it to the `handle_rx_stream` function of the `nearbyobj` object.
 */
class UartRxCallBack: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      nearbyobj->handle_rx_stream(rxValue);
    }
};

void ni_oob_tx_callback(uint8_t *data, uint8_t len){
  if(pTxCharacteristic == NULL) {
    LOG_E("pTxCharacteristic is NULL");
    return;
  }
  pTxCharacteristic->setValue(data, len);
  pTxCharacteristic->notify();
}

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
  static uint8_t lsta = true, cnt = 0;
  if((cnt++)%20 == 0) lsta = true;
  else lsta = false;
  digitalWrite(LED_PIN, lsta);
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
 * Draws text on the TFT display at the specified coordinates.
 *
 * @param x The x-coordinate of the starting position of the text.
 * @param y The y-coordinate of the starting position of the text.
 * @param text The text to be displayed.
 * @param size The size of the text (default is 1).
 * @param color The color of the text (default is ST77XX_WHITE).
 * @param cover Whether to cover the previous text with a rectangle (default is true).
 */
void tft_drawtext(int16_t x, int16_t y, char *text, uint8_t size = 1 ,uint16_t color = ST77XX_WHITE, bool cover = true) {
  tft->setCursor(x, y);
  if(cover) tft->fillRect(x, y, (strlen(text) + 1)*size*6, 8*size, TFT_BACK_COLOR);//Desired text size. 1 is default 6x8, 2 is 12x16, 3 is 18x24, etc
  tft->setTextColor(color);
  tft->setTextSize(size);//Desired text size. 1 is default 6x8, 2 is 12x16, 3 is 18x24, etc
  tft->println(text);
}

void lcd_fsm(std::map<uint64_t, uint16_t> dismap, bool header_refresh){
  static uint8_t font_size = 2, dy_step = 8 * font_size;
  static int16_t dx = 1, dy = 1;
  if(header_refresh){
    dx = 1, dy = 1;
    tft->fillScreen(TFT_BACK_COLOR);
    tft_drawtext(dx , dy, (char*)String("Example :Apple NI").c_str(), font_size, ST77XX_BLUE);  dy += dy_step;
    tft_drawtext(dx , dy, (char*)(grole == NodeRole::INITATOR ? "Role    :Initator" : "Role    :Responder"), font_size, ST77XX_BLUE);   dy += dy_step;
    tft_drawtext(dx , dy, (char*)(String("self mac:" ) + String(gself_mac, 16)).c_str(), font_size, ST77XX_BLUE);   dy += dy_step;
  }

  uint8_t dy_t = 0;
  for (auto dis = dismap.begin(); dis != dismap.end(); dis++){
    uint16_t mac = dis->first;
    uint16_t distance = dis->second;

    String header = (String("from ") + String(mac, 16) + String("->"));
    tft_drawtext(dx , dy + dy_t, (char*)header.c_str(), font_size, ST77XX_GREEN, false);
    tft_drawtext(header.length()*6*font_size, dy + dy_t, (char*)(String(distance) + String("cm")).c_str(), font_size, ST77XX_GREEN); dy_t += dy_step;
  }
}

void lcd_clear(){
  if (tft != NULL)
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
  //doesn't handle automatically pulling SS low
  pinMode(gspi_uwb->pinSS(), OUTPUT); //HSPI SS

  //initialise the spi bus for the lcd display
  gspi_lcd = new SPIClass(HSPI);
  gspi_lcd->begin(TFT_SCL, -1, TFT_SDA, TFT_CS);
  //set up slave select pins as outputs as the Arduino API
  pinMode(gspi_lcd->pinSS(), OUTPUT); 
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
  //initialize uwb instance
  uwb->init();
  //begin uwb instance with some specific gpios, except SS pin, RST，INT，RDY pin are private spi protocol for chip vendor
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
    gself_mac = (uint16_t)std::stoi(guid + 4, nullptr, 16);
    tft_drawtext(dx, dy, (char*)(String("uid:") + String(guid)).c_str(), font_size*2, ST77XX_GREEN);dy += dy_step*2;
    tft_drawtext(dx, dy, (char*)String("Activate OK!").c_str(), font_size*2, ST77XX_GREEN); dy += dy_step*2;
    delay(1000);
  }
  //if uwb return any failed result, you can get a new License by following method
  else{
      tft_drawtext(dx , dy, (char*)String("License error!").c_str(), font_size*2, ST77XX_RED);dy += dy_step*2;
      tft_drawtext(dx , dy, (char*)(String("uid => ") + String(guid)).c_str(), font_size*2, ST77XX_RED);dy += dy_step*2;
      tft_drawtext(dx , dy, "Generate a new one:", font_size*2, ST77XX_RED);dy += dy_step*2;
      tft_drawtext(dx , dy, (char*)gurl, font_size, ST77XX_RED);dy += dy_step;
      while (true){
        if (NULL != guid) {
              LOG_E(">> Jump here to generate a new license by yourself: %s", gurl);
              LOG_E("-----------------------------------------------------------------------------------");
              delay(1000);
        }
        else{
          LOG_E(">> Sorry, uwb device activate failed! can't get uid, please check the connection.");
          delay(1000);
        }
      }
    }
}

/**
 * @brief Starts the ni (NearBy) session.
 * 
 * This function initializes the ni session and configures the UWB device with the necessary parameters
 * received from the iPhone. It waits for the configuration data from the iPhone and sets up the UWB
 * range session accordingly. If the UWB device fails to initialize or the range setup fails, it enters
 * an error loop.
 */
void ni_start(){
  if(nearbyobj == NULL) nearbyobj = new NearByClass(NodeRole::RESPONDER, gself_mac, ni_oob_tx_callback);

  uint8_t cnt = 0;
  while (nearbyobj->is_oob_phone_uwb_start_phase() == false){
    // 2 minutes timeout default, the value is depend on uwb module lib
    if((++cnt)%60 == 0){
      LOG_W("Waiting the configuration data from iPhone time out, timeout = 1 minutes, reset the uwb device.");
      uwb_init();
    }
    delay(1000);
    LOG_I("Waiting for the configuration data from iPhone %ds, code = 0x%02x", cnt%60, (uint8_t)nearbyobj->get_oob_phase());
  }

  if (NULL == uwb) {
    while (true) {
      LOG_E("UWB device is not initialized!");
      delay(1000);
    }
  }

  // NINearbyShareableData_t PhoneShareableData = nearbyobj->get_shareable_data();
  //Sequence of ni configuration
  bool sta = true;

  uint16_t dest_mac = nearbyobj->get_shareable_dest_address();

  sta &= uwb->range_set_session_role(grole);

  sta &= uwb->range_set_session_tx_power(14);

  sta &= uwb->range_set_session_self_mac((uint8_t*)&gself_mac, sizeof(gself_mac)/sizeof(uint8_t));

  sta &= uwb->range_set_session_preamble_code_index(nearbyobj->get_shareable_code_index());

  sta &= uwb->range_set_session_channel_number(nearbyobj->get_shareable_channel_number());  

  sta &= uwb->range_set_session_slots_per_rr(nearbyobj->get_shareable_slots_per_rr());

  sta &= uwb->range_set_session_slot_duration(nearbyobj->get_shareable_slot_duration());

  sta &= uwb->range_set_session_ranging_interval(nearbyobj->get_shareable_ranging_interval());

  sta &= uwb->range_set_session_ranging_round_control(nearbyobj->get_shareable_ranging_round_control());

  sta &= uwb->range_set_session_sts_init_iv(nearbyobj->get_shareable_sts_init_iv());

  sta &= uwb->range_set_session_multi_node(nearbyobj->get_shareable_multi_node());

  sta &= uwb->range_set_session_vendor_id(nearbyobj->get_shareable_vendor_id());

  sta &= uwb->range_set_session_dest_mac_list((uint8_t*)(&dest_mac), 2);

  sta &= uwb->configuration_commit(SessionType::FIRA, nearbyobj->get_shareable_session_id());

  if(false == sta){
    while(true){
      LOG_E("UWB range setup failed!");
      delay(1000);
    }
  }
  uwb->range_set_session_start(nearbyobj->get_shareable_session_id());
}

/**
 * Initializes the BLE functionality and sets up the BLE server and service.
 * 
 * @param name The name to be displayed for the BLE device.
 */
void ble_init(char* name){
   // Create the BLE Device
  BLEDevice::init("uwb zero (" + String(name) + ")");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BleServerCallbacks());
  // Create the BLE Service

  /*********************************Nordic UART Service***********************************/
  nusService = pServer->createService(NUS_SERVICE_UUID);
  // Create a BLE Characteristic
  pTxCharacteristic = nusService->createCharacteristic(
                    NUS_CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic = nusService->createCharacteristic(
                      NUS_CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
                    );
  pRxCharacteristic->setCallbacks(new UartRxCallBack());
  // Start the service
  nusService->start();

  //add the service UUID to the advertising data
  pServer->getAdvertising()->addServiceUUID(NUS_SERVICE_UUID);
  // Start advertising
  pServer->getAdvertising()->start();

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
    delay(1000); // wait for serial monitor open, if you not need, just remove it
    led_init();
    spi_init();
    lcd_init();
    uwb_init();
    ble_init(guid);
    ni_start();
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
  static uint16_t err_cnt = 0, distance = 0;
  static std::map<uint64_t, uint16_t> dismap;
  static bool first_in = true;

  //init the lcd display
  if(first_in){
    lcd_fsm(dismap, true);
    first_in = false;
  }

  // //listen data from uwb device, including range data, status, etc.
  uwb->ntf_listening();

  uint16_t phoneMac = nearbyobj->get_shareable_dest_address();
  if(true == uwb->get_range_status(phoneMac)){
    if(true == uwb->get_range_distance(phoneMac, &distance)){
      dismap[phoneMac] = distance;
      lcd_fsm(dismap, false);
    }
  }
  else{
      if(((++err_cnt)%20 == 0) && (nearbyobj->is_oob_phone_uwb_stop_phase() == true)){
         dismap.clear();
         while (nearbyobj->is_oob_phone_uwb_start_phase() == false){
            LOG_W("Connection lost, Waiting for the configuration from iPhone, code = 0x%02x", (uint8_t)nearbyobj->get_oob_phase());
            delay(1000);
        }
        err_cnt = 0;
        lcd_clear();
        uwb_init();
        ni_start();
        lcd_fsm(dismap, true);
      }
  }
  led_fsm();
}
