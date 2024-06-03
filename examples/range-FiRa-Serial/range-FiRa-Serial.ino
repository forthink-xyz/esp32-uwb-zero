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
#include "EEPROM.h"

using namespace ftlib;

#define LED_PIN        38
#define ROLE_SW_PIN    35

#define UWB_RST_PIN    8
#define UWB_INT_PIN    14
#define UWB_RDY_PIN    9

#define SPI_MISO       10
#define SPI_MOSI       12
#define SPI_SCLK       11
#define UWB_SPI_SS     13

#define LICENCE_SIZE   128
#define EEPROM_SIZE    LICENCE_SIZE

static const uint16_t MAC_RESPOR_1 = 0xfbbe;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
static const uint16_t MAC_RESPOR_2 = 0xfbbc;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
static const uint16_t MAC_RESPOR_3 = 0xfbb2;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
static const uint16_t MAC_RESPOR_4 = 0xfbaf;//Respondors around the initiator which mac address is known in advance，let initator know the mac address of the responder
static const uint16_t MAC_INITOR   = 0xabcd;//set initator mac address to same value for all of the responder nodes, let responder know the mac address of the initator
static const int  spiClk           = 10*1000*1000;
static const char *gurl            = "https://licenses.forthink.com.cn";


static std::map<char*, uint16_t>   gdest_mac_map;     //mac address of the dest nodes which we know in advance, it's a map container, key is the name of the dest node, value is the mac address of the dest node
static uint16_t                    gself_mac = 0x0000;// it will be set in the uwb_init function, any value is ok except 0x0000, we set it to uid for test later
static uint64_t                    gsession_id = 0xcbc54f23;//session id for range session, it's would be better to use a random number, here we use a fixed number for test
static uint8_t                     grole       = NODE_RESPONDER;//node role, default is responder, it will be set in the role_init function, use the ROLE_SW_PIN to determine the node role
static UWBHALClass                 *uwb       = NULL;//uwb object pointer
static SPIClass                    *gspi_uwb  = NULL;//spi object pointer
static char                        *guid      = NULL;
static char                        glicense[LICENCE_SIZE + 1] = {0,}; //license buffer, it's a hex string, the length is 128 bytes, the last byte is '\0'

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
 * Initializes the serial communication and prints the ESP32 Chip ID.
 */
void serial_init(void){
	Serial.begin(115200);
  delay(3000); // wait for serial monitor open, if you not need, just remove it
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
  grole = (state == HIGH) ? NODE_INITATOR : NODE_RESPONDER;
  const char* pinfo = (grole == NODE_INITATOR) ? "Initator":"Responder";
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
  LOG_I("Forthink uwb api version = %s", ftlib::get_lib_version());
  //instance a uwb object with a specific spi bus object
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



  LOG_I("Try to activate the uwb device with license [%s]", glicense);
  if(true == uwb->dev_activate(glicense)){
    LOG_I("vvvvvvv Congratulations, uwb device activated. vvvvvvv");   
    delay(1000);
  }
  //if uwb return any failed result, you can get a new License by following method
  else{
      while (true){
          if (NULL != guid) {
                LOG_E(">> Sorry, uwb device activate failed! uid is [%s]", guid);
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

  //set node mac and dst mac via other sync method
  //Use the node role to determine the mac address
  //Self mac address is the last two bytes of the self uid while dest mac address is the last two bytes of the dest nodes uid which you know in advance
  //Here we set the dest mac address hard coded manually for test
  if(NODE_INITATOR == grole){
    gself_mac = MAC_INITOR; 
    //when we say dest mac in initator role, it means the mac address of the responder, it's multiple destination mac address
    gdest_mac_map["responder1"] =  MAC_RESPOR_1; 
    gdest_mac_map["responder2"] =  MAC_RESPOR_2; 
    gdest_mac_map["responder3"] =  MAC_RESPOR_3; 
    gdest_mac_map["responder4"] =  MAC_RESPOR_4;
  }   
  else{
    int uid = std::stoi(guid + 4, nullptr, 16);//get the last two bytes of the uid as the mac address
    gself_mac = (uint16_t)uid;
    gdest_mac_map["initator"] = MAC_INITOR;
  }
  


  //Sequence of uwb range setup if the api of uwb device is activated successfully
  bool sta = true;
  sta &= uwb->range_set_session_role(grole);
  sta &= uwb->range_set_session_param_default(FIRA_SESSION);
  sta &= uwb->range_set_session_tx_power(14);
  sta &= uwb->range_set_session_self_mac((uint8_t*)&gself_mac, sizeof(gself_mac)/sizeof(uint8_t));
  sta &= uwb->range_set_session_dest_mac_list(gdest_mac_map);
  sta &= uwb->configuration_commit(FIRA_SESSION, gsession_id);
  if(false == sta){
    while(true){
      LOG_E("UWB range setup failed!");
      delay(100);
    }
  }
  uwb->range_set_session_start(gsession_id);
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
    led_init();
    spi_init();
    role_init();
    uwb_init();
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
  static uint16_t err_cnt = 1;
  static uint16_t distance = 0;
  static uint16_t responder_mac = 0;
  static uint16_t initator_mac = 0;
  
  //listen data from uwb device, including range data, status, etc.
  uwb->ntf_listening();
  //Logic for different node role choice via 'ROLE_SW_PIN' 
  switch (grole)
  {
    case NODE_INITATOR:
            for(auto mac = gdest_mac_map.begin(); mac != gdest_mac_map.end(); mac++){
              responder_mac = mac->second;
              if(true == uwb->get_range_status(responder_mac)){
                if(true == uwb->get_range_distance(responder_mac, &distance)){
                  dismap[responder_mac] = distance;
                }
              }
            }
      break;
    case NODE_RESPONDER:
            initator_mac = gdest_mac_map["initator"];
            if(true == uwb->get_range_status(initator_mac)){
              err_cnt = 1;
              if(true == uwb->get_range_distance(initator_mac, &distance)){
                dismap[initator_mac] = distance;
              }
            }
            else{
              err_cnt++;
            }
            //If the signal seems lost (err_cnt reached 20), restart the range session every 500 milliseconds until the signal is regained
            //Initiator node will not restart the range session if no range data return, Initator is like a router in the network, responder have to follow the initiator
            //Some powerful and easy sync method will be added in the future
            if(err_cnt % 20 == 0){
              do{
                uwb->range_set_session_restart(FIRA_SESSION, gsession_id);
                delay(500);
              }while(false == uwb->get_range_status(initator_mac));
              err_cnt++;//avoid the infinite loop, add 1 to err_cnt
            }
      break;
    default:
          LOG_E("Unknown node role!");
          delay(1000);
      break;
  }
  led_fsm();
  delay(1);
}
