# Forthink_ESP32_UWB



Forthink ESP32 UWB contains an ESP32 and a NCJ29D5 chip.

# Introduce

Ultra-wideband (UWB) is a short-range, wireless  communication protocol that operates through radio waves, enables secure  reliable ranging and precision sensing, creating a new dimension of  spatial context for wireless devices.

Forthink ESP32 UWB module, which is based on IC NXP NCJ29D5, has been greatly popular and liked by many Makers. NCJ29D5 is an automotive-grade chip, that is widely used in many UWB digital key systems.

Yes, the NCJ29D5 is an automotive-grade chip that is widely used in many UWB (Ultra-Wideband) digital key systems in cars.

In the automotive industry, UWB technology is used to implement highly secure digital key systems. These systems utilize UWB chips to recognize and authenticate vehicles, providing enhanced security and theft prevention.

As an automotive-grade UWB chip, the NCJ29D5 possesses characteristics that are suitable for the automotive environment, such as resistance to interference and high temperature tolerance. It enables fast and reliable vehicle recognition, ensuring that only authorized digital keys can unlock and start the vehicle. The widespread adoption of this technology has enhanced vehicle security while providing a more convenient user experience.

Compares to the other UWB Modules, the NCJ29D5 has advantages as below:

1. Most important: Interoperable with Apple U1 chip, that makes it possible to work with the Apple system;
2. Fully aligned with FiRa™ PHY, MAC, and certification development, which make it more suitable for further applications;
3. Capable to operate as specified by the Car Connectivity Consortium (CCC) to enable interoperability with the car access ecosystem;
4. Supports UWB channels 5 (6.5 GHz) and 9 (8 GHz), while DWM1000 does not support Channel 9;



# Feature

- Integrated Heltec ESP32 2.4G WiFi and Bluetooth
- UWB chip from NXP



# Usage

## Install Library

Copy the project to Arduino library directory.  Default :  `C:\Users\<username>\Documents\Arduino\libraries`

## Device activate
Base on range-xxx.ino 

**1. get your device uid :**

	1. uwb = new UWBHALClass(gspi, SPISettings(spiClk, MSBFIRST, SPI_MODE0));

	2. uwb->init();

	3. uwb->begin(UWB_RST_PIN, UWB_SPI_SS, UWB_INT_PIN, UWB_RDY_PIN);

	4. char* puid = uwb->dev_get_uid();

 	5. LOG_I("uid = %s",puid);
  
**2. Jump to https://www.forthink.com.cn/ to generate a new license by yourself, it's a 128 characters string, just like:**

	plicense = 'ce2b41269ed55c386c404831e04aea13562386d5ab533366169bcc6cf9d503f8c71034f52c75a527ec8372c4edbfeca00c6b56a68eed149a42af982545906dd6'
	
**3. save this license in your code , we called 'plicense' here, you have to call it when you want to control the uwb device.**

## Examples
Assume that we actived the uwb firmware already.
### 1.Node Role Select

Default Role ： RESPONDER

| GPIO         | State         |Describe|
| -------------  | ------------- |--------|
| GPIO26| HIGH		|INITATOR|
|GPIO26| LOW		|RESPONDER|

### 2.Interaction with iPhone


coming soon...


### 3.FiRa Ranging

**setup phase：**

	1. uwb = new UWBHALClass(gspi, SPISettings(spiClk, MSBFIRST, SPI_MODE0));

	2. uwb->init();

	3. uwb->begin(UWB_RST_PIN, UWB_SPI_SS, UWB_INT_PIN, UWB_RDY_PIN);

	4. char* puid = uwb->dev_get_uid();

	5. bool sta = uwb->dev_actived(plicense);
 
  	//We have to active the uwb device before next configurations
   	//Without Bluetooth to transform the handshake frame, we have to add a sector of test code as blow to tell each other the mac address around.
	//Hopefully we can add a Bluetooth handshake method soon...
	if(node_role == NODE_INITATOR){
		nodemac = {0x23, 0xbc};
		dstmacmap["R1"] = {0xb5, 0xac};
		dstmacmap["R2"] = {0x75, 0x82};
	}   
	else{
		nodemac = {0xb5, 0xac};
		//nodemac = {0x75, 0x82};
		dstmacmap["R1"] = {0x23, 0xbc};
	}

	6. bool sta = false;
	
	7. sta |= uwb->range_set_role(role);
	
	8. sta |= uwb->range_set_session_type(FIRA_SESSION);
	
	9. sta |= uwb->range_set_tx_power(14);
	
	10. sta |= uwb->range_set_self_mac(nodemac);
	
	11. sta |= uwb->range_set_dest_mac_list(dstmacmap);
	
	12. sta |= uwb->range_apply_parameters(0xcbc54f23);
	
	13. if(true == sta)  uwb->range_start();
    
**loop phase:**

	static std::map<uint64_t, uint16_t> dismap;
 
	1. uwb->range_listen_data();
	
	2. uwb->range_get_distance(dismap);

### 4.CCC Ranging
**setup phase：**

	coming soon...
 
**loop phase:**

	coming soon...
