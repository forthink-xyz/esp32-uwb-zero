/*
 forthink.h - Main include file for the UWB Library
 Copyright (c) 2024-2032 Forthink Team.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 */

#ifndef _FORTHINK_H_
#define _FORTHINK_H_

#include <cstring>
#include <iostream>
#include <algorithm>
#include "logger.h"
#include <vector>
#include <SPI.h>
#include <map>



/**
 * @brief The role of the node.
 * 
 * This enumeration defines the possible roles of a node in the system.
 * - `NODE_INITATOR`: Represents the initiator node.
 * - `NODE_RESPONDER`: Represents the responder node.
 */
enum{
    NODE_INITATOR = 0x01,
    NODE_RESPONDER = 0x00,
}NODE_ROLE;

/**
 * @brief Enumeration representing different session types.
 * 
 * This enumeration is used to define different session types in the Forthink library.
 * The available session types are CCC_SESSION and FIRA_SESSION.
 */
enum{
    CCC_SESSION  = (1<<0),
    FIRA_SESSION = (1<<1)
}SESSION_TYPE;


namespace ftlib{
#define IN
#define OUT
const char* get_lib_version();
class SPIPortClass;

/**
 * @class UWBHALClass
 * @brief This class represents the UWB Hardware Abstraction Layer (HAL).
 *
 * The UWBHALClass provides an interface to interact with the UWB hardware and perform various operations,
 * such as initialization, activation, setting session parameters, and more.
 */
class UWBHALClass{
        private:
            SPIPortClass *dev;
        public:

            UWBHALClass(IN SPIClass *spibus , IN SPISettings spiset);

            ~UWBHALClass();

            bool init(void);

            bool deinit(void);

            void begin(IN int8_t prst, IN int8_t cs, IN int8_t pint, IN int8_t prdy);

            void hardware_reset(void);

            char* dev_get_uid(void);

            /**
             * @brief Activates the device using the provided license.
             *
             * This function activates the device using the specified license.
             *
             * @param lics The license string to activate the device.
             * @return true if the device is successfully activated, false otherwise.
             */
            bool dev_activate(IN const char *lics);

            /**
             * @brief Sets the default session parameter for a given session type.
             *
             * This function sets the default session parameter for the specified session type.
             *
             * @param session_type The session type for which the default parameter needs to be set: CCC_SESSION or FIRA_SESSION.
             * @return True if the default session parameter was successfully set, false otherwise.
             */
            bool range_set_session_param_default(IN int8_t session_type);

            /**
             * Sets the session preamble code index for the range.
             *
             * @param index The index of the session preamble code.
             * @return True if the session preamble code index was set successfully, false otherwise.
             */
            bool range_set_session_preamble_code_index(IN uint8_t index);

            /**
             * @brief Sets the session channel number for the range.
             *
             * This function sets the session channel number for the range. The session channel number
             * determines the channel to which the range session is assigned.
             *
             * @param channel The channel number to set for the range session.
             * @return True if the session channel number was set successfully, false otherwise.
             */
            bool range_set_session_channel_number(IN uint8_t channel);

            /**
             * @brief Sets the session for multiple nodes in the range.
             *
             * This function sets the session for multiple nodes in the range.
             *
             * @param multi_node The value indicating whether to enable or disable multi-node session.
             *                   Set to true to enable multi-node session, false otherwise.
             *
             * @return true if the session for multiple nodes was successfully set, false otherwise.
             */
            bool range_set_session_multi_node(IN uint8_t multi_node);

            /**
             * Sets the number of slots per round-robin session for the range.
             *
             * @param slots_per_rr The number of slots per round-robin session.
             * @return True if the operation was successful, false otherwise.
             */
            bool range_set_session_slots_per_rr(IN uint8_t slots_per_rr);

            /**
             * Sets the duration of a session slot for the range.
             *
             * @param slot_duration The duration of the session slot in milliseconds.
             * @return True if the slot duration was successfully set, false otherwise.
             */
            bool range_set_session_slot_duration(IN uint16_t slot_duration);

            /**
             * @brief Sets the session ranging interval.
             *
             * This function sets the session ranging interval to the specified value.
             * The ranging interval is a 32-bit value, but only the lower 2 bytes are used.
             * The higher 2 bytes should be filled with 0.
             *
             * @param ranging_interval The session ranging interval to set.
             * @return True if the session ranging interval was set successfully, false otherwise.
             */
            bool range_set_session_ranging_interval(IN uint32_t ranging_interval);

            /**
             * Sets the session ranging round control value.
             *
             * This function is used to set the ranging round control value for the session.
             *
             * @param ranging_round_control The ranging round control value to be set.
             * @return True if the ranging round control value was set successfully, false otherwise.
             */
            bool range_set_session_ranging_round_control(IN uint8_t ranging_round_control);

            /**
             * Sets the session status initialization IV for the range.
             *
             * @param sts_init_iv The pointer to the session status initialization IV.
             * @param sts_init_iv_len The length of the session status initialization IV (default is 6).
             * @return True if the session status initialization IV is successfully set, false otherwise.
             */
            bool range_set_session_sts_init_iv(IN uint8_t *sts_init_iv, IN uint8_t sts_init_iv_len = 6);

            /**
             * @brief Sets the self MAC address for the session range, FiRa only.
             *
             * This function sets the self MAC address for the session range. The self MAC address is used for internal
             * communication within the range session.
             *
             * @param self_mac A pointer to the self MAC address.
             * @param self_mac_len The length of the self MAC address.
             * @return `true` if the self MAC address was set successfully, `false` otherwise.
             */
            bool range_set_session_self_mac(IN uint8_t *self_mac, uint8_t self_mac_len);

            /**
             * Sets the destination MAC address list for the session range, FiRa only.
             *
             * @param dstmap A map containing the destination MAC addresses and their corresponding port numbers.
             *               The keys are character arrays representing the MAC addresses, and the values are
             *               unsigned integers representing the port numbers.
             * @return True if the destination MAC address list was successfully set, false otherwise.
             */
            bool range_set_session_dest_mac_list(IN std::map<char*, uint16_t> dstmap);

            /**
             * @brief Sets the destination MAC address list for the session range.
             *
             * This function sets the destination MAC address list for the session range.
             *
             * @param dest_mac Pointer to the destination MAC address list.
             * @param dest_mac_len Length of the destination MAC address list.
             * @return `true` if the destination MAC address list was set successfully, `false` otherwise.
             */
            bool range_set_session_dest_mac_list(IN uint8_t *dest_mac, uint8_t dest_mac_len);

            /**
             * @brief Sets the vendor ID for the session range.
             *
             * This function sets the vendor ID for the session range. The vendor ID is a 16-bit value
             * that uniquely identifies the vendor of the session range.
             *
             * @param vendor_id The vendor ID to set for the session range.
             * @return `true` if the vendor ID was set successfully, `false` otherwise.
             */
            bool range_set_session_vendor_id(IN uint16_t vendor_id);

            /**
             * @brief Sets the session role for the range.
             *
             * This function sets the session role for the range. The session role determines the permissions and privileges
             * associated with the range.
             *
             * @param role The session role to be set. This should be a valid uint8_t value.
             * @return True if the session role was set successfully, false otherwise.
             */
            bool range_set_session_role(IN uint8_t role);

            /**
             * Sets the session number responder for the range.
             *
             * @param num The session number to set.
             * @return True if the session number was set successfully, false otherwise.
             */
            bool range_set_session_num_responder(IN uint8_t num);

            /**
             * Sets the session index for the range. CCC only.
             *
             * @param index The session index to set.
             * @return True if the session index was set successfully, false otherwise.
             */
            bool range_set_session_index_responder(IN uint8_t index);

            /**
             * @brief Sets the session restart for a given session type and session ID.
             *
             * This function is used to set the session restart for a specific session type and session ID.
             *
             * @param session_type The type of the session.
             * @param session_id The ID of the session.
             * @return true if the session restart was successfully set, false otherwise.
             */
            bool range_set_session_restart(IN int8_t session_type, IN uint32_t session_id);

            /**
             * Enables or disables the transmission log for the session range.
             *
             * @param en The flag indicating whether to enable or disable the transmission log.
             * @return True if the transmission log was successfully enabled or disabled, false otherwise.
             */
            bool range_set_session_tx_log_enable(IN bool en);

            /**
             * Enables or disables the logging of received session data.
             *
             * @param en The flag indicating whether to enable or disable the logging.
             * @return True if the logging was successfully enabled or disabled, false otherwise.
             */
            bool range_set_session_rx_log_enable(IN bool en);

            /**
             * @brief Sets the transmission power for the session range.
             *
             * This function allows you to set the transmission power for the session range.
             * The transmission power determines the signal strength of the radio waves used
             * for communication. A higher power level can result in a longer range, but it
             * also consumes more energy.
             *
             * @param power The transmission power level to set. This should be an integer
             *              value between -12dBm to +14dBm, where a higher value represents a
             *              higher power level.
             * @return True if the transmission power was successfully set, false otherwise.
             */
            bool range_set_session_tx_power(IN int8_t power);

            /**
             * @brief Sets the start of a session for range calculation.
             *
             * This function sets the start of a session for range calculation. It takes a session ID as input and returns a boolean value indicating whether the operation was successful or not.
             *
             * @param session_id The ID of the session to set the start for.
             * @return True if the session start was set successfully, false otherwise.
             */
            bool range_set_session_start(IN uint32_t session_id);


            /**
             * Retrieves the range status for a given MAC address. FiRa only.
             *
             * @param mac The MAC address for which to retrieve the range status.
             * @return The range status of the specified MAC address.
             */
            bool get_range_status(uint64_t mac);

            /**
             * Retrieves the range distance for a given MAC address. specified MAC address should be given in FiRa mode, 
             * the MAC address should be given as 0x0000 in CCC mode.
             *
             * @param mac The MAC address for which to retrieve the range distance.
             * @param dist A pointer to a variable where the range distance will be stored.
             * @return True if the range distance was successfully retrieved, false otherwise.
             */
            bool get_range_distance(IN uint64_t mac, OUT uint16_t *dist);


            /**
             * @brief Commits the configuration changes for a specific session.
             *
             * This function is used to commit the configuration changes made before a session start an after the session app configuration set.
             *
             * @param session_type The type of the session.
             * @param session_id The ID of the session.
             * @return True if the configuration changes were successfully committed, false otherwise.
             */
            bool configuration_commit(IN int8_t session_type, IN uint32_t session_id);

            /**
             * listen for the notification from the device.
             *
             * @return true if the callback is executing, false otherwise.
             */
            bool ntf_listening(void);
            
            /**
             * @brief Performs firmware update for the device.
             * 
             * This function is responsible for updating the firmware of the uwb device.
             * 
             * @return true if the firmware update is successful, false otherwise.
             */
            bool dev_fw_update(void);
    };
}


struct NINearbyShareableData;
typedef NINearbyShareableData* NINearbyShareableData_t;
enum class NINearbyMessageId;

/**
 * @class NearByClass
 * @brief Represents a class for handling nearby communication.
 *
 * The NearByClass class provides methods for managing nearby communication, including
 * initializing the class, getting and setting the out-of-band (OOB) phase, handling
 * received data streams, and retrieving shareable data.
 */
class NearByClass{
public:
    NearByClass(uint8_t devrole , uint16_t devmac, void (*oob_tx_cb)(uint8_t *data, uint8_t len));
    ~NearByClass();
    uint8_t  get_oob_phase();
    uint16_t get_phone_mac();
    void     handle_rx_stream(String rx_stream);
    void    set_oob_phase(uint8_t phase);
    NINearbyShareableData_t get_shareable_data();
private:
    uint8_t  dev_role;
    uint16_t dev_mac;
    NINearbyMessageId oobConfiguredPhase;
    void (*oob_tx)(uint8_t *data, uint8_t len);
    NINearbyShareableData_t  ni_shareable_data_from_phone = NULL;
};

#endif
