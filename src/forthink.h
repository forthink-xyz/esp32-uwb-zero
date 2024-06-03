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
enum class NodeRole{
    INITATOR  = 0x01,
    RESPONDER = 0x00,
};

/**
 * @brief Enumeration representing different session types.
 * 
 * This enumeration is used to define different session types in the Forthink library.
 * The available session types are CCC_SESSION and FIRA_SESSION.
 */
enum class SessionType{
    CCC  = 0xA0,
    FIRA = 0x00
};



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

            /**
             * @brief Resets the hardware.
             * 
             * This function performs a hardware reset.
             * 
             * @return void
             */
            void hardware_reset(void);

            /**
             * @brief Retrieves the unique identifier of the device.
             * 
             * @return A pointer to a character array containing the device's unique identifier.
             */
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
            bool range_set_session_param_default(IN SessionType session_type);

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
            bool range_set_session_role(IN NodeRole role);

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
            bool range_set_session_restart(IN SessionType session_type, IN uint32_t session_id);

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
            bool configuration_commit(IN SessionType session_type, IN uint32_t session_id);

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
    NearByClass(NodeRole devrole , uint16_t devmac, void (*oob_tx_cb)(uint8_t *data, uint8_t len));
    ~NearByClass();

    /**
     * @brief Handles the received stream of data.
     *
     * This function is responsible for processing the received stream of data.
     * It takes a String parameter `rx_stream` which represents the received stream.
     * 
     * @param rx_stream The received stream of data to be processed.
     */
    void  handle_rx_stream(String rx_stream);

    /**
     * @brief Retrieves the index of the shareable code.
     * 
     * This function returns the index of the shareable code.
     * 
     * @return The index of the shareable code.
     */
    uint8_t get_shareable_code_index();

    /**
     * @brief Retrieves the shareable channel number.
     * 
     * This function returns the shareable channel number as a uint8_t value.
     * The shareable channel number is used for communication purposes.
     * 
     * @return The shareable channel number.
     */
    uint8_t get_shareable_channel_number();

    /**
     * Retrieves the number of shareable slots per round-robin.
     *
     * @return The number of shareable slots per round-robin.
     */
    uint8_t get_shareable_slots_per_rr();

    /**
     * Retrieves the duration of a shareable slot.
     *
     * @return The duration of a shareable slot in milliseconds.
     */
    uint16_t get_shareable_slot_duration();

    /**
     * Retrieves the shareable ranging interval.
     *
     * @return The shareable ranging interval as a 16-bit unsigned integer.
     */
    uint16_t get_shareable_ranging_interval();

    /**
     * @brief Retrieves the shareable ranging round control value.
     *
     * This function returns the shareable ranging round control value, which is a uint8_t.
     * The shareable ranging round control value determines the behavior of the ranging round.
     *
     * @return The shareable ranging round control value.
     */
    uint8_t get_shareable_ranging_round_control();

    /**
     * @brief Retrieves the shareable initialization vector (IV) for the sts_init function.
     * 
     * @return A pointer to the shareable initialization vector.
     */
    uint8_t* get_shareable_sts_init_iv();

    /**
     * @brief Retrieves the shareable multi-node value.
     * 
     * This function returns the shareable multi-node value as a uint8_t.
     * 
     * @return The shareable multi-node value.
     */
    uint8_t get_shareable_multi_node();

    /**
     * @brief Retrieves the shareable vendor ID.
     *
     * This function returns the shareable vendor ID as a 16-bit unsigned integer.
     *
     * @return The shareable vendor ID.
     */
    uint16_t get_shareable_vendor_id();

    /**
     * @brief Retrieves the shareable destination address.
     *
     * This function returns the shareable destination address as a 16-bit unsigned integer.
     *
     * @return The shareable destination address.
     */
    uint16_t get_shareable_dest_address();

    /**
     * @brief Retrieves the shareable session ID.
     *
     * This function returns a 16-bit unsigned integer representing the shareable session ID.
     *
     * @return The shareable session ID.
     */
    uint16_t get_shareable_session_id();

    /**
     * @brief Retrieves the out-of-band phase.
     * 
     * This function returns the out-of-band phase as a uint8_t value.
     * 
     * @return The out-of-band phase as a uint8_t value.
     */
    uint8_t  get_oob_phase();

    /**
     * Sets the out-of-band (OOB) phone UWB stop phase.
     * 
     * This function is responsible for setting the UWB stop phase for the out-of-band (OOB) phone.
     * 
     * @return true if the UWB stop phase was successfully set, false otherwise.
     */
    bool set_oob_phone_uwb_stop_phase();

    /**
     * Checks if the out-of-band (OOB) phone UWB initialization phase is ongoing.
     *
     * @return true if the OOB phone UWB initialization phase is ongoing, false otherwise.
     */
    bool is_oob_phone_uwb_init_phase();

    /**
     * Checks if the out-of-band (OOB) phone UWB start phase is active.
     * 
     * @return true if the OOB phone UWB start phase is active, false otherwise.
     */
    bool is_oob_phone_uwb_start_phase();

    /**
     * Checks if the OOB phone UWB stop phase is active.
     *
     * @return true if the OOB phone UWB stop phase is active, false otherwise.
     */
    bool is_oob_phone_uwb_stop_phase();

private:
    NodeRole  dev_role;
    uint16_t  dev_mac;
    /**
     * Sets the out-of-band (OOB) phase.
     *
     * This function sets the out-of-band phase to the specified value.
     *
     * @param phase The value of the out-of-band phase to set.
     */
    void      set_oob_phase(uint8_t phase);
    /**
     * @brief Function pointer for out-of-band transmission.
     *
     * This function pointer is used to transmit data out-of-band.
     * It takes a pointer to the data and the length of the data as parameters.
     *
     * @param data Pointer to the data to be transmitted.
     * @param len Length of the data.
     */
    void      (*oob_tx)(uint8_t *data, uint8_t len);

    /**
     * @brief Represents the identifier for a nearby message.
     * 
     * This class is used to uniquely identify a nearby message in the Forthink library.
     * It is typically used in conjunction with other classes and functions to perform
     * operations related to nearby messages.
     */
    NINearbyMessageId oobConfiguredPhase;
    NINearbyShareableData_t  shareable_data_from_phone = NULL;
};

#endif
