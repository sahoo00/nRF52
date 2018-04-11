/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This file contains the source code for a sample client application using the LED Button service.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "nrf_ble_gatt.h"
#include "nrf_gpio.h"

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SCAN_INTERVAL                   0x00A0                              /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     0x0050                              /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT                    0x0000                              /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                   /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

#define UUID16_SIZE                     2                                   /**< Size of a UUID, in bytes. */

#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */


#define TRG1_UUID_BASE        {0xc6, 0x1d, 0x93, 0x6e, 0x09, 0x03, 0x63, 0x8b, \
                              0xe8, 0x49, 0xa5, 0xcf, 0x00, 0x00, 0xba, 0xde}
#define SMPH_UUID_SERVICE     0x0000
#define TRG2_UUID_SERVICE     0x0001
#define MN_UUID_SERVICE       0x0002
#define TRG1_SAFETY_ALERT     0x0010
#define TRG1_ALERT_STATUS     0x0011

#define TRG1_DEVICE_ID        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
static uint8_t device_id[17] = {0x00, TRG1_DEVICE_ID};

/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t * p_data;   /**< Pointer to data. */
    uint16_t  data_len; /**< Length of data. */
} data_t;

/**@brief TRG1 Client event type. */
typedef enum
{
    BLE_TRG1_EVT_DISCOVERY_COMPLETE = 1,  /**< Event indicating that the LED Button Service has been discovered at the peer. */
    BLE_TRG1_EVT_BUTTON_NOTIFICATION      /**< Event indicating that a notification of the LED Button Button characteristic has been received from the peer. */
} ble_trg1_evt_type_t;

/**@brief Structure containing the Button value received from the peer. */
typedef struct
{
    uint8_t button_state;  /**< Button Value. */
} ble_button_t;

/**@brief Structure containing the handles related to the LED Button Service found on the peer. */
typedef struct
{
    uint16_t safety_alert_handle;       /**< Handle of the Button characteristic as provided by the SoftDevice. */
    uint16_t alert_status_handle;          /**< Handle of the LED characteristic as provided by the SoftDevice. */
    uint16_t cccd_handle;
} trg1_db_t;

/**@brief LED Button Event structure. */
typedef struct
{
    ble_trg1_evt_type_t evt_type;    /**< Type of the event. */
    uint16_t             conn_handle; /**< Connection handle on which the event occured.*/
    union
    {
        ble_button_t button;          /**< Button Value received. This will be filled if the evt_type is @ref BLE_trg1_EVT_BUTTON_NOTIFICATION. */
        trg1_db_t    peer_db;         /**< LED Button Service related handles found on the peer device. This will be filled if the evt_type is @ref BLE_trg1_EVT_DISCOVERY_COMPLETE.*/
    } params;
} ble_trg1_evt_t;

// Forward declaration of the ble_trg1_t type.
typedef struct ble_trg1_s ble_trg1_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_trg1_evt_handler_t) (ble_trg1_t * p_ble_trg1, ble_trg1_evt_t * p_evt);

/**@brief LED Button Client structure. */
struct ble_trg1_s
{
    uint16_t                conn_handle;  /**< Connection handle as provided by the SoftDevice. */
    trg1_db_t                peer_trg1_db;
    ble_trg1_evt_handler_t evt_handler;  /**< Application event handler to be called when there is an event related to the LED Button service. */
    uint8_t                 uuid_type;    /**< UUID type. */
    uint8_t					ready_to_send;
    uint8_t					max_num_send;
};

/**@brief LED Button Client initialization structure. */
typedef struct
{
    ble_trg1_evt_handler_t evt_handler;  /**< Event handler to be called by the LED Button Client module whenever there is an event related to the LED Button Service. */
} ble_trg1_init_t;

#define TX_BUFFER_MASK         0x07                  /**< TX Buffer mask, must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE         (TX_BUFFER_MASK + 1)  /**< Size of send buffer, which is 1 higher than the mask. */

#define WRITE_MESSAGE_LENGTH   64    				 /**< Length of the write message for CCCD. */

typedef enum
{
    READ_REQ,  /**< Type identifying that this tx_message is a read request. */
    WRITE_REQ  /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

/**@brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct
{
    uint8_t                  gattc_value[WRITE_MESSAGE_LENGTH];  /**< The message to write. */
    ble_gattc_write_params_t gattc_params;                       /**< GATTC parameters for this message. */
} write_params_t;

/**@brief Structure for holding data to be transmitted to the connected central.
 */
typedef struct
{
    uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
    tx_request_t type;         /**< Type of this message, i.e. read or write message. */
    union
    {
        uint16_t       read_handle;  /**< Read request message. */
        write_params_t write_req;    /**< Write request message. */
    } req;
} tx_message_t;

static tx_message_t m_tx_buffer[TX_BUFFER_SIZE];  /**< Transmit buffer for messages to be transmitted to the central. */
static uint32_t     m_tx_insert_index = 0;        /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t     m_tx_index = 0;               /**< Current index in the transmit buffer from where the next message to be transmitted resides. */


/**@brief Function for passing any pending request from the buffer to the stack.
 */
static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index)
    {
        uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ)
        {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        }
        else
        {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
        }
        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_DEBUG("SD Read/Write API returns Success..");
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
        }
        else
        {
            NRF_LOG_DEBUG("SD Read/Write API returns error. This message sending will be "
                "attempted again..");
        }
    }
}


/**@brief Function for handling write response events.
 *
 * @param[in] p_ble_trg1 Pointer to the Led Button Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_write_rsp(ble_trg1_t * p_ble_trg1, ble_evt_t const * p_ble_evt)
{
    // Check if the event if on the link for this instance
    if (p_ble_trg1->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
    // Check if there is any message to be sent across to the peer and send it.
    tx_buffer_process();
}


/**@brief Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details This function will uses the Handle Value Notification received from the SoftDevice
 *          and checks if it is a notification of Button state from the peer. If
 *          it is, this function will decode the state of the button and send it to the
 *          application.
 *
 * @param[in] p_ble_trg1 Pointer to the Led Button Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_trg1_t * p_ble_trg1, ble_evt_t const * p_ble_evt)
{
    // Check if the event is on the link for this instance
    if (p_ble_trg1->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
}


/**@brief Function for handling Disconnected event received from the SoftDevice.
 *
 * @details This function check if the disconnect event is happening on the link
 *          associated with the current instance of the module, if so it will set its
 *          conn_handle to invalid.
 *
 * @param[in] p_ble_trg1 Pointer to the Led Button Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_trg1_t * p_ble_trg1, ble_evt_t const * p_ble_evt)
{
    if (p_ble_trg1->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_ble_trg1->conn_handle                    = BLE_CONN_HANDLE_INVALID;
        p_ble_trg1->peer_trg1_db.cccd_handle 			= BLE_GATT_HANDLE_INVALID;
        p_ble_trg1->peer_trg1_db.safety_alert_handle   = BLE_GATT_HANDLE_INVALID;
        p_ble_trg1->peer_trg1_db.alert_status_handle   = BLE_GATT_HANDLE_INVALID;
    }
}

void ble_trg1_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_trg1_t * p_ble_trg1 = (ble_trg1_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_trg1, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(p_ble_trg1, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_trg1, p_ble_evt);
            break;

        default:
            break;
    }
}

static ble_trg1_t m_ble_smph;
static ble_trg1_t m_ble_trg2;
static ble_trg1_t m_ble_mn;
NRF_SDH_BLE_OBSERVER(m_ble_smph_obs, 2, ble_trg1_on_ble_evt, &m_ble_smph);
NRF_SDH_BLE_OBSERVER(m_ble_trg2_obs, 2, ble_trg1_on_ble_evt, &m_ble_trg2);
NRF_SDH_BLE_OBSERVER(m_ble_mn_obs, 2, ble_trg1_on_ble_evt, &m_ble_mn);
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_discovery, 2);                      /**< Database discovery module instances. */

static char const m_target_smph[] = "Shanvi";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/
static char const m_target_trg2[] = "Sh_TRG2";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/
static char const m_target_mn[] = "Sh_MN";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/

/**@brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION <= 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION >= 3)
        .use_whitelist = 0,
    #endif
};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};

/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  type       Type of data to be looked for in advertisement data.
 * @param[in]  p_advdata  Advertisement report length and pointer to report.
 * @param[out] p_typedata If data type requested is found in the data report, type data length and
 *                        pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    (void) sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);

}


/**@brief Handles events coming from the LED Button central module.
 */
static void smph_evt_handler(ble_trg1_t * p_trg1, ble_trg1_evt_t * p_trg1_evt)
{
    switch (p_trg1_evt->evt_type)
    {
        case BLE_TRG1_EVT_DISCOVERY_COMPLETE:
        {
            if (m_ble_smph.conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                m_ble_smph.conn_handle = p_trg1_evt->conn_handle;
                NRF_LOG_INFO("SMPH discovered on conn_handle 0x%x", m_ble_smph.conn_handle);
                p_trg1->conn_handle = p_trg1_evt->conn_handle;
                p_trg1->peer_trg1_db = p_trg1_evt->params.peer_db;
            }
        } break; // BLE_TRG1_EVT_DISCOVERY_COMPLETE

        case BLE_TRG1_EVT_BUTTON_NOTIFICATION:
        default:
            // No implementation needed.
            break;
    }
}

static void trg2_evt_handler(ble_trg1_t * p_trg1, ble_trg1_evt_t * p_trg1_evt)
{
    switch (p_trg1_evt->evt_type)
    {
        case BLE_TRG1_EVT_DISCOVERY_COMPLETE:
        {
            if (m_ble_trg2.conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                m_ble_trg2.conn_handle = p_trg1_evt->conn_handle;
                NRF_LOG_INFO("TRG2 discovered on conn_handle 0x%x", m_ble_trg2.conn_handle);
                p_trg1->conn_handle = p_trg1_evt->conn_handle;
                p_trg1->peer_trg1_db = p_trg1_evt->params.peer_db;
            }
        } break; // BLE_TRG1_EVT_DISCOVERY_COMPLETE

        case BLE_TRG1_EVT_BUTTON_NOTIFICATION:
        default:
            // No implementation needed.
            break;
    }
}

static void mn_evt_handler(ble_trg1_t * p_trg1, ble_trg1_evt_t * p_trg1_evt)
{
    switch (p_trg1_evt->evt_type)
    {
        case BLE_TRG1_EVT_DISCOVERY_COMPLETE:
        {
            if (m_ble_mn.conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                m_ble_mn.conn_handle = p_trg1_evt->conn_handle;
                NRF_LOG_INFO("MN discovered on conn_handle 0x%x", m_ble_mn.conn_handle);
                p_trg1->conn_handle = p_trg1_evt->conn_handle;
                p_trg1->peer_trg1_db = p_trg1_evt->params.peer_db;
            }
        } break; // BLE_TRG1_EVT_DISCOVERY_COMPLETE

        case BLE_TRG1_EVT_BUTTON_NOTIFICATION:
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_adv_report(const ble_evt_t * const p_ble_evt)
{
    ret_code_t err_code;
    data_t     adv_data;
    data_t     dev_name;
    bool       do_connect = false;

    // For readibility.
    ble_gap_evt_t  const * p_gap_evt  = &p_ble_evt->evt.gap_evt;
    ble_gap_addr_t const * peer_addr  = &p_gap_evt->params.adv_report.peer_addr;

    // Initialize advertisement report for parsing
    adv_data.p_data   = (uint8_t *)p_gap_evt->params.adv_report.data;
    adv_data.data_len = p_gap_evt->params.adv_report.dlen;

    // Search for advertising names.
    bool name_found = false;
    err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &dev_name);

    if (err_code != NRF_SUCCESS)
    {
        // Look for the short local name if it was not found as complete.
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
        if (err_code != NRF_SUCCESS)
        {
            // If we can't parse the data, then exit
            return;
        }
        else
        {
            name_found = true;
        }
    }
    else
    {
        name_found = true;
    }

    if (name_found)
    {
        if (strlen(m_target_smph) != 0)
        {
            if (m_ble_smph.max_num_send > 0 &&
            		memcmp(m_target_smph, dev_name.p_data, dev_name.data_len )== 0)
            {
                do_connect = true;
            }
        }
        if (strlen(m_target_trg2) != 0)
        {
            if (m_ble_trg2.max_num_send > 0 &&
            		memcmp(m_target_trg2, dev_name.p_data, dev_name.data_len )== 0)
            {
                do_connect = true;
            }
        }
        if (strlen(m_target_mn) != 0)
        {
            if (m_ble_mn.max_num_send > 0 &&
            		memcmp(m_target_mn, dev_name.p_data, dev_name.data_len )== 0)
            {
                do_connect = true;
            }
        }
    }

    if (do_connect)
    {
    	NRF_LOG_INFO("Connecting to %d %s", dev_name.data_len, dev_name.p_data);
    	NRF_LOG_HEXDUMP_INFO(dev_name.p_data, dev_name.data_len);
        // Initiate connection.
        err_code = sd_ble_gap_connect(peer_addr,
                                      &m_scan_params,
                                      &m_connection_param,
                                      APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

	static uint16_t last_evt = 0;
	uint16_t cur_evt = p_ble_evt->header.evt_id + p_gap_evt->conn_handle +
			m_ble_smph.conn_handle + m_ble_trg2.conn_handle + m_ble_mn.conn_handle;
	if (last_evt != cur_evt) {
	    NRF_LOG_DEBUG("ble_evt_handler 0x%x %d %d %d %d", p_ble_evt->header.evt_id,
	    		p_gap_evt->conn_handle, m_ble_smph.conn_handle,
	    		m_ble_trg2.conn_handle, m_ble_mn.conn_handle);
	}
    last_evt = cur_evt;
    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected.");
            if (   (m_ble_smph.conn_handle  == BLE_CONN_HANDLE_INVALID)
                           || (m_ble_trg2.conn_handle == BLE_CONN_HANDLE_INVALID)
                        		   || (m_ble_mn.conn_handle == BLE_CONN_HANDLE_INVALID)) {
            	NRF_LOG_INFO("Attempt to find SMPH, TRG2, or MN on conn_handle 0x%x", p_gap_evt->conn_handle);
            	err_code = ble_db_discovery_start(&m_db_discovery[0], p_gap_evt->conn_handle);
            	if (err_code == NRF_ERROR_BUSY)
            	{
            		err_code = ble_db_discovery_start(&m_db_discovery[1], p_gap_evt->conn_handle);
            		APP_ERROR_CHECK(err_code);
            	}
            	else
            	{
            		APP_ERROR_CHECK(err_code);
            	}
            }
        } break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            if (p_gap_evt->conn_handle == m_ble_smph.conn_handle) {
                NRF_LOG_INFO("SMPH central disconnected (reason: %d)",
                             p_gap_evt->params.disconnected.reason);

                m_ble_smph.conn_handle = BLE_CONN_HANDLE_INVALID;
                m_ble_smph.ready_to_send = 0;
            }
            if (p_gap_evt->conn_handle == m_ble_trg2.conn_handle) {
                NRF_LOG_INFO("TRG2 central disconnected (reason: %d)",
                             p_gap_evt->params.disconnected.reason);

                m_ble_trg2.conn_handle = BLE_CONN_HANDLE_INVALID;
                m_ble_trg2.ready_to_send = 0;
            }
            if (p_gap_evt->conn_handle == m_ble_mn.conn_handle) {
                NRF_LOG_INFO("MN central disconnected (reason: %d)",
                             p_gap_evt->params.disconnected.reason);

                m_ble_mn.conn_handle = BLE_CONN_HANDLE_INVALID;
                m_ble_mn.ready_to_send = 0;
            }

            if (   (m_ble_smph.conn_handle == BLE_CONN_HANDLE_INVALID)
                || (m_ble_trg2.conn_handle  == BLE_CONN_HANDLE_INVALID)
				|| (m_ble_mn.conn_handle  == BLE_CONN_HANDLE_INVALID))
            {
                // Start scanning
                scan_start();
            }
        } break;

        case BLE_GAP_EVT_ADV_REPORT:
        {
            on_adv_report(p_ble_evt);
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_trg1_init(ble_trg1_t * p_ble_trg1, ble_trg1_init_t * p_ble_trg1_init, uint16_t uuid)
{
    uint32_t      err_code;
    ble_uuid_t    trg1_uuid;
    ble_uuid128_t trg1_base_uuid = {TRG1_UUID_BASE};

    VERIFY_PARAM_NOT_NULL(p_ble_trg1);
    VERIFY_PARAM_NOT_NULL(p_ble_trg1_init);
    VERIFY_PARAM_NOT_NULL(p_ble_trg1_init->evt_handler);

    p_ble_trg1->peer_trg1_db.cccd_handle 			= BLE_GATT_HANDLE_INVALID;
    p_ble_trg1->peer_trg1_db.safety_alert_handle   = BLE_GATT_HANDLE_INVALID;
    p_ble_trg1->peer_trg1_db.alert_status_handle   = BLE_GATT_HANDLE_INVALID;

    p_ble_trg1->conn_handle                    = BLE_CONN_HANDLE_INVALID;
    p_ble_trg1->evt_handler                    = p_ble_trg1_init->evt_handler;
    p_ble_trg1->ready_to_send                  = 0;
    p_ble_trg1->max_num_send                   = 2;

    err_code = sd_ble_uuid_vs_add(&trg1_base_uuid, &p_ble_trg1->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    VERIFY_SUCCESS(err_code);

    trg1_uuid.type = p_ble_trg1->uuid_type;
    trg1_uuid.uuid = uuid;

    return ble_db_discovery_evt_register(&trg1_uuid);
}

/**@brief TRG1 client initialization.
 */
static void trg1_init(void)
{
    ret_code_t       err_code;
    ble_trg1_init_t smph_init_obj;
    ble_trg1_init_t trg2_init_obj;
    ble_trg1_init_t mn_init_obj;

    smph_init_obj.evt_handler = smph_evt_handler;
    trg2_init_obj.evt_handler = trg2_evt_handler;
    mn_init_obj.evt_handler = mn_evt_handler;

    err_code = ble_trg1_init(&m_ble_smph, &smph_init_obj, SMPH_UUID_SERVICE);
    APP_ERROR_CHECK(err_code);

    err_code = ble_trg1_init(&m_ble_trg2, &trg2_init_obj, TRG2_UUID_SERVICE);
    APP_ERROR_CHECK(err_code);

    err_code = ble_trg1_init(&m_ble_mn, &mn_init_obj, MN_UUID_SERVICE);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


void ble_trg1_on_db_disc_evt(ble_trg1_t * p_ble_trg1, ble_db_discovery_evt_t const * p_evt, uint16_t uuid)
{
	 NRF_LOG_INFO("ble_trg1_on_db_disc_evt 0x%x 0x%x 0x%x %d %d", p_evt->evt_type,
			 p_evt->params.discovered_db.srv_uuid.uuid, uuid,
			 p_evt->params.discovered_db.srv_uuid.type, p_ble_trg1->uuid_type);
    // Check if the Safety Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        (p_evt->params.discovered_db.srv_uuid.uuid == uuid) &&
        p_evt->params.discovered_db.srv_uuid.type == p_ble_trg1->uuid_type)
    {
        ble_trg1_evt_t evt;

        evt.evt_type    = BLE_TRG1_EVT_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;

		for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++) {
			const ble_gatt_db_char_t * p_char =
					&(p_evt->params.discovered_db.charateristics[i]);

			switch (p_char->characteristic.uuid.uuid) {
            case TRG1_SAFETY_ALERT:
            	NRF_LOG_HEXDUMP_INFO(&p_char->characteristic.uuid.uuid, 2);
                evt.params.peer_db.safety_alert_handle = p_char->characteristic.handle_value;
                break;
            case TRG1_ALERT_STATUS:
                evt.params.peer_db.alert_status_handle      = p_char->characteristic.handle_value;
                evt.params.peer_db.cccd_handle 				= p_char->cccd_handle;
                break;
			default:
				break;
			}
		}
		if (p_ble_trg1->conn_handle != BLE_CONN_HANDLE_INVALID) {
			if ((p_ble_trg1->peer_trg1_db.cccd_handle == BLE_GATT_HANDLE_INVALID)
					&& (p_ble_trg1->peer_trg1_db.safety_alert_handle == BLE_GATT_HANDLE_INVALID)
					&& (p_ble_trg1->peer_trg1_db.alert_status_handle	== BLE_GATT_HANDLE_INVALID)) {
				p_ble_trg1->peer_trg1_db = evt.params.peer_db;
			}
		}
        p_ble_trg1->evt_handler(p_ble_trg1, &evt);
        p_ble_trg1->ready_to_send = 1;
    }
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_trg1_on_db_disc_evt(&m_ble_smph, p_evt, SMPH_UUID_SERVICE);
    ble_trg1_on_db_disc_evt(&m_ble_trg2, p_evt, TRG2_UUID_SERVICE);
    ble_trg1_on_db_disc_evt(&m_ble_mn, p_evt, MN_UUID_SERVICE);
}


/**@brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the log.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Power manager. */
static void power_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

static void disconnect(ble_trg1_t * p_ble_trg1) {
	uint32_t      err_code;
	err_code = sd_ble_gap_disconnect(p_ble_trg1->conn_handle,
	                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	APP_ERROR_CHECK(err_code);
}

static uint32_t send_data(ble_trg1_t * p_ble_trg1, uint8_t status) {
	if (p_ble_trg1->max_num_send <= 0) {
		return NRF_ERROR_INVALID_STATE;
	}
    VERIFY_PARAM_NOT_NULL(p_ble_trg1);

    if (p_ble_trg1->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    NRF_LOG_INFO("writing device ID 0x%x", status);
    device_id[0] = status;

    tx_message_t * p_msg;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = p_ble_trg1->peer_trg1_db.safety_alert_handle;
    p_msg->req.write_req.gattc_params.len      = sizeof(device_id);
    p_msg->req.write_req.gattc_params.p_value  = device_id;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_CMD;
    p_msg->conn_handle                         = p_ble_trg1->conn_handle;
    p_msg->type                                = WRITE_REQ;

    tx_buffer_process();
    if (p_ble_trg1->max_num_send > 0) {
    	p_ble_trg1->max_num_send --;
    }
    if (p_ble_trg1->max_num_send <= 0) {
    	p_ble_trg1->ready_to_send = 0;
    	disconnect(p_ble_trg1);
    }
    return NRF_SUCCESS;
}

int main(void)
{
    log_init();
    timer_init();
    power_init();

	ble_stack_init();
	gatt_init();
	db_discovery_init();
	trg1_init();


    NRF_LOG_INFO("TRG1 started.");

	uint8_t button = 18;
	nrf_gpio_cfg_input(button, NRF_GPIO_PIN_PULLUP);

	// Start scanning for peripherals and initiate connection to devices which
    // advertise.
    scan_start();

    uint8_t status = 0;
    for (;;)
    {
    	NRF_LOG_INFO("TRG1 %d %d %d", m_ble_smph.ready_to_send, m_ble_trg2.ready_to_send, m_ble_mn.ready_to_send);
    	NRF_LOG_INFO("TRG1 %d %d %d", m_ble_smph.max_num_send, m_ble_trg2.max_num_send, m_ble_mn.max_num_send);
    	status = nrf_gpio_pin_read(button);
    	send_data(&m_ble_smph, 1 - status);
    	send_data(&m_ble_trg2, 1 - status);
    	send_data(&m_ble_mn, 1 - status);
    	status++;
    	nrf_delay_ms(2000);
        NRF_LOG_FLUSH();
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
    }
}
