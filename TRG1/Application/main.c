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
#include "ble_nus_c.h"
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


#define SMPH_UUID_BASE        {0xc6, 0x1d, 0x93, 0x6e, 0x09, 0x03, 0x63, 0x8b, \
                              0xe8, 0x49, 0xa5, 0xcf, 0x00, 0x00, 0xba, 0xde}
#define TRG2_UUID_BASE        {0xc6, 0x1d, 0x93, 0x6e, 0x09, 0x03, 0x63, 0x8b, \
                              0xe8, 0x49, 0xa5, 0xcf, 0x01, 0x00, 0xba, 0xde}
#define MN_UUID_BASE          {0xc6, 0x1d, 0x93, 0x6e, 0x09, 0x03, 0x63, 0x8b, \
                              0xe8, 0x49, 0xa5, 0xcf, 0x02, 0x00, 0xba, 0xde}

#define SMPH_UUID_SERVICE     0x0000
#define TRG2_UUID_SERVICE     0x0001
#define MN_UUID_SERVICE       0x0002
#define TRG1_SAFETY_ALERT     0x0010
#define TRG1_ALERT_STATUS     0x0011

#define BLE_UUID_TRG1_SERVICE 0xbade

#define TRG1_DEVICE_ID        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
static uint8_t device_id[17] = {0x00, TRG1_DEVICE_ID};


NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_NUS_C_DEF(m_ble_smph_c);                                             /**< BLE NUS service client instance. */
BLE_NUS_C_DEF(m_ble_trg2_c);                                             /**< BLE NUS service client instance. */
BLE_NUS_C_DEF(m_ble_mn_c);                                             /**< BLE NUS service client instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< DB discovery module instance. */
//BLE_DB_DISCOVERY_ARRAY_DEF(m_db_discovery, 3);

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static ble_uuid128_t const m_trg2_uuid = {TRG2_UUID_BASE};
static ble_uuid128_t const m_smph_uuid = {SMPH_UUID_BASE};
static ble_uuid128_t const m_mn_uuid = {MN_UUID_BASE};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
    (uint16_t)SLAVE_LATENCY,            // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
};

/** @brief Parameters used when scanning. */
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

static char const m_target_smph[] = "Shanvi";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/
static char const m_target_trg2[] = "Sh_TRG2";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/
static char const m_target_mn[] = "Sh_MN";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/

/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t * p_data;   /**< Pointer to data. */
    uint16_t  data_len; /**< Length of data. */
} data_t;

uint8_t isSMPHConnected() {
	return m_ble_smph_c.conn_handle != BLE_CONN_HANDLE_INVALID;
}

uint8_t isTRG2Connected() {
	return m_ble_trg2_c.conn_handle != BLE_CONN_HANDLE_INVALID;
}
uint8_t isMNConnected() {
	return m_ble_mn_c.conn_handle != BLE_CONN_HANDLE_INVALID;
}

void trg1_disconnect() {
    uint32_t      err_code;
    if (isSMPHConnected()) {
    	err_code = sd_ble_gap_disconnect(m_ble_smph_c.conn_handle,
    	                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    	APP_ERROR_CHECK(err_code);
    }
    if (isTRG2Connected()) {
    	err_code = sd_ble_gap_disconnect(m_ble_trg2_c.conn_handle,
    	                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    	APP_ERROR_CHECK(err_code);
    }
    if (isMNConnected()) {
    	err_code = sd_ble_gap_disconnect(m_ble_mn_c.conn_handle,
    	                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    	APP_ERROR_CHECK(err_code);
    }
}

void send_trigger_smph(void) {
	NRF_LOG_INFO("Sending trigger SMPH\r\n");
    uint32_t ret_val;
	do {
		ret_val = ble_nus_c_string_send(&m_ble_smph_c, (uint8_t *)&device_id, sizeof(device_id));
		if ((ret_val != NRF_ERROR_INVALID_STATE)
				&& (ret_val != NRF_ERROR_BUSY)) {
			APP_ERROR_CHECK(ret_val);
		}
	} while (ret_val == NRF_ERROR_BUSY);
	uint32_t err_code = sd_ble_gap_disconnect(m_ble_smph_c.conn_handle,
	                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	APP_ERROR_CHECK(err_code);
}

void send_trigger_trg2(void) {
	NRF_LOG_INFO("Sending trigger TRG2\r\n");
    uint32_t ret_val;
	do {
		ret_val = ble_nus_c_string_send(&m_ble_trg2_c, (uint8_t *)&device_id, sizeof(device_id));
		if ((ret_val != NRF_ERROR_INVALID_STATE)
				&& (ret_val != NRF_ERROR_BUSY)) {
			APP_ERROR_CHECK(ret_val);
		}
	} while (ret_val == NRF_ERROR_BUSY);
	uint32_t err_code = sd_ble_gap_disconnect(m_ble_trg2_c.conn_handle,
	                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	APP_ERROR_CHECK(err_code);
}

void send_trigger_mn(void) {
	NRF_LOG_INFO("Sending trigger MN\r\n");
    uint32_t ret_val;
	do {
		ret_val = ble_nus_c_string_send(&m_ble_mn_c, (uint8_t *)&device_id, sizeof(device_id));
		if ((ret_val != NRF_ERROR_INVALID_STATE)
				&& (ret_val != NRF_ERROR_BUSY)) {
			APP_ERROR_CHECK(ret_val);
		}
	} while (ret_val == NRF_ERROR_BUSY);
	uint32_t err_code = sd_ble_gap_disconnect(m_ble_mn_c.conn_handle,
	                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	APP_ERROR_CHECK(err_code);
}

/**@brief Callback handling NUS Client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS Client Handle. This identifies the NUS client
 * @param[in]   p_ble_nus_evt Pointer to the NUS Client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_smph_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    NRF_LOG_INFO("ble_nus_c_evt_handler 0x%x", p_ble_nus_evt->evt_type);
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete. SMPH");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            //err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            //APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Safety Service.");
            send_trigger_smph();
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected. SMPH");
            break;
    }
}
static void ble_trg2_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    NRF_LOG_INFO("ble_nus_c_evt_handler 0x%x", p_ble_nus_evt->evt_type);
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete. TRG2");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            //err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            //APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Safety Service.");
            send_trigger_trg2();
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected. TRG2");
            break;
    }
}
static void ble_mn_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    NRF_LOG_INFO("ble_nus_c_evt_handler 0x%x", p_ble_nus_evt->evt_type);
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete. MN");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            //err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            //APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Safety Service.");
            send_trigger_mn();
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected. MN");
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */

uint32_t ble_trg1_init(ble_nus_c_t * p_ble_nus_c, ble_nus_c_init_t * p_ble_nus_c_init,
		const ble_uuid128_t * uuid_base, const uint16_t uuid)
{
    uint32_t      err_code;
    ble_uuid_t    uart_uuid;

    VERIFY_PARAM_NOT_NULL(p_ble_nus_c);
    VERIFY_PARAM_NOT_NULL(p_ble_nus_c_init);

    err_code = sd_ble_uuid_vs_add(uuid_base, &p_ble_nus_c->uuid_type);
    VERIFY_SUCCESS(err_code);

    uart_uuid.type = p_ble_nus_c->uuid_type;
    uart_uuid.uuid = uuid;

    p_ble_nus_c->conn_handle           = BLE_CONN_HANDLE_INVALID;
    p_ble_nus_c->evt_handler           = p_ble_nus_c_init->evt_handler;
    p_ble_nus_c->handles.nus_tx_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_nus_c->handles.nus_rx_handle = BLE_GATT_HANDLE_INVALID;

    return ble_db_discovery_evt_register(&uart_uuid);
}

/**@brief Function for initializing the NUS Client. */
void trg1_client_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init_smph;
    ble_nus_c_init_t init_trg2;
    ble_nus_c_init_t init_mn;

    init_smph.evt_handler = ble_smph_c_evt_handler;
    init_trg2.evt_handler = ble_trg2_c_evt_handler;
    init_mn.evt_handler = ble_mn_c_evt_handler;

    err_code = ble_trg1_init(&m_ble_smph_c, &init_smph, &m_smph_uuid, SMPH_UUID_SERVICE);
    APP_ERROR_CHECK(err_code);
    err_code = ble_trg1_init(&m_ble_trg2_c, &init_trg2, &m_trg2_uuid, TRG2_UUID_SERVICE);
    APP_ERROR_CHECK(err_code);
    err_code = ble_trg1_init(&m_ble_mn_c, &init_mn, &m_mn_uuid, MN_UUID_SERVICE);
    APP_ERROR_CHECK(err_code);
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


/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static bool is_name_present(const ble_evt_t * const p_ble_evt)
{
    ret_code_t err_code;
    data_t     adv_data;
    data_t     dev_name;
    bool       do_connect = false;

    // For readibility.
    ble_gap_evt_t  const * p_gap_evt  = &p_ble_evt->evt.gap_evt;

    // Initialize advertisement report for parsing
    adv_data.p_data   = (uint8_t *)p_gap_evt->params.adv_report.data;
    adv_data.data_len = p_gap_evt->params.adv_report.dlen;

    // Search for advertising names.
    bool name_found = false;
    err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &dev_name);

    if (err_code != NRF_SUCCESS) {
        // Look for the short local name if it was not found as complete.
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
        if (err_code != NRF_SUCCESS) {
            // If we can't parse the data, then exit
            return false;
        }
        else {
            name_found = true;
        }
    }
    else {
        name_found = true;
    }

    if (name_found) {
        if (strlen(m_target_smph) != 0) {
            if (memcmp(m_target_smph, dev_name.p_data, dev_name.data_len )== 0)
            {
                do_connect = true;
            }
        }
        if (strlen(m_target_trg2) != 0) {
            if (memcmp(m_target_trg2, dev_name.p_data, dev_name.data_len )== 0)
            {
                do_connect = true;
            }
        }
        if (strlen(m_target_mn) != 0) {
            if (memcmp(m_target_mn, dev_name.p_data, dev_name.data_len )== 0)
            {
                do_connect = true;
            }
        }
    }

    return do_connect;
}

static bool is_uuid128_present(ble_uuid128_t               const * p_target_uuid,
                            ble_gap_evt_adv_report_t const * p_adv_report)
{
    uint16_t     index  = 0;
    uint8_t    * p_data = (uint8_t *)p_adv_report->data;

    while (index < p_adv_report->dlen)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];
        if (   (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
                 || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE))
        {
        	bool match = true;
        	//NRF_LOG_HEXDUMP_INFO(&p_data[index + 2], 16);
        	//NRF_LOG_HEXDUMP_INFO(&p_target_uuid->uuid128, 16);
            for (uint8_t i = 0; i < 16; i++) {
            	if (p_data[index + 2 + i] != p_target_uuid->uuid128[i]) {
            		match = false;
            	}
            }
            if (match) {
                return true;
            }
        }
        index += field_length + 1;
    }
    return false;
}

static bool is_uuid128_present_any(ble_gap_evt_adv_report_t const * p_adv_report) {
	return is_uuid128_present(&m_smph_uuid, p_adv_report) || is_uuid128_present(&m_trg2_uuid, p_adv_report) ||
			is_uuid128_present(&m_mn_uuid, p_adv_report);
}

/**@brief Function to start scanning. */
void scan_start(void)
{
    ret_code_t ret;

    NRF_LOG_INFO("scan_start");

    ret = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(ret);

}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
	static uint16_t last_evt = 0;
	if (last_evt != p_ble_evt->header.evt_id) {
		NRF_LOG_INFO("on_ble_central_evt 0x%x", p_ble_evt->header.evt_id);
	}
    last_evt = p_ble_evt->header.evt_id;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            ble_gap_evt_adv_report_t const * p_adv_report = &p_gap_evt->params.adv_report;

            if (is_name_present(p_ble_evt) || is_uuid128_present_any(p_adv_report)) {

                err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                              &m_scan_params,
                                              &m_connection_param,
                                              APP_BLE_CONN_CFG_TAG);
                NRF_LOG_INFO("Found %ld\r\n", err_code);
				if (err_code == NRF_SUCCESS) {
					// scan is automatically stopped by the connect
					NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
							p_adv_report->peer_addr.addr[0],
							p_adv_report->peer_addr.addr[1],
							p_adv_report->peer_addr.addr[2],
							p_adv_report->peer_addr.addr[3],
							p_adv_report->peer_addr.addr[4],
							p_adv_report->peer_addr.addr[5]);
				}
            }
        }
        break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected to target");
            /*err_code = ble_nus_c_handles_assign(&m_ble_smph_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

        	NRF_LOG_INFO("Attempt to find SMPH or TRG2 or MN on conn_handle 0x%x", p_gap_evt->conn_handle);
        	err_code = ble_db_discovery_start(&m_db_discovery[0], p_gap_evt->conn_handle);
        	if (0 && err_code == NRF_ERROR_BUSY) {
        		err_code = ble_db_discovery_start(&m_db_discovery[1], p_gap_evt->conn_handle);
        		if (err_code == NRF_ERROR_BUSY) {
        			err_code = ble_db_discovery_start(&m_db_discovery[2], p_gap_evt->conn_handle);
        		}
        	} */
        	err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
        	APP_ERROR_CHECK(err_code);

            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                NRF_LOG_INFO("Scan timed out.");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

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
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

void ble_trg1_on_db_disc_evt(ble_nus_c_t * p_ble_nus_c, ble_db_discovery_evt_t * p_evt, uint16_t uuid) {
    ble_nus_c_evt_t nus_c_evt;
    memset(&nus_c_evt,0,sizeof(ble_nus_c_evt_t));

    ble_gatt_db_char_t * p_chars = p_evt->params.discovered_db.charateristics;

    NRF_LOG_INFO("ble_trg1_on_db_disc_evt 0x%x", p_evt->evt_type);

    // Check if the NUS was discovered.
    if (    (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
        &&  (p_evt->params.discovered_db.srv_uuid.uuid == uuid)
        &&  (p_evt->params.discovered_db.srv_uuid.type == p_ble_nus_c->uuid_type))
    {
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            switch (p_chars[i].characteristic.uuid.uuid)
            {
                case TRG1_SAFETY_ALERT:
                	NRF_LOG_HEXDUMP_INFO(&p_chars[i].characteristic.uuid.uuid, 2);
                    nus_c_evt.handles.nus_rx_handle = p_chars[i].characteristic.handle_value;
                    break;

                case TRG1_ALERT_STATUS:
                    nus_c_evt.handles.nus_tx_handle = p_chars[i].characteristic.handle_value;
                    nus_c_evt.handles.nus_tx_cccd_handle = p_chars[i].cccd_handle;
                    break;

                default:
                    break;
            }
        }
        if (p_ble_nus_c->evt_handler != NULL)
        {
            nus_c_evt.conn_handle = p_evt->conn_handle;
            nus_c_evt.evt_type    = BLE_NUS_C_EVT_DISCOVERY_COMPLETE;
            p_ble_nus_c->evt_handler(p_ble_nus_c, &nus_c_evt);
        }
    }
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
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
    ble_trg1_on_db_disc_evt(&m_ble_smph_c, p_evt, SMPH_UUID_SERVICE);
    ble_trg1_on_db_disc_evt(&m_ble_trg2_c, p_evt, TRG2_UUID_SERVICE);
    ble_trg1_on_db_disc_evt(&m_ble_mn_c, p_evt, MN_UUID_SERVICE);
}


/** @brief Function for initializing the Database Discovery Module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
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


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
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


int main(void)
{
    log_init();
    timer_init();
    power_init();

	ble_stack_init();
	gatt_init();
	db_discovery_init();
	trg1_client_init();

	scan_start();
    NRF_LOG_INFO("TRG1 started.");

    for (;;)
    {
        NRF_LOG_FLUSH();
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
    }
}
