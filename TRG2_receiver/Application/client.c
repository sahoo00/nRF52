/*
 * client.c
 *
 *  Created on: Mar 11, 2018
 *      Author: Debashis Sahoo
 */


#include "nordic_common.h"
#include "app_uart.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "ble_nus_c.h"
#include "ble_hci.h"
#include "math.h"

#include "nrf_delay.h"
#include "nrf_nvic.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "common.h"

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define SCAN_INTERVAL           0x0030                                  /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0030                                  /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT            0x0000                                  /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS)         /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS)         /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           0                                       /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Determines supervision time-out in units of 10 millisecond. */

#define UUID16_SIZE             2                                       /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                                       /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                                      /**< Size of 128 bit UUID */

#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service back to the sender. */

uint8_t button_state = 0;

APP_TIMER_DEF(m_led_timer_id);
APP_TIMER_DEF(m_trg_timer_id);

BLE_NUS_C_DEF(m_ble_smph_c);                                             /**< BLE NUS service client instance. */
BLE_NUS_C_DEF(m_ble_mn_c);                                             /**< BLE NUS service client instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< DB discovery module instance. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static uint32_t trg2c_index = 0;
static uint8_t ready_to_send = 0;
static uint32_t trg2c_event_start = 0;

static uint8_t device_id[17] = {0x00, TRG2_DEVICE_ID};


static char const m_target_smph[] = "Shanvi";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/
static char const m_target_trg2[] = "Sh_TRG2";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/
static char const m_target_mn[] = "Sh_MN";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/

/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t * p_data;   /**< Pointer to data. */
    uint16_t  data_len; /**< Length of data. */
} data_t;

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

/**@brief NUS uuid. */
static ble_uuid_t const m_nus_uuid =
{
    .uuid = BLE_UUID_TRG2C1_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE
};

static ble_uuid128_t const m_smph_uuid = {SMPH_UUID_BASE};
static ble_uuid128_t const m_mn_uuid = {MN_UUID_BASE};
static uint8_t m_smph_addr[BLE_GAP_ADDR_LEN];
static uint8_t m_mn_addr[BLE_GAP_ADDR_LEN];

int8_t array[64];
uint16_t indexArray = 0;
uint16_t countArray = 0;
uint16_t totalArray = 50;
uint16_t window = 5;
uint8_t trg2c_trigger = 0;

uint8_t get_smph_hop() { return m_smph_addr[0]; }
uint8_t get_mn_hop() { return m_mn_addr[0]; }

static uint16_t getValue(int8_t rssi) {
    array[indexArray] = rssi;
    indexArray ++;
    countArray ++;
    if (indexArray > totalArray) { indexArray = 0; }
    if (countArray > totalArray) { countArray = totalArray; }
    uint16_t sum = 0;
    int16_t j = indexArray - 1;
    for (uint16_t i =0; i < countArray && i < window; i++) {
    	if ( j < 0) {
    		j = countArray - 1;
    	}
    	sum = sum + (150 + array[j]);
    	j -- ;
    }
    return sum;
}

static float getScore(int8_t rssi) {
	if (0) { // DFT transformation
	    float sum1 = 0;
	    float sum2 = 0;
		int16_t j = indexArray - 1;
		float pi = 3.14159;
		for (uint16_t i = 0; i < countArray; i++) {
			if (j < 0) {
				j = countArray - 1;
			}
			sum1 = sum1 + array[j] * cos(2 * pi * 3 * i / totalArray);
			sum2 = sum2 + array[j] * sin(2 * pi * 3 * i / totalArray);
			j--;
		}
	    return sum1 * sum1 + sum2 * sum2;
	}
	if (1) { // Correlation based
		int sum[4] = {0, 0, 0, 0};
		int16_t j = indexArray - 1;
		for (uint16_t i = 0; i < 40; i++) {
			if (j < 0) {
				j = countArray - 1;
			}
			sum[i/10] = sum[i/10] + array[j];
			j--;
		}
		int m1 = sum[0]; m1 = m1 > sum[2]? sum[2]: m1;
		int m2 = sum[1]; m2 = m2 < sum[3]? sum[3]: m2;
	    float score1 = (m1 - m2)*2.0*(m1-m2)*(m1-m2)/100;
		m1 = sum[0]; m1 = m1 > sum[3]? sum[3]: m1;
		m2 = sum[1]; m2 = m2 < sum[2]? sum[2]: m2;
		float score2 = (m1 - m2)*2.0*(m1-m2)*(m1-m2)/100;
		float score = score1;
		score = score < score2? score2:score;
		return score;
	}
	return 0;
}


/**
 * @brief Function for shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Shutdown handler");

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

/**@brief Function to start scanning. */
void scan_start_forced(void)
{
    ret_code_t ret;
    ret = sd_ble_gap_scan_start(&m_scan_params);
    NRF_LOG_INFO("scan_start: %d", ret);
    APP_ERROR_CHECK(ret);
}

/**@brief Function to start scanning. */
void scan_start(void)
{
    ret_code_t ret;
/*
    ret = sd_ble_gap_scan_stop();
	NRF_LOG_INFO("scan_stop: %d", ret);
    if ( ret == NRF_SUCCESS) {
    */
    	ret = sd_ble_gap_scan_start(&m_scan_params);
    	NRF_LOG_INFO("scan_start: %d", ret);
    	/*
    	APP_ERROR_CHECK(ret);
    }
    	 */
}

void ble_trg2_on_db_disc_evt(ble_nus_c_t * p_ble_nus_c, ble_db_discovery_evt_t * p_evt, uint16_t uuid) {
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
                case TRG2_SAFETY_ALERT:
                	NRF_LOG_HEXDUMP_INFO(&p_chars[i].characteristic.uuid.uuid, 2);
                    nus_c_evt.handles.nus_rx_handle = p_chars[i].characteristic.handle_value;
                    break;

                case TRG2_ALERT_STATUS:
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

/**@brief Function for handling characters received by the Nordic UART Service.
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(ble_nus_c_t * p_ble_nus_c, uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_DEBUG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
            ret_val = app_uart_put(p_data[i]);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if (p_data[data_len-1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }
    if (ECHOBACK_BLE_UART_DATA)
    {
        // Send data back to peripheral.
        do
        {
            ret_val = ble_nus_c_string_send(p_ble_nus_c, p_data, data_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
}

void send_smph_c_data(void * p, uint16_t len) {
    uint32_t ret_val;
    do
    {
    	uint16_t length = len;
        ret_val = ble_nus_c_string_send(&m_ble_smph_c, p, length);
        if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_BUSY) )
        {
            APP_ERROR_CHECK(ret_val);
        }
    } while (ret_val == NRF_ERROR_BUSY);
}

void send_mn_c_data(void * p, uint16_t len) {
    uint32_t ret_val;
    do
    {
    	uint16_t length = len;
        ret_val = ble_nus_c_string_send(&m_ble_mn_c, p, length);
        if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_BUSY) )
        {
            APP_ERROR_CHECK(ret_val);
        }
    } while (ret_val == NRF_ERROR_BUSY);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint16_t index = 0;
    //NRF_LOG_INFO("uart_event_handle (client.c): 0x%x", p_event->evt_type);
    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
            {
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);
#if 0
                send_smph_c_data(data_array, index);
#else
                send_nus_data(data_array, index);
#endif

                index = 0;
            }
            break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            //NRF_LOG_ERROR("Communication error occurred while handling UART. %d", p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            //NRF_LOG_ERROR("Error occurred in FIFO module used by UART. %d", p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the UART. */
void uart_init(void)
{
    ret_code_t err_code;
#if 1
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = UART_RX_TRG2_S_PIN, // 4, // 12, // RX_PIN_NUMBER,
        .tx_pin_no    = UART_TX_TRG2_S_PIN, // 5, // 18, // TX_PIN_NUMBER,
        .rts_pin_no   = UART_RTS_TRG2_S_PIN, // RTS_PIN_NUMBER,
        .cts_pin_no   = UART_CTS_TRG2_S_PIN, // CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };
#else
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };
#endif

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

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

    NRF_LOG_INFO("ble_smph_c_evt_handler 0x%x", p_ble_nus_evt->evt_type);
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete. SMPH");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Safety Service.");
            if (get_trg2_state() == TRG2_TRIGGER) {
            	ready_to_send = 2;
            	send_trigger();
            }
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            ble_nus_chars_received_uart_print(p_ble_nus_c, p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected. SMPH");
            //ready_to_send = 0;
            //scan_start();
            break;
    }
}

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_mn_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    NRF_LOG_INFO("ble_mn_c_evt_handler 0x%x", p_ble_nus_evt->evt_type);
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete. MN");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Safety Service.");
            if (get_trg2_state() == TRG2_TRIGGER) {
            	ready_to_send = 2;
            	send_trigger();
            }
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            ble_nus_chars_received_uart_print(p_ble_nus_c, p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected. MN");
            //ready_to_send = 0;
            //scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

void print_bytes(const void * p, int len) {
	for (uint32_t i = 0; i < len; i++) {
		printf(" %02x", ((uint8_t*) p)[i]);
	}
}

/**@brief Reads an advertising report and checks if a UUID is present in the service list.
 *
 * @details The function is able to search for 16-bit, 32-bit and 128-bit service UUIDs.
 *          To see the format of a advertisement packet, see
 *          https://www.bluetooth.org/Technical/AssignedNumbers/generic_access_profile.htm
 *
 * @param[in]   p_target_uuid The UUID to search for.
 * @param[in]   p_adv_report  Pointer to the advertisement report.
 *
 * @retval      true if the UUID is present in the advertisement report. Otherwise false
 */
static bool is_uuid_present(ble_uuid_t               const * p_target_uuid,
                            ble_gap_evt_adv_report_t const * p_adv_report)
{
    ret_code_t   err_code;
    ble_uuid_t   extracted_uuid;
    uint16_t     index  = 0;
    uint8_t    * p_data = (uint8_t *)p_adv_report->data;

    while (index < p_adv_report->dlen)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];
        //printf("%d %02x %d\r\n", field_length, field_type, p_adv_report->dlen);
        if ( field_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA) {
            err_code = sd_ble_uuid_decode(UUID16_SIZE,
                                          &p_data[4 + index + 2],
                                          &extracted_uuid);
            if (extracted_uuid.uuid == p_target_uuid->uuid) {
            	return true;
            }
        }
        else if (   (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
            || (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE))
        {
            for (uint32_t i = 0; i < (field_length / UUID16_SIZE); i++)
            {
                err_code = sd_ble_uuid_decode(UUID16_SIZE,
                                              &p_data[i * UUID16_SIZE + index + 2],
                                              &extracted_uuid);
                //printf("16 %04x\r\n", extracted_uuid.uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if (extracted_uuid.uuid == p_target_uuid->uuid)
                    {
                        return true;
                    }
                }
            }
        }
        else if (   (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_MORE_AVAILABLE)
                 || (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_COMPLETE))
        {
            for (uint32_t i = 0; i < (field_length / UUID32_SIZE); i++)
            {
                err_code = sd_ble_uuid_decode(UUID32_SIZE,
                                              &p_data[i * UUID32_SIZE + index + 2],
                                              &extracted_uuid);
                //printf("32 %04x\r\n", extracted_uuid.uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if (   (extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if (   (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
                 || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE))
        {
            err_code = sd_ble_uuid_decode(UUID128_SIZE, &p_data[index + 2], &extracted_uuid);
            //printf("128 %04x\r\n", extracted_uuid.uuid);
            if (err_code == NRF_SUCCESS)
            {
                if (   (extracted_uuid.uuid == p_target_uuid->uuid)
                    && (extracted_uuid.type == p_target_uuid->type))
                {
                    return true;
                }
            }
        }
        index += field_length + 1;
    }
    return false;
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
    	//NRF_LOG_INFO("Name: %s", dev_name.p_data);
    	NRF_LOG_HEXDUMP_INFO(dev_name.p_data, 16);
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
        //NRF_LOG_HEXDUMP_INFO(&p_data[index + 2], field_length);
        //NRF_LOG_INFO("f_len: %d, f_type: %d", field_length, field_type);
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
	return is_uuid128_present(&m_smph_uuid, p_adv_report) ||
			is_uuid128_present(&m_mn_uuid, p_adv_report);
}


void trg2c_event_trigger() {
	NRF_LOG_INFO("trg2c_event_trigger : %d", trg2c_trigger);
	if (trg2c_trigger == 0) {
		trg2c_event_start = trg2c_index;
		if (trg2c_index == trg2c_event_start || trg2c_index > (trg2c_event_start + 10)) {
			trg2c_trigger++;
		}
		schedule_test();
	}
	else {
		vibrate();
		if (trg2c_index == trg2c_event_start || trg2c_index > (trg2c_event_start + 10)) {
			trg2c_trigger++;
		}
	}

}

void trg2c_event_reset() {
	trg2c_trigger = 0;
}

void schedule_test() {
	NRF_LOG_INFO("schedule test");
	ret_code_t            err_code;
	err_code = app_timer_start(m_trg_timer_id, APP_TIMER_TICKS(10000), NULL);
	APP_ERROR_CHECK(err_code);
	vibrate();
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
void on_ble_central_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
	float score = 0;
	uint16_t sum = 0;
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

            if (get_trg2_state() == TRG2_SIGNAL && is_uuid_present(&m_nus_uuid, p_adv_report)) {
            	sum = getValue(p_adv_report->rssi);
            	score = getScore(p_adv_report->rssi);
            	trg2c_index++;
            	if (score > 200) {
            		trg2c_event_trigger();
            		score = 200;
            	}
            	trg2_signal_data_t sd = {trg2c_trigger, trg2c_index, sum, p_adv_report->rssi, score};
            	//NRF_LOG_INFO("%ld %d %d %f\r\n", sd.trg2c_index, sd.sum, sd.rssi, sd.score);
            	//if ((indexArray & 0x1) == 0) {
            		send_signal_data(&sd);
            	//}

            }
            if ( (get_trg2_state() == TRG2_TRIGGER ||  get_trg2_state() == TRG2_GPS ||
            		get_trg2_state() == TRG2_ROUTER) &&
            		(is_name_present(p_ble_evt) || is_uuid128_present_any(p_adv_report)) ) {

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
					if (is_uuid128_present(&m_smph_uuid, p_adv_report)) {
						memcpy(m_smph_addr, p_adv_report->peer_addr.addr, BLE_GAP_ADDR_LEN);
					}
					if (is_uuid128_present(&m_mn_uuid, p_adv_report)) {
						memcpy(m_mn_addr, p_adv_report->peer_addr.addr, BLE_GAP_ADDR_LEN);
					}
				}
            }
        }
        break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected to target");
            //nrf_delay_ms(600);
            /*err_code = ble_nus_c_handles_assign(&m_ble_smph_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code); */

            // Discover peer's services.
            memset(&m_db_disc, 0x00, sizeof(m_db_disc));
            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            NRF_LOG_INFO("Starting discovery: %d", err_code);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GAP_EVT_DISCONNECTED:
        	NRF_LOG_INFO("Disconnected from target");
        	memset(&m_db_disc, 0x00, sizeof(m_db_disc));
        	scan_start();
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


uint8_t isSMPHConnected() {
	return m_ble_smph_c.conn_handle != BLE_CONN_HANDLE_INVALID;
}
uint8_t isMNConnected() {
	return m_ble_mn_c.conn_handle != BLE_CONN_HANDLE_INVALID;
}
uint8_t isConnected() {
	return isSMPHConnected() || isMNConnected();
}

void trg2c_disconnect_smph() {
    uint32_t      err_code;
    if (isSMPHConnected()) {
    	err_code = sd_ble_gap_disconnect(m_ble_smph_c.conn_handle,
    	                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    	NRF_LOG_INFO("SMPH disconnect: %d", err_code);
    	APP_ERROR_CHECK(err_code);
    }
}
void trg2c_disconnect_mn() {
    uint32_t      err_code;
    if (isMNConnected()) {
        err_code = sd_ble_gap_disconnect(m_ble_mn_c.conn_handle,
        	                                BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        NRF_LOG_INFO("MN disconnect: %d", err_code);
        APP_ERROR_CHECK(err_code);
    }
}
void trg2c_disconnect() {
	trg2c_disconnect_smph();
	trg2c_disconnect_mn();
}

void reset_client() {
	disable_gps();
	set_trg2_state(TRG2_SIGNAL);
	trg2c_event_reset();
	trg2c_disconnect();
	scan_start();
}

void send_trigger_smph(void) {
	if (!isSMPHConnected()) {
		return;
	}
	NRF_LOG_INFO("Sending trigger SMPH\r\n");
    uint32_t ret_val;
	do {
		ret_val = ble_nus_c_string_send(&m_ble_smph_c, (uint8_t *)&device_id, sizeof(device_id));
		if ((ret_val != NRF_ERROR_INVALID_STATE)
				&& (ret_val != NRF_ERROR_BUSY)) {
			APP_ERROR_CHECK(ret_val);
		}
	} while (ret_val == NRF_ERROR_BUSY);
}
void send_trigger_mn(void) {
	if (!isMNConnected()) {
		return;
	}
	NRF_LOG_INFO("Sending trigger MN\r\n");
    uint32_t ret_val;
	do {
		ret_val = ble_nus_c_string_send(&m_ble_mn_c, (uint8_t *)&device_id, sizeof(device_id));
		if ((ret_val != NRF_ERROR_INVALID_STATE)
				&& (ret_val != NRF_ERROR_BUSY)) {
			APP_ERROR_CHECK(ret_val);
		}
	} while (ret_val == NRF_ERROR_BUSY);
}


void send_trigger(void) {

	NRF_LOG_INFO("Sending trigger %d %d\r\n", ready_to_send, get_trg2_state());
	NRF_LOG_FLUSH();
	if (!ready_to_send) return;
	if (get_trg2_state() != TRG2_TRIGGER) {
		// reset
		reset_client();
	}
	send_trigger_smph();
	send_trigger_mn();
	trg2c_event_reset();
	nrf_delay_ms(100);
	if (ready_to_send > 0) {
		ready_to_send--;
	}
	if (ready_to_send > 0) {
		schedule_test();
	}
	else {
		set_trg2_state(TRG2_GPS);
		ready_to_send = 2;
		enable_gps();
	}
}

void send_gps_data_smph() {
	if (!isSMPHConnected()) {
		return;
	}
	NRF_LOG_INFO("Sending GPS data SMPH\r\n");
    uint32_t ret_val;
	do {
		/*
		struct nmea_gga_data * p_gga_data = get_gga_data();
		trg2_gps_data_t gps_data = { {0x02, TRG2_DEVICE_ID}, p_gga_data[0] };
		*/
		sh_gps_packet_t loc = get_sh_gps_packet();
		ret_val = ble_nus_c_string_send(&m_ble_smph_c, (uint8_t *)&loc, sizeof(loc));
		if ((ret_val != NRF_ERROR_INVALID_STATE)
				&& (ret_val != NRF_ERROR_BUSY)) {
			APP_ERROR_CHECK(ret_val);
		}
	} while (ret_val == NRF_ERROR_BUSY);
}
void send_gps_data_mn() {
	if (!isMNConnected()) {
		return;
	}
	NRF_LOG_INFO("Sending GPS data MN\r\n");
    uint32_t ret_val;
	do {
		/*
		struct nmea_gga_data * p_gga_data = get_gga_data();
		trg2_gps_data_t gps_data = { {0x02, TRG2_DEVICE_ID}, p_gga_data[0] };
		*/
		sh_gps_packet_t loc = get_sh_gps_packet();
		ret_val = ble_nus_c_string_send(&m_ble_mn_c, (uint8_t *)&loc, sizeof(loc));
		if ((ret_val != NRF_ERROR_INVALID_STATE)
				&& (ret_val != NRF_ERROR_BUSY)) {
			APP_ERROR_CHECK(ret_val);
		}
	} while (ret_val == NRF_ERROR_BUSY);
}

void send_gps_data(void) {
	if (!isConnected()) {
		//scan_start();
		return;
	}
	if (!ready_to_send) return;
	if (get_trg2_state() != TRG2_GPS) {
		// reset
		reset_client();
	}

	send_gps_data_smph();
	send_gps_data_mn();

	trg2c_event_reset();
	nrf_delay_ms(100);
	if (ready_to_send > 0) {
		ready_to_send--;
	}
	if (ready_to_send <= 0) {
		reset_client();
	}
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    NRF_LOG_INFO("bsp_event_handler: 0x%x", event);
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_ble_smph_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

void test_trg2_handler(void * p_context) {
	NRF_LOG_INFO("Test trg2 handler: %d %d %d", get_trg2_state(), trg2c_trigger, ready_to_send);
	if (get_trg2_state() == TRG2_SIGNAL && trg2c_trigger > 1) {
		set_trg2_state(TRG2_TRIGGER);
		vibrate();
		vibrate();
		send_trigger();
	}
	if (get_trg2_state() == TRG2_TRIGGER) {
		send_trigger();
	}
	trg2c_event_reset();
}

uint32_t ble_trg2_init(ble_nus_c_t * p_ble_nus_c, ble_nus_c_init_t * p_ble_nus_c_init,
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
void trg2_client_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init_smph;
    ble_nus_c_init_t init_mn;

    init_smph.evt_handler = ble_smph_c_evt_handler;
    init_mn.evt_handler = ble_mn_c_evt_handler;

    err_code = ble_trg2_init(&m_ble_smph_c, &init_smph, &m_smph_uuid, SMPH_UUID_SERVICE);
    APP_ERROR_CHECK(err_code);
    err_code = ble_trg2_init(&m_ble_mn_c, &init_mn, &m_mn_uuid, MN_UUID_SERVICE);
    APP_ERROR_CHECK(err_code);

	/* Define a timer id used for 10Hz sample rate */
	err_code = app_timer_create(&m_trg_timer_id, APP_TIMER_MODE_SINGLE_SHOT, test_trg2_handler);
	APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{

    switch (pin_no)
    {
        case BUTTON_TRG2_S_PIN:
        	NRF_LOG_INFO("Send button state change %d", button_action);
        	if (button_action == 1) {
        		button_state = button_state ? 0:1;
        		if (button_state == 1) {
        			NRF_LOG_INFO("stop: %d", button_state);
        			leds_reset();
        		}
        		else {
        			NRF_LOG_INFO("start: %d", button_state);
        			//set_trg2_state(TRG2_SIGNAL);
        			//trg2c_event_reset();
        			//trg2c_disconnect();
        			//scan_start();
        			sd_nvic_SystemReset();
        		}
        	}
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}

void led_handler(void * p_context) {
	router(NULL);
	if (button_state == 0) {
		nrf_gpio_pin_toggle(LED_TRG2_S_PIN);
	}
}

/**@brief Function for initializing LEDs. */
void leds_reset(void)
{
	nrf_gpio_pin_write(LED_TRG2_S_PIN, 1);
}

/**@brief Function for initializing buttons and leds. */
void buttons_leds_init(void)
{
    APP_GPIOTE_INIT(1);
    /* Configure LED pin */
	nrf_gpio_cfg_output(LED_TRG2_S_PIN);
	leds_reset();

	ret_code_t err_code;

	/* Define a timer id used for 10Hz sample rate */
	err_code = app_timer_create(&m_led_timer_id, APP_TIMER_MODE_REPEATED, led_handler);
	APP_ERROR_CHECK(err_code);
	err_code = app_timer_start(m_led_timer_id, APP_TIMER_TICKS(5000), NULL);
	APP_ERROR_CHECK(err_code);

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {BUTTON_TRG2_S_PIN, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
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
void db_client_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_trg2_on_db_disc_evt(&m_ble_smph_c, p_evt, SMPH_UUID_SERVICE);
    ble_trg2_on_db_disc_evt(&m_ble_mn_c, p_evt, MN_UUID_SERVICE);
}

void vibrate(void) {
	nrf_gpio_cfg_output(VIB_MOT_TRG2_S_PIN);
	nrf_gpio_pin_write(VIB_MOT_TRG2_S_PIN, 1);
	nrf_delay_ms(60);
	nrf_gpio_pin_write(VIB_MOT_TRG2_S_PIN, 0);
}

