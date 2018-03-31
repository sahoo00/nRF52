/*
 * common.h
 *
 *  Created on: Mar 11, 2018
 *      Author: Debashis Sahoo
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <strings.h>
#include "nordic_common.h"
#include "ble.h"
#include "ble_db_discovery.h"
#include "nrf_ble_gatt.h"

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */
#define BUTTON_TRG2_S_PIN          		4                     			  /**< Pin number assigned to button */
#define LED_TRG2_S_PIN          		18                     			  /**< Pin number assigned to LED */
#define VIB_MOT_TRG2_S_PIN          	29                     			  /**< Pin number assigned to vibration motor */
#define GPS_TRG2_S_PIN          		3                     			  /**< Pin number assigned to GPS Enable */
#define GPS_TX_TRG2_S_PIN          		5                     			  /**< Pin number assigned to GPS TX */
#define GPS_RX_TRG2_S_PIN          		2                     			  /**< Pin number assigned to GPS RX */
#define GPS_RTS_TRG2_S_PIN          	11                     			  /**< Pin number assigned to RTS not used */
#define GPS_CTS_TRG2_S_PIN          	12                     			  /**< Pin number assigned to CTS not used */

#define UART_RX_TRG2_S_PIN          	14                     			  /**< Pin number assigned to UART RX */
#define UART_TX_TRG2_S_PIN          	15                     			  /**< Pin number assigned to UART TX */
#define UART_RTS_TRG2_S_PIN          	16                     			  /**< Pin number assigned to RTS not used */
#define UART_CTS_TRG2_S_PIN          	6                     			  /**< Pin number assigned to CTS not used */

#define APP_BLE_CONN_CFG_TAG    1                                       /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< Application's BLE observer priority. You shoulnd't need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define BLE_UUID_TRG2C1_SERVICE 0xbade

#define TRG2_UUID_BASE        {0xc6, 0x1d, 0x93, 0x6e, 0x09, 0x03, 0x63, 0x8b, \
                              0xe8, 0x49, 0xa5, 0xcf, 0x00, 0x00, 0xba, 0xde}
#define SMPH_UUID_SERVICE     0x0000
#define TRG2_UUID_SERVICE     0x0001
#define TRG2_SAFETY_ALERT     0x0010
#define TRG2_ALERT_STATUS     0x0011
#define TRG2_CMD_CHARSTIC     0x0012
#define TRG2_SIG_CHARSTIC     0x0013

#define TRG2_DEVICE_ID        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00


enum trg2_state_type {
	TRG2_IDLE,
 	TRG2_SIGNAL,
 	TRG2_TRIGGER,
 	TRG2_GPS
};

typedef struct trg2_signal_data {
	uint32_t trg2c_index;
	uint16_t sum;
	int8_t rssi;
	float score;
} trg2_signal_data_t;


uint8_t get_trg2_state();
void set_trg2_state(uint8_t state);
void schedule_test();

void leds_reset(void);
void vibrate(void);

void uart_init(void);
void uart_gps_init(void);
void print_gps_info();
void send_trigger(void);
void disable_gps();
void enable_gps();

void on_ble_peripheral_evt(ble_evt_t const * p_ble_evt, void * p_context);
void on_ble_central_evt(ble_evt_t const * p_ble_evt, void * p_context);
void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
void db_client_disc_handler(ble_db_discovery_evt_t * p_evt);
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);

void scan_start(void);
void buttons_leds_init(void);
void trg2_client_init(void);
void send_nus_c_data(void * p, uint16_t len);

void gap_params_init(void);
void advertising_init(void);
void services_init(void);
void advertising_start();
void conn_params_init(void);
void send_nus_data(void * p, uint16_t len);
void send_signal_data(trg2_signal_data_t * data);

void print_bytes(const void * p, int len);

