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

#define SMPH_UUID_BASE        {0xc6, 0x1d, 0x93, 0x6e, 0x09, 0x03, 0x63, 0x8b, \
                              0xe8, 0x49, 0xa5, 0xcf, 0x00, 0x00, 0xba, 0xde}
#define SMPH_UUID_SERVICE     0x0000
#define TRG2_UUID_SERVICE     0x0001
#define MN_UUID_SERVICE       0x0002
#define TRG2_SAFETY_ALERT     0x0010
#define TRG2_ALERT_STATUS     0x0011
#define TRG2_CMD_CHARSTIC     0x0012
#define TRG2_SIG_CHARSTIC     0x0013
#define TRG2_GPS_CHARSTIC     0x0014

#define MN_UUID_BASE          {0xc6, 0x1d, 0x93, 0x6e, 0x09, 0x03, 0x63, 0x8b, \
                              0xe8, 0x49, 0xa5, 0xcf, 0x02, 0x00, 0xba, 0xde}

#define TRG2_DEVICE_ID        0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define TRG2_CKEY        	  "73b1ed59ebfdd3e3"

#define MAX_HOPS_DATA 5

enum trg2_state_type {
	TRG2_IDLE,
 	TRG2_SIGNAL,
 	TRG2_TRIGGER,
 	TRG2_GPS,
	TRG2_ROUTER,
};

typedef struct trg2_signal_data {
	uint8_t type;
	uint32_t trg2c_index;
	uint16_t sum;
	int8_t rssi;
	float score;
} trg2_signal_data_t;

struct nmea_position {
	uint16_t degree;
	uint8_t minute_value;
	uint32_t minute_decimal;
};

struct nmea_time {
	uint8_t hh;
	uint8_t mm;
	uint8_t ss_value;
	uint8_t ss_decimal;
};

enum nmea_gll_type {
	DTM = 0,
	GBQ,
	GBS,
	GGA,
	GLL,
	GLQ,
	GNQ,
	GNS,
	GPQ,
	GRS,
	GSA,
	GST,
	GSV,
	RMC,
	TXT,
	VLW,
	VTG,
	ZDA,
};

enum nmea_gll_status {
    GLL_STATUS_DATA_VALID = 'A',
    GLL_STATUS_DATA_NOT_VALID = 'V',
};

enum nmea_faa_mode {
    FAA_MODE_AUTONOMOUS = 'A',
    FAA_MODE_DIFFERENTIAL = 'D',
    FAA_MODE_ESTIMATED = 'E',
    FAA_MODE_MANUAL = 'M',
    FAA_MODE_SIMULATED = 'S',
    FAA_MODE_NOT_VALID = 'N',
    FAA_MODE_PRECISE = 'P',
};

struct nmea_gll_data {
	uint8_t type;
	struct nmea_position latitude;
	uint8_t ns;
	struct nmea_position longitude;
	uint8_t ew;
	struct nmea_time utc;
	uint8_t status;
	uint8_t mode;
	uint8_t checksum;
};

//$GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B
struct nmea_gga_data {
	uint8_t type;
	struct nmea_time utc;
	struct nmea_position latitude;
	uint8_t ns;
	struct nmea_position longitude;
	uint8_t ew;
	uint8_t quality;
	uint8_t numSV;
	float HDOP;
	float alt;
	uint8_t uAlt;
	float sep;
	uint8_t uSep;
};

typedef struct trg2_gps_data {
	uint8_t device_id[17];
	struct nmea_gga_data gga_data;
} trg2_gps_data_t;

enum {
	PKT_TRGN,
	PKT_TRGC,
	PKT_LOC,
	PKT_ACK,
	PKT_CMD
};

typedef struct sh_trg_packet {
	uint8_t type;
	uint8_t device_id[16];
	int ckey;
	uint8_t q_count;
	uint8_t h_count;
	uint8_t hops[MAX_HOPS_DATA];
} sh_trg_packet_t;

typedef struct sh_gps_packet {
	uint8_t type;
	uint8_t device_id[16];
	float lat;
	float lon;
	float alt;
	uint8_t q_count;
	uint8_t h_count;
	uint8_t hops[MAX_HOPS_DATA];
} sh_gps_packet_t;

typedef struct sh_ack_packet {
	uint8_t type;
	uint8_t device_id[16];
	uint8_t q_count;
	uint8_t h_count;
	uint8_t hops[MAX_HOPS_DATA];
} sh_ack_packet_t;

typedef struct sh_info_packet {
	uint8_t type;
	long long device_id;
	long long device_id_2;
} sh_info_packet_t;


typedef struct sh_packet {
	union {
		sh_trg_packet_t trgn;
		sh_trg_packet_t trgc;
		sh_gps_packet_t location;
		sh_ack_packet_t ack;
		sh_info_packet_t info;
	};
} sh_packet_t;

uint8_t get_trg2_state();
void set_trg2_state(uint8_t state);
void schedule_test();
void trg2c_event_trigger();
void trg2c_event_reset();

void leds_reset(void);
void vibrate(void);

void uart_init(void);
void uart_gps_init(void);
void print_gps_info();
void send_trigger(void);
void disable_gps();
void enable_gps();

void send_gps_data(void);
struct nmea_gll_data * get_gll_data();
struct nmea_gga_data * get_gga_data();
sh_gps_packet_t get_sh_gps_packet();

void on_ble_peripheral_evt(ble_evt_t const * p_ble_evt, void * p_context);
void on_ble_central_evt(ble_evt_t const * p_ble_evt, void * p_context);
void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
void db_client_disc_handler(ble_db_discovery_evt_t * p_evt);
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);

void scan_start(void);
void buttons_leds_init(void);
void trg2_client_init(void);
void send_smph_c_data(void * p, uint16_t len);
void send_mn_c_data(void * p, uint16_t len);
uint8_t isSMPHConnected();
uint8_t isMNConnected();
void reset_client();
void trg2c_disconnect_smph();
void trg2c_disconnect_mn();
void trg2c_disconnect();
uint8_t get_smph_hop();
uint8_t get_mn_hop();

void gap_params_init(void);
void advertising_init(void);
void services_init(void);
void advertising_start();
void conn_params_init(void);
void send_nus_data(void * p, uint16_t len);
void send_signal_data(trg2_signal_data_t * data);
void send_gps_notify_data(uint8_t * data, uint16_t len);
bool gps_notification_enabled();

void print_bytes(const void * p, int len);

void processData(const uint8_t *data, int len);
void router_start();
void router(void * p_context);
