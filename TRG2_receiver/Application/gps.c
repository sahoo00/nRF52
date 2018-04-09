/*
 * gps.c
 *
 *  Created on: Mar 11, 2018
 *      Author: Debashis Sahoo
 */

#include "ble_nus.h"
#include "ble_nus_c.h"
#include "app_uart.h"

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "common.h"

#define GPS_MAX_DATA_LEN 256
char data_gps_array[GPS_MAX_DATA_LEN];
uint16_t index_gps = 0;
uint16_t error_gps = 0;


struct nmea_gll_data init_gll_data() {
	struct nmea_gll_data res;
	bzero(&res, sizeof(res));
	res.status = GLL_STATUS_DATA_NOT_VALID;
	res.mode = FAA_MODE_NOT_VALID;
	return res;
}

struct nmea_gga_data init_gga_data() {
	struct nmea_gga_data res;
	bzero(&res, sizeof(res));
	return res;
}

struct nmea_gll_data gll_data;
struct nmea_gga_data gga_data;

struct nmea_gll_data * get_gll_data() {
	return &gll_data;
}

struct nmea_gga_data * get_gga_data() {
	return &gga_data;
}

static uint8_t hex2int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

static float read_float(const char* array, int size) {
	float res = 0;
	int val = 0;
	int dec = 0;
	int scale = 1;
	uint8_t decimal = 0;
	for (uint8_t i = 0; i < size; i++) {
		if (array[i] >= '0' && array[i] <= '9' && decimal == 0) {
			val = val * 10 + (array[i] - '0');
		}
		if (array[i] >= '0' && array[i] <= '9' && decimal == 1) {
			dec = dec * 10 + (array[i] - '0');
			scale = scale * 10;
		}
		if (array[i] == '.') {
			decimal = 1;
		}
		if (array[i] == '-') {
			scale = -1;
		}
		if (array[i] != '-' || array[i] != '.' || array[i] < '0' || array[i] > '9') {
			//break;
		}
	}
	res = val + dec * 1.0 / scale;
	return res;
}

//$GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B
struct nmea_gga_data parse_gga_data(const char* array, int size) {
	struct nmea_gga_data res = init_gga_data();
	uint8_t indices[20];
	uint8_t index_size = 0;
	const char * gll = "$GNGGA,";
	uint8_t index_cur = 0;
	//NRF_LOG_HEXDUMP_INFO(array, 8);
	//if (strncmp(array, "$GNGGA", 6) == 0) {
		//NRF_LOG_HEXDUMP_INFO(array, 20);
	//}
	for (uint8_t i = 0; i < 7; i++) {
		if (array[i] == gll[i] || i == 2) {
			index_cur++;
		}
	}
	if (index_cur != 7) {
		return res;
	}
	res.type = GBQ;
	for (uint8_t i =0; i < size; i++) {
		if (array[i] == ',' || array[i] == '*') {
			indices[index_size] = i;
			index_size++;
		}
	}
	if (index_size != 15) {
		return res;
	}
	res.type = GGA;
    if ( (indices[1]+1) == indices[2] ) {
    	res.type = GBQ;
    }
	uint8_t start = indices[0];
	for (uint8_t i = start + 1; i < indices[1]; i++) {
		if (i < start + 3) {
			res.utc.hh = res.utc.hh * 10 + (array[i] - '0');
		}
		if (i >= start + 3 && i < start + 5) {
			res.utc.mm = res.utc.mm * 10 + (array[i] - '0');
		}
		if (i >= start + 5 && i < start + 7) {
			res.utc.ss_value = res.utc.ss_value * 10 + (array[i] - '0');
		}
		if (i >= start + 8 && i < start + 10) {
			res.utc.ss_decimal = res.utc.ss_decimal * 10 + (array[i] - '0');
		}
	}
	//NRF_LOG_INFO("%d %d %d %d", res.utc.hh, res.utc.mm, res.utc.ss_value, res.utc.ss_decimal);
	start = indices[1];
	for (uint8_t i = start + 1; i < indices[2]; i++) {
		if (i < start + 3) {
			res.latitude.degree = res.latitude.degree * 10 + (array[i] - '0');
		}
		if (i >= start + 3 && i < start + 5) {
			res.latitude.minute_value = res.latitude.minute_value * 10
					+ (array[i] - '0');
		}
		if (i >= start + 6 && i < start + 11) {
			res.latitude.minute_decimal = res.latitude.minute_decimal * 10
					+ (array[i] - '0');
		}
	}
	//NRF_LOG_INFO("%d %d %d", res.latitude.degree, res.latitude.minute_value, res.latitude.minute_decimal);
	start = indices[2];
	for (uint8_t i = start + 1; i < indices[3]; i++) {
		res.ns = array[i];
	}
	//NRF_LOG_INFO("%c", res.ns);
	start = indices[3];
	for (uint8_t i = start + 1; i < indices[4]; i++) {
		if (i < start + 4) {
			res.longitude.degree = res.longitude.degree * 10 + (array[i] - '0');
		}
		if (i >= start + 4 && i < start + 6) {
			res.longitude.minute_value = res.longitude.minute_value * 10
					+ (array[i] - '0');
		}
		if (i >= start + 7 && i < start + 12) {
			res.longitude.minute_decimal = res.longitude.minute_decimal * 10
					+ (array[i] - '0');
		}
	}
	//NRF_LOG_INFO("%d %d %d", res.longitude.degree, res.longitude.minute_value, res.longitude.minute_decimal);
	start = indices[4];
	for (uint8_t i = start + 1; i < indices[5]; i++) {
		res.ew = array[i];
	}
	//NRF_LOG_INFO("%c", res.ew);
	start = indices[5];
	for (uint8_t i = start + 1; i < indices[6]; i++) {
		res.quality = array[i];
	}
	//NRF_LOG_INFO("%c", res.quality);
	start = indices[6];
	for (uint8_t i = start + 1; i < indices[7]; i++) {
		res.numSV = (array[i] - '0') + 10 * res.numSV ;
	}
	//NRF_LOG_INFO("%d", res.numSV);
	start = indices[7];
	//NRF_LOG_HEXDUMP_INFO(array + indices[7], 8);
	res.HDOP = read_float(array + indices[7]+1, indices[8] - indices[7]);
	//NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(res.HDOP));
	res.alt = read_float(array + indices[8]+1, indices[9] - indices[8]);
	//NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(res.alt));
	res.uAlt = array[indices[9] + 1];
	res.sep = read_float(array + indices[10]+1, indices[11] - indices[10]);
	//NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(res.sep));
	res.uSep = array[indices[11] + 1];

	return res;
}

//$GNGLL,3250.77698,N,11713.06689,W,011608.00,A,A*6C
struct nmea_gll_data parse_gll_data(const char* array, int size) {
	struct nmea_gll_data res = init_gll_data();
	uint8_t indices[13];
	uint8_t index_size = 0;
	const char * gll = "$GNGLL,";
	uint8_t index_cur = 0;
	//NRF_LOG_HEXDUMP_INFO(array, 10);
	if (strncmp(array, "$GNGGA", 6) == 0) {
		NRF_LOG_HEXDUMP_INFO(array, 20);
	}
	for (uint8_t i = 0; i < 7; i++) {
		if (array[i] == gll[i] || i == 2) {
			index_cur++;
		}
	}
	if (index_cur != 7) {
		return res;
	}
	res.type = GLL;

	for (uint8_t i =0; i < size; i++) {
		if (array[i] == ',' || array[i] == '*') {
			indices[index_size] = i;
			index_size++;
		}
	}
	if (index_size != 8) {
		return res;
	}

	uint8_t start = indices[0];
	for (uint8_t i = start + 1; i < indices[1]; i++) {
		if (i < start + 3) {
			res.latitude.degree = res.latitude.degree * 10 + (array[i] - '0');
		}
		if (i >= start + 3 && i < start + 5) {
			res.latitude.minute_value = res.latitude.minute_value * 10
					+ (array[i] - '0');
		}
		if (i >= start + 6 && i < start + 11) {
			res.latitude.minute_decimal = res.latitude.minute_decimal * 10
					+ (array[i] - '0');
		}
	}
	//NRF_LOG_INFO("%d %d %d", res.latitude.degree, res.latitude.minute_value, res.latitude.minute_decimal);
	start = indices[1];
	for (uint8_t i = start + 1; i < indices[2]; i++) {
		res.ns = array[i];
	}
	//NRF_LOG_INFO("%c", res.ns);
	start = indices[2];
	for (uint8_t i = start + 1; i < indices[3]; i++) {
		if (i < start + 4) {
			res.longitude.degree = res.longitude.degree * 10 + (array[i] - '0');
		}
		if (i >= start + 4 && i < start + 6) {
			res.longitude.minute_value = res.longitude.minute_value * 10
					+ (array[i] - '0');
		}
		if (i >= start + 7 && i < start + 12) {
			res.longitude.minute_decimal = res.longitude.minute_decimal * 10
					+ (array[i] - '0');
		}
	}
	//NRF_LOG_INFO("%d %d %d", res.longitude.degree, res.longitude.minute_value, res.longitude.minute_decimal);
	start = indices[3];
	for (uint8_t i = start + 1; i < indices[4]; i++) {
		res.ew = array[i];
	}
	//NRF_LOG_INFO("%c", res.ew);
	start = indices[4];
	for (uint8_t i = start + 1; i < indices[5]; i++) {
		if (i < start + 3) {
			res.utc.hh = res.utc.hh * 10 + (array[i] - '0');
		}
		if (i >= start + 3 && i < start + 5) {
			res.utc.mm = res.utc.mm * 10 + (array[i] - '0');
		}
		if (i >= start + 5 && i < start + 7) {
			res.utc.ss_value = res.utc.ss_value * 10 + (array[i] - '0');
		}
		if (i >= start + 8 && i < start + 10) {
			res.utc.ss_decimal = res.utc.ss_decimal * 10 + (array[i] - '0');
		}
	}
	//NRF_LOG_INFO("%d %d %d %d", res.utc.hh, res.utc.mm, res.utc.ss_value, res.utc.ss_decimal);
	start = indices[5];
	for (uint8_t i = start + 1; i < indices[6]; i++) {
		res.status = array[i];
	}
	//NRF_LOG_INFO("%c", res.status);
	start = indices[6];
	for (uint8_t i = start + 1; i < indices[7]; i++) {
		res.mode = array[i];
	}
	//NRF_LOG_INFO("%c", res.mode);
	start = indices[7];
	for (uint8_t i = start + 1; i < size && i < start + 3; i++) {
		res.checksum = res.checksum * 16 + hex2int(array[i]);
	}
	//NRF_LOG_INFO("%2x", res.checksum);

	return res;
}

void uart_gps_event_handle(app_uart_evt_t * p_event)
{
    //static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint16_t index = 0;

    error_gps = 0;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get((uint8_t*)&data_gps_array[index]));
            index++;

            if ((data_gps_array[index - 1] == '\n') || (index >= (GPS_MAX_DATA_LEN)))
            {
            	//data_gps_array[index] = '\0';
				gga_data = parse_gga_data(data_gps_array, index);

				if (gga_data.type == GGA || gga_data.type == GBQ) {
					NRF_LOG_INFO("%s", data_gps_array);
					if (gga_data.type == GGA) {
						send_gps_data();
					}
					index_gps = index;
					nrf_delay_ms(2000); // This is important otherwise gps data doesn't work.
					//app_uart_flush();
					//app_uart_close();
				}
				if (0) {
					gll_data = parse_gll_data(data_gps_array, index);
					if (gll_data.type == GLL) {
						//NRF_LOG_INFO("%s", data_gps_array);
						index_gps = index;
						nrf_delay_ms(3000); // This is important otherwise gps data doesn't work.
						//app_uart_flush();
						//app_uart_close();
					}
				}
				index = 0;
            }
            break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            //NRF_LOG_ERROR("Communication error occurred while handling UART.");
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            error_gps = 1;
            break;

        case APP_UART_FIFO_ERROR:
            //NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            //APP_ERROR_HANDLER(p_event->data.error_code);
            error_gps = 1;
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the UART. */
void uart_gps_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = GPS_TX_TRG2_S_PIN, // 4, // 12, // RX_PIN_NUMBER,
        .tx_pin_no    = GPS_RX_TRG2_S_PIN, // 5, // 18, // TX_PIN_NUMBER,
        .rts_pin_no   = GPS_RTS_TRG2_S_PIN, // RTS_PIN_NUMBER,
        .cts_pin_no   = GPS_CTS_TRG2_S_PIN, // CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud9600
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_gps_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

void print_gps_info() {
	if (index_gps > 0) {
		NRF_LOG_INFO("%s", data_gps_array);
		NRF_LOG_FLUSH(); // This is important otherwise NRF LOG %s doesn't work
	    /*uart_init();
	    printf("%d %d.%ld\r\n", gll_data.latitude.degree, gll_data.latitude.minute_value, gll_data.latitude.minute_decimal);
	    nrf_delay_ms(3000); // This is important otherwise printf %s doesn't work.
	    app_uart_flush();
	    app_uart_close();
	    index_gps = 0;
	    data_gps_array[0] = '\0';
	    uart_gps_init();*/
	}
}

void disable_gps() {
	NRF_LOG_INFO("GPS disabled");
	app_uart_flush();
    app_uart_close();
	nrf_gpio_cfg_output(GPS_TRG2_S_PIN);
	nrf_gpio_pin_write(GPS_TRG2_S_PIN, 0);
	uart_init();
}

void enable_gps() {
	NRF_LOG_INFO("GPS enabled");
	app_uart_flush();
    app_uart_close();
	nrf_gpio_cfg_output(GPS_TRG2_S_PIN);
	nrf_gpio_pin_write(GPS_TRG2_S_PIN, 1);
	uart_gps_init();
}
