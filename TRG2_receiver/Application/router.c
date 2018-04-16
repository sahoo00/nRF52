/*
 * router.c
 *
 *  Created on: Apr 15, 2018
 *      Author: Debashis
 */

#include <stdbool.h>
#include <stdint.h>

#include "nordic_common.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "app_timer.h"

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "common.h"

static sh_packet_t sh_queue[10];
static uint8_t sh_num = 0;
static uint8_t sh_start = 0;
static uint8_t sh_total = 10;

bool sendData(sh_packet_t * p_pkt);

uint8_t get_pkt_len(sh_packet_t * p_pkt) {
	switch (p_pkt->trgn.type) {
	case PKT_TRGN: return sizeof(sh_trg_packet_t);
	case PKT_TRGC: return sizeof(sh_trg_packet_t);
	case PKT_ACK: return sizeof(sh_ack_packet_t);
	case PKT_LOC: return sizeof(sh_gps_packet_t);
	default:
		break;
	}
	return 0;
}

uint8_t get_q_count(sh_packet_t * p_pkt) {
	switch (p_pkt->trgn.type) {
	case PKT_TRGN: return p_pkt->trgn.q_count;
	case PKT_TRGC: return p_pkt->trgc.q_count;
	case PKT_ACK: return p_pkt->ack.q_count;
	case PKT_LOC: return p_pkt->location.q_count;
	default:
		break;
	}
	return 0;
}

void set_q_count(sh_packet_t * p_pkt, uint8_t c) {
	switch (p_pkt->trgn.type) {
	case PKT_TRGN: p_pkt->trgn.q_count = c; break;
	case PKT_TRGC: p_pkt->trgc.q_count = c; break;
	case PKT_ACK: p_pkt->ack.q_count = c; break;
	case PKT_LOC: p_pkt->location.q_count = c; break;
	default:
		break;
	}
}

uint8_t get_h_count(sh_packet_t * p_pkt) {
	switch (p_pkt->trgn.type) {
	case PKT_TRGN: return p_pkt->trgn.h_count;
	case PKT_TRGC: return p_pkt->trgc.h_count;
	case PKT_ACK: return p_pkt->ack.h_count;
	case PKT_LOC: return p_pkt->location.h_count;
	default:
		break;
	}
	return 0;
}

uint8_t * get_h_ptr(sh_packet_t * p_pkt) {
	switch (p_pkt->trgn.type) {
	case PKT_TRGN: return &p_pkt->trgn.h_count;
	case PKT_TRGC: return &p_pkt->trgc.h_count;
	case PKT_ACK: return &p_pkt->ack.h_count;
	case PKT_LOC: return &p_pkt->location.h_count;
	default:
		break;
	}
	NRF_LOG_INFO( "Pkt type: %d", p_pkt->trgn.type);
	return NULL;
}

bool isPresentInQueue(sh_packet_t * p_pkt) {
	bool found = false;
	for (uint8_t i = 0; i < sh_num; i++) {
		uint8_t pos = (sh_start + i) % sh_total;
		if (memcmp(&sh_queue[pos], p_pkt, 17) == 0) {
			found = true;
		}
	}
	return found;
}

void addQueue(sh_packet_t * p_pkt) {
	NRF_LOG_INFO( "Add begin start:%d, num:%d", sh_start, sh_num);
	if (p_pkt->trgn.type > PKT_ACK) {
		return;
	}
	uint8_t hc = get_h_count(p_pkt);
	uint8_t qc = get_q_count(p_pkt);
	NRF_LOG_INFO( "Add begin hc:%d, qc:%d", hc, qc);
	if (hc > 10) {
		return;
	}
	if (qc > 10) {
		return;
	}
	set_q_count(p_pkt, qc + 1);

	bool found = isPresentInQueue(p_pkt);
	if (!found) {
		uint8_t pos = (sh_start + sh_num) % sh_total;
		sh_queue[pos] = p_pkt[0];
		if (sh_num < sh_total) {
			sh_num ++;
		}
		else {
			sh_start = (sh_start + 1) % sh_total;
		}

	}

	NRF_LOG_INFO( "Add end start:%d, num:%d", sh_start, sh_num);
}

uint32_t removeQueue(sh_packet_t * p_pkt) {

	uint32_t status = NRF_SUCCESS;
	if (sh_num <= 0) {
		status = NRF_ERROR_NOT_FOUND;
		return status;
	}

	NRF_LOG_INFO( "Remove begin start:%d, num:%d", sh_start, sh_num);

	if (sh_num > 0) {
		p_pkt[0] = sh_queue[sh_start];
		sh_num --;
		sh_start = (sh_start + 1) % sh_total;
	}
	else {
		status = NRF_ERROR_NOT_FOUND;
	}

	//esp_log_buffer_hex(TAG, sh_queue, sizeof(sh_trg_packet_t));
	NRF_LOG_INFO( "Remove end start:%d, num:%d", sh_start, sh_num);
	return status;
}

void router(void * p_context)
{
	if (sh_num > 0) {
		set_trg2_state(TRG2_ROUTER);
	}
    	sh_packet_t pkt;
    	if (removeQueue(&pkt) == NRF_SUCCESS) {
    		if (!sendData(&pkt)) {
    			addQueue(&pkt);
    		}
    	}
    	if (sh_num <= 0) {
    		set_trg2_state(TRG2_SIGNAL);
    	}
}


void router_start()
{
	NRF_LOG_INFO( "Router start:%d, num:%d", sh_start, sh_num);
}

void router_stop()
{

}

void processData(const uint8_t *data, int len) {
	sh_packet_t pkt;
	bzero(&pkt, sizeof(pkt));
	memcpy(&pkt, data, len);
	set_q_count(&pkt, 0);
	addQueue(&pkt);
	if (pkt.trgn.type == PKT_CMD && len == 2) {
		if (data[1] == 0x00) {
			router_stop();
		}
		if (data[1] == 0x01) {
			router_start();
		}
		if (data[1] == 0x04) {
			sh_packet_t pkt;
			if (removeQueue(&pkt) == NRF_SUCCESS) {
				if (!sendData(&pkt)) {
					addQueue(&pkt);
				}
			}
		}
	}
}

void add_hop(void * data, uint8_t h) {
	NRF_LOG_INFO( "Add hop:%d", h);
	sh_packet_t * p_pkt = (sh_packet_t *) data;
	uint8_t * hptr = get_h_ptr(p_pkt);
	hptr[1 + hptr[0]] = h;
	hptr[0] = (hptr[0]+1) % MAX_HOPS_DATA;
}

bool check_hop(void * data, uint8_t h) {
	sh_packet_t * p_pkt = (sh_packet_t *) data;
	uint8_t * hptr = get_h_ptr(p_pkt);
	NRF_LOG_INFO( "h count:0x%x", (int)hptr);
	bool found = false;
	for (uint8_t i = 0; i < hptr[0]; i++) {
		if (hptr[1 + i] == h) {
			found = true;
		}
	}
	return found;
}

void get_hex_buffer(char * str, const void * p, int len) {
	for (uint32_t i = 0; i < len; i++) {
		sprintf(&str[2 * i], "%02x", ((uint8_t*) p)[i]);
	}
}

bool sendData(sh_packet_t * p_pkt) {
	if (isSMPHConnected()) {
		uint8_t hop = get_smph_hop();
		add_hop(p_pkt, hop);
		send_smph_c_data(p_pkt, get_pkt_len(p_pkt));
		return true;
	}
	if (isMNConnected()) {
		uint8_t hop = get_mn_hop();
		add_hop(p_pkt, hop);
		send_mn_c_data(p_pkt, get_pkt_len(p_pkt));
		return true;
	}
	return false;
}


