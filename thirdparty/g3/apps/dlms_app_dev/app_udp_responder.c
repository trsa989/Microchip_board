/**
 * \file
 *
 * \brief UDP responder application, required in G3 Conformance test
 *
 * Copyright (c) 2018 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "compiler.h"

#include "app_udp_responder.h"
#include "Logger.h"
#include "error.h"

#include "app_adp_mng.h"
#include "ipv6_mng.h"
#include "drivers/g3/network_adapter_g3.h"
#include "async_ping.h"

#ifdef DLMS_DEBUG_CONSOLE
#       define LOG_APP_DEBUG(a)   printf a
#else
#       define LOG_APP_DEBUG(a)   (void)0
#endif

/* Vars for conformance socket communication */
/* Socket for the UDP over PLC communication */
Socket *spx_conf_socket;
/* Local IP address */
IpAddr sx_conf_local_ip_addr;
/* Local Socket PORT */
uint16_t sus_conf_udp_port;
/* Socket opened */
bool b_is_conf_socket_open = false;

extern uint8_t g_u8NetworkJoinStatus;
extern bool b_ipv6_initialized;

/* Maximum length of IPv6 PDUs for DLMS application */
#define MAX_LENGTH_IPv6_PDU 1200

uint8_t udp_buffer[MAX_LENGTH_IPv6_PDU]; /* Serial data buffer */
uint16_t u16_length = 0;

/**
 * \brief Function to decide whether Conformance configuration has to be set
 *        This function is intended to be implemented by the user using its own mechanisms
 *
 */
static bool _enter_conformance_mode(void)
{
#ifdef APP_CONFORMANCE_TEST
	return true;

#else
	return false;
#endif
}

/**
 * \brief Process UDP message for G3-PLC Conformance test
 */
static void _udp_responder_process_message(uint8_t *pUdpPayload, uint16_t u16UdpPayloadLength)
{
	LOG_APP_DEBUG(("[UDP_RESPONDER] _udp_responder_process_message()"));
	bool x_error;
	size_t x_transmitted_bytes;

	if (u16UdpPayloadLength > 0) {
		if (pUdpPayload[0] == 0x01) {
			/*
			 * In order to validate RFC6282 UDP header compression, an exchange of frames transported over UDP is required. For this purpose a very
			 * simple UDP responder needs to be implemented.
			 * - The device listen to port 0xF0BF over UDP.
			 * - The first byte of UDP payload indicate the message type, the rest of the UDP payload correspond to the message data:
			 * - 0x01(UDP request): upon reception, the device must send back an UDP frame to the original sender, using the received frame source
			 * and destination ports for the destination and source
			 * ports (respectively) of the response frame, setting the message type to 0x02 (UDP reply) and copying the message data from the
			 * request;
			 * - 0x02 (UDP reply): this message is dropped upon reception;
			 * - other value: this message is dropped upon reception;
			 */

			/* UDP responder needed for conformance testing */
			/* update UPD payload */
			pUdpPayload[0] = 0x02;

			udp_socket_send(spx_conf_socket, &sx_conf_local_ip_addr, sus_conf_udp_port, pUdpPayload, u16UdpPayloadLength, &x_transmitted_bytes);
		} else if (pUdpPayload[0] == 0x03) {
			/*
			 * The following extension is added to the UDP responder, in order to make the IUT generate ICMPv6
			 * ECHO Request frames.
			 * The new message type 0x03 (ICMPv6 ECHO request trigger) is added: upon reception, the device
			 * must send back an ICMPv6 ECHO request frame to the original sender. The ICMPv6 Identifier,
			 * Sequence Number and Data fields are filled (in that order) using the received message data.
			 * Example: If an UDP message with a payload of ?03 010203040506070809? is received, then an
			 * ICMPv6 echo request is sent back with an ICMPv6 content of ?80 00 xxxx 0102 0304 0506070809?
			 * (where xxxx correspond to the ICMP checksum).
			 */

			x_error = async_ping_send_with_content(&netInterface[0], &sx_conf_local_ip_addr, u16UdpPayloadLength - 1, &pUdpPayload[1], CONFORMANCE_PING_TTL,
					CONFORMANCE_PING_TTL * 1000);

			if (x_error == NO_ERROR) {
				LOG_APP_DEBUG(("\r\n[UDP_RESPONDER] Ping Sent ok!\r\n"));
				/* ul_pings_sent += 1; */
				/* uc_ping_sent = 1; */
			} else {
				LOG_APP_DEBUG(("\r\n[UDP_RESPONDER] Fail sending Ping!  Error: %d \r\n", x_error));
			}
		} else {
			LOG_APP_DEBUG(("[UDP_RESPONDER] Drop UDP message: invalid protocol"));
		}
	} else {
		LOG_APP_DEBUG(("[UDP_RESPONDER] Drop UDP message: invalid header / length"));
	}
}

/**
 * \brief Timers of UDP Responder Application
 *
 */
void udp_responder_app_timers_update(void)
{
}

/**
 * \brief Process of UDP Responder Application
 *
 */
void udp_responder_app_process(void)
{
	if (_enter_conformance_mode()) {
		if (ipv6_mng_ready()) {
			if (b_is_conf_socket_open && udp_socket_is_open(spx_conf_socket)) {
				/* Process UDP messages for conformance */
				if (udp_socket_receive(spx_conf_socket, &sx_conf_local_ip_addr, &sus_conf_udp_port, udp_buffer, MAX_LENGTH_IPv6_PDU, (size_t *)(&u16_length))) {
					if (u16_length > 0) {
						_udp_responder_process_message(udp_buffer, u16_length);
					}
				}
			} else {
				/* Open sockets */
				b_is_conf_socket_open = udp_socket_open(spx_conf_socket, CONFORMANCE_SOCKET_PORT);
			}
		}
	}
}

/**
 * \brief Initialize UDP Responder Application
 *
 */
void udp_responder_app_init(void)
{
	if (_enter_conformance_mode()) {
		/* Initialize socket */
		spx_conf_socket = udp_socket_initialize();
	}
}
