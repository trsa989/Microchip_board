/**
 * \file
 *
 * \brief Manufacture mode of the ATPL360 PLC transceiver.
 * This file manages the accesses to the ATPL360 component.
 *
 * Copyright (c) 2017 Atmel Corporation. All rights reserved.
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

/* ATPL360 includes */
#include "atpl360.h"
#include "atpl360_mnf.h"
#include "atpl360_hal_spi.h"

static atpl360_hal_wrapper_t *spx_hal_wrp;
static uint8_t spuc_mnf_test_buf[256];

#define ATPL360_MNF_DELAY_CMD             1

/**
 * \brief Function reverse buffer of 16 bytes.
 *
 * \param px_hal_wrp       Pointer to hardware abstraction component
 */
static void _inverse_buf_16(uint8_t *puc_buf)
{
	uint8_t tmp_buf[16];

	for (uint8_t uc_idx = 0; uc_idx < 16; uc_idx++) {
		tmp_buf[uc_idx] = puc_buf[15 - uc_idx];
	}

	memcpy(puc_buf, tmp_buf, 16);
}

/**
 * \brief Function to init manufacturing mode.
 *
 * \param px_hal_wrp       Pointer to hardware abstraction component
 */
void atpl360_mnf_init(atpl360_hal_wrapper_t *px_hal_wrp)
{
	uint8_t auc_value[4];

	/* Capture hardware spi wrapper */
	spx_hal_wrp = px_hal_wrp;
	spx_hal_wrp->plc_reset();

	/* Set in Boot command mode */
	auc_value[3] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 24);
	auc_value[2] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 16);
	auc_value[1] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 8);
	auc_value[0] = (uint8_t)(ATPL360_BOOT_WRITE_KEY);
	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_CMD_ENABLE_WRITE, 0, sizeof(auc_value), auc_value, NULL);

	auc_value[3] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 8);
	auc_value[2] = (uint8_t)(ATPL360_BOOT_WRITE_KEY);
	auc_value[1] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 24);
	auc_value[0] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 16);
	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_CMD_ENABLE_WRITE, 0, sizeof(auc_value), auc_value, NULL);
}

static bool atpl360_mnf_boot_is_rdy(void)
{
	uint8_t puc_value[4];

	memset(puc_value, 0, sizeof(puc_value));
	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_READ_BOOT_STATUS, 0, sizeof(puc_value), puc_value, puc_value);
	if (*(uint32_t *)puc_value) {
		return false;
	} else {
		return true;
	}
}

void atpl360_mnf_write_word(uint32_t ul_address, uint32_t ul_data)
{
	uint8_t auc_value[4];

	/* Set in Boot command mode */
	auc_value[3] = (uint8_t)(ul_data >> 24);
	auc_value[2] = (uint8_t)(ul_data >> 16);
	auc_value[1] = (uint8_t)(ul_data >> 8);
	auc_value[0] = (uint8_t)(ul_data);
	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_CMD_WRITE_WORD, ul_address, sizeof(auc_value), auc_value, NULL);
}

void atpl360_mnf_write_buff(uint32_t ul_address, uint8_t *puc_data, uint16_t us_len)
{
	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_CMD_WRITE_BUF, ul_address, us_len, puc_data, NULL);
}

uint32_t atpl360_mnf_read_word(uint32_t ul_address)
{
	uint8_t auc_value[4];

	/* Set in Boot command mode */
	memset(spuc_mnf_test_buf, 0, sizeof(spuc_mnf_test_buf));
	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_CMD_READ_WORD, ul_address, sizeof(auc_value), spuc_mnf_test_buf, auc_value);

	return(*((uint32_t *)auc_value));
}

void atpl360_mnf_read_buff(uint32_t ul_address, uint8_t *puc_data, uint16_t us_len)
{
	memset(spuc_mnf_test_buf, 0, sizeof(spuc_mnf_test_buf));
	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_CMD_READ_BUF, ul_address, us_len, spuc_mnf_test_buf, puc_data);
}

void atpl360_mnf_write_enc_fuses(uint8_t *puc_data)
{
	uint8_t puc_enc_fuse[16];

	memcpy(puc_enc_fuse, puc_data, 16);
	_inverse_buf_16(puc_enc_fuse);

	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_SET_128_FUSES_VALUE, 0, 16, puc_enc_fuse, NULL);
	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_SET_ENC_FUSES, 0, 4, puc_enc_fuse, NULL);
	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_BLOWN_FUSES, 0, 4, puc_enc_fuse, NULL);

	/* Test Bootloader status : Fuses */
	while (!atpl360_mnf_boot_is_rdy()) {
		spx_hal_wrp->plc_delay(DELAY_TREF_MS, ATPL360_MNF_DELAY_CMD);
	}
}

void atpl360_mnf_read_enc_fuses(uint8_t *puc_data)
{
	uint8_t puc_enc_fuse[16];

	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_GET_ENC_FUSES, 0, 4, puc_enc_fuse, NULL);

	/* Test Bootloader status : Fuses */
	while (!atpl360_mnf_boot_is_rdy()) {
		spx_hal_wrp->plc_delay(DELAY_TREF_MS, ATPL360_MNF_DELAY_CMD);
	}

	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_READ_TAMPER_REG, 0, 16, puc_enc_fuse, puc_enc_fuse);

	memcpy(puc_data, puc_enc_fuse, 16);
	_inverse_buf_16(puc_data);
}

void atpl360_mnf_write_tag_fuses(uint8_t *puc_data)
{
	uint8_t puc_enc_fuse[16];

	memcpy(puc_enc_fuse, puc_data, 16);
	_inverse_buf_16(puc_enc_fuse);

	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_SET_128_FUSES_VALUE, 0, 16, puc_enc_fuse, NULL);
	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_SET_TAG_FUSES, 0, 4, puc_enc_fuse, NULL);
	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_BLOWN_FUSES, 0, 4, puc_enc_fuse, NULL);

	/* Test Bootloader status : Fuses */
	while (!atpl360_mnf_boot_is_rdy()) {
		spx_hal_wrp->plc_delay(DELAY_TREF_MS, ATPL360_MNF_DELAY_CMD);
	}
}

void atpl360_mnf_read_tag_fuses(uint8_t *puc_data)
{
	uint8_t puc_enc_fuse[16];

	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_GET_TAG_FUSES, 0, 4, puc_enc_fuse, NULL);

	/* Test Bootloader status : Fuses */
	while (!atpl360_mnf_boot_is_rdy()) {
		spx_hal_wrp->plc_delay(DELAY_TREF_MS, ATPL360_MNF_DELAY_CMD);
	}

	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_READ_TAMPER_REG, 0, 16, puc_enc_fuse, puc_enc_fuse);

	memcpy(puc_data, puc_enc_fuse, 16);
	_inverse_buf_16(puc_data);
}

void atpl360_mnf_write_ctrl_fuses(uint32_t ul_ctrl_mask)
{
	uint8_t puc_value[16];

	memset(puc_value, 0, sizeof(puc_value));

	puc_value[0] = (uint8_t)(ul_ctrl_mask);
	puc_value[1] = (uint8_t)(ul_ctrl_mask >> 8);
	puc_value[2] = (uint8_t)(ul_ctrl_mask >> 16);
	puc_value[3] = (uint8_t)(ul_ctrl_mask >> 24);

	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_SET_128_FUSES_VALUE, 0, 16, puc_value, NULL);
	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_SET_CTRL_FUSES, 0, 4, puc_value, NULL);
	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_BLOWN_FUSES, 0, 4, puc_value, NULL);

	/* Test Bootloader status : Fuses */
	while (!atpl360_mnf_boot_is_rdy()) {
		spx_hal_wrp->plc_delay(DELAY_TREF_MS, ATPL360_MNF_DELAY_CMD);
	}

	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_GET_CTRL_FUSES, 0, 4, puc_value, NULL);

	/* Test Bootloader status : Fuses */
	while (!atpl360_mnf_boot_is_rdy()) {
		spx_hal_wrp->plc_delay(DELAY_TREF_MS, ATPL360_MNF_DELAY_CMD);
	}

	printf("Write CTRL_FUSES: 0x%08x\r\n", ul_ctrl_mask);
}

uint32_t atpl360_mnf_read_ctrl_fuses(void)
{
	uint8_t puc_value[16];
	uint32_t ul_res;

	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_GET_CTRL_FUSES, 0, 4, puc_value, NULL);

	/* Test Bootloader status : Fuses */
	while (!atpl360_mnf_boot_is_rdy()) {
		spx_hal_wrp->plc_delay(DELAY_TREF_MS, ATPL360_MNF_DELAY_CMD);
	}

	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_READ_TAMPER_REG, 0, 16, puc_value, puc_value);

	ul_res = (uint32_t)puc_value[3] << 24;
	ul_res += (uint32_t)puc_value[2] << 16;
	ul_res += (uint32_t)puc_value[1] << 8;
	ul_res += (uint32_t)puc_value[0];

	printf("Read CTRL_FUSES: 0x%08x\r\n", ul_res);

	return ul_res;
}

void atpl360_mnf_boot_window(void)
{
	uint8_t puc_value[4];

	spx_hal_wrp->plc_send_boot_cmd(ATPL360_BOOT_STOP_WINDOW, 0, 4, puc_value, NULL);
}
