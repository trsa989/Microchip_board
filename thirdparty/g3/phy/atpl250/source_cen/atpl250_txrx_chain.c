/**
 * \file
 *
 * \brief ATPL250 Tx/Rx Chain Configuration
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
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

/* System includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Phy layer includes */
#include "atpl250.h"
#include "atpl250_common.h"
#include "atpl250_txrx_chain.h"

/* Pointers to external defined arrays */
extern const uint8_t cauc_abcd_fullgain[];
static const uint8_t *const puc_abcd_fullgain = cauc_abcd_fullgain;

/* Extern variables needed */
extern uint8_t uc_used_carriers;
extern uint8_t uc_num_symbols_fch;
extern uint8_t uc_agc_ext;
extern uint16_t us_agc_int;
extern uint8_t uc_legacy_mode;

/* Static variables */
static uint8_t auc_interleaver_regs[8];

/**
 * \brief Writes raw header to buffer for transmission
 *
 * \param p_fch_buf  Pointer to buffer containing FCH
 *
 */
void write_raw_header(uint8_t *p_fch_buf)
{
	/* Write from bit 0 */
	pplc_if_write16(REG_ATPL250_RAW_DATA_H16, 0);
	/* Write Header */
	pplc_if_write_rep(REG_ATPL250_RAW_DATA_VL8, 1, p_fch_buf, FCH_LEN);

	/* Config interleaver parameters */
	interleaver_config(uc_used_carriers, uc_num_symbols_fch, MOD_TYPE_BPSK_ROBO);
	/* Set start bit and end bit for hw chain, start bit not set because it is 0 */
	pplc_if_write32(REG_ATPL250_INTERLEAVER_CFG3_32, FCH_LEN_BITS - 1);

	/* Config chain to load data to interleaver */
	pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x152F);
	/* 15: Bypass second scrambler, set 6 repetitions */
	/* 2F: Move data to interleaver, load mode, tx mode, G3 mode, enable HW chain */
}

/**
 * \brief Writes raw payload to buffer for transmission
 *
 * \param p_tx_ctl        Pointer to Tx control structure
 * \param us_buf_offset   Offset from where to start writing from tx buffer
 *
 */
void write_raw_payload(struct phy_tx_ctl *p_tx_ctl, uint16_t us_buf_offset)
{
	uint32_t ul_rs_cfg;

	/* Disable HW chain to write raw payload */
	pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~0x01u));

	/* Write from bit 0 again, FCH is overwritten because it is not used anymore */
	pplc_if_write16(REG_ATPL250_RAW_DATA_H16, 0);
	/* Write Payload */
	pplc_if_write_rep(REG_ATPL250_RAW_DATA_VL8, 1, &p_tx_ctl->auc_tx_buf[FCH_LEN + us_buf_offset], p_tx_ctl->m_us_payload_len);

	/* Config interleaver parameters */
	interleaver_config(p_tx_ctl->m_uc_payload_carriers, p_tx_ctl->m_us_tx_payload_symbols, p_tx_ctl->e_mod_type);
	pplc_if_write32(REG_ATPL250_INTERLEAVER_CFG3_32, (p_tx_ctl->m_us_payload_len << 3) - 1);

	/* Config RS encoder */
	ul_rs_cfg = (uint32_t)0x06 << 24; /* Add 6 zeroes to flush convolutional encoder */
	ul_rs_cfg |= (uint32_t)(p_tx_ctl->m_us_payload_len) << 16; /* Payload length */
	ul_rs_cfg |= (0x40 | p_tx_ctl->m_uc_rs_parity); /* Enable encoder and set parity */
	pplc_if_write32(REG_ATPL250_RS_CFG_32, ul_rs_cfg);

	/* Config Tx chain depending on modulation */
	switch (p_tx_ctl->e_mod_type) {
	case MOD_TYPE_BPSK_ROBO:
		pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x132F);
		/* 13: Bypass second scrambler, set 4 repetitions */
		/* 2F: BPSK, Move data to interleaver, load mode, tx mode, G3 mode, enable HW chain */
		break;

	case MOD_TYPE_BPSK:
		pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x102F);
		/* 10: Bypass second scrambler, no repetitions */
		/* 2F: BPSK, Move data to interleaver, load mode, tx mode, G3 mode, enable HW chain */
		break;

	case MOD_TYPE_QPSK:
		pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x106F);
		/* 10: Bypass second scrambler, no repetitions */
		/* 6F: QPSK, Move data to interleaver, load mode, tx mode, G3 mode, enable HW chain */
		break;

	case MOD_TYPE_8PSK:
		pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x10AF);
		/* 10: Bypass second scrambler, no repetitions */
		/* AF: 8PSK, Move data to interleaver, load mode, tx mode, G3 mode, enable HW chain */
		break;

	case MOD_TYPE_QAM:
		pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x10EF);
		/* 10: Bypass second scrambler, no repetitions */
		/* EF: QAM, Move data to interleaver, load mode, tx mode, G3 mode, enable HW chain */
		break;
	}
}

/**
 * \brief Calculates Great Common Divisor between 2 numbers
 *
 * \param us_a  One of the numbers
 * \param us_b  The other number
 *
 * \return Value of Great Common Divisor
 */
static inline uint16_t _gcd(uint16_t us_a, uint16_t us_b)
{
	uint16_t us_r;

	do {
		us_r = us_a % us_b;
		us_a = us_b;
		us_b = us_r;
	} while (us_r);

	return us_a;
}

/**
 * \brief Configures Interleaver block depending on used carriers, number of symbols and modulation
 *
 * \param uc_num_carriers  Number of carriers
 * \param us_num_symbols   Number of symbols
 * \param e_mod_type       Modulation type
 *
 */
void interleaver_config(uint8_t uc_num_carriers, uint16_t us_num_symbols, enum mod_types e_mod_type)
{
	uint8_t uc_mi, uc_mj, uc_ni, uc_nj;
	uint16_t us_i;
	uint8_t uc_i_10, uc_swap_ij;

	/* Note n: Symbols, m: carriers */
	/* Initialize values */
	uc_nj = 1;
	uc_ni = 1;
	uc_mi = 1;
	uc_mj = 1;

	if (uc_legacy_mode) {
		switch (e_mod_type) {
		case MOD_TYPE_QPSK:
			us_num_symbols <<= 1;
			break;

		case MOD_TYPE_8PSK:
			us_num_symbols *= 3;
			break;

		case MOD_TYPE_QAM:
			us_num_symbols <<= 2;
			break;

		default:
			break;
		}
	} else {
		switch (e_mod_type) {
		case MOD_TYPE_BPSK_ROBO:
		case MOD_TYPE_BPSK:
			pplc_if_write8(REG_ATPL250_INTERLEAVER_BPSCR_H8, 0x00);
			break;

		case MOD_TYPE_QPSK:
			pplc_if_write8(REG_ATPL250_INTERLEAVER_BPSCR_H8, 0x01);
			break;

		case MOD_TYPE_8PSK:
			pplc_if_write8(REG_ATPL250_INTERLEAVER_BPSCR_H8, 0x02);
			break;

		case MOD_TYPE_QAM:
			pplc_if_write8(REG_ATPL250_INTERLEAVER_BPSCR_H8, 0x03);
			break;

		default:
			break;
		}
	}

	for (us_i = 3; us_i < us_num_symbols; us_i++) {
		if (_gcd(us_num_symbols, us_i) == 1) {
			uc_nj = us_i;
			break;
		}
	}

	for (us_i++; us_i < us_num_symbols; us_i++) {
		if (_gcd(us_num_symbols, us_i) == 1) {
			uc_ni = us_i;
			break;
		}
	}

	for (us_i = 3; us_i < uc_num_carriers; us_i++) {
		if (_gcd(uc_num_carriers, us_i) == 1) {
			uc_mi = us_i;
			break;
		}
	}

	for (us_i++; us_i < uc_num_carriers; us_i++) {
		if (_gcd(uc_num_carriers, us_i) == 1) {
			uc_mj = us_i;
			break;
		}
	}

	/* Check legacy mode */
	uc_swap_ij = 0;
	if (uc_legacy_mode) {
		/* Calculate I_10: I(1,0) = ( mi + ( ni  % n) × mj ) % m */
		uc_i_10 = (uc_mi + (uc_ni % us_num_symbols) * uc_mj) % uc_num_carriers;
		if (uc_i_10 == 0) {
			uc_swap_ij = 1;
		}
	}

	auc_interleaver_regs[0] = 0x00;
	auc_interleaver_regs[1] = uc_num_carriers;
	auc_interleaver_regs[2] = us_num_symbols >> 8;
	auc_interleaver_regs[3] = us_num_symbols;
	if (uc_swap_ij) {
		auc_interleaver_regs[4] = uc_nj;
		auc_interleaver_regs[5] = uc_ni;
	} else {
		auc_interleaver_regs[4] = uc_ni;
		auc_interleaver_regs[5] = uc_nj;
	}

	auc_interleaver_regs[6] = uc_mi;
	auc_interleaver_regs[7] = uc_mj;
	/* Write to registers */
	pplc_if_write_buf(REG_ATPL250_INTERLEAVER_CFG0_32, auc_interleaver_regs, 8);
}

/**
 * \brief Ends a reception for FFT and TxRx
 *
 */
void end_rx_fft_txrx(void)
{
	/*unforce agcs*/
	atpl250_unforce_agc_ext(uc_agc_ext);
	atpl250_unforce_agc_int(us_agc_int);
	/* Look for chirp */
	pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x01);
	/* Reset Rotator, TXRX and FFT */
	atpl250_txrxb_rotator_and_fft_push_rst();
	/* reconfigure rotator to NO bypass and rotation 0 */
	pplc_if_write32(REG_ATPL250_ROTATOR_CONFIG0_32, 0x00000000);
	pplc_if_write32(REG_ATPL250_ROTATOR_CONFIG3_32, 0x0502007F);
	pplc_if_write8(REG_ATPL250_ROTATOR_CONFIG3_H8, 0x01);
}

/**
 * \brief Ends a reception for IOB
 *
 */
void end_rx_iob(void)
{
	/* Reset IOB */
	atpl250_iob_push_rst();
	atpl250_clear_iobuf_to_fft();
	atpl250_iob_release_rst();

	/* Avoid overflow to 8th symbol when reading */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, 0x033A);

	/* Reconfigure source_h as it affects to preamble detection */
	pplc_if_and8(REG_ATPL250_INOUTB_CONF2_H8, 0xFE);  /* SOURCE_H='0' */

	/* RX_FULL = 0 */
	atpl250_clear_rx_full();
}

/**
 * \brief Configures HW for the next possible reception
 *
 */
void prepare_next_rx(void)
{
	/* Look for chirp */
	pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x01);
	/* Release Reset for Rotator, TXRXB and FFT */
	atpl250_txrxb_rotator_and_fft_release_rst();
	/* Disable HW TxRx chain */
	pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~0x01u));
	/* Reset Rx Chain */
	pplc_if_or8(REG_ATPL250_INTERLEAVER_CTL_H8, 0x0F);
	/* Disable Reed Solomon */
	pplc_if_and8(REG_ATPL250_RS_CFG_VL8, (uint8_t)(~0x60u));
	/* Configure interleaver for FCH */
	interleaver_config(uc_used_carriers, uc_num_symbols_fch, MOD_TYPE_BPSK_ROBO);
	pplc_if_write16(REG_ATPL250_INTERLEAVER_CFG2_L16, FCH_INTERLEAVER_USEFUL_SIZE);
	/* Change value to modulator abcd points (full gain) */
	pplc_if_write_buf(REG_ATPL250_INOUTB_PSK_VALUES1_32, (uint8_t *)puc_abcd_fullgain, ABCD_POINTS_LEN);
	pplc_if_write8(REG_ATPL250_COH_PILOTCNUM_L8, 0x10); /* clear consecutive pilots bit */
}

/**
 * \brief Ends a reception
 *
 */
void end_rx(void)
{
	end_rx_fft_txrx();
	end_rx_iob();
	prepare_next_rx();
	#if (LOG_SPI == 1)
	dumpLogSpi();
	#endif
}

/**
 * \brief Ends a transmission
 *
 */
void end_tx(void)
{
	/* TX_MODE = 0, RX_MODE = 1 */
	atpl250_clear_tx_mode();
	atpl250_set_rx_mode();
	atpl250_clear_iobuf_to_fft();
	/* PATH="00", IFFT=0 */
	pplc_if_or8(REG_ATPL250_FFT_CONFIG_VL8, 0x08); /* FFT_DSIABLE=1 */
	pplc_if_and8(REG_ATPL250_FFT_CONFIG_VL8, 0x74); /* IFFT=0; FFT_DISABLE=0; PATH=0 */
	/* Set FFT_SHIFT for reception */
	pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, RX_FFT_SHIFT);

	if (pplc_if_read8(REG_ATPL250_INT_FLAGS_VH8) & INT_TX_ERROR_MASK_8) {
		atpl250_clear_tx_error_int();
		LOG_PHY(("too slow tx\n\r"));
	}

	/* Configure SYNCM detection */
	pplc_if_or8(REG_ATPL250_SYNCM_CTL_L8, 0x80); /* Enable SYNCM */

	/* Push Reset for Rotator, TXRXB and FFT */
	atpl250_txrxb_rotator_and_fft_push_rst();
	/* Prepare for next rx */
	prepare_next_rx();
}

void clear_tx_chain(void)
{
	uint8_t uc_sym_ready = 0;
	uint16_t us_i = 0;
	atpl250_txrxb_rotator_and_fft_push_rst();
	atpl250_rotator_and_fft_release_rst();
	pplc_if_and8(REG_ATPL250_TXRXB_STATE_VL8, (uint8_t)(~0x02u));

	uc_sym_ready = atpl250_read_sym_ready();

	while (uc_sym_ready) {
		uc_sym_ready = atpl250_read_sym_ready();
		if (++us_i > 1000) {
			break;
		}
	}

	if (++us_i > 1000) {
		LOG_PHY(("clear_tx_chain error"));
	}

	/* Clear interrupt flag */
	atpl250_clear_iob_int();

	atpl250_iob_rotator_and_fft_push_rst();
	atpl250_iob_rotator_and_fft_release_rst();

	atpl250_txrxb_release_rst();
}
