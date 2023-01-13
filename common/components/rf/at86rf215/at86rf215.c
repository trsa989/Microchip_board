/**
 *
 * \file
 *
 * \brief Management of the AT86RF215 RF transceiver.
 * This file manages the accesses to the AT86RF215 component.
 *
 * Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/* AT86RF215 includes */
#include "at86rf.h"
#include "at86rf_version.h"
#include "rf215_phy_defs.h"
#include "rf215_conf_check.h"
#include "rf215_irq.h"
#include "rf215_pll.h"
#include "rf215_fe.h"
#include "rf215_bbc.h"
#include "rf215_trx_ctl.h"
#include "rf215_tx_rx.h"
#include "ieee_15_4_sun_fsk.h"
#include "ieee_15_4_sun_ofdm.h"
#ifdef AT86RF_ADDONS_ENABLE
# include "rf215_addon.h"
#endif

/* System includes */
#include "string.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/* RF215 base addresses (one per TRX) */
const rf215_base_addr_t gx_rf215_base_addr[AT86RF_NUM_TRX] = RF215_BASE_ADDRESSES;

/* HAL wrapper (SPI, IRQ and timer access functions) */
at86rf_hal_wrapper_t gx_rf215_hal_wrp;

/* Uppler layer callbacks */
at86rf_callbacks_t gx_rf215_callbacks;

/* PHY configuration struct (one per TRX) */
at86rf_phy_cfg_t gpx_phy_cfg[AT86RF_NUM_TRX];

/* TRX and PHY control struct (one per TRX) */
volatile rf215_phy_ctl_t gpx_phy_ctl[AT86RF_NUM_TRX];

/* PHY statistics struct (one per TRX) */
rf215_phy_stats_t gpx_phy_stats[AT86RF_NUM_TRX];

/* Time of 1 byte SPI transaction in us [uQ3.5] */
uint8_t guc_spi_byte_time_us_q5;

/* Exception bitmask */
volatile uint8_t guc_rf215_exception_mask;

/* Component state */
rf215_component_state_t guc_rf215_comp_state = RF215_COMPONENT_OFF;

/* Counter of consecutive RF interrupts without any reported flag */
uint8_t guc_irqs_empty_count;

/**
 * \brief Convert band and operating mode enumerate
 * to PHY configuration struct
 *
 * \param us_band_opm IEEE 802.15.4 Band and operating mode (see
 * at86rf_phy_band_opm_t)
 * \px_phy_cfg Pointer to store PHY configuration struct
 *
 * \retval true Correct configuration
 * \retval false Wrong configuration or not supported
 */
static bool _rf215_band_opm_to_phy_config(at86rf_phy_band_opm_t us_band_opm, at86rf_phy_cfg_t *px_phy_cfg)
{
	switch (us_band_opm) {
	case AT86RF_SUN_FSK_BAND_863_OPM1:
		*px_phy_cfg = SUN_FSK_BAND_863_870_OPM1;
		break;

	case AT86RF_SUN_FSK_BAND_863_OPM2:
		*px_phy_cfg = SUN_FSK_BAND_863_870_OPM2;
		break;

	case AT86RF_SUN_FSK_BAND_863_OPM3:
		*px_phy_cfg = SUN_FSK_BAND_863_870_OPM3;
		break;

	case AT86RF_SUN_OFDM_BAND_863_OPT4:
		*px_phy_cfg = SUN_OFDM_BAND_863_870_OPT4;
		break;

	case AT86RF_SUN_FSK_BAND_866_OPM1:
		*px_phy_cfg = SUN_FSK_BAND_865_867_OPM1;
		break;

	case AT86RF_SUN_FSK_BAND_866_OPM2:
		*px_phy_cfg = SUN_FSK_BAND_865_867_OPM2;
		break;

	case AT86RF_SUN_FSK_BAND_866_OPM3:
		*px_phy_cfg = SUN_FSK_BAND_865_867_OPM3;
		break;

	case AT86RF_SUN_OFDM_BAND_866_OPT4:
		*px_phy_cfg = SUN_OFDM_BAND_865_867_OPT4;
		break;

	case AT86RF_SUN_FSK_BAND_870_OPM1:
		*px_phy_cfg = SUN_FSK_BAND_870_876_OPM1;
		break;

	case AT86RF_SUN_FSK_BAND_870_OPM2:
		*px_phy_cfg = SUN_FSK_BAND_870_876_OPM2;
		break;

	case AT86RF_SUN_FSK_BAND_870_OPM3:
		*px_phy_cfg = SUN_FSK_BAND_870_876_OPM3;
		break;

	case AT86RF_SUN_OFDM_BAND_870_OPT4:
		*px_phy_cfg = SUN_OFDM_BAND_870_876_OPT4;
		break;

	case AT86RF_SUN_FSK_BAND_915_OPM1:
		*px_phy_cfg = SUN_FSK_BAND_902_928_OPM1;
		break;

	case AT86RF_SUN_FSK_BAND_915_OPM2:
		*px_phy_cfg = SUN_FSK_BAND_902_928_OPM2;
		break;

	case AT86RF_SUN_FSK_BAND_915_OPM3:
		*px_phy_cfg = SUN_FSK_BAND_902_928_OPM3;
		break;

	case AT86RF_SUN_OFDM_BAND_915_OPT4:
		*px_phy_cfg = SUN_OFDM_BAND_902_928_OPT4;
		break;

	case AT86RF_SUN_OFDM_BAND_915_OPT3:
		*px_phy_cfg = SUN_OFDM_BAND_902_928_OPT3;
		break;

	case AT86RF_SUN_OFDM_BAND_915_OPT2:
		*px_phy_cfg = SUN_OFDM_BAND_902_928_OPT2;
		break;

	case AT86RF_SUN_OFDM_BAND_915_OPT1:
		*px_phy_cfg = SUN_OFDM_BAND_902_928_OPT1;
		break;

	case AT86RF_SUN_FSK_BAND_915A_OPM1:
		*px_phy_cfg = SUN_FSK_BAND_902_928_ALT_OPM1;
		break;

	case AT86RF_SUN_FSK_BAND_915A_OPM2:
		*px_phy_cfg = SUN_FSK_BAND_902_928_ALT_OPM2;
		break;

	case AT86RF_SUN_FSK_BAND_915A_OPM3:
		*px_phy_cfg = SUN_FSK_BAND_902_928_ALT_OPM3;
		break;

	case AT86RF_SUN_FSK_BAND_915A_OPM4:
		*px_phy_cfg = SUN_FSK_BAND_902_928_ALT_OPM4;
		break;

	case AT86RF_SUN_FSK_BAND_915A_OPM5:
		*px_phy_cfg = SUN_FSK_BAND_902_928_ALT_OPM5;
		break;

	case AT86RF_SUN_OFDM_BAND_915A_OPT4:
		*px_phy_cfg = SUN_OFDM_BAND_902_928_ALT_OPT4;
		break;

	case AT86RF_SUN_OFDM_BAND_915A_OPT3:
		*px_phy_cfg = SUN_OFDM_BAND_902_928_ALT_OPT3;
		break;

	case AT86RF_SUN_OFDM_BAND_915A_OPT2:
		*px_phy_cfg = SUN_OFDM_BAND_902_928_ALT_OPT2;
		break;

	case AT86RF_SUN_OFDM_BAND_915A_OPT1:
		*px_phy_cfg = SUN_OFDM_BAND_902_928_ALT_OPT1;
		break;

	case AT86RF_SUN_FSK_BAND_915B_OPM1:
		*px_phy_cfg = SUN_FSK_BAND_902_907_915_928_OPM1;
		break;

	case AT86RF_SUN_FSK_BAND_915B_OPM2:
		*px_phy_cfg = SUN_FSK_BAND_902_907_915_928_OPM2;
		break;

	case AT86RF_SUN_FSK_BAND_915B_OPM3:
		*px_phy_cfg = SUN_FSK_BAND_902_907_915_928_OPM3;
		break;

	case AT86RF_SUN_FSK_BAND_915B_OPM4:
		*px_phy_cfg = SUN_FSK_BAND_902_907_915_928_OPM4;
		break;

	case AT86RF_SUN_FSK_BAND_915B_OPM5:
		*px_phy_cfg = SUN_FSK_BAND_902_907_915_928_OPM5;
		break;

	case AT86RF_SUN_OFDM_BAND_915B_OPT4:
		*px_phy_cfg = SUN_OFDM_BAND_902_907_915_928_OPT4;
		break;

	case AT86RF_SUN_OFDM_BAND_915B_OPT3:
		*px_phy_cfg = SUN_OFDM_BAND_902_907_915_928_OPT3;
		break;

	case AT86RF_SUN_OFDM_BAND_915B_OPT2:
		*px_phy_cfg = SUN_OFDM_BAND_902_907_915_928_OPT2;
		break;

	case AT86RF_SUN_OFDM_BAND_915B_OPT1:
		*px_phy_cfg = SUN_OFDM_BAND_902_907_915_928_OPT1;
		break;

	case AT86RF_SUN_FSK_BAND_915C_OPM1:
		*px_phy_cfg = SUN_FSK_BAND_915_928_OPM1;
		break;

	case AT86RF_SUN_FSK_BAND_915C_OPM2:
		*px_phy_cfg = SUN_FSK_BAND_915_928_OPM2;
		break;

	case AT86RF_SUN_FSK_BAND_915C_OPM3:
		*px_phy_cfg = SUN_FSK_BAND_915_928_OPM3;
		break;

	case AT86RF_SUN_FSK_BAND_915C_OPM4:
		*px_phy_cfg = SUN_FSK_BAND_915_928_OPM4;
		break;

	case AT86RF_SUN_FSK_BAND_915C_OPM5:
		*px_phy_cfg = SUN_FSK_BAND_915_928_OPM5;
		break;

	case AT86RF_SUN_OFDM_BAND_915C_OPT4:
		*px_phy_cfg = SUN_OFDM_BAND_915_928_OPT4;
		break;

	case AT86RF_SUN_OFDM_BAND_915C_OPT3:
		*px_phy_cfg = SUN_OFDM_BAND_915_928_OPT3;
		break;

	case AT86RF_SUN_OFDM_BAND_915C_OPT2:
		*px_phy_cfg = SUN_OFDM_BAND_915_928_OPT2;
		break;

	case AT86RF_SUN_OFDM_BAND_915C_OPT1:
		*px_phy_cfg = SUN_OFDM_BAND_915_928_OPT1;
		break;

	default:
		return false;
	}

	return true;
}

/**
 * \brief Initialize RF215 PHY internal variables
 *
 * \param uc_trx_id TRX identifier
 * \param px_ini_params Pointer to PHY initial configuration
 *
 * \retval true Correct configuration
 * \retval false Wrong configuration or not supported
 */
__always_inline static bool _rf215_phy_init(uint8_t uc_trx_id, at86rf_phy_ini_params_t *px_ini_params)
{
	at86rf_phy_cfg_t x_phy_cfg;
	at86rf_phy_cfg_t *px_phy_cfg;
	at86rf_phy_band_opm_t us_band_opm;
	bool b_cfg_ok;

	/* Get PHY configuration depending on initialization parameters */
	us_band_opm = px_ini_params->us_band_opm;
	if (us_band_opm == AT86RF_BAND_OPM_CUSTOM) {
		px_phy_cfg = px_ini_params->px_phy_cfg;
	} else {
		px_phy_cfg = &x_phy_cfg;
		b_cfg_ok = _rf215_band_opm_to_phy_config(us_band_opm, px_phy_cfg);
		if (!b_cfg_ok) {
			px_phy_cfg = NULL;
		}
	}

	/* Check pointer */
	if (px_phy_cfg == NULL) {
		return false;
	}

	/* Initialize PHY and TRX control */
	memset(&gpx_phy_stats[uc_trx_id], 0, sizeof(rf215_phy_stats_t));
	gpx_phy_ctl[uc_trx_id].uc_phy_state = RF_PHY_STATE_RESET;
	gpx_phy_ctl[uc_trx_id].uc_trx_state = RF215_RFn_STATE_RF_TRXOFF;
	gpx_phy_ctl[uc_trx_id].us_band_opm = us_band_opm;

	/* Initialize TX&RX control */
	rf215_tx_init(uc_trx_id);
	rf215_rx_init(uc_trx_id);

	/* Initialize Baseband Core and Frequency Synthesizer modules */
	b_cfg_ok = rf215_bbc_init(uc_trx_id, px_phy_cfg, px_ini_params->us_chn_num_ini);

	return b_cfg_ok;
}

/**
 * \brief Wait to IRQ indicating TRX reset (PHY state RX_LISTEN). IRQ must be
 * disabled before entering this function and it will be enabled when it returns
 *
 * \param uc_trx_id TRX identifier
 *
 * \return Result
 */
__always_inline static at86rf_res_t _rf215_wait_trx_reset(uint8_t uc_trx_id)
{
	uint32_t ul_timeout;
	uint32_t ul_basepri_prev;
	at86rf_res_t uc_result;

	/* Set PHY state to RESET */
	gpx_phy_ctl[uc_trx_id].uc_phy_state = RF_PHY_STATE_RESET;

	/* Enable IRQ */
	gx_rf215_hal_wrp.rf_enable_int(true);
	gx_rf215_hal_wrp.timer_enable_int(true);

	/* Make sure that interrupt is enabled in BASEPRI */
	ul_basepri_prev = __get_BASEPRI();
	if (ul_basepri_prev != 0) {
		__set_BASEPRI(0);
	}

	/* Wait to IRQ indicating TRX reset (TRX initialization done in IRQ) */
	ul_timeout = 5000000;
	uc_result = AT86RF_SUCCESS;
	while (gpx_phy_ctl[uc_trx_id].uc_phy_state != RF_PHY_STATE_RX_LISTEN) {
		if (ul_timeout == 0) {
			/* Protection to avoid infinite loop */
			uc_result = AT86RF_ERROR;
			break;
		} else {
			ul_timeout--;
		}
	}

	/* Restore BASEPRI */
	__set_BASEPRI(ul_basepri_prev);

	return uc_result;
}

/**
 * \brief Wait to IRQ indicating chip reset (exception reset). IRQ must be
 * disabled before entering this function and it will remain disabled when it
 * returns
 *
 * \return Result
 */
__always_inline static at86rf_res_t _rf215_wait_chip_reset(void)
{
	uint32_t ul_basepri_prev;
	uint32_t ul_timeout;
	at86rf_res_t uc_result;

	/* Initialize exception bitmask */
	guc_rf215_exception_mask = 0;

	/* Enable IRQ in NVIC */
	gx_rf215_hal_wrp.rf_enable_int(true);

	/* Make sure that interrupt is enabled in BASEPRI */
	ul_basepri_prev = __get_BASEPRI();
	if (ul_basepri_prev != 0) {
		__set_BASEPRI(0);
	}

	/* Wait to IRQ indicating chip reset (device initialization is done
	 * inside interrupt) */
	ul_timeout = 5000000;
	uc_result = AT86RF_SUCCESS;
	while (guc_rf215_exception_mask == 0) {
		if (ul_timeout == 0) {
			/* Protection to avoid infinite loop. Something
			 * is wrong, return error. */
			uc_result = AT86RF_ERROR;
			break;
		} else {
			ul_timeout--;
		}
	}

	/* Restore BASEPRI */
	__set_BASEPRI(ul_basepri_prev);

	/* Critical region to avoid unexpected interrupt */
	gx_rf215_hal_wrp.rf_enable_int(false);

	if (guc_rf215_exception_mask != AT86RF_EXCEPTION_RESET) {
		/* Something went wrong during initialization (inside
		 * interrupt), return error. */
		uc_result = AT86RF_ERROR;
	}

	/* Clear exception bitmask */
	guc_rf215_exception_mask = 0;

	return uc_result;
}

/**
 * \brief Sleep TRX. Send SLEEP command and update internal variables
 *
 * \param uc_trx_id TRX identifier
 */
__always_inline static void _rf215_trx_sleep(uint8_t uc_trx_id)
{
	/* Wait for SPI free to avoid disabling interrupts more than needed */
	rf215_spi_wait_free();

	/* Critical region to avoid state changes from IRQ */
	gx_rf215_hal_wrp.rf_enable_int(false);
	gx_rf215_hal_wrp.timer_enable_int(false);

	/* Abort TX auto procedure with control, avoiding RFn_CMD confict */
	rf215_tx_auto_stop(uc_trx_id, AT86RF_TX_ABORTED);

	/* Switch TRX to TRXOFF state before sending SLEEP command and abort
	 * TX/RX in progress, if needed */
	rf215_trx_switch_trxoff(uc_trx_id);

	/* Send SLEEP command */
	rf215_trx_cmd_sleep(uc_trx_id);

	/* Leave critical region */
	gx_rf215_hal_wrp.timer_enable_int(true);
	gx_rf215_hal_wrp.rf_enable_int(true);
}

/**
 * \brief Wake-up TRX. Send TRXOFF command and update internal variables
 *
 * \param uc_trx_id TRX identifier
 *
 * \return Wake-up result
 */
__always_inline static at86rf_res_t _rf215_trx_wakeup(uint8_t uc_trx_id)
{
	at86rf_res_t uc_result;

	/* Wait for SPI free to avoid disabling interrupts more than needed */
	rf215_spi_wait_free();

	/* Critical region to avoid state changes from IRQ */
	gx_rf215_hal_wrp.rf_enable_int(false);
	gx_rf215_hal_wrp.timer_enable_int(false);

	/* Send TRXOFF command to wake-up TRX */
	rf215_trx_cmd_trxoff(uc_trx_id);

	/* Wait to IRQ indicating TRX reset. IRQ will be enabled after calling
	 * this function */
	uc_result = _rf215_wait_trx_reset(uc_trx_id);
	return uc_result;
}

/**
 * \brief Reset entire chip.
 *
 * \return Result
 */
__always_inline static at86rf_res_t _rf215_chip_reset(void)
{
	at86rf_res_t uc_result;

	/* Wait for SPI free to avoid reset in the middle of SPI transaction */
	rf215_spi_wait_free();

	/* Critical region to avoid state changes from IRQ */
	gx_rf215_hal_wrp.rf_enable_int(false);
	gx_rf215_hal_wrp.timer_enable_int(false);

	/* Reset RF215 */
	gx_rf215_hal_wrp.rf_reset();

	for (uint8_t uc_trx_id = 0; uc_trx_id < AT86RF_NUM_TRX; uc_trx_id++) {
		/* Check current PHY state and abort TX/RX if needed */
		rf215_tx_rx_check_aborts(uc_trx_id, true);

		/* Set PHY state to RESET and TRX state to TRXOFF */
		gpx_phy_ctl[uc_trx_id].uc_phy_state = RF_PHY_STATE_RESET;
		gpx_phy_ctl[uc_trx_id].uc_trx_state = RF215_RFn_STATE_RF_TRXOFF;
	}

	/* It is safe to allow timer IRQ here */
	gx_rf215_hal_wrp.timer_enable_int(true);

	/* Wait to IRQ indicating chip reset (device initialization is done
	 * inside interrupt) */
	uc_result = _rf215_wait_chip_reset();

	/* Enable RF IRQ */
	gx_rf215_hal_wrp.rf_enable_int(true);

	return uc_result;
}

/**
 * \brief Sleep/wake-up TRX.
 *
 * \param uc_trx_id TRX identifier
 * \param b_sleep Sleep (true) or wake-up (false)
 *
 * \return Result
 */
__always_inline static at86rf_res_t _rf215_sleep_req(uint8_t uc_trx_id, bool b_sleep)
{
	at86rf_res_t uc_result;
	bool b_sleep_state;

	/* Check TRX current sleep state */
	b_sleep_state = (bool)(gpx_phy_ctl[uc_trx_id].uc_phy_state == RF_PHY_STATE_SLEPT);
	if (b_sleep_state == b_sleep) {
		return AT86RF_SUCCESS;
	}

	if (b_sleep) {
		/* Enter TRX in SLEEP state with SLEEP command */
		_rf215_trx_sleep(uc_trx_id);
		uc_result = AT86RF_SUCCESS;
	} else {
		/* Leave TRX from SLEEP state with TRXOFF command */
		uc_result = _rf215_trx_wakeup(uc_trx_id);
	}

	return uc_result;
}

/**
 * \brief Reset single TRX.
 *
 * \param uc_trx_id TRX identifier
 *
 * \return Result
 */
__always_inline static at86rf_res_t _rf215_trx_reset(uint8_t uc_trx_id)
{
	at86rf_res_t uc_result;

	/* Wait for SPI free to avoid disabling interrupts more than needed */
	rf215_spi_wait_free();

	/* Critical region to avoid state changes from IRQ */
	gx_rf215_hal_wrp.rf_enable_int(false);
	gx_rf215_hal_wrp.timer_enable_int(false);

	/* Abort TX auto procedure with control, avoiding RFn_CMD confict */
	rf215_tx_auto_stop(uc_trx_id, AT86RF_TX_ABORTED);

	/* Send RESET command to single TRX */
	rf215_trx_cmd_reset(uc_trx_id);

	/* Check current PHY state and abort TX/RX if needed */
	rf215_tx_rx_check_aborts(uc_trx_id, false);

	/* Wait to IRQ indicating TRX reset. IRQ will be enabled */
	uc_result = _rf215_wait_trx_reset(uc_trx_id);
	return uc_result;
}

/**
 * \brief Function to initialize AT86RF215 component.
 *
 * \param[in] px_hal_wrp Pointer to HAL wrapper (hardware abstraction layer
 * functions)
 * \param[in] px_callbacks Pointer to callbacks for upper layer
 */
void at86rf_init(at86rf_hal_wrapper_t *px_hal_wrp, at86rf_callbacks_t *px_callbacks)
{
	/* Initialize component state */
	guc_rf215_comp_state = RF215_COMPONENT_INIT;

	/* Fill upper layer callbacks */
	gx_rf215_callbacks = *px_callbacks;

	/* Fill HAL wrapper functions to access hardware peripherals */
	gx_rf215_hal_wrp = *px_hal_wrp;

	/* Initialize TRX interface (SPI, and RST and IRQ pins). RST pin is kept
	 * pushed. IRQ pin interrupt is disabled (in PIO, not in NVIC) */
	guc_spi_byte_time_us_q5 = gx_rf215_hal_wrp.rf_init();
}

/**
 * \brief Enable the AT86RF215 component. PHY initialization.
 *
 * \param px_ini_params_09 Pointer to Sub-1GHz (RF09) PHY initial configuration
 * \param px_ini_params_24 Pointer to 2.4GHz (RF24) PHY initial configuration
 *
 * \return Initialization result
 */
at86rf_res_t at86rf_enable(at86rf_phy_ini_params_t *px_ini_params_09, at86rf_phy_ini_params_t *px_ini_params_24)
{
	at86rf_res_t uc_result;
	bool b_cfg_ok;

	if (guc_rf215_comp_state < RF215_COMPONENT_INIT) {
		/* Component not initialized yet */
		return AT86RF_NOT_INIT;
	}

	/* Disable IRQ in NVIC (critical region) */
	gx_rf215_hal_wrp.rf_enable_int(false);
	gx_rf215_hal_wrp.timer_enable_int(false);

	/* Reset RF215 */
	gx_rf215_hal_wrp.rf_reset();
	uc_result = AT86RF_SUCCESS;

#ifndef AT86RF215_DISABLE_RF09_TRX
	/* Initialize PHY configuration (RF09) */
	b_cfg_ok = _rf215_phy_init(AT86RF_TRX_RF09_ID, px_ini_params_09);
	if (!b_cfg_ok) {
		/* Invalid PHY configuration */
		uc_result = AT86RF_INVALID_CFG;
	}

#else
	UNUSED(px_ini_params_09);
#endif

#ifndef AT86RF215_DISABLE_RF24_TRX
	/* Initialize PHY configuration (RF24) */
	b_cfg_ok = _rf215_phy_init(AT86RF_TRX_RF24_ID, px_ini_params_24);
	if (!b_cfg_ok) {
		/* Invalid PHY configuration */
		uc_result = AT86RF_INVALID_CFG;
	}

#else
	UNUSED(px_ini_params_24);
#endif

	/* Initialize component state */
	guc_rf215_comp_state = RF215_COMPONENT_ENABLING;
	guc_irqs_empty_count = 0;

	/* It is safe to allow timer IRQ here */
	gx_rf215_hal_wrp.timer_enable_int(true);

	if (uc_result == AT86RF_SUCCESS) {
		/* Set handler for TRX IRQ. Enable IRQ pin interrupt in PIO */
		gx_rf215_hal_wrp.rf_set_handler(rf215_irq_handler);

		/* Wait to IRQ indicating chip reset (device initialization is
		 * done inside interrupt) */
		uc_result = _rf215_wait_chip_reset();
	}

	if (uc_result == AT86RF_SUCCESS) {
		/* Successful RF215 initialization */
		guc_rf215_comp_state = RF215_COMPONENT_ENABLED;
	} else {
		/* Initialize again TRX interface (SPI, and RST/IRQ pins). RST
		 * pin is kept pushed. IRQ pin interrupt is disabled (in PIO,
		 * not in NVIC) */
		gx_rf215_hal_wrp.rf_init();
		guc_rf215_comp_state = RF215_COMPONENT_INIT;

		/* Turn on TX and RX LEDs to indicate error */
		gx_rf215_hal_wrp.rf_led(RF215_LED_TX, true);
		gx_rf215_hal_wrp.rf_led(RF215_LED_RX, true);
	}

	/* Enable IRQ in NVIC */
	gx_rf215_hal_wrp.rf_enable_int(true);

#ifdef AT86RF_ADDONS_ENABLE
	/* Initialize addon */
	rf215_addon_init();
#endif

	return uc_result;
}

/**
 * \brief Disable the AT86RF215 component.
 *
 * \return Result
 */
at86rf_res_t at86rf_disable(void)
{
	if (guc_rf215_comp_state != RF215_COMPONENT_ENABLED) {
		/* Component not enabled yet */
		return AT86RF_NOT_ENABLED;
	}

	/* Enter in DEEP_SLEEP state by sending SLEEP command to both TRX (if
	 * one of them is disabled, it will be already in SLEEP state) */
	for (uint8_t uc_trx_id = 0; uc_trx_id < AT86RF_NUM_TRX; uc_trx_id++) {
		_rf215_trx_sleep(uc_trx_id);
	}

	/* Update component state */
	guc_rf215_comp_state = RF215_COMPONENT_DISABLED;

	return AT86RF_SUCCESS;
}

/**
 * \brief Function to Check AT86RF215 pending events
 */
void at86rf_event_handler(void)
{
	if ((guc_rf215_comp_state != RF215_COMPONENT_ENABLED) && (guc_rf215_comp_state != RF215_COMPONENT_ERROR)) {
		/* Component not enabled yet */
		return;
	}

	/* Check exception bitmask */
	if (guc_rf215_exception_mask != 0) {
		uint8_t uc_exception_mask;

		/* Critical region to avoid modification of
		 * guc_rf215_exception_mask from interrupt */
		gx_rf215_hal_wrp.rf_enable_int(false);
		gx_rf215_hal_wrp.timer_enable_int(false);
		uc_exception_mask = guc_rf215_exception_mask;
		guc_rf215_exception_mask = 0;
		gx_rf215_hal_wrp.timer_enable_int(true);
		gx_rf215_hal_wrp.rf_enable_int(true);

		if (gx_rf215_callbacks.rf_exception_cb) {
			/* Indicate exception to upper layer */
			gx_rf215_callbacks.rf_exception_cb(uc_exception_mask);
		}
	}

	if (guc_rf215_comp_state != RF215_COMPONENT_ENABLED) {
		/* Component not enabled yet */
		return;
	}

	for (uint8_t uc_trx_id = 0; uc_trx_id < AT86RF_NUM_TRX; uc_trx_id++) {
		/* Check pending TX confirm and RX indication events */
		rf215_tx_event_handler(uc_trx_id);
		rf215_rx_event_handler(uc_trx_id);
	}
}

/**
 * \brief Get length of AT86RF215 PHY PIB
 *
 * \param us_attr PIB attribute (at86rf_pib_attr_t)
 *
 * \return PIB length in bytes
 */
uint8_t at86rf_pib_get_len(at86rf_pib_attr_t us_attr)
{
	uint16_t us_len;

	switch (us_attr) {
	case AT86RF_PIB_PHY_STATS_RESET:
	case AT86RF_PIB_DEVICE_RESET:
	case AT86RF_PIB_TRX_RESET:
	case AT86RF_PIB_TRX_SLEEP:
		us_len = sizeof(uint8_t);
		break;

	case AT86RF_PIB_DEVICE_ID:
	case AT86RF_PIB_PHY_CHANNEL_NUM:
	case AT86RF_PIB_PHY_TX_PAY_SYMBOLS:
	case AT86RF_PIB_PHY_RX_PAY_SYMBOLS:
	case AT86RF_PIB_PHY_CCA_ED_DURATION:
	case AT86RF_PIB_PHY_TURNAROUND_TIME:
	case AT86RF_PIB_MAC_UNIT_BACKOFF_PERIOD:
		us_len = sizeof(uint16_t);
		break;

	case AT86RF_PIB_PHY_CHANNEL_FREQ_HZ:
	case AT86RF_PIB_PHY_TX_TOTAL:
	case AT86RF_PIB_PHY_TX_TOTAL_BYTES:
	case AT86RF_PIB_PHY_TX_ERR_TOTAL:
	case AT86RF_PIB_PHY_TX_ERR_BUSY_TX:
	case AT86RF_PIB_PHY_TX_ERR_BUSY_RX:
	case AT86RF_PIB_PHY_TX_ERR_BUSY_CHN:
	case AT86RF_PIB_PHY_TX_ERR_BAD_LEN:
	case AT86RF_PIB_PHY_TX_ERR_BAD_FORMAT:
	case AT86RF_PIB_PHY_TX_ERR_TIMEOUT:
	case AT86RF_PIB_PHY_TX_ERR_ABORTED:
	case AT86RF_PIB_PHY_TX_CFM_NOT_HANDLED:
	case AT86RF_PIB_PHY_RX_TOTAL:
	case AT86RF_PIB_PHY_RX_TOTAL_BYTES:
	case AT86RF_PIB_PHY_RX_ERR_TOTAL:
	case AT86RF_PIB_PHY_RX_ERR_FALSE_POSITIVE:
	case AT86RF_PIB_PHY_RX_ERR_BAD_LEN:
	case AT86RF_PIB_PHY_RX_ERR_BAD_FORMAT:
	case AT86RF_PIB_PHY_RX_ERR_BAD_FCS_PAY:
	case AT86RF_PIB_PHY_RX_ERR_ABORTED:
	case AT86RF_PIB_PHY_RX_OVERRIDE:
	case AT86RF_PIB_PHY_RX_IND_NOT_HANDLED:
		us_len = sizeof(uint32_t);
		break;

	case AT86RF_PIB_PHY_CCA_ED_THRESHOLD:
		us_len = sizeof(int8_t);
		break;

	case AT86RF_PIB_FW_VERSION:
		us_len = sizeof(at86rf_fw_version_t);
		break;

	case AT86RF_PIB_PHY_CONFIG:
		us_len = sizeof(at86rf_phy_cfg_t);
		break;

	case AT86RF_PIB_PHY_BAND_OPERATING_MODE:
		us_len = sizeof(at86rf_phy_band_opm_t);
		break;

	case AT86RF_PIB_PHY_CCA_ED_CONFIG:
		us_len = sizeof(at86rf_cca_ed_cfg_t);
		break;

	default:
		us_len = 0;
		break;
	}

	return us_len;
}

/**
 * \brief Get value of AT86RF215 PHY PIB
 *
 * \param uc_trx_id TRX identifier
 * \param us_attr PIB attribute (at86rf_pib_attr_t)
 * \param p_val Pointer to store PIB value (size written corresponds to
 * at86rf_pib_get_len)
 *
 * \return PIB get result
 */
at86rf_res_t at86rf_pib_get(uint8_t uc_trx_id, at86rf_pib_attr_t us_attr, void *p_val)
{
	at86rf_res_t uc_pib_result;

	/* Check component enabled */
	if (guc_rf215_comp_state != RF215_COMPONENT_ENABLED) {
		return AT86RF_NOT_ENABLED;
	}

	/* Check pointer */
	if (p_val == NULL) {
		return AT86RF_INVALID_PARAM;
	}

	/* PIBs not associated to TRX ID */
	if (us_attr == AT86RF_PIB_FW_VERSION) {
		const at86rf_fw_version_t x_fw_version = AT86RF_FW_VERSION;
		*((at86rf_fw_version_t *)p_val) = x_fw_version;
		return AT86RF_SUCCESS;
	} else if (us_attr == AT86RF_PIB_DEVICE_ID) {
		*((uint16_t *)p_val) = AT86RF_DEVICE_ID;
		return AT86RF_SUCCESS;
	} else if (us_attr == AT86RF_PIB_DEVICE_RESET) {
		return AT86RF_WRITE_ONLY;
	}

	/* Check TRX ID */
	if (uc_trx_id >= AT86RF_NUM_TRX) {
		return AT86RF_INVALID_TRX_ID;
	}

	/* PIBs associated to TRX ID */
	uc_pib_result = AT86RF_SUCCESS;
	switch (us_attr) {
	case AT86RF_PIB_TRX_SLEEP:
		*((bool *)p_val) = (gpx_phy_ctl[uc_trx_id].uc_phy_state == RF_PHY_STATE_SLEPT);
		break;

	case AT86RF_PIB_PHY_CONFIG:
		*((at86rf_phy_cfg_t *)p_val) = gpx_phy_cfg[uc_trx_id];
		break;

	case AT86RF_PIB_PHY_BAND_OPERATING_MODE:
		*((at86rf_phy_band_opm_t *)p_val) = gpx_phy_ctl[uc_trx_id].us_band_opm;
		break;

	case AT86RF_PIB_PHY_CHANNEL_NUM:
		*((uint16_t *)p_val) = gpx_phy_ctl[uc_trx_id].us_chn_num;
		break;

	case AT86RF_PIB_PHY_CHANNEL_FREQ_HZ:
		*((uint32_t *)p_val) = rf215_pll_get_chn_freq(uc_trx_id);
		break;

	case AT86RF_PIB_PHY_CCA_ED_CONFIG:
		*((at86rf_cca_ed_cfg_t *)p_val) = gpx_phy_cfg[uc_trx_id].x_cca_ed_cfg;
		break;

	case AT86RF_PIB_PHY_CCA_ED_DURATION:
		*((uint16_t *)p_val) = gpx_phy_cfg[uc_trx_id].x_cca_ed_cfg.us_duration_us;
		break;

	case AT86RF_PIB_PHY_CCA_ED_THRESHOLD:
		*((int8_t *)p_val) = gpx_phy_cfg[uc_trx_id].x_cca_ed_cfg.sc_threshold_dBm;
		break;

	case AT86RF_PIB_PHY_TURNAROUND_TIME:
		*((uint16_t *)p_val) = gpx_phy_ctl[uc_trx_id].us_turnaround_time_us;
		break;

	case AT86RF_PIB_PHY_TX_PAY_SYMBOLS:
		*((uint16_t *)p_val) = gpx_phy_ctl[uc_trx_id].us_tx_pay_symbols;
		break;

	case AT86RF_PIB_PHY_RX_PAY_SYMBOLS:
		*((uint16_t *)p_val) = gpx_phy_ctl[uc_trx_id].us_rx_pay_symbols;
		break;

	case AT86RF_PIB_PHY_TX_TOTAL:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_tx_total;
		break;

	case AT86RF_PIB_PHY_TX_TOTAL_BYTES:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_tx_total_bytes;
		break;

	case AT86RF_PIB_PHY_TX_ERR_TOTAL:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_tx_err_total;
		break;

	case AT86RF_PIB_PHY_TX_ERR_BUSY_TX:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_tx_err_busy_tx;
		break;

	case AT86RF_PIB_PHY_TX_ERR_BUSY_RX:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_tx_err_busy_rx;
		break;

	case AT86RF_PIB_PHY_TX_ERR_BUSY_CHN:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_tx_err_busy_chn;
		break;

	case AT86RF_PIB_PHY_TX_ERR_BAD_LEN:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_tx_err_bad_len;
		break;

	case AT86RF_PIB_PHY_TX_ERR_BAD_FORMAT:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_tx_err_bad_format;
		break;

	case AT86RF_PIB_PHY_TX_ERR_TIMEOUT:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_tx_err_timeout;
		break;

	case AT86RF_PIB_PHY_TX_ERR_ABORTED:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_tx_err_aborted;
		break;

	case AT86RF_PIB_PHY_TX_CFM_NOT_HANDLED:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_tx_cfm_not_handled;
		break;

	case AT86RF_PIB_PHY_RX_TOTAL:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_rx_total;
		break;

	case AT86RF_PIB_PHY_RX_TOTAL_BYTES:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_rx_total_bytes;
		break;

	case AT86RF_PIB_PHY_RX_ERR_TOTAL:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_rx_err_total;
		break;

	case AT86RF_PIB_PHY_RX_ERR_FALSE_POSITIVE:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_rx_err_false_positive;
		break;

	case AT86RF_PIB_PHY_RX_ERR_BAD_LEN:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_rx_err_bad_len;
		break;

	case AT86RF_PIB_PHY_RX_ERR_BAD_FORMAT:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_rx_err_bad_format;
		break;

	case AT86RF_PIB_PHY_RX_ERR_BAD_FCS_PAY:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_rx_err_bad_fcs_pay;
		break;

	case AT86RF_PIB_PHY_RX_ERR_ABORTED:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_rx_err_aborted;
		break;

	case AT86RF_PIB_PHY_RX_OVERRIDE:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_rx_override;
		break;

	case AT86RF_PIB_PHY_RX_IND_NOT_HANDLED:
		*((uint32_t *)p_val) = gpx_phy_stats[uc_trx_id].ul_rx_ind_not_handled;
		break;

	case AT86RF_PIB_MAC_UNIT_BACKOFF_PERIOD:
		*((uint16_t *)p_val) = gpx_phy_ctl[uc_trx_id].us_turnaround_time_us + gpx_phy_cfg[uc_trx_id].x_cca_ed_cfg.us_duration_us;
		break;

	case AT86RF_PIB_PHY_STATS_RESET:
	case AT86RF_PIB_TRX_RESET:
		uc_pib_result = AT86RF_WRITE_ONLY;
		break;

	default:
		uc_pib_result = AT86RF_INVALID_ATTR;
		break;
	}

	return uc_pib_result;
}

/**
 * \brief Set value of AT86RF215 PHY PIB
 *
 * \param uc_trx_id TRX identifier
 * \param us_attr PIB attribute (at86rf_pib_attr_t)
 * \param p_val Pointer to PIB value (size read corresponds to
 * at86rf_pib_get_len)
 *
 * \return PIB set result
 */
at86rf_res_t at86rf_pib_set(uint8_t uc_trx_id, at86rf_pib_attr_t us_attr, void *p_val)
{
	at86rf_res_t uc_pib_result;

	/* Check component enabled */
	if (guc_rf215_comp_state != RF215_COMPONENT_ENABLED) {
		return AT86RF_NOT_ENABLED;
	}

	/* Check pointer */
	if (p_val == NULL) {
		return AT86RF_INVALID_PARAM;
	}

	/* PIBs not associated to TRX ID */
	if (us_attr == AT86RF_PIB_FW_VERSION) {
		return AT86RF_READ_ONLY;
	} else if (us_attr == AT86RF_PIB_DEVICE_ID) {
		return AT86RF_READ_ONLY;
	} else if (us_attr == AT86RF_PIB_DEVICE_RESET) {
		return _rf215_chip_reset();
	}

	/* Check TRX ID */
	if (uc_trx_id >= AT86RF_NUM_TRX) {
		return AT86RF_INVALID_TRX_ID;
	}

	uc_pib_result = AT86RF_SUCCESS;
	switch (us_attr) {
	case AT86RF_PIB_PHY_CHANNEL_FREQ_HZ:
	case AT86RF_PIB_PHY_TX_PAY_SYMBOLS:
	case AT86RF_PIB_PHY_RX_PAY_SYMBOLS:
	case AT86RF_PIB_PHY_TURNAROUND_TIME:
	case AT86RF_PIB_PHY_TX_TOTAL:
	case AT86RF_PIB_PHY_TX_TOTAL_BYTES:
	case AT86RF_PIB_PHY_TX_ERR_TOTAL:
	case AT86RF_PIB_PHY_TX_ERR_BUSY_TX:
	case AT86RF_PIB_PHY_TX_ERR_BUSY_RX:
	case AT86RF_PIB_PHY_TX_ERR_BUSY_CHN:
	case AT86RF_PIB_PHY_TX_ERR_BAD_LEN:
	case AT86RF_PIB_PHY_TX_ERR_BAD_FORMAT:
	case AT86RF_PIB_PHY_TX_ERR_TIMEOUT:
	case AT86RF_PIB_PHY_TX_ERR_ABORTED:
	case AT86RF_PIB_PHY_TX_CFM_NOT_HANDLED:
	case AT86RF_PIB_PHY_RX_TOTAL:
	case AT86RF_PIB_PHY_RX_TOTAL_BYTES:
	case AT86RF_PIB_PHY_RX_ERR_TOTAL:
	case AT86RF_PIB_PHY_RX_ERR_FALSE_POSITIVE:
	case AT86RF_PIB_PHY_RX_ERR_BAD_LEN:
	case AT86RF_PIB_PHY_RX_ERR_BAD_FORMAT:
	case AT86RF_PIB_PHY_RX_ERR_BAD_FCS_PAY:
	case AT86RF_PIB_PHY_RX_ERR_ABORTED:
	case AT86RF_PIB_PHY_RX_OVERRIDE:
	case AT86RF_PIB_PHY_RX_IND_NOT_HANDLED:
	case AT86RF_PIB_MAC_UNIT_BACKOFF_PERIOD:
		uc_pib_result = AT86RF_READ_ONLY;
		break;

	case AT86RF_PIB_TRX_RESET:
		uc_pib_result = _rf215_trx_reset(uc_trx_id);
		break;

	case AT86RF_PIB_TRX_SLEEP:
		uc_pib_result = _rf215_sleep_req(uc_trx_id, *((bool *)p_val));
		break;

	case AT86RF_PIB_PHY_CONFIG:
		uc_pib_result = rf215_bbc_set_phy_cfg(uc_trx_id, p_val, 0);
		if (uc_pib_result == AT86RF_SUCCESS) {
			gpx_phy_ctl[uc_trx_id].us_band_opm = AT86RF_BAND_OPM_CUSTOM;
		}

		break;

	case AT86RF_PIB_PHY_BAND_OPERATING_MODE:
	{
		at86rf_phy_band_opm_t us_band_opm_new = *((at86rf_phy_band_opm_t *)p_val);
		at86rf_phy_cfg_t x_phy_cfg;
		bool b_cfg_ok = _rf215_band_opm_to_phy_config(us_band_opm_new, &x_phy_cfg);
		if (b_cfg_ok) {
			uc_pib_result = rf215_bbc_set_phy_cfg(uc_trx_id, &x_phy_cfg, 0);
			if (uc_pib_result == AT86RF_SUCCESS) {
				gpx_phy_ctl[uc_trx_id].us_band_opm = us_band_opm_new;
			}
		} else {
			uc_pib_result = AT86RF_INVALID_PARAM;
		}

		break;
	}

	case AT86RF_PIB_PHY_CHANNEL_NUM:
	{
		/* Only update channel number */
		uint16_t us_chn_num_new = *((uint16_t *)p_val);
		uc_pib_result = rf215_bbc_set_phy_cfg(uc_trx_id, &gpx_phy_cfg[uc_trx_id], us_chn_num_new);
		break;
	}

	case AT86RF_PIB_PHY_CCA_ED_CONFIG:
		gpx_phy_cfg[uc_trx_id].x_cca_ed_cfg = *((at86rf_cca_ed_cfg_t *)p_val);
		rf215_fe_upd_phy_cfg(uc_trx_id);
		break;

	case AT86RF_PIB_PHY_CCA_ED_DURATION:
		gpx_phy_cfg[uc_trx_id].x_cca_ed_cfg.us_duration_us = *((uint16_t *)p_val);
		rf215_fe_upd_phy_cfg(uc_trx_id);
		break;

	case AT86RF_PIB_PHY_CCA_ED_THRESHOLD:
		gpx_phy_cfg[uc_trx_id].x_cca_ed_cfg.sc_threshold_dBm = *((int8_t *)p_val);
		break;

	case AT86RF_PIB_PHY_STATS_RESET:
		memset(&gpx_phy_stats[uc_trx_id], 0, sizeof(rf215_phy_stats_t));
		break;

	default:
		uc_pib_result = AT86RF_INVALID_ATTR;
		break;
	}

	return uc_pib_result;
}

/**
 * \brief Perform Clear Channel Assessment (CCA).
 * TBD: Energy Detection (CCA Mode 1 and 3)
 *
 * \param uc_trx_id TRX identifier
 * \param uc_cca_mode Clear Channel Assessment (CCA) mode
 *
 * \return CCA result (busy/free channel)
 */
at86rf_cca_res_t at86rf_cca_req(uint8_t uc_trx_id, at86rf_cca_t uc_cca_mode)
{
	at86rf_cca_res_t uc_cca_result;

	/* Check component enabled */
	if (guc_rf215_comp_state != RF215_COMPONENT_ENABLED) {
		return AT86RF_CCA_NOT_ENABLED;
	}

	/* Check CCA mode */
	if (uc_cca_mode > AT86RF_CCA_OFF) {
		return AT86RF_CCA_INVALID_PARAM;
	}

	/* Check TRX ID */
	if (uc_trx_id >= AT86RF_NUM_TRX) {
		return AT86RF_CCA_INVALID_TRX_ID;
	}

	uc_cca_result = AT86RF_CCA_FREE;

	switch (gpx_phy_ctl[uc_trx_id].uc_phy_state) {
	case RF_PHY_STATE_RX_HEADER:
		/* RX in progress: Busy RX (if CCA uses carrier sense) */
		if ((uc_cca_mode == AT86RF_CCA_MODE_2) || (uc_cca_mode == AT86RF_CCA_MODE_3)) {
			uc_cca_result = AT86RF_CCA_BUSY_RX;
		}

		break;

	case RF_PHY_STATE_RX_PAYLOAD:
		/* RX payload in progress: Busy RX (except CCA off) */
		if (uc_cca_mode != AT86RF_CCA_OFF) {
			uc_cca_result = AT86RF_CCA_BUSY_RX;
		}

		break;

	case RF_PHY_STATE_TX_TXPREP:
	case RF_PHY_STATE_TX_CCA_ED:
	case RF_PHY_STATE_TX:
		uc_cca_result = AT86RF_CCA_BUSY_TX;
		break;

	case RF_PHY_STATE_SLEPT:
		uc_cca_result = AT86RF_CCA_TRX_SLEPT;
		break;

	case RF_PHY_STATE_RESET:
		uc_cca_result =  AT86RF_CCA_NOT_ENABLED;
		break;

	default:
		break;
	}

	/* TBD: Energy Detection for CCA Modes 1 and 3 */

	return uc_cca_result;
}

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
