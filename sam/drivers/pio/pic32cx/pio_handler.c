/**
 * \file
 *
 * \brief Parallel Input/Output (PIO) interrupt handler for PIC32CX.
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

/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "pio.h"
#include "pio_handler.h"

/**
 * Maximum number of interrupt sources that can be defined. This
 * constant can be increased, but the current value is the smallest possible one
 * that will be compatible with all existing projects.
 */
#define MAX_INTERRUPT_SOURCES       7

/**
 * Describes a PIO interrupt source, including the PIO instance triggering the
 * interrupt and the associated interrupt handler.
 */
struct s_interrupt_source {
	uint32_t id;
	uint32_t mask;
	uint32_t attr;

	/* Interrupt handler. */
	void (*handler)(const uint32_t, const uint32_t);
};

/* List of interrupt sources. */
static struct s_interrupt_source gs_interrupt_sources[MAX_INTERRUPT_SOURCES];

/* Number of currently defined interrupt sources. */
static uint32_t gs_ul_nb_sources = 0;

/**
 * \brief Process an interrupt request on the given PIO controller.
 *
 * \param p_pio_group PIO controller base address.
 * \param ul_id PIO controller ID.
 */
void pio_handler_process(PioGroup *p_pio_group, uint32_t ul_id)
{
	uint32_t status;
	uint32_t i;

	/* Read PIO controller status */
	status = pio_get_interrupt_status(p_pio_group);
	status &= pio_get_interrupt_mask(p_pio_group);

	/* Check pending events */
	if (status != 0) {
		/* Find triggering source */
		i = 0;
		while (status != 0) {
			/* Source is configured on the same controller */
			if (gs_interrupt_sources[i].id == ul_id) {
				/* Source has PIOs whose statuses have changed */
				if ((status & gs_interrupt_sources[i].mask) != 0) {
					gs_interrupt_sources[i].handler(gs_interrupt_sources[i].id,
							gs_interrupt_sources[i].mask);
					status &= ~(gs_interrupt_sources[i].mask);
				}
			}

			i++;
			if (i >= MAX_INTERRUPT_SOURCES) {
				break;
			}
		}
	}
}

/**
 * \brief Set an interrupt handler for the provided pins.
 * The provided handler will be called with the triggering pin as its parameter
 * as soon as an interrupt is detected.
 *
 * \param p_pio_group PIO controller base address.
 * \param ul_id PIO ID.
 * \param ul_mask Pins (bit mask) to configure.
 * \param ul_attr Pins attribute to configure.
 * \param p_handler Interrupt handler function pointer.
 *
 * \return 0 if successful, 1 if the maximum number of sources has been defined.
 */
uint32_t pio_handler_set(PioGroup *p_pio_group, uint32_t ul_id, uint32_t ul_mask,
		uint32_t ul_attr, void (*p_handler)(uint32_t, uint32_t))
{
	uint8_t i;
	struct s_interrupt_source *pSource;

	if (gs_ul_nb_sources >= MAX_INTERRUPT_SOURCES) {
		return 1;
	}

	/* Check interrupt for this pin, if already defined, redefine it. */
	for (i = 0; i <= gs_ul_nb_sources; i++) {
		pSource = &(gs_interrupt_sources[i]);
		if (pSource->id == ul_id && pSource->mask == ul_mask) {
			break;
		}
	}

	/* Define new source */
	pSource->id = ul_id;
	pSource->mask = ul_mask;
	pSource->attr = ul_attr;
	pSource->handler = p_handler;
	if (i == gs_ul_nb_sources + 1) {
		gs_ul_nb_sources++;
	}

	/* Configure interrupt mode */
	pio_configure_interrupt(p_pio_group, ul_mask, ul_attr);

	return 0;
}

/**
 * \brief Set an interrupt handler for the specified pin.
 * The provided handler will be called with the triggering pin as its parameter
 * as soon as an interrupt is detected.
 *
 * \param ul_pin Pin index to configure.
 * \param ul_flag Pin flag.
 * \param p_handler Interrupt handler function pointer.
 *
 * \return 0 if successful, 1 if the maximum number of sources has been defined.
 */
uint32_t pio_handler_set_pin(uint32_t ul_pin, uint32_t ul_flag,
		void (*p_handler)(uint32_t, uint32_t))
{
	PioGroup *p_pio_group = pio_get_pin_group(ul_pin);
	uint32_t group_id =  pio_get_pin_group_id(ul_pin);
	uint32_t group_mask = pio_get_pin_group_mask(ul_pin);

	return pio_handler_set(p_pio_group, group_id, group_mask, ul_flag, p_handler);
}

#ifdef ID_PIOA

/**
 * \brief Parallel IO Controller A interrupt handler.
 * Redefined PIOA interrupt handler for NVIC interrupt table.
 */
void PIOA_Handler(void)
{
	pio_handler_process(PIOA, ID_PIOA);
}

#endif

#ifdef ID_PIOB

/**
 * \brief Parallel IO Controller B interrupt handler
 * Redefined PIOB interrupt handler for NVIC interrupt table.
 */
void PIOB_Handler(void)
{
	pio_handler_process(PIOB, ID_PIOB);
}

#endif

#ifdef ID_PIOC

/**
 * \brief Parallel IO Controller C interrupt handler.
 * Redefined PIOC interrupt handler for NVIC interrupt table.
 */
void PIOC_Handler(void)
{
	pio_handler_process(PIOC, ID_PIOC);
}

#endif

#ifdef ID_PIOD

/**
 * \brief Parallel IO Controller D interrupt handler.
 * Redefined PIOD interrupt handler for NVIC interrupt table.
 */
void PIOD_Handler(void)
{
	pio_handler_process(PIOD, ID_PIOD);
}

#endif

/**
 * \brief Initialize PIO interrupt management logic.
 *
 * \param p_pio_group PIO controller base address.
 * \param ul_irqn NVIC line number.
 * \param ul_priority PIO controller interrupts priority.
 */
void pio_handler_set_priority(PioGroup *p_pio_group, IRQn_Type ul_irqn, uint32_t ul_priority)
{
	uint32_t bitmask = 0;

	bitmask = pio_get_interrupt_mask(p_pio_group);
	pio_disable_interrupt(p_pio_group, 0xFFFFFFFF);
	pio_get_interrupt_status(p_pio_group);
	NVIC_DisableIRQ(ul_irqn);
	NVIC_ClearPendingIRQ(ul_irqn);
	NVIC_SetPriority(ul_irqn, ul_priority);
	NVIC_EnableIRQ(ul_irqn);
	pio_enable_interrupt(p_pio_group, bitmask);
}
