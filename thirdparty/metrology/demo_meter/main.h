/**
 * \file
 *
 * \brief Meter Demo Application Header
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

#ifndef _MAIN_H_INCLUDED
#define _MAIN_H_INCLUDED

#include "rtcproc.h"

/* Maximum size of edited string. */
/* #define MAX_EDIT_SIZE			10 */

typedef enum {
	IDLE_MODE = 0,
	BACKUP_MODE,
	WAIT_MODE,
	RESTART_MODE,
	NORMAL_MODE,
} MODE_TYPE;

/* system parameters */
typedef struct {
	MODE_TYPE mode;
	uint32_t arch_id;
	rtc_t stop_time;
} sys_t;

extern sys_t Vsys;

void battery_mode_init(void);
void battery_mode(void);
void restart_mode(void);
void idle_mode(void);
void LCD_mode(void);
void normal_mode(void);
int main(void);

#endif /* _MAIN_H_INCLUDED */
