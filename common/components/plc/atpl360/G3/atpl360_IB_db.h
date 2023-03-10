/**
 * \file
 *
 * \brief ATPL360 IB (Information Base) Database for G3.
 *
 * Copyright (c) 2020 Atmel Corporation. All rights reserved.
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

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef ATPL360_IB_DB_H_INCLUDED
#define ATPL360_IB_DB_H_INCLUDED

#include "general_defs.h"
#include "conf_atpl360.h"

#include "compiler.h"

#if SAM4CMS16_0
#define ATPL360_HOST_DESCRIPTION                   "SAM4CMS16C"
#elif SAM4C16_0
#define ATPL360_HOST_DESCRIPTION                   "SAM4C16C  "
#elif SAMG55
#define ATPL360_HOST_DESCRIPTION                   "SAMG55    "
#elif SAME70
#define ATPL360_HOST_DESCRIPTION                   "SAME70    "
#elif PIC32CX
#define ATPL360_HOST_DESCRIPTION                   "PIC32CX   "
#else
# error Unsupported chip type
#endif

/* ! \name G3 configuration band */
#define ATPL360_HOST_PRODUCT               0x3601
#define ATPL360_HOST_MODEL                 0x0002
#define ATPL360_HOST_VERSION               0x36010400
#define ATPL360_HOST_BAND                  ATPL360_WB

#endif /* ATPL360_IB_DB_H_INCLUDED */
