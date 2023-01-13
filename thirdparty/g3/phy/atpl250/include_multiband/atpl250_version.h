/**
 * \file
 *
 * \brief ATPL250 Physical layer version info
 *
 * Copyright (c) 2013 Atmel Corporation. All rights reserved.
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

#ifndef ATPL250VERSION_H_INCLUDE
#define ATPL250VERSION_H_INCLUDE

#include "compiler.h"

#if SAM4CP16_0
#define ATPL250_PRODID                  "SAM4CP16C "
#elif SAM4CMP16_0
#define ATPL250_PRODID                  "SAM4CMP16C"
#elif SAM4CMS16_0
#define ATPL250_PRODID                  "SAM4CMS16C"
#elif SAM4CMS32_0
#define ATPL250_PRODID                  "SAM4CMS32C"
#elif SAM4C16_0
#define ATPL250_PRODID                  "SAM4C16C  "
#elif SAM4C32_0
#define ATPL250_PRODID                  "SAM4C32C  "
#elif SAM4S
#define ATPL250_PRODID                  "SAM4SD32C "
#elif SAM4E
#define ATPL250_PRODID                  "ATPL250ABN"
#elif SAMG55
#define ATPL250_PRODID                  "ATPL250CH"
#elif SAME70
#define ATPL250_PRODID                  "SAME70Q21"
#else
# error Unsupported chip type
#endif

#define ATPL250_MODEL                   0x0001
#define ATPL250_COPYMSG                 "Copyright (c) 2019 Atmel Corporation. All rights reserved."

#define ATPL250_VERSION_NUM_CENELEC_A   0x25010502
#define ATPL250_VERSION_NUM_FCC         0x25020502
#define ATPL250_VERSION_NUM_ARIB        0x25030502

#endif  /* ATPL250VERSION_H_INCLUDE */
