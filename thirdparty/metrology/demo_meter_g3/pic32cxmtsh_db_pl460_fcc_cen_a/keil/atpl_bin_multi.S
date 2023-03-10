/**
 * \file
 *
 * \brief Link binary image of PL360 multiband.
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

/*
 * For EWARM, there is another link method supported by EWARM IDE.
 */
#if defined (__GNUC__)

	.section .rodata
	.global atpl_bin_cena_start
	.global atpl_bin_cena_end
	.global atpl_bin_fcc_start
	.global atpl_bin_fcc_end
	.global core1_metlib_start
	.global core1_metlib_end

	.align 8
atpl_bin_cena_start:
    .incbin "./ATPL360CENA.bin"
	.align 8
atpl_bin_cena_end:
atpl_bin_fcc_start:
    .incbin "./ATPL360FCC.bin"
	.align 8
atpl_bin_fcc_end:

	.align 8
core1_metlib_start:
    .incbin "./MET.bin"
	.align 8
core1_metlib_end:

#elif defined(__CC_ARM)
	PRESERVE8
	AREA atpl_bin, CODE, READONLY
	EXPORT atpl_bin_cena_start
	EXPORT atpl_bin_cena_end
	EXPORT atpl_bin_fcc_start
	EXPORT atpl_bin_fcc_end
	EXPORT core1_metlib_start
	EXPORT core1_metlib_end
	
	ALIGN 4
atpl_bin_cena_start
    INCBIN ATPL360CENA.bin
	ALIGN 4
atpl_bin_cena_end
atpl_bin_fcc_start
    INCBIN ATPL360FCC.bin
	ALIGN 4
atpl_bin_fcc_end

	ALIGN 4
core1_metlib_start
    INCBIN MET.bin
	ALIGN 4
core1_metlib_end

	END
#endif
