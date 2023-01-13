/**
 * \file
 *
 * \brief DLMS_SRV_LIB : DLMS server lib: G3 Profile
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
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

#include "dlms_srv_lib.h"
#include "dlms_srv_data.h"
#include "dlms_srv_data_example.h"

data_access_result_t obis_0_0_29_2_1_128_cb(assoc_info_t *px_assoc_info, uint8_t uc_attr)
{
	return obis_0_0_29_2_1_128_example_cb(px_assoc_info, uc_attr);
}

data_access_result_t obis_0_0_29_2_2_128_cb(assoc_info_t *px_assoc_info, uint8_t uc_attr)
{
	return obis_0_0_29_2_2_128_example_cb(px_assoc_info, uc_attr);
}

data_access_result_t obis_0_0_29_2_3_128_cb(assoc_info_t *px_assoc_info, uint8_t uc_attr)
{
	return obis_0_0_29_2_3_128_example_cb(px_assoc_info, uc_attr);
}

data_access_result_t obis_0_0_29_1_0_255_cb(assoc_info_t *px_assoc_info, uint8_t uc_attr)
{
	return obis_0_0_29_1_0_255_example_cb(px_assoc_info, uc_attr);
}

data_access_result_t obis_0_0_29_2_0_255_cb(assoc_info_t *px_assoc_info, uint8_t uc_attr)
{
	return obis_0_0_29_2_0_255_example_cb(px_assoc_info, uc_attr);
}

data_access_result_t obis_1_0_0_2_0_255_cb(assoc_info_t *px_assoc_info, uint8_t uc_attr)
{
	return obis_1_0_0_2_0_255_example_cb(px_assoc_info, uc_attr);
}

data_access_result_t obis_0_0_1_0_0_255_cb(assoc_info_t *px_assoc_info, uint8_t uc_attr)
{
	return obis_0_0_1_0_0_255_example_cb(px_assoc_info, uc_attr);
}

data_access_result_t obis_0_0_40_0_0_255_cb(assoc_info_t *px_assoc_info, uint8_t uc_attr)
{
	return obis_0_0_40_0_0_255_example_cb(px_assoc_info, uc_attr);
}

void dlms_srv_data_init(meter_params_t *px_meter_params)
{
	dlms_srv_data_init_example(px_meter_params);
}
