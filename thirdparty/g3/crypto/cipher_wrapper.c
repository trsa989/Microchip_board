/**
 * \file
 *
 * \brief aes_wrapper : Wrapper layer between G3 stack and Cipher module
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

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "cipher_wrapper.h"
#include "mbedtls/cipher.h"
#include "mbedtls/cmac.h"
#include "mbedtls/ccm.h"

#if defined(__cplusplus)
extern "C"
{
#endif

const cipher_wrapper_cipher_info_t *cipher_wrapper_cipher_info_from_type(const cipher_wrapper_cipher_type_t cipher_type)
{
	return (const cipher_wrapper_cipher_info_t *)(mbedtls_cipher_info_from_type((mbedtls_cipher_type_t)cipher_type));
}

int cipher_wrapper_cipher_setup(cipher_wrapper_cipher_context_t *ctx, const cipher_wrapper_cipher_info_t *cipher_info)
{
	return mbedtls_cipher_setup((mbedtls_cipher_context_t *)ctx, (const mbedtls_cipher_info_t *)cipher_info);
}

int cipher_wrapper_cipher_cmac_starts(cipher_wrapper_cipher_context_t *ctx, const unsigned char *key, size_t keybits)
{
	return mbedtls_cipher_cmac_starts((mbedtls_cipher_context_t *)ctx, key, keybits);
}

int cipher_wrapper_cipher_cmac_update(cipher_wrapper_cipher_context_t *ctx, const unsigned char *input, size_t ilen)
{
	return  mbedtls_cipher_cmac_update((mbedtls_cipher_context_t *)ctx, input, ilen);
}

int cipher_wrapper_cipher_cmac_finish(cipher_wrapper_cipher_context_t *ctx, unsigned char *output)
{
	return mbedtls_cipher_cmac_finish((mbedtls_cipher_context_t *)ctx, output);
}

int cipher_wrapper_cipher_cmac_reset(cipher_wrapper_cipher_context_t *ctx)
{
	return mbedtls_cipher_cmac_reset((mbedtls_cipher_context_t *)ctx );
}

void cipher_wrapper_cipher_free(cipher_wrapper_cipher_context_t *ctx)
{
	mbedtls_cipher_free((mbedtls_cipher_context_t *)ctx);
}

void cipher_wrapper_ccm_init(cipher_wrapper_ccm_context *ctx)
{
	mbedtls_ccm_init((mbedtls_ccm_context *)ctx);
}

int cipher_wrapper_ccm_setkey(cipher_wrapper_ccm_context *ctx, cipher_wrapper_cipher_id_t cipher,
		const unsigned char *key, unsigned int keybits)
{
	return mbedtls_ccm_setkey((mbedtls_ccm_context *)ctx, (mbedtls_cipher_id_t)cipher, key, keybits);
}

int cipher_wrapper_ccm_auth_decrypt(cipher_wrapper_ccm_context *ctx, size_t length, const unsigned char *iv,
		size_t iv_len, const unsigned char *add, size_t add_len, const unsigned char *input,
		unsigned char *output, const unsigned char *tag, size_t tag_len)
{
	return mbedtls_ccm_auth_decrypt((mbedtls_ccm_context *)ctx, length, iv, iv_len, add, add_len, input, output, tag, tag_len);
}

int cipher_wrapper_ccm_encrypt_and_tag(cipher_wrapper_ccm_context *ctx, size_t length, const unsigned char *iv,
		size_t iv_len, const unsigned char *add, size_t add_len, const unsigned char *input,
		unsigned char *output, unsigned char *tag, size_t tag_len)
{
	return mbedtls_ccm_encrypt_and_tag((mbedtls_ccm_context *)ctx, length, iv, iv_len, add, add_len, input, output, tag, tag_len);
}

void cipher_wrapper_ccm_free(cipher_wrapper_ccm_context *ctx)
{
	mbedtls_ccm_free((mbedtls_ccm_context *)ctx);
}

#if defined(__cplusplus)
}
#endif
