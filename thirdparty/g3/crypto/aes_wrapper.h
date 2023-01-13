/**
 * \file
 *
 * \brief aes_wrapper : Wrapper layer between G3 stack and AES module
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

#ifndef _AES_WRAPPER_H
#define _AES_WRAPPER_H

#include <stdlib.h>

/*  This include is used to find 8 & 32 bit unsigned integer types  */
#include "brg_types.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/**
 * \brief The AES context-type definition.
 */
typedef struct
{
    int nr;                     /*!< The number of rounds. */
    uint32_t *rk;               /*!< AES round keys. */
    uint32_t buf[68];           /*!< Unaligned data buffer. This buffer can
                                     hold 32 extra Bytes, which can be used for
                                     one of the following purposes:
                                     Alignment if VIA padlock is used.
                                     Simplifying key expansion in the 256-bit
                                         case by generating an extra round key. */
}
aes_wrapper_context;

#define AES_VAR     /* if variable key size scheduler is needed     */

#define AES_ENCRYPT /* if support for encryption is needed          */
#define AES_DECRYPT /* if support for decryption is needed          */

#define AES_BLOCK_SIZE  16  /* the AES block size in bytes          */
#define N_COLS           4  /* the number of columns in the state   */

/* The key schedule length is 11, 13 or 15 16-byte blocks for 128,  */
/* 192 or 256-bit keys respectively. That is 176, 208 or 240 bytes  */
/* or 44, 52 or 60 32-bit words.                                    */
#define KS_LENGTH       44

#define AES_RETURN INT_RETURN

/* This routine must be called before first use if non-static       */
/* tables are being used                                            */
void crypto_init(void);

AES_RETURN aes_encrypt(const unsigned char *in, unsigned char *out);
AES_RETURN aes_decrypt(const unsigned char *in, unsigned char *out);
AES_RETURN aes_key(const unsigned char *key, int key_len);

/* API functions to map on source file to mbedTLS or library used */
void aes_wrapper_aes_init(aes_wrapper_context *ctx);
int aes_wrapper_aes_setkey_enc(aes_wrapper_context *ctx, const unsigned char *key, unsigned int keybits);
void aes_wrapper_aes_encrypt(aes_wrapper_context *ctx, const unsigned char input[16], unsigned char output[16]);
void aes_wrapper_aes_free(aes_wrapper_context *ctx);

#if defined(__cplusplus)
}
#endif

#endif
