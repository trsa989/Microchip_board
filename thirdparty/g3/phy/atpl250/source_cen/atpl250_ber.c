/**
 * \file
 *
 * \brief ATPL250 BER Configuration
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
#include <string.h>

/* Phy layer includes */
#include "atpl250.h"
#include "atpl250_ber.h"
#include "atpl250_reg.h"
#include "atpl250_common.h"

/* Pointer to static notching array defined in atpl250 module */
extern uint8_t auc_static_notching_pos[];
static uint8_t *puc_static_notching = auc_static_notching_pos;

/* Extern variables needed */
extern uint8_t uc_used_carriers;
extern uint8_t uc_num_symbols_fch;
extern uint8_t uc_legacy_mode;
extern uint8_t uc_impulsive_noise_detected;

/* ------------------------- Modulation and tone map selection -------------------------- */
/* Number of elements in the BER->SNR conversion tables */
#define NUM_ELEM_TABLE_BER2SNR  256
#define NUM_ELEM_TABLE_SNR_LOG2SNR_LIN  251

/* Threshold to identify corrupted carriers (in dB in Q7.25) */
#define THRESHOLD_SNR_DB        201326592L  /* 6 dB */
/* #define THRESHOLD_SNR_DB	100663296L  //3dB */

/* Range limit for LQI */
#define MIN_SNR_LQI     -335544320L /* -10 in Q7.25 */
#define MAX_SNR_LQI     1803550720L /* 53.75 in Q7.25 */
#define STEP_SNR_LQI    8388608L /* 0.25 in Q7.25 */
/* Step and max value for per-carrier SNR */
#define MAX_PER_CARRIER_SNR             1778384896L /* 53 in Q7.25 */
#define STEP_PER_CARRIER_SNR    33554432L /* 1 in Q7.25 */

/* --Minimum values of SNR for FER=10% in Q7.25 */
/* CENELEC-DIFFERENTIAL */
#define SNR_MIN_CEN_DIF_8PSK    359032422L      /* 10.7dB */
#define SNR_MIN_CEN_DIF_QPSK    218103808L      /* 6.5dB */
#define SNR_MIN_CEN_DIF_BPSK    128513475L      /* 3.83dB */
/* FCC-DIFFERENTIAL */
#define SNR_MIN_FCC_DIF_8PSK    359032422L      /* 10.7dB */
#define SNR_MIN_FCC_DIF_QPSK    228170138L      /* 6.8dB */
#define SNR_MIN_FCC_DIF_BPSK    127506842L      /* 3.8dB */
/* %CENELEC-COHERENT */
#define SNR_MIN_CEN_COH_8PSK    427819008L      /* 12.75dB */
#define SNR_MIN_CEN_COH_QPSK    251658240L      /* 7.5dB */
#define SNR_MIN_CEN_COH_BPSK    159383552L      /* 4.75dB */
/* %FCC-COHERENT */
#define SNR_MIN_FCC_COH_8PSK    427819008L      /* 12.75dB */
#define SNR_MIN_FCC_COH_QPSK    251658240L      /* 7.5dB */
#define SNR_MIN_FCC_COH_BPSK    159383552L      /* 4.75dB */

/* Minimum number of groups in the tone map for which the baud rate of the current modulation is lower than with */
/* the next low-order modulation */
#define MAX_TONE_MAP_GROUPS_DELETED_CEN_8PSK    2
#define MAX_TONE_MAP_GROUPS_DELETED_CEN_QPSK    3
#define MAX_TONE_MAP_GROUPS_DELETED_CEN_BPSK    4

#define MAX_TONE_MAP_GROUPS_DELETED_FCC_8PSK    8
#define MAX_TONE_MAP_GROUPS_DELETED_FCC_QPSK    12
#define MAX_TONE_MAP_GROUPS_DELETED_FCC_BPSK    19

/* Limits for stopping deleting groups of carriers from the tone map */
#define MAX_TONE_MAP_GROUPS_DELETED_8PSK        MAX_TONE_MAP_GROUPS_DELETED_CEN_8PSK
#define MAX_TONE_MAP_GROUPS_DELETED_QPSK        MAX_TONE_MAP_GROUPS_DELETED_CEN_QPSK
#define MAX_TONE_MAP_GROUPS_DELETED_BPSK        MAX_TONE_MAP_GROUPS_DELETED_CEN_BPSK

#define NUM_BYTES_TONE_MAP_FCH  1       /* Size (bytes) of the tone map field in the FCH */

/* Fractional parts of different Q formats */
#define FRAC_PART_Q31   31
#define FRAC_PART_Q25   25
#define FRAC_PART_Q26   26

/* Conversion scales from different Q formats */
#define Q_6_26_TO_Q_7_25        1

#define VALUE_01_Q7_25  3355443L
#define VALUE_1_Q7_25   33554432L
#define VALUE_9_Q7_25   301989888L
#define VALUE_12_Q7_25  402653184L
#define VALUE_15_Q7_25  503316480L
#define VALUE_18_Q7_25  603979776L
#define VALUE_21_Q7_25  704643072L
#define VALUE_24_Q7_25  805306368L
#define VALUE_27_Q7_25  905969664L
#define VALUE_30_Q7_25  1006632960L
#define VALUE_33_Q7_25  1107296256L
#define VALUE_36_Q7_25  1207959552L

#define MIN_SNR_LOG_IN_TABLE_LOG_LIN    -536870912L  /* -16 dB in Q7.25 */
#define VALUE_STEP_TABLE_LOG_LIN        6710886L        /* 0.2 in Q7.25 */

static uint8_t uc_notched_carriers_ber = 0, uc_num_subbands_tone_map = 0, uc_num_corr_carr;

static uint16_t aus_snr_distance_accum[NUMBER_SUBBANDS_TONE_MAP];

/* Defines fixed point multiplication of real values in Quc_q_int.uc_q_frac using half-up rounding. */
#define mult_real_q(sl_a, sl_b, uc_q_frac)  (int32_t)((((int64_t)sl_a * (int64_t)sl_b) + (1 << (uc_q_frac - 1))) >> uc_q_frac)

/* Defines division of Q(16-uc_q_frac values).uc_q_frac values and Q(32-uc_q_frac values).uc_q_frac */
/* div_real_q(dividend,divisor,quotient), the sum of divisor >>1 is for rounding instead of truncating. */
#define div_real_q(sl_dividend, sl_divisor, uc_q_frac, psl_quotient) do { \
		*psl_quotient = ((((int64_t)sl_dividend) << uc_q_frac) + (((int64_t)sl_divisor) >> 1)) / ((int64_t)sl_divisor);	\
} \
	while (0)

/* BER->SNR conversion tables in Q7.25 */
static const q31_t asl_ber2snr_mix[NUM_ELEM_TABLE_BER2SNR]
	= {1140850688, 1015021568, 939524096, 880803840, 830472192, 796917760, 763363328, 729808896, 704643072, 687865856, 662700032, 645922816,
	   620756992, 603979776,
	   587202560, 570425344, 562036736, 545259520, 528482304, 520093696, 503316480, 494927872, 478150656,
	   469762048, 461373440, 444596224, 436207616, 427819008, 419430400,
	   411041792, 402653184, 394264576, 385875968, 377487360, 377487360, 369098752, 360710144, 352321536,
	   343932928, 335544320, 327155712, 327155712, 318767104, 318767104,
	   310378496, 301989888, 293601280, 293601280, 285212672, 285212672, 276824064, 268435456, 268435456,
	   260046848, 260046848, 251658240, 243269632, 243269632, 234881024,
	   234881024, 234881024, 226492416, 226492416, 226492416, 218103808, 209715200, 209715200, 209715200,
	   201326592, 201326592, 192937984, 192937984, 192937984, 184549376,
	   184549376, 184549376, 176160768, 176160768, 176160768, 167772160, 167772160, 167772160, 159383552,
	   159383552, 159383552, 150994944, 150994944, 150994944, 142606336,
	   142606336, 142606336, 134217728, 134217728, 134217728, 125829120, 125829120, 125829120, 117440512,
	   117440512, 117440512, 117440512, 117440512, 109051904, 109051904,
	   109051904, 100663296, 100663296, 100663296, 92274688, 92274688, 92274688, 92274688, 92274688, 83886080,
	   83886080, 83886080, 83886080, 75497472, 75497472, 75497472, 67108864,
	   67108864, 67108864, 58720256, 58720256, 58720256, 58720256, 58720256, 50331648, 50331648, 50331648,
	   41943040, 41943040, 41943040, 41943040, 41943040, 33554432, 33554432,
	   33554432, 25165824, 25165824, 25165824, 25165824, 25165824, 16777216, 16777216, 16777216, 8388608,
	   8388608, 8388608, 0, 0, 0, 0, -8388608, -8388608, -8388608, -8388608, -8388608,
	   -16777216, -16777216, -16777216, -25165824, -25165824, -25165824, -33554432, -33554432, -33554432,
	   -33554432, -33554432, -41943040, -41943040, -41943040, -50331648,
	   -50331648, -50331648, -58720256, -58720256, -58720256, -58720256, -67108864, -67108864, -67108864,
	   -67108864, -75497472, -75497472, -75497472, -83886080, -83886080,
	   -83886080, -92274688, -92274688, -92274688, -92274688, -100663296, -100663296, -100663296, -109051904,
	   -109051904, -117440512, -117440512, -117440512, -125829120,
	   -125829120, -125829120, -134217728, -134217728, -134217728, -142606336, -142606336, -150994944,
	   -150994944, -150994944, -159383552, -159383552, -167772160, -176160768,
	   -176160768, -184549376, -184549376, -192937984, -192937984, -192937984, -201326592, -201326592,
	   -209715200, -218103808, -218103808, -226492416, -226492416, -234881024,
	   -243269632, -243269632, -251658240, -260046848, -268435456, -268435456, -276824064, -276824064,
	   -285212672, -293601280, -301989888, -310378496, -327155712, -335544320,
	   -343932928, -352321536, -369098752, -377487360, -394264576, -411041792, -427819008, -444596224,
	   -469762048, -494927872, -511705088};

static const q31_t asl_ber2snr_coh[NUM_ELEM_TABLE_BER2SNR]
	= {922746880, 889192448, 855638016, 822083584, 805306368, 788529152, 771751936, 754974720, 738197504, 721420288, 713031680, 696254464,
	   679477248, 671088640, 654311424,
	   637534208, 629145600, 612368384, 603979776, 587202560, 578813952, 562036736, 545259520, 536870912,
	   528482304, 511705088, 503316480, 494927872, 478150656, 469762048,
	   461373440, 452984832, 444596224, 436207616, 427819008, 419430400, 411041792, 402653184, 394264576,
	   385875968, 377487360, 369098752, 360710144, 352321536, 343932928,
	   335544320, 335544320, 327155712, 318767104, 310378496, 301989888, 301989888, 293601280, 285212672,
	   276824064, 276824064, 268435456, 268435456, 260046848, 251658240,
	   251658240, 243269632, 234881024, 226492416, 226492416, 218103808, 218103808, 209715200, 201326592,
	   192937984, 192937984, 184549376, 184549376, 176160768, 167772160,
	   167772160, 159383552, 159383552, 150994944, 142606336, 142606336, 134217728, 134217728, 134217728,
	   125829120, 117440512, 117440512, 109051904, 109051904, 100663296,
	   92274688, 92274688, 83886080, 83886080, 83886080, 75497472, 75497472, 67108864, 58720256, 58720256,
	   50331648, 50331648, 50331648, 41943040, 41943040, 33554432, 33554432,
	   25165824, 16777216, 16777216, 8388608, 8388608, 8388608, 0, 0, 0, -8388608, -16777216, -16777216,
	   -16777216, -25165824, -25165824, -25165824, -33554432, -33554432, -41943040,
	   -41943040, -41943040, -50331648, -50331648, -58720256, -58720256, -67108864, -67108864, -67108864,
	   -75497472, -75497472, -75497472, -83886080, -83886080, -83886080,
	   -92274688, -92274688, -100663296, -100663296, -100663296, -109051904, -109051904, -117440512,
	   -117440512, -125829120, -125829120, -125829120, -134217728, -134217728,
	   -134217728, -142606336, -142606336, -142606336, -150994944, -150994944, -159383552, -159383552,
	   -159383552, -167772160, -167772160, -167772160, -176160768, -176160768,
	   -176160768, -184549376, -184549376, -184549376, -192937984, -192937984, -192937984, -201326592,
	   -201326592, -201326592, -209715200, -209715200, -209715200, -218103808,
	   -218103808, -218103808, -226492416, -226492416, -226492416, -234881024, -234881024, -234881024,
	   -243269632, -243269632, -243269632, -251658240, -251658240, -251658240,
	   -260046848, -260046848, -260046848, -268435456, -268435456, -268435456, -268435456, -276824064,
	   -276824064, -276824064, -285212672, -285212672, -285212672, -293601280,
	   -293601280, -293601280, -301989888, -301989888, -301989888, -310378496, -310378496, -318767104,
	   -318767104, -318767104, -327155712, -327155712, -327155712, -335544320,
	   -335544320, -335544320, -343932928, -343932928, -352321536, -352321536, -352321536, -360710144,
	   -360710144, -360710144, -369098752, -369098752, -377487360, -377487360,
	   -377487360, -385875968, -385875968, -385875968, -394264576, -394264576, -394264576, -402653184,
	   -402653184, -411041792, -411041792, -411041792, -419430400, -419430400,
	   -427819008, -444596224, -469762048};

/* SNR_log to SNR_linear conversion table in Q13.19. Valures range from -16 dB to 34 dB in steps of 0.2 dB */
static const q31_t asl_snr_log2snr_lin[NUM_ELEM_TABLE_SNR_LOG2SNR_LIN]
	= {13170, 13790, 14440, 15121, 15833, 16579, 17361, 18179, 19036, 19933, 20872, 21856, 22886, 23965, 25094, 26277,
	   27515, 28812, 30170, 31591, 33080, 34639, 36272, 37981,
	   39771, 41646, 43608, 45664, 47816, 50069, 52429, 54900, 57487, 60196, 63033, 66004, 69115,
	   72372, 75783, 79354, 83094, 87010, 91111, 95405, 99901, 104609, 109539, 114702, 120107,
	   125768, 131695, 137902, 144401, 151206, 158332, 165794, 173608, 181790, 190357, 199329,
	   208723, 218560, 228860, 239646, 250940, 262766, 275150, 288118, 301696, 315915, 330803,
	   346394, 362719, 379813, 397713, 416457, 436084, 456636, 478156, 500691, 524288, 548997,
	   574870, 601963, 630333, 660039, 691146, 723719, 757827, 793542, 830940, 870102, 911108,
	   954047, 999010, 1046092, 1095393, 1147017, 1201074, 1257679, 1316952, 1379018, 1444009,
	   1512063, 1583324, 1657944, 1736081, 1817900, 1903575, 1993287, 2087228, 2185596, 2288600,
	   2396459, 2509400, 2627665, 2751503, 2881177, 3016963, 3159148, 3308034, 3463936, 3627187,
	   3798131, 3977131, 4164568, 4360838, 4566358, 4781563, 5006912, 5242880, 5489969, 5748704,
	   6019632, 6303328, 6600395, 6911462, 7237189, 7578267, 7935420, 8309405, 8701015, 9111081,
	   9540473, 9990102, 10460921, 10953929, 11470172, 12010744, 12576793, 13169519, 13790179,
	   14440091, 15120631, 15833244, 16579442, 17360807, 18178997, 19035747, 19932874, 20872281,
	   21855962, 22886001, 23964585, 25094001, 26276645, 27515025, 28811769, 30169625, 31591476,
	   33080336, 34639365, 36271868, 37981308, 39771312, 41645676, 43608376, 45663576, 47815634,
	   50069115, 52428800, 54899693, 57487036, 60196317, 63033282, 66003949, 69114619, 72371891,
	   75782673, 79354200, 83094048, 87010150, 91110812, 95404732, 99901019, 104609209, 109539289,
	   114701717, 120107442, 125767931, 131695191, 137901794, 144400906, 151206311, 158332445,
	   165794423, 173608073, 181789970, 190357467, 199328738, 208722812, 218559615, 228860013,
	   239645853, 250940014, 262766452, 275150254, 288117686, 301696254, 315914760, 330803364,
	   346393646, 362718676, 379813081, 397713120, 416456762, 436083764, 456635759, 478156339,
	   500691153, 524288000, 548996932, 574870360, 601963166, 630332816, 660039486, 691146189,
	   723718905, 757826727, 793542000, 830940482, 870101501, 911108121, 954047324, 999010189,
	   1046092089, 1095392890, 1147017166, 1201074420, 1257679313, 1316951913};

/* Parameters of the fitting functions */
/* Coefficients are in Q6.26 */
/* CENELEC-DIFFERENTIAL */
/* 8PSK */
static const q31_t asl_k1_cen_dif_8psk_sqr[][3] = {
	{0, 494663, -1775895},
	{1005, -64837, 1155407}
};

static const q31_t asl_k2_cen_dif_8psk_sqr[][3] = {
	{0, -527221, 18358114},
	{-78234, 4404455, -2668494}
};

static const q31_t asl_k3_cen_dif_8psk_sqr[1] = {0};

static const q31_t asl_k_cen_dif_8psk_lin[][2] = {
	{0, 33554432},
	{0, 50331648},
	{0, 147639501},
	{67108864, 0}
};

/* QPSK */
static const q31_t asl_k1_cen_dif_qpsk_sqr[][3] = {
	{ 899, -56499, 956855}
};

static const q31_t asl_k2_cen_dif_qpsk_sqr[][3] = {
	{-77843, 3995029, 9180005}
};

static const q31_t asl_k3_cen_dif_qpsk_sqr[1] = {0};

static const q31_t asl_k_cen_dif_qpsk_lin[][2] = {
	{ 0, 83886080},
	{0, 201326592},
	{0, 100663296},
	{0, 301989888},
	{0, 483183821},
	{67108864, 0},
};

/* BPSK */
static const q31_t asl_k1_cen_dif_bpsk_sqr[][3] = {
	{ 0, 384271, -1975825},
	{ 1430, -82770, 1287605},
};

static const q31_t asl_k2_cen_dif_bpsk_sqr[][3] = {
	{0, 274299, 14757509},
	{-110308, 5620457, -11215152}
};

static const q31_t asl_k3_cen_dif_bpsk_sqr[1] = {0};

static const q31_t asl_k_cen_dif_bpsk_lin[][2] = {
	{ 0, 67108864},
	{0, 53687091},
	{0, 147639501},
	{ 0, 134217728},
	{0, 107374182},
	{0, 201326592},
	{67108864, 0}
};

/* CENELEC-COHERENT */
/* 8PSK */
static const q31_t asl_k1_cen_coh_8psk_sqr[][3] = {
	{0, -42477, 897934},
	{820, -49381, 839497}
};

static const q31_t asl_k2_cen_coh_8psk_sqr[][3] = {
	{0, 4463105, 10509156},
	{-47573, 2783208, 21010905}
};

static const q31_t asl_k3_cen_coh_8psk_sqr[1] = {0};

static const q31_t asl_k_cen_coh_8psk_lin[][2] = {
	{0, 53687091},
	{0, 255013683},
	{0, 389231411},
	{67108864, 0}
};

/* QPSK */
static const q31_t asl_k1_cen_coh_qpsk_sqr[][3] = {
	{0, 0, 921783},
	{ 1147, -55159, 801890}
};

static const q31_t ass_k2_cen_coh_qpsk_sqr[][3] = {
	{0, 0, 11564917},
	{-59423, 2866840, 25627933}
};

static const q31_t asl_k3_cen_coh_qpsk_sqr[1] = {0};

static const q31_t asl_k_cen_coh_qpsk_lin[][2] = {{67108864, 0}};

/* BPSK */
static const q31_t asl_k1_cen_coh_bpsk_sqr[][3] = {
	{ 0, 0, 997194},
	{ -167195, 1450316, -2487657},
	{ 1358, -56819, 73802},
};

static const q31_t asl_k2_cen_coh_bpsk_sqr[][3] = {
	{0, 0, 18607416},
	{6774158, -55669769, 145823806},
	{-73802, 3050034, 28434449}
};

static const q31_t asl_k3_cen_coh_bpsk_sqr[1] = {0};

static const q31_t asl_k_cen_coh_bpsk_lin[][2] = {
	{0, 33554432},
	{0, 67108864},
	{0, 1006632960},
	{67108864, 0},
};

static void _correct_snr_value(q31_t *sll_val, uint8_t uc_mod_scheme)
{
	if (uc_mod_scheme == MOD_SCHEME_COHERENT) {
		if (*sll_val < 0) {
			*sll_val += (1 << 27); /* +4 dB */
		} else if (*sll_val < 268435456) { /* >0, <8 */
			*sll_val += (1 << 26); /* +2 dB */
		} else { /* >8 */
			*sll_val += (1 << 25); /* +1 dB */
		}
	}
}

/**
 * \brief Computes the SNR loss using a squared-law approximation
 *
 */
static inline q31_t _snr_loss_sqr_aprox(uint8_t num_carr_corr, q31_t sll_snr_be, const q31_t *psl_ka, const q31_t *psl_kb, const q31_t psl_kc)
{
	q31_t sl_snr_loss, sl_k1, sl_k2;

	sl_k1 = ((*psl_ka) * num_carr_corr * num_carr_corr + (*(psl_ka + 1)) * num_carr_corr + *(psl_ka + 2)) >> Q_6_26_TO_Q_7_25;
	sl_k2 = ((*psl_kb) * num_carr_corr * num_carr_corr + (*(psl_kb + 1)) * num_carr_corr + *(psl_kb + 2)) >> Q_6_26_TO_Q_7_25;

	sl_snr_loss
		= mult_real_q(sll_snr_be,
			mult_real_q(sl_k1, sll_snr_be,
			FRAC_PART_Q25), FRAC_PART_Q25) + mult_real_q(sl_k2, sll_snr_be, FRAC_PART_Q25) + (psl_kc >> Q_6_26_TO_Q_7_25);

	return sl_snr_loss;
}

/* --Functions for computing the snr_loss */
/* CENELEC-DIFFERENTIAL */
/* 8PSK */

/**
 * \brief Computes the SNR loss when using 8DPSK in CENELEC
 *
 */
static inline q31_t _snr_loss_cen_dif_8psk(uint8_t num_carriers_corr, q31_t sll_snr_be)
{
	q31_t sl_snr_loss = 0;

	if (num_carriers_corr == 1) {
		sl_snr_loss = mult_real_q(sll_snr_be, (asl_k_cen_dif_8psk_lin[0][0]), FRAC_PART_Q26) + (asl_k_cen_dif_8psk_lin[0][1] >> Q_6_26_TO_Q_7_25);
	} else if (num_carriers_corr == 2) {
		if (sll_snr_be < VALUE_18_Q7_25) {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_dif_8psk_lin[1][0]),
					FRAC_PART_Q26) + (asl_k_cen_dif_8psk_lin[1][1] >> Q_6_26_TO_Q_7_25);
		} else {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_dif_8psk_lin[2][0]),
					FRAC_PART_Q26) + (asl_k_cen_dif_8psk_lin[2][1] >> Q_6_26_TO_Q_7_25);
		}
	} else if ((num_carriers_corr > 2) && (num_carriers_corr <= 5)) {
		sl_snr_loss = _snr_loss_sqr_aprox(num_carriers_corr, sll_snr_be, asl_k1_cen_dif_8psk_sqr[0], asl_k2_cen_dif_8psk_sqr[0],
				asl_k3_cen_dif_8psk_sqr[0]);
	} else if ((num_carriers_corr > 5) && (num_carriers_corr <= 26)) {
		sl_snr_loss = _snr_loss_sqr_aprox(num_carriers_corr, sll_snr_be, asl_k1_cen_dif_8psk_sqr[1], asl_k2_cen_dif_8psk_sqr[1],
				asl_k3_cen_dif_8psk_sqr[0]);
	} else if (num_carriers_corr > 26) {
		sl_snr_loss = mult_real_q(sll_snr_be, (asl_k_cen_dif_8psk_lin[3][0]), FRAC_PART_Q26) + (asl_k_cen_dif_8psk_lin[3][1] >> Q_6_26_TO_Q_7_25);
	}

	if (sl_snr_loss < 0) {
		sl_snr_loss = 0;
	}

	return sl_snr_loss;
}

/* QPSK */

/**
 * \brief Computes the SNR loss when using 8QPSK in CENELEC
 *
 */
static inline q31_t _snr_loss_cen_dif_qpsk(uint8_t num_carriers_corr, q31_t sll_snr_be)
{
	q31_t sl_snr_loss = 0;

	if (num_carriers_corr == 1) {
		if (sll_snr_be < VALUE_18_Q7_25) {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_dif_qpsk_lin[0][0]),
					FRAC_PART_Q26) + (asl_k_cen_dif_qpsk_lin[0][1] >> Q_6_26_TO_Q_7_25);
		} else {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_dif_qpsk_lin[1][0]),
					FRAC_PART_Q26) + (asl_k_cen_dif_qpsk_lin[1][1] >> Q_6_26_TO_Q_7_25);
		}
	} else if (num_carriers_corr == 2) {
		if (sll_snr_be <= VALUE_12_Q7_25) {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_dif_qpsk_lin[2][0]),
					FRAC_PART_Q26) + (asl_k_cen_dif_qpsk_lin[2][1] >> Q_6_26_TO_Q_7_25);
		} else if (sll_snr_be <= VALUE_24_Q7_25) {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_dif_qpsk_lin[3][0]),
					FRAC_PART_Q26) + (asl_k_cen_dif_qpsk_lin[3][1] >> Q_6_26_TO_Q_7_25);
		} else {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_dif_qpsk_lin[4][0]),
					FRAC_PART_Q26) + (asl_k_cen_dif_qpsk_lin[4][1] >> Q_6_26_TO_Q_7_25);
		}
	} else if ((num_carriers_corr > 2) && (num_carriers_corr <= 24)) {
		sl_snr_loss = _snr_loss_sqr_aprox(num_carriers_corr, sll_snr_be, asl_k1_cen_dif_qpsk_sqr[0], asl_k2_cen_dif_qpsk_sqr[0],
				asl_k3_cen_dif_qpsk_sqr[0]);
	} else if (num_carriers_corr > 24) {
		sl_snr_loss = mult_real_q(sll_snr_be, (asl_k_cen_dif_qpsk_lin[5][0]), FRAC_PART_Q26) + (asl_k_cen_dif_qpsk_lin[5][1] >> Q_6_26_TO_Q_7_25);
	}

	if (sl_snr_loss < 0) {
		sl_snr_loss = 0;
	}

	return sl_snr_loss;
}

/* BPSK */

/**
 * \brief Computes the SNR loss when using 8BPSK in CENELEC
 *
 */
static inline q31_t _snr_loss_cen_dif_bpsk(uint8_t num_carriers_corr, q31_t sll_snr_be)
{
	q31_t sl_snr_loss = 0;

	if (num_carriers_corr == 1) {
		sl_snr_loss = mult_real_q(sll_snr_be, (asl_k_cen_dif_bpsk_lin[0][0]), FRAC_PART_Q26) + (asl_k_cen_dif_bpsk_lin[0][1] >>  Q_6_26_TO_Q_7_25);
	} else if (num_carriers_corr == 2) {
		if (sll_snr_be <= VALUE_12_Q7_25) {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_dif_bpsk_lin[1][0]),
					FRAC_PART_Q26) + (asl_k_cen_dif_bpsk_lin[1][1] >>  Q_6_26_TO_Q_7_25);
		} else {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_dif_bpsk_lin[2][0]),
					FRAC_PART_Q26) + (asl_k_cen_dif_bpsk_lin[2][1] >>  Q_6_26_TO_Q_7_25);
		}
	} else if (num_carriers_corr == 3) {
		sl_snr_loss = mult_real_q(sll_snr_be, (asl_k_cen_dif_bpsk_lin[3][0]), FRAC_PART_Q26) + (asl_k_cen_dif_bpsk_lin[3][1] >>  Q_6_26_TO_Q_7_25);
	} else if (num_carriers_corr == 4) {
		if (sll_snr_be <= VALUE_15_Q7_25) {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_dif_bpsk_lin[4][0]),
					FRAC_PART_Q26) + (asl_k_cen_dif_bpsk_lin[4][1] >>  Q_6_26_TO_Q_7_25);
		} else {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_dif_bpsk_lin[5][0]),
					FRAC_PART_Q26) + (asl_k_cen_dif_bpsk_lin[5][1] >>  Q_6_26_TO_Q_7_25);
		}
	} else if ((num_carriers_corr > 4) && (num_carriers_corr <= 7)) {
		sl_snr_loss = _snr_loss_sqr_aprox(num_carriers_corr, sll_snr_be, asl_k1_cen_dif_bpsk_sqr[0], asl_k2_cen_dif_bpsk_sqr[0],
				asl_k3_cen_dif_bpsk_sqr[0]);
	} else if ((num_carriers_corr > 7) && (num_carriers_corr <= 23)) {
		sl_snr_loss = _snr_loss_sqr_aprox(num_carriers_corr, sll_snr_be, asl_k1_cen_dif_bpsk_sqr[1], asl_k2_cen_dif_bpsk_sqr[1],
				asl_k3_cen_dif_bpsk_sqr[0]);
	} else if (num_carriers_corr > 23) {
		sl_snr_loss = mult_real_q(sll_snr_be, (asl_k_cen_dif_bpsk_lin[6][0]), FRAC_PART_Q26) + (asl_k_cen_dif_bpsk_lin[6][1] >>  Q_6_26_TO_Q_7_25);
	}

	if (sl_snr_loss < 0) {
		sl_snr_loss = 0;
	}

	return sl_snr_loss;
}

/* CENELEC-COHERENT */
/* 8PSK */

/**
 * \brief Computes the SNR loss when using 8PSK in CENELEC
 *
 */
static inline q31_t _snr_loss_cen_coh_8psk(uint8_t num_carriers_corr, q31_t sll_snr_be)
{
	q31_t sl_snr_loss = 0;

	if (num_carriers_corr == 1) {
		if (sll_snr_be <= VALUE_9_Q7_25) {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_coh_8psk_lin[0][0]),
					FRAC_PART_Q26) + (asl_k_cen_coh_8psk_lin[0][1] >> Q_6_26_TO_Q_7_25);
		} else if (sll_snr_be <= VALUE_18_Q7_25) {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_coh_8psk_lin[1][0]),
					FRAC_PART_Q26) + (asl_k_cen_coh_8psk_lin[1][1] >> Q_6_26_TO_Q_7_25);
		} else {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_coh_8psk_lin[2][0]),
					FRAC_PART_Q26) + (asl_k_cen_coh_8psk_lin[2][1] >> Q_6_26_TO_Q_7_25);
		}
	} else if ((num_carriers_corr == 2) | (num_carriers_corr == 3)) {
		sl_snr_loss = _snr_loss_sqr_aprox(num_carriers_corr, sll_snr_be, asl_k1_cen_coh_8psk_sqr[0], asl_k2_cen_coh_8psk_sqr[0],
				asl_k3_cen_coh_8psk_sqr[0]);
	} else if ((num_carriers_corr >= 4) && (num_carriers_corr <= 27)) {
		sl_snr_loss = _snr_loss_sqr_aprox(num_carriers_corr, sll_snr_be, asl_k1_cen_coh_8psk_sqr[1], asl_k2_cen_coh_8psk_sqr[1],
				asl_k3_cen_coh_8psk_sqr[0]);
	} else if (num_carriers_corr > 27) {
		sl_snr_loss = mult_real_q(sll_snr_be, (asl_k_cen_coh_8psk_lin[3][0]), FRAC_PART_Q26) + (asl_k_cen_coh_8psk_lin[3][1] >> Q_6_26_TO_Q_7_25);
	}

	if (sl_snr_loss < 0) {
		sl_snr_loss = 0;
	}

	return sl_snr_loss;
}

/* QPSK */

/**
 * \brief Computes the SNR loss when using QPSK in CENELEC
 *
 */
static inline q31_t _snr_loss_cen_coh_qpsk(uint8_t num_carriers_corr, q31_t sll_snr_be)
{
	q31_t sl_snr_loss = 0;

	if (num_carriers_corr == 1) {
		sl_snr_loss = _snr_loss_sqr_aprox(num_carriers_corr, sll_snr_be, asl_k1_cen_coh_qpsk_sqr[0], ass_k2_cen_coh_qpsk_sqr[0],
				asl_k3_cen_coh_qpsk_sqr[0]);
	} else if ((num_carriers_corr > 1) && (num_carriers_corr < 24)) {
		sl_snr_loss = _snr_loss_sqr_aprox(num_carriers_corr, sll_snr_be, asl_k1_cen_coh_qpsk_sqr[1], ass_k2_cen_coh_qpsk_sqr[1],
				asl_k3_cen_coh_qpsk_sqr[0]);
	} else if (num_carriers_corr >= 24) {
		sl_snr_loss = mult_real_q(sll_snr_be, (asl_k_cen_coh_qpsk_lin[0][0]), FRAC_PART_Q26) + (asl_k_cen_coh_qpsk_lin[0][1] >> Q_6_26_TO_Q_7_25);
	}

	if (sl_snr_loss < 0) {
		sl_snr_loss = 0;
	}

	return sl_snr_loss;
}

/* BPSK */

/**
 * \brief Computes the SNR loss when using BPSK in CENELEC
 *
 */
static inline q31_t _snr_loss_cen_coh_bpsk(uint8_t num_carriers_corr, q31_t sll_snr_be)
{
	q31_t sl_snr_loss = 0;
	if (num_carriers_corr == 1) {
		if (sll_snr_be <= VALUE_15_Q7_25) {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_coh_bpsk_lin[0][0]),
					FRAC_PART_Q26) + (asl_k_cen_coh_bpsk_lin[0][1] >> Q_6_26_TO_Q_7_25);
		} else {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_coh_bpsk_lin[1][0]),
					FRAC_PART_Q26) + (asl_k_cen_coh_bpsk_lin[1][1] >> Q_6_26_TO_Q_7_25);
		}
	} else if (num_carriers_corr == 2) {
		if (sll_snr_be < VALUE_24_Q7_25) {
			sl_snr_loss = _snr_loss_sqr_aprox(num_carriers_corr, sll_snr_be, asl_k1_cen_coh_bpsk_sqr[0], asl_k2_cen_coh_bpsk_sqr[0],
					asl_k3_cen_coh_bpsk_sqr[0]);
		} else {
			sl_snr_loss
				= mult_real_q(sll_snr_be, (asl_k_cen_coh_bpsk_lin[2][0]),
					FRAC_PART_Q26) + (asl_k_cen_coh_bpsk_lin[2][1] >> Q_6_26_TO_Q_7_25);
		}
	} else if ((num_carriers_corr > 2) && (num_carriers_corr <= 5)) {
		sl_snr_loss = _snr_loss_sqr_aprox(num_carriers_corr, sll_snr_be, asl_k1_cen_coh_bpsk_sqr[1], asl_k2_cen_coh_bpsk_sqr[1],
				asl_k3_cen_coh_bpsk_sqr[0]);
	} else if ((num_carriers_corr > 5) && (num_carriers_corr <= 20)) {
		sl_snr_loss = _snr_loss_sqr_aprox(num_carriers_corr, sll_snr_be, asl_k1_cen_coh_bpsk_sqr[2], asl_k2_cen_coh_bpsk_sqr[2],
				asl_k3_cen_coh_bpsk_sqr[0]);
	} else if (num_carriers_corr >= 21) {
		sl_snr_loss = mult_real_q(sll_snr_be, (asl_k_cen_coh_bpsk_lin[3][0]), FRAC_PART_Q26) + (asl_k_cen_coh_bpsk_lin[3][1] >> Q_6_26_TO_Q_7_25);
	}

	if (sl_snr_loss < 0) {
		sl_snr_loss = 0;
	}

	return sl_snr_loss;
}

/**
 * \brief Obtain the list with the indexes of the corrupted carriers,
 *        the average snr value of the good carriers and the equivalent snr of the corrupted carriers contained in auc_ber_k_16
 *
 */
static inline void _find_good_and_corrupted_carriers(const q31_t *psl_ber2snr, q31_t *psl_snr_good, q31_t *psl_snr_be_eq, uint8_t uc_mod_scheme)
{
	uint8_t uc_i, uc_j, uc_num_good_carriers;
	uint8_t uc_address;
	uint8_t uc_current_subband;
	uint32_t ul_avg_ber = 0;
	q31_t sl_avg_snr = 0, sl_avg_lin = 0, sl_distance, sl_aux;

	/* Computation of the average BER */
	for (uc_i = 0; uc_i < uc_num_subbands_tone_map; uc_i++) {
		for (uc_j = 0; uc_j < NUM_CARRIERS_IN_SUBBAND; uc_j++) {
			ul_avg_ber
				+= *(u_shared_buffers.s_ber_info.auc_ber_k_16 +
					*(u_shared_buffers.s_ber_info.auc_ber_new_tone_map + uc_i) * NUM_CARRIERS_IN_SUBBAND + uc_j);
		}
	}
	ul_avg_ber = ul_avg_ber / (uc_num_subbands_tone_map * NUM_CARRIERS_IN_SUBBAND) + 1; /* Implements ceil */

	sl_avg_snr = *(psl_ber2snr + ul_avg_ber);
	_correct_snr_value(&sl_avg_snr, uc_mod_scheme);

	/* Find corrupted carriers */
	uc_num_corr_carr = 0;
	uc_num_good_carriers = 0;
	for (uc_i = 0; uc_i < uc_num_subbands_tone_map; uc_i++) {
		for (uc_j = 0; uc_j < NUM_CARRIERS_IN_SUBBAND; uc_j++) {
			if ((*(u_shared_buffers.s_ber_info.asl_ber_snr_k + (*(u_shared_buffers.s_ber_info.auc_ber_new_tone_map + uc_i))
					* NUM_CARRIERS_IN_SUBBAND + uc_j)) <= (sl_avg_snr - THRESHOLD_SNR_DB)) {
				*(u_shared_buffers.s_ber_info.auc_ber_list_corr_carr + uc_num_corr_carr)
					= (*(u_shared_buffers.s_ber_info.auc_ber_new_tone_map + uc_i)) * NUM_CARRIERS_IN_SUBBAND + uc_j;
				uc_num_corr_carr++;
			} else {
				u_shared_buffers.s_ber_info.asc_ber_list_good_carriers[uc_num_good_carriers]
					= (*(u_shared_buffers.s_ber_info.auc_ber_new_tone_map + uc_i)) * NUM_CARRIERS_IN_SUBBAND + uc_j;
				uc_num_good_carriers++;
			}
		}
	}

	/* Find the average SNR of the good carriers */
	*psl_snr_good = 0;
	for (uc_i = 0; uc_i < uc_num_good_carriers; uc_i++) {
		/* The 7 is to avoid overflow in the average */
		*psl_snr_good += (*(u_shared_buffers.s_ber_info.asl_ber_snr_k + u_shared_buffers.s_ber_info.asc_ber_list_good_carriers[uc_i]) >> 7);
	}
	div_real_q((*psl_snr_good), uc_num_good_carriers, 0, psl_snr_good);
	*psl_snr_good = (*psl_snr_good << 7);

	/* Computes 10*log10(mean(10.^(snr_good-snr_k)/10))) for the corrupted carriers */
	memset(aus_snr_distance_accum, 0, NUMBER_SUBBANDS_TONE_MAP * sizeof(aus_snr_distance_accum[0]));
	if (uc_num_corr_carr > 0) {
		for (uc_i = 0; uc_i < uc_num_corr_carr; uc_i++) {
			/* Difference between snr_good and snr_k of the corrupted carriers */
			sl_aux = *psl_snr_good - *(u_shared_buffers.s_ber_info.asl_ber_snr_k + *(u_shared_buffers.s_ber_info.auc_ber_list_corr_carr + uc_i));
			uc_current_subband = (*(u_shared_buffers.s_ber_info.auc_ber_list_corr_carr + uc_i)) / NUM_CARRIERS_IN_SUBBAND;

			if (sl_aux < MIN_SNR_LOG_IN_TABLE_LOG_LIN) {
				sl_aux = MIN_SNR_LOG_IN_TABLE_LOG_LIN;
			}

			/* Log->Lin conversion and sum */
			sl_distance = sl_aux - MIN_SNR_LOG_IN_TABLE_LOG_LIN;
			div_real_q(sl_distance, VALUE_STEP_TABLE_LOG_LIN, 0, &sl_distance);
			uc_address = (uint8_t)sl_distance;
			aus_snr_distance_accum[uc_current_subband] += uc_address;

			sl_avg_lin = sl_avg_lin + (asl_snr_log2snr_lin [uc_address] >> 7); /* Q13.19 */
		}
		div_real_q(sl_avg_lin, uc_num_corr_carr, 0, &sl_avg_lin); /* Value in Q13.19 */
		sl_avg_lin = sl_avg_lin << 7;

		/* Linear to log conversion */
		uc_i = 0;
		while (sl_avg_lin >= asl_snr_log2snr_lin [uc_i]) {
			uc_i++;
		}

		*psl_snr_be_eq = MIN_SNR_LOG_IN_TABLE_LOG_LIN + uc_i * VALUE_STEP_TABLE_LOG_LIN; /* Q7.25 */
	} else {
		*psl_snr_be_eq = MIN_SNR_LOG_IN_TABLE_LOG_LIN;
	}
}

/**
 * \brief Deletes a subband of corrupted carriers from u_shared_buffers.s_ber_info.auc_ber_new_tone_map
 *
 */
static inline void _delete_carriers_from_tone_map(void)
{
	uint8_t uc_i, uc_j, uc_index_first_carr_group_tone_map;
	uint16_t us_snr_accum, us_snr_accum_max, us_snr_accum_sum;
	q31_t sl_snr_min, sl_snr_i;

	us_snr_accum_sum = 0;
	for (uc_i = 0; uc_i < uc_num_subbands_tone_map; uc_i++) {
		us_snr_accum_sum += aus_snr_distance_accum[u_shared_buffers.s_ber_info.auc_ber_new_tone_map[uc_i]];
	}

	if (us_snr_accum_sum) {
		/* There is narrow band noise. Find the subband most affected by narrow band */
		uc_index_first_carr_group_tone_map = 0;
		us_snr_accum_max = aus_snr_distance_accum[u_shared_buffers.s_ber_info.auc_ber_new_tone_map[0]];
		for (uc_i = 0; uc_i < uc_num_subbands_tone_map; uc_i++) {
			us_snr_accum = aus_snr_distance_accum[u_shared_buffers.s_ber_info.auc_ber_new_tone_map[uc_i]];
			if (us_snr_accum > us_snr_accum_max) {
				uc_index_first_carr_group_tone_map = uc_i;
				us_snr_accum_max = us_snr_accum;
			}
		}
	} else {
		/* No narrow band noise found. Find the index of the carrier with the lowest snr */
		uc_index_first_carr_group_tone_map = 0;
		sl_snr_min = *(u_shared_buffers.s_ber_info.asl_ber_snr_k +  (*u_shared_buffers.s_ber_info.auc_ber_new_tone_map) * NUM_CARRIERS_IN_SUBBAND);
		for (uc_i = 0; uc_i < uc_num_subbands_tone_map; uc_i++) {
			for (uc_j = 0; uc_j < NUM_CARRIERS_IN_SUBBAND; uc_j++) {
				sl_snr_i = *(u_shared_buffers.s_ber_info.asl_ber_snr_k
						+ (*(u_shared_buffers.s_ber_info.auc_ber_new_tone_map + uc_i)) * NUM_CARRIERS_IN_SUBBAND + uc_j);
				if (sl_snr_i < sl_snr_min) {
					uc_index_first_carr_group_tone_map = uc_i;
					sl_snr_min = sl_snr_i;
				}
			}
		}
	}

	/* Creates the new tone map */
	for (uc_i = (uc_index_first_carr_group_tone_map + 1); uc_i < uc_num_subbands_tone_map; uc_i++) {
		*(u_shared_buffers.s_ber_info.auc_ber_new_tone_map + uc_i - 1) = *(u_shared_buffers.s_ber_info.auc_ber_new_tone_map + uc_i);
	}
	uc_num_subbands_tone_map--;
}

/**
 * \brief Determines the modulation and tone map to be used
 *
 */
void select_modulation_tone_map(uint8_t uc_mod_scheme, uint8_t *puc_static_and_dynamic_notching, uint8_t *puc_pilot_pos, uint8_t *puc_inactive_carriers_pos,
		uint8_t *puc_tone_map_fch, uint8_t *puc_mod_type, uint16_t us_payload_symbols, uint8_t uc_payload_carriers, uint8_t uc_rrc_notch_index)
{
	uint8_t uc_i, uc_j, uc_byte_index_i, uc_bit_index_in_byte_i, uc_value_tone_map;
	uint8_t uc_bad_channel = 0;
	uint8_t uc_non_head_carriers, uc_byte_index, uc_bit_index_in_byte;

	uint16_t us_tmp;
	uint8_t uc_sel_modulation = MOD_TYPE_BPSK_ROBO;
	const q31_t *psl_table_ber2snr;
	q31_t sl_snr_min_8psk, sl_snr_min_qpsk, sl_snr_min_bpsk;
	q31_t sl_snr_good, sl_snr_be, sl_snr_eq;

	uint8_t uc_num_tone_map_disabled_8psk = 0, uc_num_tone_map_disabled_qpsk = 0, uc_num_tone_map_disabled_bpsk = 0;

	/* Pointers to functions */
	q31_t (*snr_loss_8psk)(uint8_t, q31_t);
	q31_t (*snr_loss_qpsk)(uint8_t, q31_t);
	q31_t (*snr_loss_bpsk)(uint8_t, q31_t);

	/* In case impulsive noise is detected, select always robust mode */
	if (uc_impulsive_noise_detected) {
		memset(puc_tone_map_fch, 0, NUM_BYTES_TONE_MAP_FCH);
		*puc_tone_map_fch = 0x3F;
		*puc_mod_type = MOD_TYPE_BPSK_ROBO;
		return;
	}

	if (uc_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
		/* Minimum snr values for each modulation */
		sl_snr_min_8psk = SNR_MIN_CEN_DIF_8PSK;
		sl_snr_min_qpsk = SNR_MIN_CEN_DIF_QPSK;
		sl_snr_min_bpsk = SNR_MIN_CEN_DIF_BPSK;

		/* Pointers to functions */
		snr_loss_8psk = _snr_loss_cen_dif_8psk;
		snr_loss_qpsk = _snr_loss_cen_dif_qpsk;
		snr_loss_bpsk = _snr_loss_cen_dif_bpsk;

		psl_table_ber2snr = asl_ber2snr_mix;
	} else {
		/* Minimum snr values for each modulation */
		sl_snr_min_8psk = SNR_MIN_CEN_COH_8PSK;
		sl_snr_min_qpsk = SNR_MIN_CEN_COH_QPSK;
		sl_snr_min_bpsk = SNR_MIN_CEN_COH_BPSK;

		/* Pointers to functions */
		snr_loss_8psk = _snr_loss_cen_coh_8psk;
		snr_loss_qpsk = _snr_loss_cen_coh_qpsk;
		snr_loss_bpsk = _snr_loss_cen_coh_bpsk;

		psl_table_ber2snr = asl_ber2snr_mix;
	}

	/* Creates the tone map */
	uc_num_subbands_tone_map = 0;
	uc_notched_carriers_ber = 0;
	for (uc_i = FIRST_CARRIER; uc_i < LAST_CARRIER; uc_i += NUM_CARRIERS_IN_SUBBAND) {
		uc_byte_index_i = uc_i >> 3;
		uc_bit_index_in_byte_i = uc_i & 0x07;

		uc_value_tone_map = (*(puc_static_notching  + uc_byte_index_i) >> uc_bit_index_in_byte_i) &  0x01;
		if (uc_value_tone_map == 0) {
			u_shared_buffers.s_ber_info.auc_ber_tone_map[uc_num_subbands_tone_map] = (uc_i - FIRST_CARRIER) / NUM_CARRIERS_IN_SUBBAND;
			uc_num_subbands_tone_map++;
		} else {
			uc_notched_carriers_ber++;
		}
	}

	/* Reads the values of ber_k */
	pplc_if_read_buf(BCODE_ZONE0, u_shared_buffers.s_ber_info.auc_ber_k, uc_used_carriers * 2);

	/* Obtains the values of snr_k and the average snr */
	uc_j = FIRST_CARRIER;
	uc_non_head_carriers = FIRST_CARRIER + uc_payload_carriers; /* first address of the carrier which is not in the payload */
	for (uc_i = FIRST_CARRIER; uc_i <= LAST_CARRIER; uc_i++) {
		/* Determines the carrier byte and bit */
		uc_byte_index =  uc_i >> 3;
		uc_bit_index_in_byte = uc_i & 0x07;

		if (!(puc_static_and_dynamic_notching[uc_byte_index] & (0x01 << uc_bit_index_in_byte))) {
			if ((uc_mod_scheme == MOD_SCHEME_COHERENT) && (puc_pilot_pos[uc_byte_index] & (0x01 << uc_bit_index_in_byte))) {
				/* There is no info for this carrier */
				us_tmp
					= (u_shared_buffers.s_ber_info.auc_ber_k[2 *
						(uc_non_head_carriers -
						FIRST_CARRIER)] << 8) + u_shared_buffers.s_ber_info.auc_ber_k[2 * (uc_non_head_carriers - FIRST_CARRIER) + 1];
				us_tmp = ((us_tmp << 5) + (us_tmp)) / uc_num_symbols_fch; /* the true operation is *255/31~=*8.22~=*8.25 */
				if (us_tmp & 0x0002) { /* to identify the most significative bit of the fractional part and round the result */
					us_tmp = (us_tmp >> 2) + 1;
				} else {
					us_tmp = us_tmp >> 2;
				}

				if (us_tmp > 255) { /* maximum value of the SNR table */
					us_tmp = 255;
				}

				u_shared_buffers.s_ber_info.auc_ber_k_16[uc_i - FIRST_CARRIER] = us_tmp;
				uc_non_head_carriers++;
			} else {
				us_tmp = (u_shared_buffers.s_ber_info.auc_ber_k[2 * (uc_j - FIRST_CARRIER)] << 8)
						+ u_shared_buffers.s_ber_info.auc_ber_k[2 * (uc_j - FIRST_CARRIER) + 1];
				us_tmp = ((us_tmp << 5) + (us_tmp)) / (uc_num_symbols_fch + us_payload_symbols);
				if (us_tmp & 0x0002) { /* to identify the most significative bit of the fractional part and round the result */
					us_tmp = (us_tmp >> 2) + 1;
				} else {
					us_tmp = us_tmp >> 2;
				}

				if (us_tmp > 255) { /* maximum value of the SNR table */
					us_tmp = 255;
				}

				u_shared_buffers.s_ber_info.auc_ber_k_16[uc_i - FIRST_CARRIER] = us_tmp;
				uc_j++;
			}
		} else {
			if ((puc_inactive_carriers_pos[uc_byte_index] & (0x01 << uc_bit_index_in_byte))) {
				us_tmp
					= (u_shared_buffers.s_ber_info.auc_ber_k[2 *
						(uc_non_head_carriers -
						FIRST_CARRIER)] << 8) + u_shared_buffers.s_ber_info.auc_ber_k[2 * (uc_non_head_carriers - FIRST_CARRIER) + 1];
				us_tmp = ((us_tmp << 5) + (us_tmp)) / uc_num_symbols_fch; /* the true operation is *255/31~=*8.22~=*8.25 */
				if (us_tmp & 0x0002) { /* to identify the most significative bit of the fractional part and round the result */
					us_tmp = (us_tmp >> 2) + 1;
				} else {
					us_tmp = us_tmp >> 2;
				}

				if (us_tmp > 255) { /* maximum value of the SNR table */
					us_tmp = 255;
				}

				u_shared_buffers.s_ber_info.auc_ber_k_16[uc_i - FIRST_CARRIER] = us_tmp;
				uc_non_head_carriers++;
			} else {
				/* static notching carrier, set to the minimum error. it must not be taken into account when calculating proper tonemap */
				u_shared_buffers.s_ber_info.auc_ber_k_16[uc_i - FIRST_CARRIER] = 0;
			}
		}
	}

	/* Obtains the values of snr_k */
	for (uc_i = 0; uc_i < PROTOCOL_CARRIERS; uc_i++) {
		/* asl_snr_k[uc_i] = *(psl_table_ber2snr + auc_ber_k[uc_i] ); */
		u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_i] = *(psl_table_ber2snr + u_shared_buffers.s_ber_info.auc_ber_k_16[uc_i]);
		_correct_snr_value(&u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_i], uc_mod_scheme);
	}

	/* If notch filter is applied, set lowest snr to affected carriers */
	if (uc_rrc_notch_index) {
		if (((uc_rrc_notch_index - 3) >= FIRST_CARRIER) && ((uc_rrc_notch_index - 3) <= LAST_CARRIER)) {
			u_shared_buffers.s_ber_info.auc_ber_k_16[uc_rrc_notch_index - 3 - FIRST_CARRIER] = 255;
			u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_rrc_notch_index - 3 - FIRST_CARRIER] = *(psl_table_ber2snr + 255);
		}

		if (((uc_rrc_notch_index - 2) >= FIRST_CARRIER) && ((uc_rrc_notch_index - 2) <= LAST_CARRIER)) {
			u_shared_buffers.s_ber_info.auc_ber_k_16[uc_rrc_notch_index - 2 - FIRST_CARRIER] = 255;
			u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_rrc_notch_index - 2 - FIRST_CARRIER] = *(psl_table_ber2snr + 255);
		}

		if (((uc_rrc_notch_index - 1) >= FIRST_CARRIER) && ((uc_rrc_notch_index - 1) <= LAST_CARRIER)) {
			u_shared_buffers.s_ber_info.auc_ber_k_16[uc_rrc_notch_index - 1 - FIRST_CARRIER] = 255;
			u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_rrc_notch_index - 1 - FIRST_CARRIER] = *(psl_table_ber2snr + 255);
		}

		if (((uc_rrc_notch_index) >= FIRST_CARRIER) && ((uc_rrc_notch_index) <= LAST_CARRIER)) {
			u_shared_buffers.s_ber_info.auc_ber_k_16[uc_rrc_notch_index - FIRST_CARRIER] = 255;
			u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_rrc_notch_index - FIRST_CARRIER] = *(psl_table_ber2snr + 255);
		}

		if (((uc_rrc_notch_index + 1) >= FIRST_CARRIER) && ((uc_rrc_notch_index + 1) <= LAST_CARRIER)) {
			u_shared_buffers.s_ber_info.auc_ber_k_16[uc_rrc_notch_index + 1 - FIRST_CARRIER] = 255;
			u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_rrc_notch_index + 1 - FIRST_CARRIER] = *(psl_table_ber2snr + 255);
		}
	}

	/* Tests for 8PSK */
	memcpy(u_shared_buffers.s_ber_info.auc_ber_new_tone_map, u_shared_buffers.s_ber_info.auc_ber_tone_map, uc_num_subbands_tone_map);
	uc_num_subbands_tone_map = NUMBER_SUBBANDS_TONE_MAP;
	uc_num_corr_carr = 0;
	do {
		/* Computes the SNR of the good and BE carriers and their indexes */
		_find_good_and_corrupted_carriers(psl_table_ber2snr, &sl_snr_good, &sl_snr_be, uc_mod_scheme);

		if (uc_num_corr_carr == uc_num_subbands_tone_map * NUM_CARRIERS_IN_SUBBAND) {
			uc_bad_channel = 1; /* All the carriers are below the snr threshold and are classified as corrupted */
		} else {
			/* Computes the SNR_EQ for the current modulation */
			sl_snr_eq = sl_snr_good - snr_loss_8psk(uc_num_corr_carr, sl_snr_be);

			if (sl_snr_eq >= sl_snr_min_8psk) {
				uc_sel_modulation = MOD_TYPE_8PSK;
				memcpy(u_shared_buffers.s_ber_info.auc_ber_tone_map, u_shared_buffers.s_ber_info.auc_ber_new_tone_map,
						uc_num_subbands_tone_map);
			} else {
				_delete_carriers_from_tone_map();

				/* Resulting tone map (after deleting subband) is stored to be used in the next tests. To avoid recalculation. */
				memcpy(u_shared_buffers.s_ber_info.auc_ber_tone_map_series[uc_num_tone_map_disabled_8psk],
						u_shared_buffers.s_ber_info.auc_ber_new_tone_map, uc_num_subbands_tone_map);
				u_shared_buffers.s_ber_info.auc_ber_num_subbands_tone_map_series[uc_num_tone_map_disabled_8psk] = uc_num_subbands_tone_map;
				u_shared_buffers.s_ber_info.asl_ber_snr_good[uc_num_tone_map_disabled_8psk] = sl_snr_good;
				u_shared_buffers.s_ber_info.asl_ber_snr_be[uc_num_tone_map_disabled_8psk] = sl_snr_be;
				u_shared_buffers.s_ber_info.auc_ber_num_corr_carr [uc_num_tone_map_disabled_8psk] = uc_num_corr_carr;

				uc_num_tone_map_disabled_8psk++;
			}
		}
	} while ((uc_num_tone_map_disabled_8psk < MAX_TONE_MAP_GROUPS_DELETED_8PSK) && (uc_sel_modulation == MOD_TYPE_BPSK_ROBO) && (uc_bad_channel == 0));

	/* Tests for QPSK */
	if ((uc_sel_modulation == MOD_TYPE_BPSK_ROBO) && (uc_bad_channel == 0)) {
		memcpy(u_shared_buffers.s_ber_info.auc_ber_new_tone_map, u_shared_buffers.s_ber_info.auc_ber_tone_map, NUMBER_SUBBANDS_TONE_MAP);
		uc_num_subbands_tone_map = NUMBER_SUBBANDS_TONE_MAP;
		uc_num_corr_carr = 0;

		sl_snr_good = u_shared_buffers.s_ber_info.asl_ber_snr_good[0];
		sl_snr_be = u_shared_buffers.s_ber_info.asl_ber_snr_be[0];
		uc_num_corr_carr = u_shared_buffers.s_ber_info.auc_ber_num_corr_carr [0];

		do {
			/* Computes the SNR_EQ for the current modulation */
			sl_snr_eq = sl_snr_good - snr_loss_qpsk(uc_num_corr_carr, sl_snr_be);

			if (sl_snr_eq >= sl_snr_min_qpsk) {
				uc_sel_modulation = MOD_TYPE_QPSK;
				memcpy(u_shared_buffers.s_ber_info.auc_ber_tone_map, u_shared_buffers.s_ber_info.auc_ber_new_tone_map,
						uc_num_subbands_tone_map);
			} else {
				if (uc_num_tone_map_disabled_qpsk < (uc_num_tone_map_disabled_8psk - 1)) {
					/* The number of inactive subbands in QPSK is still lower than in 8PSK. The tone map tested in 8PSK is employed */
					uc_num_subbands_tone_map
						= u_shared_buffers.s_ber_info.auc_ber_num_subbands_tone_map_series[uc_num_tone_map_disabled_qpsk];
					memcpy(u_shared_buffers.s_ber_info.auc_ber_new_tone_map,
							u_shared_buffers.s_ber_info.auc_ber_tone_map_series[uc_num_tone_map_disabled_qpsk],
							uc_num_subbands_tone_map);
					uc_num_tone_map_disabled_qpsk++;
					sl_snr_good = u_shared_buffers.s_ber_info.asl_ber_snr_good[uc_num_tone_map_disabled_qpsk];
					sl_snr_be = u_shared_buffers.s_ber_info.asl_ber_snr_be[uc_num_tone_map_disabled_qpsk];
					uc_num_corr_carr = u_shared_buffers.s_ber_info.auc_ber_num_corr_carr [uc_num_tone_map_disabled_qpsk];
				} else if (uc_num_tone_map_disabled_qpsk == (uc_num_tone_map_disabled_8psk - 1)) {
					/* The number of inactive subbands in QPSK is equal to the last one tested in 8PSK. The tone map was saved but not the
					 * snr_good and snr_be. */
					uc_num_subbands_tone_map
						= u_shared_buffers.s_ber_info.auc_ber_num_subbands_tone_map_series[uc_num_tone_map_disabled_qpsk];
					memcpy(u_shared_buffers.s_ber_info.auc_ber_new_tone_map,
							u_shared_buffers.s_ber_info.auc_ber_tone_map_series[uc_num_tone_map_disabled_qpsk],
							uc_num_subbands_tone_map);
					uc_num_tone_map_disabled_qpsk++;

					/* Computes the SNR of the good and BE carriers and their indexes */
					_find_good_and_corrupted_carriers(psl_table_ber2snr, &sl_snr_good, &sl_snr_be, uc_mod_scheme);

					u_shared_buffers.s_ber_info.asl_ber_snr_good[uc_num_tone_map_disabled_qpsk] = sl_snr_good;
					u_shared_buffers.s_ber_info.asl_ber_snr_be[uc_num_tone_map_disabled_qpsk] = sl_snr_be;
					u_shared_buffers.s_ber_info.auc_ber_num_corr_carr [uc_num_tone_map_disabled_qpsk] = uc_num_corr_carr;
				} else {
					/* The number of inactive subbands in QPSK is high than in 8PSK. A new subband has to be deleted. */
					_delete_carriers_from_tone_map();

					/* Computes the SNR of the good and BE carriers and their indexes */
					_find_good_and_corrupted_carriers(psl_table_ber2snr, &sl_snr_good, &sl_snr_be, uc_mod_scheme);

					/* Resulting tone map (after deleting subband) is stored to be used in the next tests*/
					memcpy(u_shared_buffers.s_ber_info.auc_ber_tone_map_series[uc_num_tone_map_disabled_qpsk],
							u_shared_buffers.s_ber_info.auc_ber_new_tone_map, uc_num_subbands_tone_map);
					u_shared_buffers.s_ber_info.auc_ber_num_subbands_tone_map_series[uc_num_tone_map_disabled_qpsk]
						= uc_num_subbands_tone_map;
					u_shared_buffers.s_ber_info.asl_ber_snr_good[uc_num_tone_map_disabled_qpsk] = sl_snr_good;
					u_shared_buffers.s_ber_info.asl_ber_snr_be[uc_num_tone_map_disabled_qpsk] = sl_snr_be;
					u_shared_buffers.s_ber_info.auc_ber_num_corr_carr [uc_num_tone_map_disabled_qpsk] = uc_num_corr_carr;

					uc_num_tone_map_disabled_qpsk++;
				}
			}
		} while ((uc_num_tone_map_disabled_qpsk < MAX_TONE_MAP_GROUPS_DELETED_QPSK) && (uc_sel_modulation == MOD_TYPE_BPSK_ROBO));
	}

	/* Tests for BPSK */
	if ((uc_sel_modulation == MOD_TYPE_BPSK_ROBO) && (uc_bad_channel == 0)) {
		memcpy(u_shared_buffers.s_ber_info.auc_ber_new_tone_map, u_shared_buffers.s_ber_info.auc_ber_tone_map, NUMBER_SUBBANDS_TONE_MAP);
		uc_num_subbands_tone_map = NUMBER_SUBBANDS_TONE_MAP;
		uc_num_corr_carr = 0;

		sl_snr_good = u_shared_buffers.s_ber_info.asl_ber_snr_good[0];
		sl_snr_be = u_shared_buffers.s_ber_info.asl_ber_snr_be[0];
		uc_num_corr_carr = u_shared_buffers.s_ber_info.auc_ber_num_corr_carr [0];

		do {
			/* Computes the SNR_EQ for the current modulation */
			sl_snr_eq = sl_snr_good - snr_loss_bpsk(uc_num_corr_carr, sl_snr_be);

			if (sl_snr_eq >= sl_snr_min_bpsk) {
				uc_sel_modulation = MOD_TYPE_BPSK;
				memcpy(u_shared_buffers.s_ber_info.auc_ber_tone_map, u_shared_buffers.s_ber_info.auc_ber_new_tone_map,
						uc_num_subbands_tone_map);
			} else {
				if (uc_num_tone_map_disabled_bpsk < (uc_num_tone_map_disabled_qpsk - 1)) {
					/* The number of inactive subbands in BPSK is still lower than in QPSK. The tone map tested in 8PSK is employed */
					uc_num_subbands_tone_map
						= u_shared_buffers.s_ber_info.auc_ber_num_subbands_tone_map_series[uc_num_tone_map_disabled_bpsk];
					memcpy(u_shared_buffers.s_ber_info.auc_ber_new_tone_map,
							u_shared_buffers.s_ber_info.auc_ber_tone_map_series[uc_num_tone_map_disabled_bpsk],
							uc_num_subbands_tone_map);
					uc_num_tone_map_disabled_bpsk++;
					sl_snr_good = u_shared_buffers.s_ber_info.asl_ber_snr_good[uc_num_tone_map_disabled_bpsk];
					sl_snr_be = u_shared_buffers.s_ber_info.asl_ber_snr_be[uc_num_tone_map_disabled_bpsk];
					uc_num_corr_carr = u_shared_buffers.s_ber_info.auc_ber_num_corr_carr [uc_num_tone_map_disabled_bpsk];
				} else if (uc_num_tone_map_disabled_bpsk == (uc_num_tone_map_disabled_qpsk - 1)) {
					/* The number of inactive subbands in QPSK is equal to the last one tested in 8PSK. The tone map was saved but not the
					 * snr_good and snr_be. */
					uc_num_subbands_tone_map
						= u_shared_buffers.s_ber_info.auc_ber_num_subbands_tone_map_series[uc_num_tone_map_disabled_bpsk];
					memcpy(u_shared_buffers.s_ber_info.auc_ber_new_tone_map,
							u_shared_buffers.s_ber_info.auc_ber_tone_map_series[uc_num_tone_map_disabled_bpsk],
							uc_num_subbands_tone_map);
					uc_num_tone_map_disabled_bpsk++;

					/* Computes the SNR of the good and BE carriers and their indexes */
					_find_good_and_corrupted_carriers(psl_table_ber2snr, &sl_snr_good, &sl_snr_be, uc_mod_scheme);
				} else {
					_delete_carriers_from_tone_map();

					/* Computes the SNR of the good and BE carriers and their indexes */
					_find_good_and_corrupted_carriers(psl_table_ber2snr, &sl_snr_good, &sl_snr_be, uc_mod_scheme);

					uc_num_tone_map_disabled_bpsk++;
				}
			}
		} while ((uc_num_tone_map_disabled_bpsk < MAX_TONE_MAP_GROUPS_DELETED_BPSK) && (uc_sel_modulation == MOD_TYPE_BPSK_ROBO));
	}

	if (uc_sel_modulation == MOD_TYPE_BPSK_ROBO) {
		uc_num_subbands_tone_map = NUMBER_SUBBANDS_TONE_MAP;
	}

	/* Expresses the tone map as used in the FCH */
	memset(puc_tone_map_fch, 0, NUM_BYTES_TONE_MAP_FCH);
	for (uc_i = 0; uc_i < uc_num_subbands_tone_map; uc_i++) {
		uc_byte_index_i =  u_shared_buffers.s_ber_info.auc_ber_tone_map [uc_i] >> 3;
		uc_bit_index_in_byte_i = u_shared_buffers.s_ber_info.auc_ber_tone_map [uc_i]  - uc_byte_index_i * 8;

		*(puc_tone_map_fch  + uc_byte_index_i) = (*(puc_tone_map_fch + uc_byte_index_i)) | ((0x01) << uc_bit_index_in_byte_i);
	}

	*puc_mod_type = uc_sel_modulation;
}

/**
 * \brief Computes the LQI and the per-carrier SNR
 *
 */
uint8_t get_lqi_and_per_carrier_snr(uint8_t uc_mod_scheme, uint8_t *puc_static_and_dynamic_notching, uint8_t *puc_pilot_pos, uint8_t *puc_snr_per_carrier,
		uint16_t us_payload_symbols)
{
	uint8_t uc_i, uc_j, uc_byte_index, uc_bit_index_in_byte;

	uint8_t uc_num_active_carriers = 0;
	int32_t sl_avg_snr = 0;
	const int32_t *psl_table_ber2snr;
	uint8_t uc_lqi;
	uint16_t us_tmp;

	if (uc_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
		psl_table_ber2snr = asl_ber2snr_mix;
	} else {
		psl_table_ber2snr = asl_ber2snr_coh;
	}

	/* Reads the values of ber_k */
	pplc_if_read_buf(BCODE_ZONE0, u_shared_buffers.s_ber_info.auc_ber_k, uc_used_carriers * 2);

	/* Obtains the values of snr_k and the average snr */
	uc_j = FIRST_CARRIER;
	for (uc_i = FIRST_CARRIER; uc_i <= LAST_CARRIER; uc_i++) {
		/* Determines the carrier byte and bit */
		uc_byte_index =  uc_i >> 3;
		uc_bit_index_in_byte = uc_i & 0x07;

		if (!(puc_static_and_dynamic_notching[uc_byte_index] & (0x01 << uc_bit_index_in_byte))) {
			if ((uc_mod_scheme == MOD_SCHEME_COHERENT) && (puc_pilot_pos[uc_byte_index] & (0x01 << uc_bit_index_in_byte))) {
				/* There is no info for this carrier */
				u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_i - FIRST_CARRIER] = MIN_SNR_LQI;
				*(puc_snr_per_carrier + uc_i - FIRST_CARRIER) = 0;
			} else {
				us_tmp = (u_shared_buffers.s_ber_info.auc_ber_k[2 * (uc_j - FIRST_CARRIER)] << 8)
						+ (u_shared_buffers.s_ber_info.auc_ber_k[2 * (uc_j - FIRST_CARRIER) + 1]);
				us_tmp = ((us_tmp << 5) + (us_tmp)) / (uc_num_symbols_fch + us_payload_symbols); /* 255/31~=*8.22~=*8.25 */
				if (us_tmp & 0x0002) {
					us_tmp = (us_tmp >> 2) + 1;
				} else {
					us_tmp = us_tmp >> 2;
				}

				if (us_tmp > 255) { /* maximum value of the SNR table */
					us_tmp = 255;
				}

				u_shared_buffers.s_ber_info.asl_ber_snr_k[ uc_i - FIRST_CARRIER] = *(psl_table_ber2snr + (us_tmp));
				_correct_snr_value(&u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_i - FIRST_CARRIER], uc_mod_scheme);

				/* asl_snr_k[ uc_i - FIRST_CARRIER] = *(psl_table_ber2snr + ((auc_ber_k[2*(uc_j - FIRST_CARRIER) + 1] )>>3)); //The 2*() +1 is
				 * because even values in auc_ber_k are zeros */

				/* Per-carrier SNR in the range 00-3F */
				if (u_shared_buffers.s_ber_info.asl_ber_snr_k[ uc_i - FIRST_CARRIER] > MAX_PER_CARRIER_SNR) {
					*(puc_snr_per_carrier + uc_i - FIRST_CARRIER) = 0x3F;
				} else if (u_shared_buffers.s_ber_info.asl_ber_snr_k[ uc_i - FIRST_CARRIER] < MIN_SNR_LQI) {
					*(puc_snr_per_carrier + uc_i - FIRST_CARRIER) = 0;
				} else {
					/* -VALUE_01_Q7_25 is to implement floor */
					div_real_q((u_shared_buffers.s_ber_info.asl_ber_snr_k[ uc_i - FIRST_CARRIER ] - MIN_SNR_LQI - VALUE_01_Q7_25),
							STEP_PER_CARRIER_SNR, 0,
							(puc_snr_per_carrier + uc_i - FIRST_CARRIER));

					/* Adjust CarrierSNR value on high range */
					if (*(puc_snr_per_carrier + uc_i - FIRST_CARRIER) >= 26) { /* 16+10 */
						*(puc_snr_per_carrier + uc_i - FIRST_CARRIER) += 2; /* +2dB */
					} else {
						if (*(puc_snr_per_carrier + uc_i - FIRST_CARRIER) >= 23) { /* 13+10 */
							*(puc_snr_per_carrier + uc_i - FIRST_CARRIER) += 1; /* +1dB */
						}
					}
				}

				/* Sum the snr's of the used carriers */
				sl_avg_snr += u_shared_buffers.s_ber_info.asl_ber_snr_k[ uc_i - FIRST_CARRIER ] >> 7; /* Scaling by 1/2^7 to avoid overflow */

				uc_num_active_carriers++;
				uc_j++;
			}
		} else {
			/* There is no info for this carrier */
			u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_i - FIRST_CARRIER] = MIN_SNR_LQI;
			*(puc_snr_per_carrier + uc_i - FIRST_CARRIER) = 0;
		}
	}

	if (uc_mod_scheme == MOD_SCHEME_COHERENT) {
		for (uc_i = FIRST_CARRIER; uc_i <= LAST_CARRIER; uc_i++) {
			/* Determines the carrier byte and bit */
			uc_byte_index =  uc_i >> 3;
			uc_bit_index_in_byte = uc_i & 0x07;

			if (puc_pilot_pos[uc_byte_index] & (0x01 << uc_bit_index_in_byte)) {
				/* Pilot position, set snr value to average of near carriers */
				if (uc_i == FIRST_CARRIER) {
					u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_i -
					FIRST_CARRIER] = u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_i - FIRST_CARRIER + 1];
					*(puc_snr_per_carrier + uc_i - FIRST_CARRIER) = *(puc_snr_per_carrier + uc_i - FIRST_CARRIER + 1);
				} else if (uc_i == LAST_CARRIER) {
					u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_i -
					FIRST_CARRIER] = u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_i - FIRST_CARRIER - 1];
					*(puc_snr_per_carrier + uc_i - FIRST_CARRIER) = *(puc_snr_per_carrier + uc_i - FIRST_CARRIER - 1);
				} else {
					u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_i - FIRST_CARRIER]
						= (u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_i - FIRST_CARRIER - 1] >> 1)
							+ (u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_i - FIRST_CARRIER + 1] >> 1);
					*(puc_snr_per_carrier + uc_i - FIRST_CARRIER)
						= (*(puc_snr_per_carrier + uc_i - FIRST_CARRIER -
							1) >> 1) + (*(puc_snr_per_carrier + uc_i - FIRST_CARRIER + 1) >> 1);
				}

				/* Sum the snr's of the used carriers */
				sl_avg_snr += u_shared_buffers.s_ber_info.asl_ber_snr_k[uc_i - FIRST_CARRIER] >> 7; /* Scaling by 1/2^7 to avoid overflow */

				uc_num_active_carriers++;
			}
		}
	}

	div_real_q(sl_avg_snr, uc_num_active_carriers, 0, &sl_avg_snr);
	sl_avg_snr = sl_avg_snr << 7;

	/* LQI mapping */
	if (sl_avg_snr > MAX_SNR_LQI) {
		uc_lqi = 0xFF;
	} else if (sl_avg_snr < MIN_SNR_LQI) {
		uc_lqi = 0;
	} else {
		/* -VALUE_01_Q7_25 is to implement floor */
		div_real_q((sl_avg_snr - MIN_SNR_LQI - VALUE_01_Q7_25), STEP_SNR_LQI, 0, &uc_lqi);

		/* Adjust LQI value on high range */
		if (uc_lqi >= 104) { /* 16*4+40 */
			uc_lqi = uc_lqi + 8; /* +2dB */
		} else {
			if (uc_lqi >= 92) { /* 13*4+40 */
				uc_lqi = uc_lqi + 4; /* +1dB */
			}
		}
	}

	return uc_lqi;
}

/**
 * \brief Initializes BER for incoming frame
 *
 */
void ber_init(void)
{
	/* INT_BER=0; MODUL_PY_USED=1 (BPSK); MODUL_HD_USED=1 (BPSK); MODE_BER=0 (ONLY PAYLOAD/FCH) */
	pplc_if_write8(REG_ATPL250_BER_PERIPH_CFG1_VH8, 0x14);
	pplc_if_write32(REG_ATPL250_BER_PERIPH_CFG2_32, ((uint32_t)(uc_used_carriers - 1) << 16) | (uc_num_symbols_fch - 1));
	/* Reuse auc_ber_carrier_buff_tmp to clear BER data, this buffer is always reset to 0 before used, so can be safely used */
	memset(&u_shared_buffers.s_ber_info.auc_ber_carrier_buff_tmp, 0x00, CARR_BUFFER_LEN);
	pplc_if_write_buf(REG_ATPL250_BER_PERIPH_CFG6_32, u_shared_buffers.s_ber_info.auc_ber_carrier_buff_tmp, CARR_BUFFER_LEN);  /* Reset notching */
}

/**
 * \brief Generates the array holding the useful carriers in the proper format for BER
 *
 * \param p_rx_ctl   Pointer to Rx control structure
 *
 */
static void _ber_generate_carriers_buf_cenelec_a(struct phy_rx_ctl *p_rx_ctl)
{
	uint8_t uc_i;
	uint8_t uc_byte_idx_st_not, uc_bit_idx_st_not;
	uint8_t uc_byte_idx_dest, uc_bit_idx_dest;
	uint8_t uc_temp_prev, uc_temp_post, uc_temp;

	uc_byte_idx_dest = 0;
	uc_bit_idx_dest = 0;

	memset(u_shared_buffers.s_ber_info.auc_ber_carrier_buff_tmp, 0x00, CARR_BUFFER_LEN);

	for (uc_byte_idx_st_not = 0; uc_byte_idx_st_not < CARR_BUFFER_LEN; uc_byte_idx_st_not++) {
		for (uc_bit_idx_st_not = 0; uc_bit_idx_st_not < 8; uc_bit_idx_st_not++) {
			if ((puc_static_notching[uc_byte_idx_st_not] & (1 << uc_bit_idx_st_not)) == 0) {
				uc_temp
					= (p_rx_ctl->m_auc_inactive_carriers_pos[uc_byte_idx_st_not] |
						p_rx_ctl->m_auc_pilot_pos[uc_byte_idx_st_not]) & (1 << uc_bit_idx_st_not);
				if (uc_temp) {
					u_shared_buffers.s_ber_info.auc_ber_carrier_buff_tmp[uc_byte_idx_dest]
						= u_shared_buffers.s_ber_info.auc_ber_carrier_buff_tmp[uc_byte_idx_dest] | (1 << uc_bit_idx_dest);
				}

				uc_bit_idx_dest = (uc_bit_idx_dest + 1) % 8;
				if (uc_bit_idx_dest == 0) {
					uc_byte_idx_dest++;
				}
			}
		}
	}

	/* auc_ber_carrier_buff_tmp must be aligned to carrier 0 --> 23 bit must be removed from the beggining */
	uc_temp_prev = (u_shared_buffers.s_ber_info.auc_ber_carrier_buff_tmp[2] & 0x80) >> 7;
	for (uc_i = 0; uc_i < 5; uc_i++) { /* 36 carriers are 5 bytes */
		uc_temp = u_shared_buffers.s_ber_info.auc_ber_carrier_buff_tmp[uc_i + 3];
		uc_temp_post = (uc_temp & 0x80) >> 7;
		uc_temp <<= 1;
		uc_temp |= uc_temp_prev;
		u_shared_buffers.s_ber_info.auc_ber_carrier_buff_tmp[uc_i] = uc_temp;
		uc_temp_prev = uc_temp_post;
	}
	/* Fill with 0s */
	memset(&u_shared_buffers.s_ber_info.auc_ber_carrier_buff_tmp[5], 0x00, 11 * sizeof(uint8_t));

	/* Reorder bytes */
	for (uc_i = 0; uc_i < CARR_BUFFER_LEN; uc_i++) {
		u_shared_buffers.s_ber_info.auc_ber_carrier_buff[CARR_BUFFER_LEN - uc_i - 1] = u_shared_buffers.s_ber_info.auc_ber_carrier_buff_tmp[uc_i];
	}
}

/**
 * \brief Configures BER block for Cenelec band
 *
 * \param p_rx_ctl   Pointer to Rx control structure
 *
 */
void ber_config_cenelec_a(struct phy_rx_ctl *p_rx_ctl)
{
	_ber_generate_carriers_buf_cenelec_a(p_rx_ctl);

	if (p_rx_ctl->e_mod_scheme == MOD_SCHEME_COHERENT) {
		atpl250_set_ber_payload_coh();
	} else {
		atpl250_set_ber_payload_diff();
	}

	/* BER is configured depending on mod_scheme, static and dynamic notching and payload symbols numbe */
	switch (p_rx_ctl->e_mod_type) {
	case MOD_TYPE_BPSK_ROBO:
	case MOD_TYPE_BPSK:
		pplc_if_write8(REG_ATPL250_BER_PERIPH_CFG1_VH8, 0x16); /* INT_BER='0'; MODUL_PY_USED=1 (BPSK); MODUL_HD_USED="01" (BPSK); MODE_BER="10" (G3) */
		break;

	case MOD_TYPE_QPSK:
		pplc_if_write8(REG_ATPL250_BER_PERIPH_CFG1_VH8, 0x26); /* INT_BER='0'; MODUL_PY_USED=2 (QPSK); MODUL_HD_USED="01" (BPSK); MODE_BER="10" (G3) */
		break;

	case MOD_TYPE_8PSK:
		pplc_if_write8(REG_ATPL250_BER_PERIPH_CFG1_VH8, 0x36); /* INT_BER='0'; MODUL_PY_USED=3 (8PSK); MODUL_HD_USED="01" (BPSK); MODE_BER="10" (G3) */
		break;

	case MOD_TYPE_QAM:
		pplc_if_write8(REG_ATPL250_BER_PERIPH_CFG1_VH8, 0x06); /* INT_BER='0'; MODUL_PY_USED=0 (QAM);  MODUL_HD_USED="01" (BPSK); MODE_BER="10" (G3) */
		break;
	}

	pplc_if_write_buf(REG_ATPL250_BER_PERIPH_CFG6_32, u_shared_buffers.s_ber_info.auc_ber_carrier_buff, CARR_BUFFER_LEN);
	/* CARRIERS_PAY=ber_payload_carriers SYMBOLS_PAY=ber_payload_num_symbols */
	pplc_if_write32(REG_ATPL250_BER_PERIPH_CFG2_32, ((p_rx_ctl->m_uc_payload_carriers - 1) << 16) | (p_rx_ctl->m_us_rx_payload_symbols - 1));
	pplc_if_write32(REG_ATPL250_INTERLEAVER_CFG3_32, (((p_rx_ctl->m_us_rx_len) * 8) - 1)); /* Set first and last bit */
}

/**
 * \brief Reads FCH BER data from HW registers
 *
 * \param p_ber_fch_data   Pointer to FCH BER structure
 *
 */
void ber_save_fch_info(struct s_rx_ber_fch_data_t *p_ber_fch_data)
{
	uint8_t auc_data[24];

	if (pplc_if_read_buf(REG_ATPL250_BER_PERIPH_OUT1_32, auc_data, 24)) {
		p_ber_fch_data->uc_fch_snr_worst_carrier = auc_data[0];
		p_ber_fch_data->us_fch_corrupted_carriers = auc_data[1] << 3 | (auc_data[2] & 0xE0) >> 5;
		p_ber_fch_data->us_fch_noised_symbols = (auc_data[2] & 0x03) << 8 | auc_data[3];
		p_ber_fch_data->uc_fch_snr_worst_symbol = auc_data[4];
		p_ber_fch_data->uc_fch_snr_impulsive = auc_data[5];
		p_ber_fch_data->uc_fch_snr_be = auc_data[6];
		p_ber_fch_data->uc_fch_snr_background = auc_data[7];
		p_ber_fch_data->us_fch_acum_sym_minus8 = auc_data[8] << 8 | auc_data[9];
		p_ber_fch_data->us_fch_acum_sym_minus7 = auc_data[10] << 8 | auc_data[11];
		p_ber_fch_data->us_fch_acum_sym_minus6 = auc_data[12] << 8 | auc_data[13];
		p_ber_fch_data->us_fch_acum_sym_minus5 = auc_data[14] << 8 | auc_data[15];
		p_ber_fch_data->us_fch_acum_sym_minus4 = auc_data[16] << 8 | auc_data[17];
		p_ber_fch_data->us_fch_acum_sym_minus3 = auc_data[18] << 8 | auc_data[19];
		p_ber_fch_data->us_fch_acum_sym_minus2 = auc_data[20] << 8 | auc_data[21];
		p_ber_fch_data->us_fch_acum_sym_minus1 = auc_data[22] << 8 | auc_data[23];
	}
}

/**
 * \brief Reads Payload BER data from HW registers
 *
 * \param p_ber_payload_data   Pointer to Payload BER structure
 *
 */
void ber_save_payload_info(struct s_rx_ber_payload_data_t *p_ber_payload_data)
{
	uint8_t auc_data[8];
	if (pplc_if_read_buf(REG_ATPL250_BER_PERIPH_OUT1_32, auc_data, 8)) {
		p_ber_payload_data->uc_payload_snr_worst_carrier = auc_data[0];
		p_ber_payload_data->us_payload_corrupted_carriers = auc_data[1] << 3 | (auc_data[2] & 0xE0) >> 5;
		p_ber_payload_data->us_payload_noised_symbols = (auc_data[2] & 0x03) << 8 | auc_data[3];
		p_ber_payload_data->uc_payload_snr_worst_symbol = auc_data[4];
		p_ber_payload_data->uc_payload_snr_impulsive = auc_data[5];
		p_ber_payload_data->uc_payload_snr_be = auc_data[6];
		p_ber_payload_data->uc_payload_snr_background = auc_data[7];
	}
}
