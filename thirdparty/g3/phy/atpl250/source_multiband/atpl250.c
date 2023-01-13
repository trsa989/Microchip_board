/**
 * \file
 *
 * \brief ATPL250 Physical layer
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
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

/* System includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Phy includes */
#include "conf_fw.h"
#include "atpl250.h"
#include "atpl250_common.h"
#include "atpl250_reg.h"
#include "atpl250_version.h"
#include "atpl250_reed_solomon.h"
#include "atpl250_jump_ram.h"
#include "atpl250_carrier_mapping.h"
#include "atpl250_fch.h"
#include "atpl250_mod_demod.h"
#include "atpl250_ber.h"
#include "atpl250_channel_estimation.h"
#include "atpl250_txrx_chain.h"
#include "atpl250_hw_init.h"
#include "atpl250_sampling_error_estimation.h"
#include "atpl250_channel_and_sfo_estimation_params.h"
#include "conf_project.h"
#include "conf_usi.h"

/* ASF includes */
#include "asf.h"

/* ARM DSP LIB CMSIS include */
#include "arm_math.h"

/* Include for abs, malloc, calloc and free */
#include <stdlib.h>

static bool sb_cd_int_enabled = false;
static bool sb_ber_int_received = false;

/* Management Function PHY interrupt */
void phy_interrupt(void);

/* extern reference to platform function */
extern void platform_led_int_toggle(void);

/* Default empty services */
void Dummy_serial_if_init(void);
uint8_t Dummy_phy_extension_handler_start_tx(void);
uint8_t Dummy_phy_extension_handler_iob_tx(void);
void Dummy_sniffer_if_init(void);

#ifdef __GNUC__
void serial_if_init( void ) __attribute__ ((weak, alias("Dummy_serial_if_init")));
uint8_t phy_extension_handler_start_tx( void ) __attribute__ ((weak, alias("Dummy_phy_extension_handler_start_tx")));
uint8_t phy_extension_handler_iob_tx( void ) __attribute__ ((weak, alias("Dummy_phy_extension_handler_iob_tx")));
void sniffer_if_init( void ) __attribute__ ((weak, alias("Dummy_sniffer_if_init")));

#endif

#if defined(__ICCARM__) || defined(__CC_ARM)
extern void serial_if_init(void);
extern uint8_t phy_extension_handler_start_tx(void);
extern uint8_t phy_extension_handler_iob_tx(void);
extern void sniffer_if_init(void);

#pragma weak serial_if_init=Dummy_serial_if_init
#pragma weak phy_extension_handler_start_tx=Dummy_phy_extension_handler_start_tx
#pragma weak phy_extension_handler_iob_tx=Dummy_phy_extension_handler_iob_tx
#pragma weak sniffer_if_init=Dummy_sniffer_if_init
#endif

void get_phy_tx_ctl(struct phy_tx_ctl *ps_phy_tx_ctl_dst);
void get_sym_cfg(struct sym_cfg *ps_sym_cfg);

/* Default Callbacks */
struct TPhyCallbacks g_phy_callbacks = {0};
struct TPhySnifferCallbacks g_phy_sniffer_callbacks = {0};

/* Different Interrupt handlers */
struct phy_interrupt_handler {
	void (*handler)(void);
};

void handle_iob_int(void);
void handle_peak1_int(void);
void handle_peak2_int(void);
void handle_no_peak2_int(void);
void handle_peak13_int(void);
void handle_zero_cross_int(void);
void handle_viterbi_int(void);
void handle_reed_solomon_int(void);
void handle_start_tx_1_int(void);
void handle_start_tx_2_int(void);
void handle_start_tx_3_int(void);
void handle_start_tx_4_int(void);
void handle_end_tx_1_int(void);
void handle_end_tx_2_int(void);
void handle_end_tx_3_int(void);
void handle_end_tx_4_int(void);
void handle_cd_tx_1_int(void);
void handle_cd_tx_2_int(void);
void handle_cd_tx_3_int(void);
void handle_cd_tx_4_int(void);
void handle_overlap_tx_1_int(void);
void handle_overlap_tx_2_int(void);
void handle_overlap_tx_3_int(void);
void handle_overlap_tx_4_int(void);
void handle_rx_error_int(void);
void handle_tx_error_int(void);
void handle_noise_error_int(void);
void handle_noise_capture_start_int(void);
void handle_ber_int(void);
void handle_cd_int(void);
void handle_spi_error_int(void);
void handle_start_int(void);

/* Interrupt handlers initialization */
const struct phy_interrupt_handler phy_handler[PHY_NUM_INTERRUPT_SOURCES] = {
	{handle_iob_int},
	{handle_peak1_int},
	{handle_peak2_int},
	{handle_no_peak2_int},
	{handle_peak13_int},
	{handle_zero_cross_int},
	{handle_viterbi_int},
	{handle_reed_solomon_int},
	{handle_start_tx_1_int},
	{handle_start_tx_2_int},
	{handle_start_tx_3_int},
	{handle_start_tx_4_int},
	{handle_end_tx_1_int},
	{handle_end_tx_2_int},
	{handle_end_tx_3_int},
	{handle_end_tx_4_int},
	{handle_cd_tx_1_int},
	{handle_cd_tx_2_int},
	{handle_cd_tx_3_int},
	{handle_cd_tx_4_int},
	{handle_overlap_tx_1_int},
	{handle_overlap_tx_2_int},
	{handle_overlap_tx_3_int},
	{handle_overlap_tx_4_int},
	{handle_rx_error_int},
	{handle_tx_error_int},
	{handle_noise_error_int},
	{handle_noise_capture_start_int},
	{handle_ber_int},
	{handle_cd_int},
	{handle_spi_error_int},
	{handle_start_int}
};

#define FRAME_SYMBOL_DURATION_CENELEC_A                          695
#define PREAMBLE_TIME_DURATION_CENELEC_A                         5760   /* 640*9 us */
#define HALF_SYMBOL_DURATION_CENELEC_A                           320   /* 640/2 us */

#define FRAME_SYMBOL_DURATION_FCC_ARIB                           232
#define PREAMBLE_TIME_DURATION_FCC_ARIB                          1917   /* 213*9 us */
#define HALF_SYMBOL_DURATION_FCC_ARIB                            106   /* 213/2 us */

/* Time needed before start transmitting */
#define DELAYED_TX_SAFETY_TIME_CENELEC_A   (CFG_TXRX_PLC_US_CENELEC_A + RRC_DELAY_US_CENELEC_A + 200)
#define DELAYED_TX_SAFETY_TIME_FCC_ARIB    (CFG_TXRX_PLC_US_FCC_ARIB + RRC_DELAY_US_FCC_ARIB + 50)

/* Tx Gain Values*/
static uint8_t uc_phy_tx_full_gain = 0xFF;
const uint8_t auc_txgain[16] = {0xFF, 0xB4, 0x80, 0x5A, 0x40, 0x2D, 0x20, 0x17, 0x10, 0x0B, 0x08, 0x06, 0x04, 0x03, 0x02, 0x01};
const uint16_t aus_preemphasis_att_values[32] = {0x7FFF, 0x5A82, 0x4000, 0x2D41, 0x2000, 0x16A0, 0x1000, 0x0B50,
						 0x0800, 0x05A8, 0x0400, 0x02D4, 0x0200, 0x016A, 0x0100, 0x00B5,
						 0x0080, 0x005B, 0x0040, 0x002D, 0x0020, 0x0017, 0x0010, 0x000B,
						 0x0008, 0x0006, 0x0004, 0x0003, 0x0002, 0x0001, 0x0001, 0x0001};

const uint16_t aus_ripple_correction[16][NUM_CARRIERS_CENELEC_A]
	=       /*VLO AVGEq_1_1_1.5*/
	{{0, 590, 793, 1225, 2256, 2957, 3925, 5181, 6098, 6631, 7291, 7663, 7720, 7611, 7574, 7255, 6871, 6707,
	  6153, 5828, 5356, 5078, 4796, 4569, 4751, 4971, 5194, 5430, 5536, 5274, 5235, 5533, 4883, 4773, 4194, 4140},
	 {0, 418, 561, 866, 1595, 2091, 2775, 3664, 4312, 4689, 5155, 5419, 5459, 5382, 5356, 5130, 4859, 4742,
	  4351, 4121, 3787, 3591, 3392, 3231, 3360, 3515, 3673, 3840, 3914, 3729, 3702, 3913, 3453, 3375, 2966, 2928},
	 {0, 295, 396, 613, 1128, 1479, 1962, 2591, 3049, 3316, 3645, 3832, 3860, 3805, 3787, 3627, 3436, 3354,
	  3076, 2914, 2678, 2539, 2398, 2285, 2376, 2486, 2597, 2715, 2768, 2637, 2618, 2767, 2442, 2386, 2097, 2070},
	 {0, 209, 280, 433, 798, 1045, 1388, 1832, 2156, 2344, 2578, 2709, 2730, 2691, 2678, 2565, 2429, 2371,
	  2175, 2060, 1894, 1796, 1696, 1615, 1680, 1758, 1836, 1920, 1957, 1865, 1851, 1956, 1727, 1687, 1483, 1464},
	 {0, 148, 198, 306, 564, 739, 981, 1295, 1525, 1658, 1823, 1916, 1930, 1903, 1894, 1814, 1718, 1677,
	  1538, 1457, 1339, 1270, 1199, 1142, 1188, 1243, 1298, 1358, 1384, 1318, 1309, 1383, 1221, 1193, 1049, 1035},
	 {0, 104, 140, 217, 399, 523, 694, 916, 1078, 1172, 1289, 1355, 1365, 1345, 1339, 1282, 1215, 1186,
	  1088, 1030, 947, 898, 848, 808, 840, 879, 918, 960, 979, 932, 925, 978, 863, 844, 741, 732},
	 {0, 74, 99, 153, 282, 370, 491, 648, 762, 829, 911, 958, 965, 951, 947, 907, 859, 838, 769, 729,
	  670, 635, 600, 571, 594, 621, 649, 679, 692, 659, 654, 692, 610, 597, 524, 518},
	 {0, 52, 70, 108, 199, 261, 347, 458, 539, 586, 644, 677, 682, 673, 669, 641, 607, 593, 544, 515,
	  473, 449, 424, 404, 420, 439, 459, 480, 489, 466, 463, 489, 432, 422, 371, 366},
	 {0, 37, 50, 77, 141, 185, 245, 324, 381, 414, 456, 479, 483, 476, 473, 453, 429, 419, 385, 364,
	  335, 317, 300, 286, 297, 311, 325, 339, 346, 330, 327, 346, 305, 298, 262, 259},
	 {0, 26, 35, 54, 100, 131, 173, 229, 269, 293, 322, 339, 341, 336, 335, 321, 304, 296, 272, 258,
	  237, 224, 212, 202, 210, 220, 230, 240, 245, 233, 231, 245, 216, 211, 185, 183},
	 {0, 18, 25, 38, 70, 92, 123, 162, 191, 207, 228, 239, 241, 238, 237, 227, 215, 210, 192, 182,
	  167, 159, 150, 143, 148, 155, 162, 170, 173, 165, 164, 173, 153, 149, 131, 129},
	 {0, 13, 18, 27, 50, 65, 87, 114, 135, 147, 161, 169, 171, 168, 167, 160, 152, 148, 136, 129,
	  118, 112, 106, 101, 105, 110, 115, 120, 122, 117, 116, 122, 108, 105, 93, 91},
	 {0, 9, 12, 19, 35, 46, 61, 81, 95, 104, 114, 120, 121, 119, 118, 113, 107, 105, 96, 91, 84,
	  79, 75, 71, 74, 78, 81, 85, 86, 82, 82, 86, 76, 75, 66, 65},
	 {0, 7, 9, 14, 25, 33, 43, 57, 67, 73, 81, 85, 85, 84, 84, 80, 76, 74, 68, 64, 59, 56, 53,
	  50, 52, 55, 57, 60, 61, 58, 58, 61, 54, 53, 46, 46},
	 {0, 5, 6, 10, 18, 23, 31, 40, 48, 52, 57, 60, 60, 59, 59, 57, 54, 52, 48, 46, 42, 40, 37,
	  36, 37, 39, 41, 42, 43, 41, 41, 43, 38, 37, 33, 32},
	 {0, 3, 4, 7, 12, 16, 22, 29, 34, 37, 40, 42, 43, 42, 42, 40, 38, 37, 34, 32, 30, 28, 26,
	  25, 26, 27, 29, 30, 31, 29, 29, 31, 27, 26, 23, 23}};

const uint16_t aus_ripple_correction_LO[16][NUM_CARRIERS_CENELEC_A]
	=       /*NOT APPLY*/
	{{0, 590, 793, 1225, 2256, 2957, 3925, 5181, 6098, 6631, 7291, 7663, 7720, 7611, 7574, 7255, 6871, 6707,
	  6153, 5828, 5356, 5078, 4796, 4569, 4751, 4971, 5194, 5430, 5536, 5274, 5235, 5533, 4883, 4773, 4194, 4140},
	 {0, 418, 561, 866, 1595, 2091, 2775, 3664, 4312, 4689, 5155, 5419, 5459, 5382, 5356, 5130, 4859, 4742,
	  4351, 4121, 3787, 3591, 3392, 3231, 3360, 3515, 3673, 3840, 3914, 3729, 3702, 3913, 3453, 3375, 2966, 2928},
	 {0, 295, 396, 613, 1128, 1479, 1962, 2591, 3049, 3316, 3645, 3832, 3860, 3805, 3787, 3627, 3436, 3354,
	  3076, 2914, 2678, 2539, 2398, 2285, 2376, 2486, 2597, 2715, 2768, 2637, 2618, 2767, 2442, 2386, 2097, 2070},
	 {0, 209, 280, 433, 798, 1045, 1388, 1832, 2156, 2344, 2578, 2709, 2730, 2691, 2678, 2565, 2429, 2371,
	  2175, 2060, 1894, 1796, 1696, 1615, 1680, 1758, 1836, 1920, 1957, 1865, 1851, 1956, 1727, 1687, 1483, 1464},
	 {0, 148, 198, 306, 564, 739, 981, 1295, 1525, 1658, 1823, 1916, 1930, 1903, 1894, 1814, 1718, 1677,
	  1538, 1457, 1339, 1270, 1199, 1142, 1188, 1243, 1298, 1358, 1384, 1318, 1309, 1383, 1221, 1193, 1049, 1035},
	 {0, 104, 140, 217, 399, 523, 694, 916, 1078, 1172, 1289, 1355, 1365, 1345, 1339, 1282, 1215, 1186,
	  1088, 1030, 947, 898, 848, 808, 840, 879, 918, 960, 979, 932, 925, 978, 863, 844, 741, 732},
	 {0, 74, 99, 153, 282, 370, 491, 648, 762, 829, 911, 958, 965, 951, 947, 907, 859, 838, 769, 729,
	  670, 635, 600, 571, 594, 621, 649, 679, 692, 659, 654, 692, 610, 597, 524, 518},
	 {0, 52, 70, 108, 199, 261, 347, 458, 539, 586, 644, 677, 682, 673, 669, 641, 607, 593, 544, 515,
	  473, 449, 424, 404, 420, 439, 459, 480, 489, 466, 463, 489, 432, 422, 371, 366},
	 {0, 37, 50, 77, 141, 185, 245, 324, 381, 414, 456, 479, 483, 476, 473, 453, 429, 419, 385, 364,
	  335, 317, 300, 286, 297, 311, 325, 339, 346, 330, 327, 346, 305, 298, 262, 259},
	 {0, 26, 35, 54, 100, 131, 173, 229, 269, 293, 322, 339, 341, 336, 335, 321, 304, 296, 272, 258,
	  237, 224, 212, 202, 210, 220, 230, 240, 245, 233, 231, 245, 216, 211, 185, 183},
	 {0, 18, 25, 38, 70, 92, 123, 162, 191, 207, 228, 239, 241, 238, 237, 227, 215, 210, 192, 182,
	  167, 159, 150, 143, 148, 155, 162, 170, 173, 165, 164, 173, 153, 149, 131, 129},
	 {0, 13, 18, 27, 50, 65, 87, 114, 135, 147, 161, 169, 171, 168, 167, 160, 152, 148, 136, 129,
	  118, 112, 106, 101, 105, 110, 115, 120, 122, 117, 116, 122, 108, 105, 93, 91},
	 {0, 9, 12, 19, 35, 46, 61, 81, 95, 104, 114, 120, 121, 119, 118, 113, 107, 105, 96, 91, 84,
	  79, 75, 71, 74, 78, 81, 85, 86, 82, 82, 86, 76, 75, 66, 65},
	 {0, 7, 9, 14, 25, 33, 43, 57, 67, 73, 81, 85, 85, 84, 84, 80, 76, 74, 68, 64, 59, 56, 53,
	  50, 52, 55, 57, 60, 61, 58, 58, 61, 54, 53, 46, 46},
	 {0, 5, 6, 10, 18, 23, 31, 40, 48, 52, 57, 60, 60, 59, 59, 57, 54, 52, 48, 46, 42, 40, 37,
	  36, 37, 39, 41, 42, 43, 41, 41, 43, 38, 37, 33, 32},
	 {0, 3, 4, 7, 12, 16, 22, 29, 34, 37, 40, 42, 43, 42, 42, 40, 38, 37, 34, 32, 30, 28, 26,
	  25, 26, 27, 29, 30, 31, 29, 29, 31, 27, 26, 23, 23}};

const uint16_t aus_ripple_correction_HI[16][NUM_CARRIERS_CENELEC_A]
	=       /*HI_rev01 0x1E Eq_1_1_1*/
	{{6389, 6640, 6293, 5524, 4288, 3007, 2834, 2157, 2514, 2767, 1735, 1013, 981, 1265, 1293, 1972, 1894,
	  2310, 2345, 2400, 2210, 3903, 4422, 4327, 4321, 4189, 6045, 6198, 5894, 5978, 4956, 3756, 2764, 2804, 1190, 0},
	 {4518, 4695, 4450, 3906, 3032, 2127, 2004, 1525, 1778, 1956, 1227, 716, 694, 894, 914, 1395, 1340,
	  1633, 1658, 1697, 1563, 2760, 3127, 3060, 3055, 2962, 4274, 4382, 4168, 4227, 3504, 2656, 1954, 1982, 841, 0},
	 {3195, 3320, 3147, 2762, 2144, 1504, 1417, 1078, 1257, 1383, 868, 506, 491, 632, 647, 986, 947,
	  1155, 1172, 1200, 1105, 1952, 2211, 2164, 2161, 2094, 3023, 3099, 2947, 2989, 2478, 1878, 1382, 1402, 595, 0},
	 {2259, 2348, 2225, 1953, 1516, 1063, 1002, 763, 889, 978, 614, 358, 347, 447, 457, 697, 670,
	  817, 829, 848, 781, 1380, 1564, 1530, 1528, 1481, 2137, 2191, 2084, 2113, 1752, 1328, 977, 991, 421, 0},
	 {1597, 1660, 1573, 1381, 1072, 752, 709, 539, 629, 692, 434, 253, 245, 316, 323, 493, 474,
	  577, 586, 600, 553, 976, 1106, 1082, 1080, 1047, 1511, 1549, 1473, 1494, 1239, 939, 691, 701, 297, 0},
	 {1129, 1174, 1112, 976, 758, 532, 501, 381, 444, 489, 307, 179, 173, 224, 229, 349, 335,
	  408, 414, 424, 391, 690, 782, 765, 764, 740, 1069, 1096, 1042, 1057, 876, 664, 488, 496, 210, 0},
	 {799, 830, 787, 691, 536, 376, 354, 270, 314, 346, 217, 127, 123, 158, 162, 247, 237,
	  289, 293, 300, 276, 488, 553, 541, 540, 524, 756, 775, 737, 747, 619, 470, 345, 350, 149, 0},
	 {565, 587, 556, 488, 379, 266, 250, 191, 222, 245, 153, 90, 87, 112, 114, 174, 167,
	  204, 207, 212, 195, 345, 391, 382, 382, 370, 534, 548, 521, 528, 438, 332, 244, 248, 105, 0},
	 {399, 415, 393, 345, 268, 188, 177, 135, 157, 173, 108, 63, 61, 79, 81, 123, 118,
	  144, 147, 150, 138, 244, 276, 270, 270, 262, 378, 387, 368, 374, 310, 235, 173, 175, 74, 0},
	 {282, 293, 278, 244, 189, 133, 125, 95, 111, 122, 77, 45, 43, 56, 57, 87, 84,
	  102, 104, 106, 98, 172, 195, 191, 191, 185, 267, 274, 260, 264, 219, 166, 122, 124, 53, 0},
	 {200, 208, 197, 173, 134, 94, 89, 67, 79, 86, 54, 32, 31, 40, 40, 62, 59, 72,
	  73, 75, 69, 122, 138, 135, 135, 131, 189, 194, 184, 187, 155, 117, 86, 88, 37, 0},
	 {141, 147, 139, 122, 95, 66, 63, 48, 56, 61, 38, 22, 22, 28, 29, 44, 42, 51,
	  52, 53, 49, 86, 98, 96, 95, 93, 134, 137, 130, 132, 109, 83, 61, 62, 26, 0},
	 {100, 104, 98, 86, 67, 47, 44, 34, 39, 43, 27, 16, 15, 20, 20, 31, 30, 36,
	  37, 37, 35, 61, 69, 68, 68, 65, 94, 97, 92, 93, 77, 59, 43, 44, 19, 0},
	 {71, 73, 70, 61, 47, 33, 31, 24, 28, 31, 19, 11, 11, 14, 14, 22, 21, 26,
	  26, 27, 24, 43, 49, 48, 48, 46, 67, 68, 65, 66, 55, 41, 31, 31, 13, 0},
	 {50, 52, 49, 43, 34, 23, 22, 17, 20, 22, 14, 8, 8, 10, 10, 15, 15, 18,
	  18, 19, 17, 30, 35, 34, 34, 33, 47, 48, 46, 47, 39, 29, 22, 22, 9, 0},
	 {35, 37, 35, 31, 24, 17, 16, 12, 14, 15, 10, 6, 5, 7, 7, 11, 10, 13,
	  13, 13, 12, 22, 24, 24, 24, 23, 33, 34, 33, 33, 27, 21, 15, 15, 7, 0}};

/* P symbol */
const uint8_t auc_full_psymbol_cenelec_a[P_SYMBOL_LEN_MAX_CENELEC_A] = {0x21, 0x0F, 0xEC, 0xA7, 0x3F, 0xB6,
									0x1B, 0x5E, 0x7F, 0x7F, 0x6D, 0x28,
									0xD2, 0x6A, 0xD0, 0x23, 0x56, 0x77};

const uint8_t auc_full_psymbol_fcc[P_SYMBOL_LEN_MAX_FCC] = {0x21, 0x10, 0x0F, 0xEC, 0xB9, 0x74,
							    0x1F, 0xC9, 0x51, 0xEA, 0x50, 0xC6,
							    0x1C, 0x60, 0xA3, 0xD6, 0xF7, 0x08,
							    0x08, 0xF6, 0xE4, 0xB2, 0x8E, 0x39,
							    0xF3, 0x8D, 0x15, 0x9D, 0x14, 0x7A,
							    0xDF, 0x13, 0x45, 0x77, 0x89, 0xAA};

const uint8_t auc_full_psymbol_arib[P_SYMBOL_LEN_MAX_ARIB] = {0x21, 0x10, 0xFE, 0xDB, 0x96, 0x30,
							      0xC9, 0x40, 0xC6, 0x1C, 0x6F, 0x92,
							      0xB4, 0xC4, 0xC3, 0xA1, 0x7E, 0x39,
							      0xF3, 0x8D, 0x14, 0x8B, 0xE1, 0x34,
							      0x67, 0x89, 0xAA};

const uint8_t *puc_full_psymbol;

/* BER values to dB conversion */
#define MIN_DISTANCE_SNR_IMPULSIVE_TO_BACKGROUND  (int8_t)6
#define MAX_SNR_IMPULSIVE_TO_CONSIDER_ROBO        (int8_t)6

#define IMPULSIVE_MULT_FACTOR_MIN_SYMBOLS      (uint32_t)695
#define IMPULSIVE_MULT_FACTOR_MAX_SYMBOLS_DIF  (uint32_t)5560
#define IMPULSIVE_MULT_FACTOR_MAX_SYMBOLS_COH  (uint32_t)2780

#define IMPULSIVE_DIV_FACTOR  (uint32_t)20000

const int8_t casc_ber_to_db[256] = {34, 30, 28, 26, 25, 24, 23, 22, 21, 20, 20, 19, 18, 18, 17, 17, 17, 16,
				    16, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 12, 12, 12, 11, 11,
				    11, 11, 11, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 8, 8,
				    8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6,
				    6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5,
				    5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3,
				    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2,
				    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1,
				    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
				    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2,
				    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3,
				    -3, -3, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4,
				    -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -6, -6, -6, -6, -6, -6,
				    -6, -6, -7, -7, -7, -7, -7, -7, -8, -8, -8, -8, -8, -8, -9, -9,
				    -9, -10, -10, -10, -10, -11, -11, -12, -12, -13, -13, -14, -15, -15};
uint8_t auc_psymbol[P_SYMBOL_LEN_MAX_FCC];

/* Buffer to construct FCH */
static uint8_t auc_fch[FCH_LEN_FCC]; /* Reserve max space (FCC) */

/* PHY Parameters structure instance */
static atpl250_t atpl250;

/* BER data structure instances */
static struct s_rx_ber_fch_data_t s_rx_ber_fch_data;
static struct s_rx_ber_payload_data_t s_rx_ber_payload_data;

/* TM Response data structure instance */
static struct s_tone_map_response_data_t s_tone_map_response_data;

/* Number of FCH symbols used for channel/SFO estimation already read*/
uint8_t uc_num_fch_sym_read = 0;

/* Number of FCH symbols for updating H in FCC legacy mode */
extern int16_t ss_num_sym_fch_for_H_update;

/* Number of FCH symbols since last update of the channel estimate */
extern int16_t ss_num_fch_since_last_H_update;

/* Indicates that demodulation of the first symbol of the block should be done using a phase shift (only when differential modulation is used) */
uint8_t uc_apply_phase_shift = false;

/* #define CONF_FCC_ABOVE_300_KHZ */

#ifdef CONF_FCC_ABOVE_300_KHZ
/* Static Notching Array */
uint8_t auc_static_notching_pos[CARR_BUFFER_LEN] = {0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#else
/* Default Static Notching Array */
uint8_t auc_static_notching_pos[CARR_BUFFER_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
/* SFSK notching Carriers [39-49] */
/*uint8_t auc_static_notching_pos[CARR_BUFFER_LEN] = {0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};*/
/* Notching for FCC-C-05 test carriers [50-60] */
/*uint8_t auc_static_notching_pos[CARR_BUFFER_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};*/
#endif

/*Legacy Mode Indicator*/
uint8_t uc_legacy_mode = 0x01;

/* Hard reset Flag */
uint8_t uc_hard_reset_flag = 0x01;

/* Noise capture constants and variables */
#define NOISE_CAPTURE_ADAPT_SYMBOLS_CENELEC_A   2
#define NOISE_CAPTURE_ADAPT_SYMBOLS_FCC_ARIB    6
#define NOISE_CAPTURE_SYMBOLS                   8
#define NOISE_CAPTURE_PERIOD                    60000 /* ms */
#define NOISE_CAPTURE_DELTA                     100 /* us */
#define NOISE_CAPTURE_DELTA_CORRECTED           271 /* us */
#define NOISE_CAPTURE_FFT_SHIFT                 3
#define RSSI_VALUES_TABLE_SIZE                  80
#define NOISE_CORRECTION_FACTOR_DB_CENELEC_A    49
#define NOISE_CORRECTION_FACTOR_DB_FCC_ARIB     51
#define NOISE_MIN_VALUE_DB                      30
#define MIN_NOISE_DERIVATIVE_2_DB               6
#define MIN_NOISE_DERIVATIVE_3_DB               6
#define MIN_DIFF_PREV_CARRIER_DB                4
#define MIN_NOISE_AVG_DIF_DB                    12
#define MIN_NOISE_NOTCH_DB_CENELEC_A            (73 - NOISE_CORRECTION_FACTOR_DB_CENELEC_A)
#define MIN_NOISE_NOTCH_DB_FCC_ARIB             (73 - NOISE_CORRECTION_FACTOR_DB_FCC_ARIB)
#define NUM_INTERMEDIATE_PASSES                 4
static uint8_t uc_curr_intermediate_pass;
#ifndef NOISE_CAPTURE_USING_FALSE_PEAK
static uint8_t uc_flag_correct_delta;
#endif
/* Table containing values of RSSI / AGC FACTOR to obtain db values depending on index */
const uint32_t caul_rssi_values[RSSI_VALUES_TABLE_SIZE] = {1, 1, 1, 2, 2, 3, 4, 5, 7, 8, 11, 14, 17, 22, 28,
							   35, 44, 56, 70, 89, 112, 141, 177, 223, 281, 354, 446, 562, 707, 891, 1122,
							   1412, 1778, 2238, 2818, 3548, 4466, 5623, 7079, 8912, 11220, 14125, 17782,
							   22387, 28183, 35481, 44668, 56234, 70794, 89125, 112201, 141253, 177827,
							   223872, 281838, 354813, 446683, 562341, 707945, 891250, 1122018, 1412537,
							   1778279, 2238721, 2818382, 3548133, 4466835, 5623413, 7079457, 8912509,
							   11220184, 14125375, 17782794, 22387211, 28183829, 35481338, 44668359,
							   56234132, 70794578, 89125093};
/* Array to store rssi values */
uint8_t auc_rssi_per_carrier[NUM_CARRIERS_FCC + EXTRA_CARRIERS_TO_READ_NOISE];

enum ber_status {
	BER_FCH = 0,
	BER_PAYLOAD = 1
};

/* MAX iterations at Interleaver check to consider Tx Error */
#define MAX_INTL_ITERATIONS  500

/* Internal PHY variables */
static struct phy_rx_ctl s_phy_rx_ctl;
static struct phy_tx_ctl s_phy_tx_ctl;
static volatile uint32_t ul_phy_int_flags;
static struct sym_cfg s_sym_cfg;
static uint16_t us_rs_decode_idx;

static uint32_t ul_rx_sync_time;
static uint32_t ul_rx_end_time;
#ifdef ENABLE_SNIFFER
static uint32_t ul_last_tx_time;
#endif
static uint32_t ul_tx_end_time;

static volatile uint8_t uc_set_sym_ready;
static volatile uint8_t uc_clear_rx_full;

#define PHY_RX_TIMEOUT_RESET
#define PHY_TX_TIMEOUT_RESET

#define PHY_RX_TIMEOUT_MS   300000
#define PHY_TX_TIMEOUT_MS   300000

static uint16_t us_noise_agc_factor;
static uint32_t ul_noise_capture_timer;
static uint32_t ul_rx_timeout_timer;
static uint32_t ul_tx_timeout_timer;
static uint32_t ul_ms_counter;
static volatile uint8_t uc_start_interrupt_executed;

#ifdef ENABLE_PYH_PROCESS_RECALL
static uint8_t uc_force_phy_process_recall;
#endif

static uint8_t auc_prev_preemphasis[NUM_SUBBANDS_FCC];

/* PDC ratio for PDC calculation */
static uint8_t uc_pdc_ratio;

enum ber_status e_ber_status;

#ifdef ENABLE_SNIFFER
static xPhyMsgTxResult_t tx_result_sniffer;
static xPhyMsgTx_t tx_param_sniffer;
static xPhyMsgRx_t rx_msg_sniffer;
#endif

#ifdef ENABLE_SIGNAL_DUMP
#define DUMP_BUFFERS 20 /* Each buffer captures 0,853ms */
static uint8_t auc_dump_signal[DUMP_BUFFERS][2048];
static uint8_t uc_dumped_buffers;
static volatile uint32_t ul_loop;
#endif
#define IMMEDIATE_DUMP      1
#define DUMP_ON_RECEPTION   2

/* Phy generic flags, to be treated in generic task */
static volatile uint8_t uc_phy_generic_flags;
/* Flag values for uc_phy_generic_flags */
#define PHY_GENERIC_FLAG_END_TX               0x01u
#define PHY_GENERIC_FLAG_END_RX               0x02u
#define PHY_GENERIC_FLAG_CHECK_RS             0x04u
#define PHY_GENERIC_FLAG_NOISE_CAPTURE        0x08u
#define PHY_GENERIC_FLAG_INIT_PHY_PARAMS      0x40u
#define PHY_GENERIC_FLAG_RESET_PHY            0x80u
#ifdef ENABLE_SNIFFER
#define PHY_GENERIC_FLAG_SNIFFER_INDICATION   0x10u
#define PHY_GENERIC_FLAG_SNIFFER_CONFIRM      0x20u
#endif

/* PSK points to define reference constellation */
const uint8_t cauc_abcd_fullgain[ABCD_POINTS_LEN] = {0x55, 0x08, 0x4E, 0x8F, 0x3C, 0x20, 0x20, 0x8A};
const uint8_t cauc_abcd_minus3db[ABCD_POINTS_LEN] = {0x3C, 0x20, 0x37, 0x8C, 0x2A, 0x84, 0x17, 0x02};

/* Extern variables */
extern uint8_t auc_smooth_control_preamble_fch[];
extern uint8_t uc_num_carr_smooth_preamble_fch;
extern uint8_t uc_num_sym_valid_fch;
extern uint8_t uc_num_sym_block_demod_fch;
extern union shared_preamble_and_payload_symbols u_shared_preamble_and_payload_symbols;
extern q15_t ass_average_symbol[];
extern q15_t ass_modulating_symbols[], ass_symbol_aux[];
extern uint8_t uc_num_sym_valid_preamble;
extern uint8_t auc_ones[], auc_zeros[];
extern uint8_t auc_control_avg_invert_chan[];
extern uint8_t sc_sfo_time_offset;

/* Auxiliary table to get set bits in a byte */
#define B2(n) n, n + 1, n + 1, n + 2
#define B4(n) B2(n), B2(n + 1), B2(n + 1), B2(n + 2)
#define B6(n) B4(n), B4(n + 1), B4(n + 1), B4(n + 2)

const uint8_t BitsSetTable256[256] = {
	B6(0), B6(1), B6(1), B6(2)
};

/* Auto-Detection of impedance variables */
#define NUM_TX_LEVELS 8
static uint8_t uc_last_tx_power;

/* Emit Gain Adaptation parameters */
const struct emit_gain_limits_type emit_gain_limits_cenelec_a[NUM_IMPEDANCE_STATES] = {
	{EMIT_GAIN_HI_CENELEC_A, 0x70, 0x1A},
	{EMIT_GAIN_LO_CENELEC_A, 0, 0},
	{EMIT_GAIN_VLO_CENELEC_A, 0x86, 0x3A}
};

const struct emit_gain_limits_type emit_gain_limits_fcc[NUM_IMPEDANCE_STATES] = {
	{EMIT_GAIN_HI_FCC, EMIT_GAIN_HI_FCC, EMIT_GAIN_HI_FCC},
	{EMIT_GAIN_LO_FCC, EMIT_GAIN_LO_FCC, EMIT_GAIN_LO_FCC},
	{EMIT_GAIN_VLO_FCC, EMIT_GAIN_VLO_FCC, EMIT_GAIN_VLO_FCC}
};

const struct emit_gain_limits_type emit_gain_limits_arib[NUM_IMPEDANCE_STATES] = {
	{EMIT_GAIN_HI_ARIB, EMIT_GAIN_HI_ARIB, EMIT_GAIN_HI_ARIB},
	{EMIT_GAIN_LO_ARIB, EMIT_GAIN_LO_ARIB, EMIT_GAIN_LO_ARIB},
	{EMIT_GAIN_VLO_ARIB, EMIT_GAIN_VLO_ARIB, EMIT_GAIN_VLO_ARIB}
};

const struct emit_gain_limits_type *p_emit_gain_limits;

const uint32_t caul_max_rms_hi_cenelec_a[NUM_TX_LEVELS] = {79000118, 55861519, 39500059, 27930760, 19750030, 13965380, 9875015, 6982690};
const uint32_t caul_max_rms_vlo_cenelec_a[NUM_TX_LEVELS] = {254295002, 179813720, 127147501, 89906860, 63573750, 44953430, 31786875, 22476715};

const uint32_t caul_max_rms_hi_fcc[NUM_TX_LEVELS] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
const uint32_t caul_max_rms_vlo_fcc[NUM_TX_LEVELS] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

const uint32_t caul_max_rms_hi_arib[NUM_TX_LEVELS] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
const uint32_t caul_max_rms_vlo_arib[NUM_TX_LEVELS] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

const uint32_t *p_caul_max_rms_hi;
const uint32_t *p_caul_max_rms_vlo;

/* Threshold levels using HI-VLO states */
const uint32_t caul_th1_hi_cenelec_a[NUM_TX_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint32_t caul_th2_hi_cenelec_a[NUM_TX_LEVELS] = {67150100, 47482291, 33575050, 23741146, 16787525, 11870573, 8393763, 5935287};
const uint32_t caul_th1_lo_cenelec_a[NUM_TX_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint32_t caul_th2_lo_cenelec_a[NUM_TX_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint32_t caul_th1_vlo_cenelec_a[NUM_TX_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
#ifdef DISABLE_VLO_TO_HI_JUMP
const uint32_t caul_th2_vlo_cenelec_a[NUM_TX_LEVELS] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
#else
const uint32_t caul_th2_vlo_cenelec_a[NUM_TX_LEVELS] = {333236691, 235633924, 166618345, 117816962, 83309173, 58908481, 41654586, 29454240};
#endif

const uint32_t caul_th1_hi_fcc[NUM_TX_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint32_t caul_th2_hi_fcc[NUM_TX_LEVELS] = {187000000, 187000000, 187000000, 187000000, 187000000, 187000000, 187000000, 187000000};
const uint32_t caul_th1_lo_fcc[NUM_TX_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint32_t caul_th2_lo_fcc[NUM_TX_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint32_t caul_th1_vlo_fcc[NUM_TX_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint32_t caul_th2_vlo_fcc[NUM_TX_LEVELS] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

const uint32_t caul_th1_hi_arib[NUM_TX_LEVELS] = {211000000, 211000000, 211000000, 211000000, 211000000, 211000000, 211000000, 211000000};
const uint32_t caul_th2_hi_arib[NUM_TX_LEVELS] = {54000000, 54000000, 54000000, 54000000, 54000000, 54000000, 54000000, 54000000};
const uint32_t caul_th1_lo_arib[NUM_TX_LEVELS] = {283000000, 283000000, 283000000, 283000000, 283000000, 283000000, 283000000, 283000000};
const uint32_t caul_th2_lo_arib[NUM_TX_LEVELS] = {210000000, 210000000, 210000000, 210000000, 210000000, 210000000, 210000000, 210000000};
const uint32_t caul_th1_vlo_arib[NUM_TX_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint32_t caul_th2_vlo_arib[NUM_TX_LEVELS] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

const uint32_t *p_caul_th1_hi;
const uint32_t *p_caul_th2_hi;
const uint32_t *p_caul_th1_lo;
const uint32_t *p_caul_th2_lo;
const uint32_t *p_caul_th1_vlo;
const uint32_t *p_caul_th2_vlo;

/* Shared PHY Variables */
union shared_phy_buffers u_shared_buffers;
struct band_phy_constants s_band_constants;
uint8_t uc_working_band;
uint8_t uc_notched_carriers;
uint8_t uc_used_carriers;
uint8_t uc_num_symbols_fch;
uint8_t uc_psymbol_len;
uint16_t us_fch_interleaver_size_with_padding;
uint8_t uc_fch_interleaver_byte_size;
uint8_t uc_fch_symbol_byte_size;
uint8_t uc_psymbol_no_truepoint_byte_len;
uint32_t ul_resample_var;
uint32_t ul_resample_const2_inv;
uint8_t auc_unmasked_carrier_list[NUM_CARRIERS_FCC];
uint8_t auc_masked_carrier_list[NUM_CARRIERS_FCC];
uint8_t auc_predistortion[NUM_CARRIERS_FCC * 4];
int32_t asl_freq_index[NUM_CARRIERS_FCC], asl_freq_index_squared[NUM_CARRIERS_FCC]; /*frequency indexes and squared frequency indexes in Q8.24*/
/*first sample of the FFT window of each symbol in Q1.31 scaled by 1/2^SCALING_DELAY_VALUES*/
int32_t asl_delay_symbols[(NUM_FULL_SYMBOLS_PREAMBLE + SYMBOLS_8 + 2)] = {0};
uint8_t uc_agc_ext;
uint16_t us_agc_int;
uint8_t uc_impulsive_noise_detected;

/* Variables used to store pilots position in order to write them for debugging */
#ifdef PRINT_PILOTS_DBG
uint8_t auc_pilot_pos_tmp[512][CARR_BUFFER_LEN];
uint8_t auc_pilot_pos_tx_tmp[512][NUM_CARRIERS_FCC];
uint16_t us_symbol_counter_tmp;
#endif

/* Global variables used by assembler functions */
uint8_t uc_ini_lsfr_payload_asm;
uint8_t auc_reference_payload_asm[NUM_CARRIERS_FCC];
uint8_t uc_aux_pointer_asm = 0;
uint8_t auc_state_carrier_asm[NUM_CARRIERS_FCC];
extern uint8_t auc_active_carrier_state[];

/* Address to read interleaver. Initialized here, but used and updated in atpl250_mod_demod.c */
uint32_t ul_interleaver_read_pointer;
/* Address to write deinterleaver. Initialized here, but used and updated in atpl250_mod_demod.c */
uint32_t ul_deinterleaver_write_pointer;

/* Arrays to store pilots, static notching and dynamic notching. Used to fill auc_state_carrier_asm */
uint8_t auc_pilots_state_carrier[NUM_CARRIERS_FCC];
uint8_t auc_static_notching_state_carrier[NUM_CARRIERS_FCC];
uint8_t auc_dynamic_notching_state_carrier[NUM_CARRIERS_FCC];

/* Array to read reference. Used for modulate */
uint32_t aul_reference_payload[NUM_CARRIERS_FCC];

static void _process_ber_info(void)
{
	uint16_t us_payload_symbols;
	int8_t sc_snr_background_db, sc_snr_impulsive_db;
	uint16_t us_total_symbols, us_noised_symbols;
	uint32_t ul_min_noised_symbols, ul_max_noised_symbols;

	if (uc_working_band == WB_CENELEC_A) {
		us_payload_symbols = s_phy_rx_ctl.m_us_rx_payload_symbols;
	} else {
		if (uc_working_band == WB_ARIB) {
			us_payload_symbols = s_phy_rx_ctl.m_us_rx_payload_symbols;
		} else {
			if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_1_BLOCK) {
				us_payload_symbols = s_phy_rx_ctl.m_us_rx_payload_symbols;
			} else {
				/* For 2 RS blocks, BER is computed only with second block*/
				us_payload_symbols = s_phy_rx_ctl.m_us_rx_payload_symbols;
			}
		}
	}

	s_rx_ber_payload_data.uc_lqi = get_lqi_and_per_carrier_snr((uint8_t)s_phy_rx_ctl.e_mod_scheme,
			s_phy_rx_ctl.m_auc_static_and_dynamic_notching_pos,
			s_phy_rx_ctl.m_auc_pilot_pos_first_symbol, s_rx_ber_payload_data.auc_carrier_snr,
			us_payload_symbols, s_phy_rx_ctl.m_uc_payload_carriers, s_phy_rx_ctl.e_rs_blocks);

	us_noised_symbols = s_rx_ber_payload_data.us_payload_noised_symbols;
	us_total_symbols = us_payload_symbols + uc_num_symbols_fch;
	sc_snr_impulsive_db = casc_ber_to_db[s_rx_ber_payload_data.uc_payload_snr_impulsive];
	sc_snr_background_db = casc_ber_to_db[s_rx_ber_payload_data.uc_payload_snr_background];

	/* Check conditions for impulsive noise */
	if (s_phy_rx_ctl.m_uc_rs_corrected_errors == 0) {
		uc_impulsive_noise_detected = 0;
		if (((sc_snr_background_db - sc_snr_impulsive_db) > MIN_DISTANCE_SNR_IMPULSIVE_TO_BACKGROUND) &&
				(sc_snr_impulsive_db < MAX_SNR_IMPULSIVE_TO_CONSIDER_ROBO)) {
			ul_min_noised_symbols = (IMPULSIVE_MULT_FACTOR_MIN_SYMBOLS * (uint32_t)(us_total_symbols + 1)) / IMPULSIVE_DIV_FACTOR;
			if (s_phy_rx_ctl.e_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
				ul_max_noised_symbols
					= (IMPULSIVE_MULT_FACTOR_MAX_SYMBOLS_DIF * (uint32_t)(us_total_symbols + 1)) / IMPULSIVE_DIV_FACTOR;
			} else {
				ul_max_noised_symbols
					= (IMPULSIVE_MULT_FACTOR_MAX_SYMBOLS_COH * (uint32_t)(us_total_symbols + 1)) / IMPULSIVE_DIV_FACTOR;
			}

			if ((us_noised_symbols >= ul_min_noised_symbols) && (us_noised_symbols <= ul_max_noised_symbols)) {
				uc_impulsive_noise_detected = 1;
				LOG_PHY(("Impulsive Noise detected\r\n"));
			}
		}
	}

	/* Set Valid data */
	s_rx_ber_payload_data.uc_valid_data = 1;

	sb_ber_int_received = false;
}

/**
 * \brief Function set tx_preemphasis
 *
 */
static void _set_preemphasis(uint8_t *puc_preemphasis)
{
	uint8_t uc_j;
	uint8_t uc_k, uc_preemphasis_value;
	uint16_t us_ripple_correction;
	uint8_t uc_notched_carrier;
	uint16_t us_i;
	uint16_t us_pe_value_ref, us_pe_value;
	uint8_t uc_a;

	if (uc_working_band == WB_CENELEC_A) {
		uc_a = 4;
	} else {
		uc_a = 2;
	}

	/* Clear predistortion buffer, as it is in the sahred phy buffers area */
	memset(auc_predistortion, 0, sizeof(auc_predistortion));

	uc_k = 0;
	for (us_i = 0; us_i < s_band_constants.uc_num_subbands; us_i++) {
		uc_preemphasis_value = puc_preemphasis[us_i];
		us_ripple_correction = 0;

		if (uc_working_band == WB_CENELEC_A) {
			/* Apply ripple correction */
			if (uc_preemphasis_value > 15) {
				uc_preemphasis_value = 15;
			}

			if (atpl250.m_uc_impedance_state == VLO_STATE) {
				us_ripple_correction = aus_ripple_correction[uc_preemphasis_value][uc_k++];
			} else if (atpl250.m_uc_impedance_state == LO_STATE) {
				us_ripple_correction = aus_ripple_correction_LO[uc_preemphasis_value][uc_k++];
			} else {
				us_ripple_correction = aus_ripple_correction_HI[uc_preemphasis_value][uc_k++];
			}

			us_pe_value_ref = aus_preemphasis_att_values[uc_preemphasis_value];
			us_pe_value = us_pe_value_ref - us_ripple_correction;
		} else {
			us_pe_value = aus_preemphasis_att_values[uc_preemphasis_value];
		}

		auc_predistortion[us_i * s_band_constants.uc_num_carriers_in_subband * uc_a + 0 * uc_a] = (us_pe_value & 0xFF00) >> 8;
		auc_predistortion[us_i * s_band_constants.uc_num_carriers_in_subband * uc_a + 0 * uc_a + 1] = us_pe_value & 0x00FF;

		if (uc_working_band == WB_CENELEC_A) {
			if (atpl250.m_uc_impedance_state == VLO_STATE) {
				us_ripple_correction = aus_ripple_correction[uc_preemphasis_value][uc_k++];
			} else if (atpl250.m_uc_impedance_state == LO_STATE) {
				us_ripple_correction = aus_ripple_correction_LO[uc_preemphasis_value][uc_k++];
			} else {
				us_ripple_correction = aus_ripple_correction_HI[uc_preemphasis_value][uc_k++];
			}

			us_pe_value = us_pe_value_ref - us_ripple_correction;
		}

		auc_predistortion[us_i * s_band_constants.uc_num_carriers_in_subband * uc_a + 1 * uc_a] = (us_pe_value & 0xFF00) >> 8;
		auc_predistortion[us_i * s_band_constants.uc_num_carriers_in_subband * uc_a + 1 * uc_a + 1] = us_pe_value & 0x00FF;

		if (uc_working_band == WB_CENELEC_A) {
			if (atpl250.m_uc_impedance_state == VLO_STATE) {
				us_ripple_correction = aus_ripple_correction[uc_preemphasis_value][uc_k++];
			} else if (atpl250.m_uc_impedance_state == LO_STATE) {
				us_ripple_correction = aus_ripple_correction_LO[uc_preemphasis_value][uc_k++];
			} else {
				us_ripple_correction = aus_ripple_correction_HI[uc_preemphasis_value][uc_k++];
			}

			us_pe_value = us_pe_value_ref - us_ripple_correction;
		}

		auc_predistortion[us_i * s_band_constants.uc_num_carriers_in_subband * uc_a + 2 * uc_a] = (us_pe_value & 0xFF00) >> 8;
		auc_predistortion[us_i * s_band_constants.uc_num_carriers_in_subband * uc_a + 2 * uc_a + 1] = us_pe_value & 0x00FF;

		if (uc_working_band == WB_CENELEC_A) {
			if (atpl250.m_uc_impedance_state == VLO_STATE) {
				us_ripple_correction = aus_ripple_correction[uc_preemphasis_value][uc_k++];
			} else if (atpl250.m_uc_impedance_state == LO_STATE) {
				us_ripple_correction = aus_ripple_correction_LO[uc_preemphasis_value][uc_k++];
			} else {
				us_ripple_correction = aus_ripple_correction_HI[uc_preemphasis_value][uc_k++];
			}

			us_pe_value = us_pe_value_ref - us_ripple_correction;
			auc_predistortion[us_i * s_band_constants.uc_num_carriers_in_subband * uc_a + 3 * uc_a] = (us_pe_value & 0xFF00) >> 8;
			auc_predistortion[us_i * s_band_constants.uc_num_carriers_in_subband * uc_a + 3 * uc_a + 1] = us_pe_value & 0x00FF;

			if (atpl250.m_uc_impedance_state == VLO_STATE) {
				us_ripple_correction = aus_ripple_correction[uc_preemphasis_value][uc_k++];
			} else if (atpl250.m_uc_impedance_state == LO_STATE) {
				us_ripple_correction = aus_ripple_correction_LO[uc_preemphasis_value][uc_k++];
			} else {
				us_ripple_correction = aus_ripple_correction_HI[uc_preemphasis_value][uc_k++];
			}

			us_pe_value = us_pe_value_ref - us_ripple_correction;
			auc_predistortion[us_i * s_band_constants.uc_num_carriers_in_subband * uc_a + 4 * uc_a] = (us_pe_value & 0xFF00) >> 8;
			auc_predistortion[us_i * s_band_constants.uc_num_carriers_in_subband * uc_a + 4 * uc_a + 1] = us_pe_value & 0x00FF;

			if (atpl250.m_uc_impedance_state == VLO_STATE) {
				us_ripple_correction = aus_ripple_correction[uc_preemphasis_value][uc_k++];
			} else if (atpl250.m_uc_impedance_state == LO_STATE) {
				us_ripple_correction = aus_ripple_correction_LO[uc_preemphasis_value][uc_k++];
			} else {
				us_ripple_correction = aus_ripple_correction_HI[uc_preemphasis_value][uc_k++];
			}

			us_pe_value = us_pe_value_ref - us_ripple_correction;
			auc_predistortion[us_i * s_band_constants.uc_num_carriers_in_subband * uc_a + 5 * uc_a] = (us_pe_value & 0xFF00) >> 8;
			auc_predistortion[us_i * s_band_constants.uc_num_carriers_in_subband * uc_a + 5 * uc_a + 1] = us_pe_value & 0x00FF;
		}
	}

	/* Set 0s in notched carriers */
	if (uc_working_band == WB_CENELEC_A) {
		for (uc_j = 0; uc_j < uc_notched_carriers; uc_j++) {
			uc_notched_carrier = (auc_masked_carrier_list[uc_j] << 2);
			auc_predistortion[uc_notched_carrier + 0] = 0x00;
			auc_predistortion[uc_notched_carrier + 1] = 0x00;
			auc_predistortion[uc_notched_carrier + 2] = 0x00;
			auc_predistortion[uc_notched_carrier + 3] = 0x00;
		}
	} else {
		for (uc_j = 0; uc_j < uc_notched_carriers; uc_j++) {
			uc_notched_carrier = (auc_masked_carrier_list[uc_j] << 1);
			auc_predistortion[uc_notched_carrier + 0] = 0x00;
			auc_predistortion[uc_notched_carrier + 1] = 0x00;
		}
	}

	if (uc_working_band != WB_CENELEC_A) {
		/* Access only real part of carrier */
		atpl250_set_iob_real_mode();
	}

	/* Write to Zone1 */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 0) + s_band_constants.uc_last_carrier); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((0 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * uc_a);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 1) + s_band_constants.uc_last_carrier); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((1 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * uc_a);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 2) + s_band_constants.uc_last_carrier); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((2 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * uc_a);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 3) + s_band_constants.uc_last_carrier); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((3 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * uc_a);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 4) + s_band_constants.uc_last_carrier); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((4 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * uc_a);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 5) + s_band_constants.uc_last_carrier); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((5 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * uc_a);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 6) + s_band_constants.uc_last_carrier); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((6 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * uc_a);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 7) + s_band_constants.uc_last_carrier); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((7 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier)), auc_predistortion, s_band_constants.uc_num_carriers * uc_a);

	if (uc_working_band != WB_CENELEC_A) {
		/* Access both real and imaginary part of carrier */
		atpl250_clear_iob_real_mode();
	}
}

/**
 * \brief Pilot position initialization
 *
 */
static void _pilots_init(uint8_t uc_num_pilots, uint8_t uc_num_active_carriers)
{
	uint8_t uc_i;
	uint8_t uc_pilot_pos;
	uint8_t uc_pilot_index;
	uint8_t uc_carrier_index;

	memset(auc_pilots_state_carrier, 0, sizeof(auc_pilots_state_carrier));
	for (uc_i = 0; uc_i < uc_num_pilots; uc_i++) {
		uc_pilot_pos = (s_band_constants.uc_pilot_offset + PILOT_FREQ_SPA * uc_i) % uc_num_active_carriers;
		uc_pilot_index = 0;
		uc_carrier_index = 0;
		while (uc_pilot_index < uc_pilot_pos) {
			if (auc_dynamic_notching_state_carrier[s_band_constants.uc_num_carriers - uc_carrier_index - 1] ==
					0 && auc_static_notching_state_carrier[s_band_constants.uc_num_carriers - uc_carrier_index - 1] == 0) {
				uc_pilot_index++;
			}

			uc_carrier_index++;
		}
		while (!(auc_dynamic_notching_state_carrier[s_band_constants.uc_num_carriers - uc_carrier_index - 1] ==
				0 && auc_static_notching_state_carrier[s_band_constants.uc_num_carriers - uc_carrier_index - 1] == 0)) {
			uc_carrier_index++;
		}
		auc_pilots_state_carrier[s_band_constants.uc_num_carriers - uc_carrier_index - 1] = 1;
	}
}

/**
 * \brief Fill auc_rd_dynamic_notching array for CENELEC_A
 *
 */
static void _fill_dynamic_notching_cenelec_a(uint8_t *puc_tone_map)
{
	memset(&auc_dynamic_notching_state_carrier[0], 0x00, sizeof(auc_dynamic_notching_state_carrier));
	if (!((*puc_tone_map) & 0x01)) {
		auc_dynamic_notching_state_carrier[35] = 1;
		auc_dynamic_notching_state_carrier[34] = 1;
		auc_dynamic_notching_state_carrier[33] = 1;
		auc_dynamic_notching_state_carrier[32] = 1;
		auc_dynamic_notching_state_carrier[31] = 1;
		auc_dynamic_notching_state_carrier[30] = 1;
	}

	if (!((*puc_tone_map) & 0x02)) {
		auc_dynamic_notching_state_carrier[29] = 1;
		auc_dynamic_notching_state_carrier[28] = 1;
		auc_dynamic_notching_state_carrier[27] = 1;
		auc_dynamic_notching_state_carrier[26] = 1;
		auc_dynamic_notching_state_carrier[25] = 1;
		auc_dynamic_notching_state_carrier[24] = 1;
	}

	if (!((*puc_tone_map) & 0x04)) {
		auc_dynamic_notching_state_carrier[23] = 1;
		auc_dynamic_notching_state_carrier[22] = 1;
		auc_dynamic_notching_state_carrier[21] = 1;
		auc_dynamic_notching_state_carrier[20] = 1;
		auc_dynamic_notching_state_carrier[19] = 1;
		auc_dynamic_notching_state_carrier[18] = 1;
	}

	if (!((*puc_tone_map) & 0x08)) {
		auc_dynamic_notching_state_carrier[17] = 1;
		auc_dynamic_notching_state_carrier[16] = 1;
		auc_dynamic_notching_state_carrier[15] = 1;
		auc_dynamic_notching_state_carrier[14] = 1;
		auc_dynamic_notching_state_carrier[13] = 1;
		auc_dynamic_notching_state_carrier[12] = 1;
	}

	if (!((*puc_tone_map) & 0x10)) {
		auc_dynamic_notching_state_carrier[11] = 1;
		auc_dynamic_notching_state_carrier[10] = 1;
		auc_dynamic_notching_state_carrier[9] = 1;
		auc_dynamic_notching_state_carrier[8] = 1;
		auc_dynamic_notching_state_carrier[7] = 1;
		auc_dynamic_notching_state_carrier[6] = 1;
	}

	if (!((*puc_tone_map) & 0x20)) {
		auc_dynamic_notching_state_carrier[5] = 1;
		auc_dynamic_notching_state_carrier[4] = 1;
		auc_dynamic_notching_state_carrier[3] = 1;
		auc_dynamic_notching_state_carrier[2] = 1;
		auc_dynamic_notching_state_carrier[1] = 1;
		auc_dynamic_notching_state_carrier[0] = 1;
	}
}

/**
 * \brief Fill auc_rd_dynamic_notching array for FCC/ARIB
 *
 */
static void _fill_dynamic_notching_fcc_arib(uint8_t *puc_tone_map)
{
	uint32_t ul_tone_map = 0;
	uint32_t ul_i;
	uint32_t ul_carrier_pointer = 0;

	memset(&auc_dynamic_notching_state_carrier[0], 0x00, sizeof(auc_dynamic_notching_state_carrier));
	ul_tone_map += (*puc_tone_map++);
	ul_tone_map += (*puc_tone_map++) << 8;
	ul_tone_map += (*puc_tone_map) << 16;

	for (ul_i = 0; ul_i < s_band_constants.uc_num_subbands; ul_i++) {
		if (!(ul_tone_map & (1 << ul_i))) {
			auc_dynamic_notching_state_carrier[s_band_constants.uc_num_carriers - (ul_carrier_pointer) - 1] = 1;
			auc_dynamic_notching_state_carrier[s_band_constants.uc_num_carriers - (ul_carrier_pointer + 1) - 1] = 1;
			auc_dynamic_notching_state_carrier[s_band_constants.uc_num_carriers - (ul_carrier_pointer + 2) - 1] = 1;
		}

		ul_carrier_pointer += 3;
	}
}

/**
 * \brief Fill auc_rd_static_notching array
 *
 */
static void _fill_static_notching(uint8_t *puc_tone_mask)
{
	uint32_t ul_i, ul_byte, ul_bit, ul_tone_mask_pointer;

	ul_tone_mask_pointer = s_band_constants.uc_first_carrier;
	memset(&auc_static_notching_state_carrier[0], 0x00, sizeof(auc_static_notching_state_carrier));

	for (ul_i = 0; ul_i < s_band_constants.uc_num_carriers; ul_i++) {
		ul_byte = ul_tone_mask_pointer >> 3;
		ul_bit = ul_tone_mask_pointer & 7ul;

		if ((puc_tone_mask[ul_byte] & (1 << ul_bit))) {
			auc_static_notching_state_carrier[s_band_constants.uc_num_carriers - ul_i - 1] = 1;
		}

		ul_tone_mask_pointer++;
	}
}

/**
 * \brief Fill auc_rd_state_carrier array used by PAYLOAD_MODULATION() assembler function
 *
 */
static void _fill_state_carriers(uint8_t uc_mod_type, uint8_t uc_mod_scheme)
{
	uint32_t ul_i;
	uint8_t uc_active_carrier_counter = 0;

	memset(&auc_state_carrier_asm[0], 0x00, sizeof(auc_state_carrier_asm));

	for (ul_i = 0; ul_i < s_band_constants.uc_num_carriers; ul_i++) {
		auc_state_carrier_asm[ul_i] = auc_static_notching_state_carrier[ul_i] << 7;
		auc_state_carrier_asm[ul_i] += auc_dynamic_notching_state_carrier[ul_i] << 6;
		auc_state_carrier_asm[ul_i] += auc_pilots_state_carrier[ul_i] << 5;

		auc_state_carrier_asm[ul_i] += uc_mod_scheme << 2;
		auc_state_carrier_asm[ul_i] += (uc_mod_type & 0x03) + 1;

		#ifdef UPDATE_CHN_SFO_EST_PAYLOAD
		if (uc_working_band == WB_CENELEC_A) {
			if ((auc_static_notching_state_carrier[s_band_constants.uc_num_carriers - 1 - ul_i] == 0) &&
					(auc_dynamic_notching_state_carrier[s_band_constants.uc_num_carriers - 1 - ul_i] == 0)) {
				auc_active_carrier_state[uc_active_carrier_counter++] = ul_i;
			}
		}

		#else
		UNUSED(uc_active_carrier_counter);
		#endif
	}
}

/*--------------------------------------------------------------------------------------*/
/* Declaration of assembler functions */
extern void swap_bytes_asm(int16_t *pss_input_array, uint16_t us_num_elements);
extern void shift_add_shift_asm(int16_t *pss_input_array1, int16_t *pss_input_array2, uint8_t uc_num_complex_elements);
extern void smooth_carriers_asm(uint8_t *puc_data_carriers_list, uint8_t uc_num_data_carriers, q15_t *pss_input_symbol);

/**
 * \brief Function to calculate pdc in tx
 *
 */
static void _zcd_calc_pdc_rx(void)
{
	uint32_t ul_peak2_time;
	uint32_t ul_zc_time;
	uint32_t ul_zc_time_past;
	uint32_t ul_mains_period;
	uint32_t ul_zc2peak2;
	uint16_t us_time_error = s_band_constants.ul_half_symbol_duration;

	ul_zc_time_past = pplc_if_read32(REG_ATPL250_ZC_TIME_PAST_32) - CONF_ZC_OFFSET_CORRECTION;
	ul_zc_time = pplc_if_read32(REG_ATPL250_ZC_TIME_32) - CONF_ZC_OFFSET_CORRECTION;
	ul_peak2_time = pplc_if_read32(REG_ATPL250_PEAK2_TIME_32);

	if (ul_zc_time == ul_zc_time_past || (ul_zc_time -  ul_zc_time_past) > (20000 + 200)) {
		ul_zc_time_past = pplc_if_read32(REG_ATPL250_ZC_TIME_PAST_32) - CONF_ZC_OFFSET_CORRECTION;
		ul_zc_time = pplc_if_read32(REG_ATPL250_ZC_TIME_32) - CONF_ZC_OFFSET_CORRECTION;
	}

	if (ul_zc_time_past < ul_zc_time) {
		ul_mains_period = ul_zc_time - ul_zc_time_past;
	} else {
		ul_mains_period = ul_zc_time - ul_zc_time_past + 0xFFFFFFFF;
	}

	if ((ul_peak2_time + us_time_error) > ul_zc_time) {
		ul_zc2peak2 = ul_peak2_time - ul_zc_time + us_time_error;
	} else {
		ul_zc2peak2 = ul_peak2_time - ul_zc_time + us_time_error + 0xFFFFFFFF;
	}

	if (!uc_pdc_ratio) {
		ul_zc_time_past = pplc_if_read32(REG_ATPL250_ZC_TIME_PAST_32) - CONF_ZC_OFFSET_CORRECTION;
		ul_zc_time = pplc_if_read32(REG_ATPL250_ZC_TIME_32) - CONF_ZC_OFFSET_CORRECTION;
		ul_mains_period = ul_zc_time - ul_zc_time_past;
		if (ul_mains_period > 19000) {
			uc_pdc_ratio = 78;  /*10e6/(256*50Hz)*/
		} else {
			uc_pdc_ratio = 65;  /*10e6/(256*60Hz)*/
		}
	}

	if (uc_pdc_ratio == 78) { /*50Hz*/
		if (ul_zc2peak2 > 20000) {
			ul_zc2peak2 -= 20000;
		}
	} else { /*60Hz*/
		if (ul_zc2peak2 > 16666) {
			ul_zc2peak2 -= 16666;
		}
	}

	s_phy_rx_ctl.m_uc_rx_pdc = (ul_zc2peak2 / uc_pdc_ratio);
}

static void _zcd_calc_phase_difference(void)
{
	uint8_t uc_abs_diff;

	if (s_phy_rx_ctl.m_uc_rx_pdc >= s_phy_rx_ctl.m_uc_tx_pdc) {
		uc_abs_diff = s_phy_rx_ctl.m_uc_rx_pdc - s_phy_rx_ctl.m_uc_tx_pdc;
	} else {
		uc_abs_diff = s_phy_rx_ctl.m_uc_rx_pdc - s_phy_rx_ctl.m_uc_tx_pdc + 255;
	}

	if (uc_abs_diff > 235 || uc_abs_diff <= 21) { /* between 330 deg and 30 deg */
		s_phy_rx_ctl.m_uc_zct_diff = 0;
	} else if (uc_abs_diff <= 64) { /* between 30 deg and 90 deg */
		s_phy_rx_ctl.m_uc_zct_diff = 1;
	} else if (uc_abs_diff <= 107) { /* between 90 deg and 150 deg */
		s_phy_rx_ctl.m_uc_zct_diff = 2;
	} else if (uc_abs_diff <= 149) { /* between 150 deg and 210 deg */
		s_phy_rx_ctl.m_uc_zct_diff = 3;
	} else if (uc_abs_diff <= 192) { /* between 210 deg and 270 deg */
		s_phy_rx_ctl.m_uc_zct_diff = 4;
	} else { /* between 270 deg and 330 deg */
		s_phy_rx_ctl.m_uc_zct_diff = 5;
	}
}

#if defined(CALCULATE_TX_PDC)

/**
 * \brief Function to calculate PDC in Transmission
 *
 */
static void _zcd_calc_pdc_tx_start(void)
{
	uint32_t ul_zc_time;
	uint32_t timer_ref;
	uint32_t ul_zc_time_past;
	uint32_t ul_mains_period;
	uint32_t ul_zctx;

	ul_zc_time_past = pplc_if_read32(REG_ATPL250_ZC_TIME_PAST_32) - CONF_ZC_OFFSET_CORRECTION;
	ul_zc_time = pplc_if_read32(REG_ATPL250_ZC_TIME_32) - CONF_ZC_OFFSET_CORRECTION;
	timer_ref = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);

	if (ul_zc_time == ul_zc_time_past || (ul_zc_time -  ul_zc_time_past) > (20000 + 200)) {
		ul_zc_time_past = pplc_if_read32(REG_ATPL250_ZC_TIME_PAST_32) - CONF_ZC_OFFSET_CORRECTION;
		ul_zc_time = pplc_if_read32(REG_ATPL250_ZC_TIME_32) - CONF_ZC_OFFSET_CORRECTION;
	}

	if (ul_zc_time_past < ul_zc_time) {
		ul_mains_period = ul_zc_time - ul_zc_time_past;
	} else {
		ul_mains_period = ul_zc_time - ul_zc_time_past + 0xFFFFFFFF;
	}

	if (timer_ref > ul_zc_time) {
		ul_zctx = (timer_ref - ul_zc_time) + s_band_constants.ul_txrx_plc_us + s_band_constants.ul_preamble_duration;
	} else {
		ul_zctx = (timer_ref - ul_zc_time + 0xFFFFFFFF) + s_band_constants.ul_txrx_plc_us + s_band_constants.ul_preamble_duration;
	}

	/* Calculate Mains Frequency only if its not calculated previously */
	if (!uc_pdc_ratio) {
		if (ul_zc_time_past < ul_zc_time) {
			ul_mains_period = ul_zc_time - ul_zc_time_past;
		} else {
			ul_mains_period = ul_zc_time - ul_zc_time_past + 0xFFFFFFFF;
		}

		if (ul_mains_period > 19000) {
			uc_pdc_ratio = 78;  /*10e6/(256*50Hz)*/
		} else {
			uc_pdc_ratio = 65;  /*10e6/(256*60Hz)*/
		}
	}

	if (uc_pdc_ratio == 78) { /*50Hz*/
		if (ul_zctx > 40000) {
			ul_zctx -= 40000;
		} else if (ul_zctx > 20000) {
			ul_zctx -= 20000;
		}
	} else { /*60Hz*/
		if (ul_zctx > 33332) {
			ul_zctx -= 33332;
		} else if (ul_zctx > 16666) {
			ul_zctx -= 16666;
		}
	}

	s_phy_tx_ctl.m_uc_pdc = (ul_zctx / uc_pdc_ratio); /* correction detected */
}

#endif

/**
 * \brief Configure Tx parameters
 *
 * \param uc_forced   Forced tx (no carrier sense)
 * \param uc_delayed  Delayed tx (not immediate)
 * \param ul_tx_time  Tx time (only valid if uc_delayed=1)
 *
 */
static void _config_tx(uint8_t uc_forced, uint8_t uc_delayed, uint32_t ul_tx_time)
{
	uint8_t uc_mask;
	uint8_t uc_mask_immediate;
	uint32_t ul_reg_addr;
	uint32_t ul_plc_time;

	uc_mask = 0x01;
	uc_mask_immediate = 0x11;
	ul_reg_addr = REG_ATPL250_TX_TIME1_32;
	ul_plc_time = s_band_constants.ul_txrx_plc_us + s_band_constants.ul_rrc_delay_us;

	/* Write registers */
	if (uc_forced) { /* Disable CD */
		pplc_if_and8(REG_ATPL250_TXRX_CFG_H8, (uint8_t)(~(unsigned int)uc_mask));
	} else { /* Enable CD */
		pplc_if_or8(REG_ATPL250_TXRX_CFG_H8, uc_mask);
	}

	s_phy_tx_ctl.m_ul_start_tx_watch_ms = oss_get_up_time_ms();

	if (uc_delayed) { /* Set Tx time */
		pplc_if_write32(ul_reg_addr, (ul_tx_time - ul_plc_time));
		pplc_if_or8(REG_ATPL250_GLOBAL_SRST_VL8, uc_mask); /* Program as delayed */
	} else { /* Program as immediate */
		pplc_if_or8(REG_ATPL250_GLOBAL_SRST_VL8, uc_mask_immediate);
	}
}

/**
 * \brief Configure Symbol
 *
 * \param ps_sym_cfg   Pointer to symbol configuration
 *
 */
static void _config_symbol(struct sym_cfg *ps_sym_cfg)
{
	/* Set load bit */
	ps_sym_cfg->m_uc_reserved = 1;
	/* Write buffer */
	pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)ps_sym_cfg, sizeof(struct sym_cfg));
}

#ifdef TX_PREAMBLE_HW

/**
 * \brief Writes a preamble symbol to be used as reference for header
 *
 */
static inline void _write_ref_preamble_and_header_first(void)
{
	/* Disable SYNCM DETECTOR */
	pplc_if_and8(REG_ATPL250_SYNCM_CTL_L8, 0x7F);
	/* Reset IOB, Rotator and FFT */
	atpl250_iob_rotator_and_fft_push_rst();
	/* Set Tx mode. TXRXBUF_TX_MODE = 1 */
	pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x02);
	/* IFFT=1, PATH="01" configures fft for transmission */
	pplc_if_or8(REG_ATPL250_FFT_CONFIG_VL8, 0x81); /* IFFT=1; PATH[0]=1 */
	pplc_if_and8(REG_ATPL250_FFT_CONFIG_VL8, 0xFD); /* PATH[1]=0 */
	/* Release IOB, Rotator and FFT reset */
	atpl250_iob_rotator_and_fft_release_rst();
	/* Clear Rx Mode */
	atpl250_clear_rx_mode();
	/* Set tx mode */
	atpl250_set_tx_mode();

	/* Change value to modulator abcd points (minus 3dB) */
	pplc_if_write_buf(REG_ATPL250_INOUTB_PSK_VALUES1_32, (uint8_t *)cauc_abcd_minus3db, ABCD_POINTS_LEN);

	s_sym_cfg.m_uc_gain = uc_phy_tx_full_gain;
	s_sym_cfg.m_uc_rep1514 = 0;
	s_sym_cfg.m_uc_rep1312 = 0;
	s_sym_cfg.m_uc_rep1110 = 0;
	s_sym_cfg.m_uc_rep98 = 0;
	s_sym_cfg.m_uc_rep76 = 0;
	s_sym_cfg.m_uc_rep54 = 0;
	s_sym_cfg.m_uc_rep32 = 0;
	s_sym_cfg.m_uc_rep10 = 0;
	s_sym_cfg.m_uc_overlap = 8;
	s_sym_cfg.m_uc_cyclicprefix = 30;
	s_sym_cfg.m_uc_is_last_symbol = 0;
	s_sym_cfg.m_uc_repetitions = 0;

	/* Remove reference to set P reference on IOB */
	atpl250_hold_on_reference(false);

	/* Set number of symbols */
	atpl250_set_num_symbols_cfg(1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 7) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to n-th symbol when writing */
	s_sym_cfg.m_uc_sym_idx = 0;
	_config_symbol(&s_sym_cfg);
	s_sym_cfg.m_uc_sym_idx = 1;
	_config_symbol(&s_sym_cfg);
	s_sym_cfg.m_uc_sym_idx = 2;
	_config_symbol(&s_sym_cfg);
	s_sym_cfg.m_uc_sym_idx = 3;
	_config_symbol(&s_sym_cfg);
	s_sym_cfg.m_uc_sym_idx = 4;
	_config_symbol(&s_sym_cfg);
	s_sym_cfg.m_uc_sym_idx = 5;
	_config_symbol(&s_sym_cfg);
	s_sym_cfg.m_uc_sym_idx = 6;
	_config_symbol(&s_sym_cfg);
	s_sym_cfg.m_uc_sym_idx = 7;
	_config_symbol(&s_sym_cfg);

	atpl250_set_mod_bpsk_truepoint();
	pplc_if_write_jump((BCODE_COH | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0]
			+ (7 * CFG_IOB_SAMPLES_PER_SYMBOL))), (uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);

	if (uc_working_band == WB_FCC) {
		if (uc_legacy_mode) {
			atpl250_hold_on_reference(true);
		} else {
			atpl250_hold_on_reference(false);
		}
	} else {
		atpl250_hold_on_reference(false);
	}

	/* Set interleaver to unload mode, so data can be written to modulator after preamble */
	pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x07); /* 07: BPSK, unload mode, tx mode, G3 mode, enable HW chain */

	/* Header symbols */
	s_phy_tx_ctl.m_us_tx_pending_symbols = uc_num_symbols_fch;
	feed_modulator_fch(MAX_NUM_SYM_MOD, NO_SYM_OFFSET, CHANGE_MODULATION);
	s_phy_tx_ctl.m_us_tx_pending_symbols -= MAX_NUM_SYM_MOD;
}

#endif

/**
 * \brief Transmits first part of preamble of a frame
 *
 */
static inline void _tx_preamble_first(void)
{
	atpl250_hold_on_reference(false);

	/* Disable SYNCM DETECTOR */
	pplc_if_and8(REG_ATPL250_SYNCM_CTL_L8, 0x7F);
	/* Reset IOB, Rotator and FFT */
	atpl250_iob_rotator_and_fft_push_rst();
	/* Set Tx mode. TXRXBUF_TX_MODE = 1 */
	pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x02);
	/* IFFT=1, PATH="01" configures fft for transmission */
	pplc_if_or8(REG_ATPL250_FFT_CONFIG_VL8, 0x81); /* IFFT=1; PATH[0]=1 */
	pplc_if_and8(REG_ATPL250_FFT_CONFIG_VL8, 0xFD); /* PATH[1]=0 */
	/* Release IOB, Rotator and FFT reset */
	atpl250_iob_rotator_and_fft_release_rst();
	/* Clear Rx Mode */
	atpl250_clear_rx_mode();
	/* Set tx mode */
	atpl250_set_tx_mode();

	/* Change value to modulator abcd points (full gain) */
	pplc_if_write_buf(REG_ATPL250_INOUTB_PSK_VALUES1_32, (uint8_t *)cauc_abcd_fullgain, ABCD_POINTS_LEN);

	/* Change config to send the first part of preamble - 7P */
	s_sym_cfg.m_uc_gain = uc_phy_tx_full_gain;
	s_sym_cfg.m_uc_rep1514 = 0;
	s_sym_cfg.m_uc_rep1312 = 0;
	s_sym_cfg.m_uc_rep1110 = 0;
	s_sym_cfg.m_uc_rep98 = 0;
	s_sym_cfg.m_uc_rep76 = 0;
	s_sym_cfg.m_uc_rep54 = 0;
	s_sym_cfg.m_uc_rep32 = 0;
	s_sym_cfg.m_uc_rep10 = 0;
	s_sym_cfg.m_uc_overlap = 8;
	s_sym_cfg.m_uc_cyclicprefix = 8;
	s_sym_cfg.m_uc_is_last_symbol = 0;
	s_sym_cfg.m_uc_repetitions = 6;
	s_sym_cfg.m_uc_sym_idx = 0;
	_config_symbol(&s_sym_cfg);

	/* Clear notched carriers if necessary */
	if (uc_notched_carriers) {
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, 0x03FF);
		atpl250_set_mod_bpsk();
		/* Data length = Notched carriers * 1 Bit per carrier * 8 symbols / 8 bits in byte = Notched carriers */
		/* Data written is useless, it will be set to 0s due to predistortion. */
		/* The same predistortion buffer is used as it is a long array */
		pplc_if_write_jump((BCODE_DIFT | (s_band_constants.uc_first_carrier + auc_masked_carrier_list[0])), auc_predistortion, uc_notched_carriers, JUMP_COL_2);
	}

	/* Set number of symbols */
	atpl250_set_num_symbols_cfg(1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier); /* Avoid overflow to n-th symbol when writing */
	atpl250_set_mod_bpsk_truepoint();
	pplc_if_write_jump((BCODE_COH | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);
}

/**
 * \brief Transmits last part of preamble of a frame
 *
 */
static inline void _tx_preamble_last(void)
{
	/* Change config to send the last part of preamble - 1P + 1M + 0.5M */
	s_sym_cfg.m_uc_gain = uc_phy_tx_full_gain;
	s_sym_cfg.m_uc_rep1514 = 0;
	s_sym_cfg.m_uc_rep1312 = 0;
	s_sym_cfg.m_uc_rep1110 = 0;
	s_sym_cfg.m_uc_rep98 = 0;
	s_sym_cfg.m_uc_rep76 = 0;
	s_sym_cfg.m_uc_rep54 = 0;
	s_sym_cfg.m_uc_rep32 = 0x06;
	s_sym_cfg.m_uc_rep10 = 0x40;
	s_sym_cfg.m_uc_overlap = 8;
	s_sym_cfg.m_uc_cyclicprefix = 8;
	s_sym_cfg.m_uc_is_last_symbol = 0;
	s_sym_cfg.m_uc_repetitions = 2;
	s_sym_cfg.m_uc_sym_idx = 0;
	_config_symbol(&s_sym_cfg);

	/* Set number of symbols */
	atpl250_set_num_symbols_cfg(1);
	/* P symbol is already written on IOB from _tx_preamble_first function. Overflow and modulation not need to change */
	pplc_if_write_jump((BCODE_COH | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);
}

/**
 * \brief Transmits first part of Header of a frame
 *
 */
static inline void _tx_header_first(void)
{
	/* Change value to modulator abcd points (minus 3dB) */
	pplc_if_write_buf(REG_ATPL250_INOUTB_PSK_VALUES1_32, (uint8_t *)cauc_abcd_minus3db, ABCD_POINTS_LEN);

	s_sym_cfg.m_uc_gain = uc_phy_tx_full_gain;
	s_sym_cfg.m_uc_rep1514 = 0;
	s_sym_cfg.m_uc_rep1312 = 0;
	s_sym_cfg.m_uc_rep1110 = 0;
	s_sym_cfg.m_uc_rep98 = 0;
	s_sym_cfg.m_uc_rep76 = 0;
	s_sym_cfg.m_uc_rep54 = 0;
	s_sym_cfg.m_uc_rep32 = 0;
	s_sym_cfg.m_uc_rep10 = 0;
	s_sym_cfg.m_uc_overlap = 8;
	s_sym_cfg.m_uc_cyclicprefix = 30;
	s_sym_cfg.m_uc_is_last_symbol = 0;
	s_sym_cfg.m_uc_repetitions = 0;
	/* Config affects to 8 symbols */
	s_sym_cfg.m_uc_reserved = 1;
	s_sym_cfg.m_uc_sym_idx = 0;
	pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg), sizeof(struct sym_cfg));
	s_sym_cfg.m_uc_sym_idx = 1;
	pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg), sizeof(struct sym_cfg));
	s_sym_cfg.m_uc_sym_idx = 2;
	pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg), sizeof(struct sym_cfg));
	s_sym_cfg.m_uc_sym_idx = 3;
	pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg), sizeof(struct sym_cfg));
	s_sym_cfg.m_uc_sym_idx = 4;
	pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg), sizeof(struct sym_cfg));
	s_sym_cfg.m_uc_sym_idx = 5;
	pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg), sizeof(struct sym_cfg));
	s_sym_cfg.m_uc_sym_idx = 6;
	pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg), sizeof(struct sym_cfg));
	s_sym_cfg.m_uc_sym_idx = 7;
	pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg), sizeof(struct sym_cfg));

	if (uc_working_band == WB_FCC) {
		if (uc_legacy_mode) {
			atpl250_hold_on_reference(true);
		} else {
			atpl250_hold_on_reference(false);
		}
	} else {
		atpl250_hold_on_reference(false);
	}

	/* Set interleaver to unload mode, so data can be written to modulator after preamble */
	pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x07); /* 07: BPSK, unload mode, tx mode, G3 mode, enable HW chain */

	s_phy_tx_ctl.m_us_tx_pending_symbols = uc_num_symbols_fch;
	feed_modulator_fch(MAX_NUM_SYM_MOD, NO_SYM_OFFSET, CHANGE_MODULATION);
	s_phy_tx_ctl.m_us_tx_pending_symbols -= MAX_NUM_SYM_MOD;
}

/**
 * \brief Transmits last part of Header of a frame
 *
 */
static inline void _tx_header_last(void)
{
	if ((s_phy_tx_ctl.e_delimiter_type == DT_ACK) || (s_phy_tx_ctl.e_delimiter_type == DT_NACK)) {
		/* Change symbol config to set last symbol */
		s_sym_cfg.m_uc_gain = uc_phy_tx_full_gain;
		s_sym_cfg.m_uc_rep1514 = 0;
		s_sym_cfg.m_uc_rep1312 = 0;
		s_sym_cfg.m_uc_rep1110 = 0;
		s_sym_cfg.m_uc_rep98 = 0;
		s_sym_cfg.m_uc_rep76 = 0;
		s_sym_cfg.m_uc_rep54 = 0;
		s_sym_cfg.m_uc_rep32 = 0;
		s_sym_cfg.m_uc_rep10 = 0;
		s_sym_cfg.m_uc_overlap = 8;
		s_sym_cfg.m_uc_cyclicprefix = 30;
		s_sym_cfg.m_uc_is_last_symbol = 1;
		s_sym_cfg.m_uc_repetitions = 0;
		/* Set symbols to transmit */
		/* 4 symbols, Set 4 as last symbol, no change modulation (BPSK) */
		s_sym_cfg.m_uc_sym_idx = (s_phy_tx_ctl.m_us_tx_pending_symbols - 1);
		s_sym_cfg.m_uc_reserved = 1;
		pplc_if_write_buf(REG_ATPL250_TXRXB_GAIN_32, (uint8_t *)(&s_sym_cfg), sizeof(struct sym_cfg));

		feed_modulator_fch(s_phy_tx_ctl.m_us_tx_pending_symbols, NO_SYM_OFFSET, NO_CHANGE_MODULATION);
	} else {
		if (uc_working_band == WB_FCC) {
			if (uc_legacy_mode) {
				/* 4 symbols, no changes in config, no change modulation (BPSK) */
				if (s_phy_tx_ctl.e_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
					feed_modulator_fch(s_phy_tx_ctl.m_us_tx_pending_symbols - 1, NO_SYM_OFFSET, NO_CHANGE_MODULATION);
					atpl250_hold_on_reference(false);
					feed_modulator_fch(1, s_phy_tx_ctl.m_us_tx_pending_symbols - 1, NO_CHANGE_MODULATION);
					/* Number of symbols is left incorrectly configured on _feed_modulator function, set correct value */
					atpl250_set_num_symbols_cfg(s_phy_tx_ctl.m_us_tx_pending_symbols);
				} else {
					feed_modulator_fch(s_phy_tx_ctl.m_us_tx_pending_symbols, NO_SYM_OFFSET, NO_CHANGE_MODULATION);
				}
			} else {
				feed_modulator_fch(s_phy_tx_ctl.m_us_tx_pending_symbols, NO_SYM_OFFSET, NO_CHANGE_MODULATION);
			}
		} else {
			feed_modulator_fch(s_phy_tx_ctl.m_us_tx_pending_symbols, NO_SYM_OFFSET, NO_CHANGE_MODULATION);
		}
	}
}

/**
 * \brief Transmits S1 and S2 symbols
 *
 */
static inline void _coh_tx_s1s2(void)
{
	/* Set number of symbols */
	atpl250_set_num_symbols_cfg(NUM_SYMBOLS_S1S2);

	/* Change value to modulator abcd points (full gain) */
	pplc_if_write_buf(REG_ATPL250_INOUTB_PSK_VALUES1_32, (uint8_t *)cauc_abcd_fullgain, ABCD_POINTS_LEN);

	/* Symbol configuration */
	s_sym_cfg.m_uc_gain = uc_phy_tx_full_gain;
	s_sym_cfg.m_uc_rep1514 = 0;
	s_sym_cfg.m_uc_rep1312 = 0;
	s_sym_cfg.m_uc_rep1110 = 0;
	s_sym_cfg.m_uc_rep98 = 0;
	s_sym_cfg.m_uc_rep76 = 0;
	s_sym_cfg.m_uc_rep54 = 0;
	s_sym_cfg.m_uc_rep32 = 0;
	s_sym_cfg.m_uc_overlap = 8;
	s_sym_cfg.m_uc_cyclicprefix = 30;
	s_sym_cfg.m_uc_is_last_symbol = 0;
	s_sym_cfg.m_uc_repetitions = 0;

	/* S1 is a SYNCM, inverted */
	s_sym_cfg.m_uc_rep10 = 0x04;
	s_sym_cfg.m_uc_sym_idx = 0;
	_config_symbol(&s_sym_cfg);

	/* S2 is a SYNCP, not inverted */
	s_sym_cfg.m_uc_rep10 = 0x00;
	s_sym_cfg.m_uc_sym_idx = 1;
	_config_symbol(&s_sym_cfg);

	/* Remove reference to set P reference on IOB */
	atpl250_hold_on_reference(false);

	atpl250_set_mod_bpsk_truepoint();

	disable_HW_chain();     /* Disable HW chain to write in modulator */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier); /* Avoid overflow to 1st symbol when writing */
	pplc_if_write_jump((BCODE_COH | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, CFG_IOB_SAMPLES_PER_SYMBOL + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 2nd symbol when writing */
	pplc_if_write_jump((BCODE_COH | (CFG_IOB_SAMPLES_PER_SYMBOL + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 2) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 3rd symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 2) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 3) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 4th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 3) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 4) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 5th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 4) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 5) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 6th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 5) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 6) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 7th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 6) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 7) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 7) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	enable_HW_chain();

	atpl250_hold_on_reference(true); /* Payload goes in coherent */
}

/**
 * \brief Initialize fields depending on notching
 *
 */
static void _init_notching(void)
{
	uint8_t uc_i;
	uint8_t uc_byte_index;
	uint8_t uc_bit_index;
	uint8_t uc_p_index;
	uint8_t uc_p_index_new;
	uint8_t uc_masked_index;
	uint8_t uc_carrier_value;
	uint64_t ull_resample_const2_inv;

	/* Get number of used and notched carriers */
	uc_notched_carriers = 0;
	for (uc_i = 0; uc_i < CARR_BUFFER_LEN; uc_i++) {
		uc_notched_carriers += BitsSetTable256[auc_static_notching_pos[uc_i]];
	}
	uc_used_carriers = s_band_constants.uc_num_carriers - uc_notched_carriers;

	/* Calculate number of FCH symbols = (([FCH bits] + 6)*2*6 + [used carriers] - 1) / ([used carriers]) */
	uc_num_symbols_fch = ((s_band_constants.uc_fch_len_bits + 6) * 2 * 6 + uc_used_carriers - 1) / (uc_used_carriers);

	if (uc_working_band == WB_FCC) {
		puc_full_psymbol = auc_full_psymbol_fcc;
	} else if (uc_working_band == WB_ARIB) {
		puc_full_psymbol = auc_full_psymbol_arib;
	} else {
		puc_full_psymbol = auc_full_psymbol_cenelec_a;
	}

	/* Fill P symbol */
	uc_p_index = 0;
	uc_p_index_new = 0;
	uc_masked_index = 0;
	memset(auc_psymbol, 0, P_SYMBOL_LEN_MAX_FCC);
	for (uc_i = s_band_constants.uc_first_carrier; uc_i <= s_band_constants.uc_last_carrier; uc_i++) {
		/* Calculate byte and bit position for index */
		uc_byte_index = uc_i >> 3;
		uc_bit_index = uc_i & 0x07;

		if (!(auc_static_notching_pos[uc_byte_index] & (1 << uc_bit_index))) { /* Used carrier */
			/* List with the indexes of active carriers */
			auc_unmasked_carrier_list[uc_p_index_new] = uc_i - s_band_constants.uc_first_carrier;

			/* Get carrier value from full P symbol */
			if (uc_p_index % 2) {
				/* Low nibble */
				uc_carrier_value = puc_full_psymbol[uc_p_index / 2] & 0x0F;
			} else {
				/* High nibble */
				uc_carrier_value = (puc_full_psymbol[uc_p_index / 2] & 0xF0) >> 4;
			}

			/* Write it on P symbol */
			if (uc_p_index_new % 2) {
				/* Low nibble */
				auc_psymbol[uc_p_index_new / 2] |= uc_carrier_value;
			} else {
				/* High nibble */
				auc_psymbol[uc_p_index_new / 2] |= (uc_carrier_value << 4);
			}

			/* Advance index for new p symbol (only when a value has just been written) */
			uc_p_index_new++;
		} else {
			/* List with the indexes of masked carriers */
			auc_masked_carrier_list[uc_masked_index] = uc_i - s_band_constants.uc_first_carrier;
			uc_masked_index++;
		}

		/* Advance index for full p symbol (in any case) */
		uc_p_index++;
	}

	/* Set last used carrier */
	s_band_constants.uc_last_used_carrier = s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[uc_used_carriers - 1];

	/* Set P symbol length */
	uc_psymbol_len = (uc_used_carriers + 1) / 2;

	/* Calculate variables needed for channel estimation depending on notching */
	/* ceil(s_band_constants.us_fch_interleaver_useful_size / carriers) * carriers */
	us_fch_interleaver_size_with_padding = ((s_band_constants.us_fch_interleaver_useful_size + uc_used_carriers - 1) / uc_used_carriers) * uc_used_carriers;
	/* ceil(FCH_INTERLEAVER_SIZE_WITH_PADDING / 8 bits in byte) */
	uc_fch_interleaver_byte_size = (us_fch_interleaver_size_with_padding + 7) / 8;
	/* ceil(carriers / 8 bits in byte) */
	uc_fch_symbol_byte_size = (uc_used_carriers + 7) / 8;
	uc_psymbol_no_truepoint_byte_len = uc_fch_symbol_byte_size;

	/* Calculate clock resample variable */
	ul_resample_var = 2275L + ((uc_num_symbols_fch - 8) * 278L);
	if (uc_working_band == WB_FCC) {
		ull_resample_const2_inv = (RESAMPLE_CONST_2_FCC * (uint64_t)ul_resample_var) / RESAMPLE_CONST_1;
	} else if (uc_working_band == WB_ARIB) {
		ull_resample_const2_inv = (RESAMPLE_CONST_2_ARIB * (uint64_t)ul_resample_var) / RESAMPLE_CONST_1;
	} else {
		ull_resample_const2_inv = (RESAMPLE_CONST_2_CENELEC_A * (uint64_t)ul_resample_var) / RESAMPLE_CONST_1;
	}

	ul_resample_const2_inv = (uint32_t)ull_resample_const2_inv;

	/*Calculate vectors used for SFO estimation*/
	/*Vector of frequency indexes and of the squared frequency indexes in Q1.31 scaled by 1/2^SCALING_FREQ_VALUES*/
	for (uc_i = 0; uc_i < uc_used_carriers; uc_i++) {
		asl_freq_index[uc_i] = (VALUE_1_2_Q_1_31 >> (SCALING_FREQ_VALUES - 1)) * ((int32_t)(auc_unmasked_carrier_list[uc_i] + s_band_constants.uc_first_carrier));
		asl_freq_index_squared[uc_i]
			= (((VALUE_1_2_Q_1_31 >>
				(SCALING_FREQ_VALUES -
				1)) *
				((int32_t)(auc_unmasked_carrier_list[uc_i] +
				s_band_constants.uc_first_carrier))) >> SCALING_FREQ_VALUES) * ((int32_t)(auc_unmasked_carrier_list[uc_i] + s_band_constants.uc_first_carrier));
	}
	/*Vector of starting sample of the symbols. Q1.31 scaled by 1/2^SCALING_DELAY_VALUES*/
	for (uc_i = 0; uc_i < (NUM_FULL_SYMBOLS_PREAMBLE - 1); uc_i++) {
		asl_delay_symbols[uc_i] = (((int32_t)FFT_POINTS) * ((int32_t)uc_i)) * (VALUE_1_2_Q_1_31 >> (LOG2_FFT_POINTS + SCALING_DELAY_VALUES - 1));
	}
	asl_delay_symbols[NUM_FULL_SYMBOLS_PREAMBLE -
	1]
		= (((int32_t)FFT_POINTS) *
			((int32_t)(NUM_FULL_SYMBOLS_PREAMBLE - 1)) + ((int32_t)OFFSET_M)) * (VALUE_1_2_Q_1_31 >> (LOG2_FFT_POINTS + SCALING_DELAY_VALUES - 1));
	for (uc_i = 0; uc_i < (SYMBOLS_8 + 2); uc_i++) {
		asl_delay_symbols[NUM_FULL_SYMBOLS_PREAMBLE +
		uc_i]
			= (FIRST_SAMPLE_FCH +
				(int32_t)(CP_LENGTH - WINDOW_MS1S2 -
				TIME_ADVANCE)) *
				(VALUE_1_2_Q_1_31 >>
				(LOG2_FFT_POINTS + SCALING_DELAY_VALUES -
				1)) + (SYMBOL_LENGTH_SAMPLES >> (SCALING_DELAY_VALUES - 1)) * ((int32_t)(uc_num_symbols_fch - SYMBOLS_8 + uc_i));
	}

	/* Update variables used by PAYLOAD_MODULATION() assembler function */
	_fill_static_notching(&auc_static_notching_pos[0]);
}

/**
 * \brief Initialize PHY layer variables and structures
 *
 */
static void _init_phy_layer(void)
{
	/*const uint8_t auc_empty_map[CARR_BUFFER_LEN] = {0};*/
	uint8_t auc_pilot_empty[NUM_CARRIERS_FCC] = {0};

	/* Initialize variables depending on Notching */
	_init_notching();
	atpl250.m_uc_fch_symbols = uc_num_symbols_fch;

	/* Init Phy internal values */
	s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_NO_TX;
	s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;
	s_phy_tx_ctl.m_uc_payload_carriers = uc_used_carriers;
	s_phy_tx_ctl.m_ul_start_tx_watch_ms = 0;
	s_phy_rx_ctl.m_ul_start_rx_watch_ms = 0;
	s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
	s_phy_rx_ctl.m_us_rx_len = 0;
	s_phy_rx_ctl.m_uc_payload_carriers = uc_used_carriers;
	ul_phy_int_flags = 0;
	uc_phy_generic_flags = 0;
	e_ber_status = BER_FCH;

	/* Set previous preemphasis to an impossible value to force setting on first transmission (needed for notching) */
	memset(auc_prev_preemphasis, 0xFF, sizeof(auc_prev_preemphasis));
	/* Reset Stats */
	if (uc_hard_reset_flag) {
		atpl250.m_ul_tx_total = 0;
		atpl250.m_ul_tx_total_bytes = 0;
		atpl250.m_ul_tx_total_errors = 0;
		atpl250.m_ul_tx_bad_busy_tx = 0;
		atpl250.m_ul_tx_bad_busy_channel = 0;
		atpl250.m_ul_tx_bad_len = 0;
		atpl250.m_ul_tx_bad_format = 0;
		atpl250.m_ul_tx_timeout = 0;
		atpl250.m_ul_rx_total = 0;
		atpl250.m_ul_rx_total_bytes = 0;
		atpl250.m_ul_rx_exception = 0;
		atpl250.m_ul_rx_bad_len = 0;
		atpl250.m_ul_rx_bad_crc_fch = 0;
		atpl250.m_ul_rx_false_positive = 0;
		atpl250.m_ul_rx_bad_format = 0;
		atpl250.m_ul_time_freeline = 0;
		atpl250.m_ul_time_busyline = 0;
#ifdef CONF_ENABLE_C11_CFG
		atpl250.m_uc_auto_detect_impedance = FIXED_STATE_FIXED_GAIN;
		atpl250.m_uc_impedance_state = VLO_STATE;
#else
		atpl250.m_uc_auto_detect_impedance = AUTO_STATE_VAR_GAIN;
		atpl250.m_uc_impedance_state = HI_STATE;
#endif
		atpl250.m_uc_rrc_notch_active = 0;
		atpl250.m_uc_rrc_notch_index = 0;
		atpl250.m_uc_plc_disable = 0;
		atpl250.m_uc_trigger_signal_dump = 0;
		atpl250.m_ul_time_between_noise_captures = NOISE_CAPTURE_PERIOD;
		ul_noise_capture_timer = atpl250.m_ul_time_between_noise_captures;
		atpl250.m_uc_noise_peak_power = 0;
		atpl250.m_uc_last_msg_lqi = 0;
#ifndef DISABLE_NOISE_CAPTURE_AT_STARTUP
		if (uc_working_band == WB_CENELEC_A) {
			atpl250.m_uc_enable_auto_noise_capture = 1;
			uc_phy_generic_flags |= PHY_GENERIC_FLAG_NOISE_CAPTURE;
		}
#endif
		atpl250.m_uc_delay_noise_capture_after_rx = 1;
		atpl250.m_ul_last_rmscalc_corrected = 0x00000000;
		uc_hard_reset_flag = 0x00;
	}

	uc_pdc_ratio = 0; /* Invalid value, it will be used in order to detect if it has been initialized */

	/* Configure Jump RAM */
	init_jump_ram((uint8_t *)auc_static_notching_pos); /* Static notching as parameter is equivalent to configure jumps for a full tone map */

	/* HW Registers Initialization */
	atpl250_hw_init(atpl250.m_uc_impedance_state);

	/* Configure Jump RAM */
	init_jump_ram((uint8_t *)auc_static_notching_pos); /* Static notching as parameter is equivalent to configure jumps for a full tone map */

	/* Calculates the list of carriers to be smoothed and with unnotched predecessor and sucessor (for invert_channel_asm) */
	uc_num_carr_smooth_preamble_fch = 0;
	control_smooth_and_order_pilot_list(auc_static_notching_pos, auc_pilot_empty, auc_smooth_control_preamble_fch,
			&uc_num_carr_smooth_preamble_fch, auc_pilot_empty, auc_control_avg_invert_chan, 0, 0);

	/* Configures the resampling register */
	pplc_if_write32(REG_ATPL250_RESAMP24BITS_1_32, RESAMPLE_STEP);
	pplc_if_write32(REG_ATPL250_RESAMP24BITS_2_32, RESAMPLING_24BITS_2_VALUE);

	/* Set P symbol phases as reference for later coherent demodulation */
	set_p_symbol_phases(auc_psymbol, COH_REF_ALL_SYMBOLS);

	/* Prepare for Rx */
	prepare_next_rx();

	atpl250_set_mod_bpsk_truepoint();

	disable_HW_chain();     /* Disable HW chain to write in modulator */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier); /* Avoid overflow to 1st symbol when writing */
	pplc_if_write_jump((BCODE_COH | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, CFG_IOB_SAMPLES_PER_SYMBOL + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 2nd symbol when writing */
	pplc_if_write_jump((BCODE_COH | (CFG_IOB_SAMPLES_PER_SYMBOL + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 2) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 3rd symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 2) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 3) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 4th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 3) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 4) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 5th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 4) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 5) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 6th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 5) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 6) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 7th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 6) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 7) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 7) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_2);
	atpl250_set_mod_bpsk();

	/* Dump log spi */
#if (LOG_SPI == 1)
	dumpLogSpi();
#endif
}

/**
 * \brief Function to trigger Noise Capture
 *
 */
static void _trigger_noise_capture(void)
{
#ifdef NOISE_CAPTURE_USING_FALSE_PEAK
	/* Init control variable */
	uc_curr_intermediate_pass = 0;
	/* Change FFT_SHIFT to avoid saturation */
	pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, NOISE_CAPTURE_FFT_SHIFT);
	/* Config IOB to interrupt after NOISE_CAPTURE_SYMBOLS symbols */
	atpl250_set_num_symbols_cfg(s_band_constants.uc_noise_capture_adapt_symbols);
	/* Disable possible configured notch filters */
	atpl250_enable_rrc_notch_filter(0);
	/* Set Rx step */
	s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_ADAPT;
	/* Get reference time */
	s_phy_rx_ctl.m_ul_start_rx_watch_ms = oss_get_up_time_ms();
	/* Force false peak */
	atpl250_force_peak();
	/* Set Path FFT -> IOB */
	atpl250_set_iobuf_to_fft();
#else
	uint32_t ul_curr_time;

	/* Init control variable */
	uc_curr_intermediate_pass = 0;
	/* Read current time */
	ul_curr_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
	/* Set capture time */
	if (uc_flag_correct_delta) {
		uc_flag_correct_delta = 0;
		pplc_if_write32(REG_ATPL250_TX_NOISE_TIME_32, ul_curr_time + NOISE_CAPTURE_DELTA_CORRECTED);
	} else {
		pplc_if_write32(REG_ATPL250_TX_NOISE_TIME_32, ul_curr_time + NOISE_CAPTURE_DELTA);
	}

	/* Activate Noise measure */
	pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x10);
#endif
}

/**
 * \brief Function to handle Noise Capture
 *
 */
static void _analyse_noise_capture(void)
{
	uint8_t uc_i, uc_i_filter_applied;
	uint8_t uc_idx, uc_idx_max, uc_idx_min;
	uint8_t uc_max_noise, uc_selected_carrier_to_notch;
	uint8_t uc_perform_third_derivative = 1;
	uint16_t us_sum_noise_db, us_avg_noise_db;
	uint16_t *puc_noise_per_carrier;
	uint32_t ul_int_flags;
	uint32_t ul_aux_squared_module;
	uint32_t ul_agc_factor_squared;
	uint32_t ul_rssi_sifted_by_agc;

#ifdef NOISE_CAPTURE_USING_FALSE_PEAK
	if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_SECOND_PASS) {
		atpl250_rotator_and_fft_push_rst();
		atpl250_clear_iobuf_to_fft();

		/* Look for chirp */
		pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x01);

		/* Release resets */
		atpl250_rotator_and_fft_release_rst();
	}

	/* Set overflow to max value */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, 0x03FF);

	/* Get power per carrier */
	if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_FIRST_PASS) {
		/* Clear avg */
		memset(u_shared_buffers.s_noise.ass_noise_avg, 0, sizeof(u_shared_buffers.s_noise.ass_noise_avg));
	}

	/* Read raw info in symbols */
	for (uc_i = 0; uc_i < SYMBOLS_8; uc_i++) {
		if (uc_i == 0) {
			pplc_if_read_buf((BCODE_ZONE4 | ((uc_i * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier - (EXTRA_CARRIERS_TO_READ_NOISE >> 1))),
					(uint8_t *)(u_shared_buffers.s_noise.ass_noise_capture), (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE) << 2, true);
		} else {
			pplc_if_do_read((uint8_t *)(u_shared_buffers.s_noise.ass_noise_capture), (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE) << 2);
		}

		/* Invert byte order */
		swap_bytes_asm(u_shared_buffers.s_noise.ass_noise_capture, (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE));
		/* Get squared magnitude */
		arm_cmplx_mag_squared_q15(u_shared_buffers.s_noise.ass_noise_capture, u_shared_buffers.s_noise.ass_noise_squared_mag,
				s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
		if (uc_i < (SYMBOLS_8 - 1)) {
			pplc_if_read_buf((BCODE_ZONE4 | (((uc_i + 1) * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier - (EXTRA_CARRIERS_TO_READ_NOISE >> 1))),
					(uint8_t *)(u_shared_buffers.s_noise.ass_noise_capture), (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE) << 2,
					false);
		}

		if (uc_working_band == WB_CENELEC_A) {
			/* Shift and average. Shift -4, because the final add will be 8 + 8 symbols */
			arm_shift_q15(u_shared_buffers.s_noise.ass_noise_squared_mag, -4, u_shared_buffers.s_noise.ass_noise_squared_mag,
					s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
			arm_add_q15(u_shared_buffers.s_noise.ass_noise_squared_mag, u_shared_buffers.s_noise.ass_noise_avg,
					u_shared_buffers.s_noise.ass_noise_avg, s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
		} else {
			/* Shift and average. Final add will be 48 symbols, so we do an approximation */

			/*
			 * aux = captured
			 * aux = aux + (captured >> 2)
			 * aux = aux + (captured >> 4)
			 * aux = aux >> 6
			 * avg = avg + aux
			 */
			memcpy(u_shared_buffers.s_noise.ass_noise_aux_sum, u_shared_buffers.s_noise.ass_noise_squared_mag,
					s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
			arm_shift_q15(u_shared_buffers.s_noise.ass_noise_squared_mag, -2, u_shared_buffers.s_noise.ass_noise_aux_shift,
					s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
			arm_add_q15(u_shared_buffers.s_noise.ass_noise_aux_sum, u_shared_buffers.s_noise.ass_noise_aux_shift,
					u_shared_buffers.s_noise.ass_noise_aux_sum, s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
			arm_shift_q15(u_shared_buffers.s_noise.ass_noise_squared_mag, -4, u_shared_buffers.s_noise.ass_noise_aux_shift,
					s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
			arm_add_q15(u_shared_buffers.s_noise.ass_noise_aux_sum, u_shared_buffers.s_noise.ass_noise_aux_shift,
					u_shared_buffers.s_noise.ass_noise_aux_sum, s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);

			/* Instead of shifting 6 here, to save some precision we shift 4. Later a shift of 16 in the other direction
			 * is set to 14 to compensate this */
			arm_shift_q15(u_shared_buffers.s_noise.ass_noise_aux_sum, -4, u_shared_buffers.s_noise.ass_noise_aux_sum,
					s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
			arm_add_q15(u_shared_buffers.s_noise.ass_noise_aux_sum, u_shared_buffers.s_noise.ass_noise_avg,
					u_shared_buffers.s_noise.ass_noise_avg, s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
		}
	}

	if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_FIRST_PASS) {
		/* Set Rx step */
		if (uc_working_band == WB_CENELEC_A) {
			s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_SECOND_PASS;
		} else {
			s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_INTERMEDIATE_PASS;
		}
	} else if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_INTERMEDIATE_PASS) {
		/* Increase pass and change step if necessary */
		uc_curr_intermediate_pass++;
		if (uc_curr_intermediate_pass == NUM_INTERMEDIATE_PASSES) {
			s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_SECOND_PASS;
		}
	} else { /* STEP_RX_NOISE_CAPTURE_SECOND_PASS */
		 /* Transform values to dB */
		ul_agc_factor_squared = (uint32_t)((us_noise_agc_factor * us_noise_agc_factor)) >> 12;
		us_sum_noise_db = 0;
		for (uc_i = 0; uc_i < (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE); uc_i++) {
			ul_aux_squared_module = (uint32_t)(u_shared_buffers.s_noise.ass_noise_avg[uc_i]);

			if (ul_aux_squared_module > 0) {
				ul_rssi_sifted_by_agc = (ul_aux_squared_module << 14) / ul_agc_factor_squared;
				/* Perform a search in table to find dB value */
				uc_idx_min = 0;
				uc_idx_max = RSSI_VALUES_TABLE_SIZE;
				while (uc_idx_min != uc_idx_max) {
					uc_idx = (uc_idx_max + uc_idx_min) / 2;
					if (ul_rssi_sifted_by_agc < caul_rssi_values[uc_idx]) {
						uc_idx_max = uc_idx;
					} else {
						uc_idx_min = uc_idx + 1;
					}
				}
				u_shared_buffers.s_noise.ass_noise_avg[uc_i] = uc_idx_max;
			} else {
				u_shared_buffers.s_noise.ass_noise_avg[uc_i] = 0;
			}

			/* Sum values for carriers in band to get avg later */
			if ((uc_i > (EXTRA_CARRIERS_TO_READ_NOISE >> 1)) && (uc_i <= (s_band_constants.uc_num_carriers + (EXTRA_CARRIERS_TO_READ_NOISE >> 1)))) {
				us_sum_noise_db += u_shared_buffers.s_noise.ass_noise_avg[uc_i];
			}
		}

		/* Get average value */
		us_avg_noise_db = us_sum_noise_db / s_band_constants.uc_num_carriers;

		/* Calculate noise derivative and look for noised tones */
		/* Derivative 2 */
		puc_noise_per_carrier = (uint16_t *)(&u_shared_buffers.s_noise.ass_noise_avg[2]);
		uc_max_noise = 0;
		uc_i_filter_applied = 0xFF;
		for (uc_i = 0; uc_i < (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE - 4); uc_i++) {
			/* Derivative */
			if ((*(puc_noise_per_carrier + uc_i) > *(puc_noise_per_carrier + uc_i - 2)) &&
					(*(puc_noise_per_carrier + uc_i) > *(puc_noise_per_carrier + uc_i + 2)) &&
					((*(puc_noise_per_carrier + uc_i) - *(puc_noise_per_carrier + uc_i - 2)) > MIN_NOISE_DERIVATIVE_2_DB) &&
					((*(puc_noise_per_carrier + uc_i) - *(puc_noise_per_carrier + uc_i + 2)) > MIN_NOISE_DERIVATIVE_2_DB)) {
				/* Difference with average */
				if ((*(puc_noise_per_carrier + uc_i) > us_avg_noise_db) &&
						((*(puc_noise_per_carrier + uc_i) - us_avg_noise_db) > MIN_NOISE_AVG_DIF_DB)) {
					/* Carrier noise */
					if (*(puc_noise_per_carrier + uc_i) > s_band_constants.uc_min_noise_notch_db) {
						/* Noised carrier, if it has the maximum noise value, select it to be notched */
						if (*(puc_noise_per_carrier + uc_i) > uc_max_noise) {
							uc_max_noise = *(puc_noise_per_carrier + uc_i);
							uc_selected_carrier_to_notch = s_band_constants.uc_first_carrier + uc_i - (EXTRA_CARRIERS_TO_READ_NOISE >> 1) + 2;
							uc_i_filter_applied = uc_i;
							LOG_PHY(("C2 %u %udB\r\n", uc_selected_carrier_to_notch,
									*(puc_noise_per_carrier + uc_i) + s_band_constants.uc_noise_correction_factor_db));
						}
					}
				}
			}
		}
		if (uc_max_noise) {
			/* Check if noise in next carrier is near the max, to apply filter on that carrier */
			if (*(puc_noise_per_carrier + uc_i_filter_applied + 1) > (uc_max_noise - MIN_DIFF_PREV_CARRIER_DB)) {
				uc_selected_carrier_to_notch = s_band_constants.uc_first_carrier + uc_i_filter_applied + 1 - (EXTRA_CARRIERS_TO_READ_NOISE >> 1) + 2;
				uc_perform_third_derivative = 0;
				LOG_PHY(("C2 LESS %u %udB\r\n", uc_selected_carrier_to_notch,
						*(puc_noise_per_carrier + uc_i) + s_band_constants.uc_noise_correction_factor_db));
			}
		}

		if (uc_perform_third_derivative) {
			/* Derivative 3. Look for noises greater than in derivative 2 */
			puc_noise_per_carrier = (uint16_t *)(&u_shared_buffers.s_noise.ass_noise_avg[3]);
			for (uc_i = 0; uc_i < (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE - 3); uc_i++) {
				/* Derivative */
				if ((*(puc_noise_per_carrier + uc_i) > *(puc_noise_per_carrier + uc_i - 3)) &&
						(*(puc_noise_per_carrier + uc_i) > *(puc_noise_per_carrier + uc_i + 3)) &&
						((*(puc_noise_per_carrier + uc_i) - *(puc_noise_per_carrier + uc_i - 3)) > MIN_NOISE_DERIVATIVE_3_DB) &&
						((*(puc_noise_per_carrier + uc_i) - *(puc_noise_per_carrier + uc_i + 3)) > MIN_NOISE_DERIVATIVE_3_DB)) {
					/* Difference with average */
					if ((*(puc_noise_per_carrier + uc_i) > us_avg_noise_db) &&
							((*(puc_noise_per_carrier + uc_i) - us_avg_noise_db) > MIN_NOISE_AVG_DIF_DB)) {
						/* Carrier noise */
						if (*(puc_noise_per_carrier + uc_i) > s_band_constants.uc_min_noise_notch_db) {
							/* Noised carrier, if it has the maximum noise value, select it to be notched */
							if (*(puc_noise_per_carrier + uc_i) > uc_max_noise) {
								uc_max_noise = *(puc_noise_per_carrier + uc_i);
								uc_selected_carrier_to_notch = s_band_constants.uc_first_carrier + uc_i - (EXTRA_CARRIERS_TO_READ_NOISE >> 1) + 3;
								LOG_PHY(("C3 %u %udB\r\n", uc_selected_carrier_to_notch,
										*(puc_noise_per_carrier + uc_i) + s_band_constants.uc_noise_correction_factor_db));
							}
						}
					}
				}
			}
		}

		/* If Peak1 has been detected, calculated filter is not valid */
		ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
		if (!(ul_int_flags & INT_PEAK1_MASK_32)) {
			/* If noise has been detected, apply filter and reset TXRXB */
			if (uc_max_noise) {
				atpl250.m_uc_rrc_notch_active = 1;
				atpl250.m_uc_rrc_notch_index = uc_selected_carrier_to_notch;
				for (uc_i = 0; uc_i <= FILTER_CONFIG_NUM_STEPS; uc_i++) {
					/* <= because last step is for writing to HW */
					atpl250_set_rrc_notch_filter(uc_selected_carrier_to_notch, uc_i);
					ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
					if ((ul_int_flags & INT_PEAK1_MASK_32)) {
						/* Abort */
						s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
						return;
					}
				}
				atpl250_sync_pream_txrxb_push_rst();
				ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
				if ((ul_int_flags & INT_PEAK1_MASK_32)) {
					/* Reset FFT IOB Rotator and SyncM */
					atpl250_iob_rotator_and_fft_push_rst();
					atpl250_syncm_detector_push_rst();
					atpl250_syncm_detector_release_rst();
					/* Clear interrupts */
					ul_phy_int_flags &= (~INT_PEAK1_MASK_32);
					atpl250_clear_peak1_int();
					/* SyncM reset raises a No Peak2 int */
					atpl250_clear_no_peak2_int();
					/* Look for chirp again */
					pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x01);
					/* Release all resets */
					atpl250_iob_rotator_and_fft_release_rst();
				}

				/* Apply filter */
				pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x80);  /* SRST_DEC=1 */
				atpl250_enable_rrc_notch_filter(1);
				atpl250_sync_pream_txrxb_release_rst();
				pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);  /* SRST_DEC=0 */
			} else {
				atpl250.m_uc_rrc_notch_active = 0;
				atpl250.m_uc_rrc_notch_index = 0;
			}
		}

		/* Set Rx step */
		s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;

		/* Reset IOB and FFT, unless peak already detected */
		ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
		if (!(ul_int_flags & INT_PEAK1_MASK_32)) {
			atpl250_iob_push_rst();
			atpl250_iob_release_rst();
		}
	}

	/* Clear Rx full */
	uc_clear_rx_full = true;

#else
	if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_FIRST_PASS) {
		atpl250_clear_iobuf_to_fft();
		atpl250_clear_rx_full();
		/* Check Peak1 */
		ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
		if (ul_int_flags & INT_PEAK1_MASK_32) {
			/* Restore FFT_SHIFT value */
			pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, s_band_constants.uc_rx_fft_shift);
			/* Clear Rx Full */
			atpl250_clear_rx_full();
			/* Set Rx step */
			s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
			/* Return to receive frame */
			return;
		}
	} else if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_INTERMEDIATE_PASS) {
		atpl250_clear_iobuf_to_fft();
		atpl250_clear_rx_full();
	} else if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_SECOND_PASS) {
		atpl250_rotator_and_fft_push_rst();
		atpl250_clear_iobuf_to_fft();
		atpl250_clear_rx_full();

		/* Restore FFT_SHIFT value */
		pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, s_band_constants.uc_rx_fft_shift);

		/* Just in case rx error has raised at the same time as IOB, clear it */
		ul_phy_int_flags &= (~INT_RX_ERROR_MASK_32);
		atpl250_clear_rx_error_int();

		/* Look for chirp (this will abort noise capture and Noise Error will be generated) */
		pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x01);

		ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
		if (ul_int_flags & INT_PEAK1_MASK_32) {
			/* Clear peak1 and Noise Error interrupts and return */
			ul_phy_int_flags &= (~INT_PEAK1_MASK_32);
			ul_phy_int_flags &= (~INT_NOISE_ERROR_MASK_32);
			atpl250_clear_peak1_int();
			atpl250_clear_noise_error_int();
			atpl250_iob_push_rst();
			atpl250_syncm_detector_push_rst();
			atpl250_syncm_detector_release_rst();
			/* SyncM reset raises a No Peak2 int */
			atpl250_clear_no_peak2_int();
			/* Look for chirp again */
			pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x01);
			/* Release all resets */
			atpl250_iob_rotator_and_fft_release_rst();
			/* Set Rx step */
			s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
			return;
		}

		/* Release resets */
		atpl250_rotator_and_fft_release_rst();
	}

	/* Set overflow to max value */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, 0x03FF);

	/* Get power per carrier */
	if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_FIRST_PASS) {
		/* Clear avg */
		memset(u_shared_buffers.s_noise.ass_noise_avg, 0, sizeof(u_shared_buffers.s_noise.ass_noise_avg));
	}

	/* Read raw info in symbols */
	for (uc_i = 0; uc_i < SYMBOLS_8; uc_i++) {
		if (uc_i == 0) {
			pplc_if_read_buf((BCODE_ZONE4 | ((uc_i * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier - (EXTRA_CARRIERS_TO_READ_NOISE >> 1))),
					(uint8_t *)(u_shared_buffers.s_noise.ass_noise_capture), (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE) << 2, true);
		} else {
			pplc_if_do_read((uint8_t *)(u_shared_buffers.s_noise.ass_noise_capture), (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE) << 2);
		}

		/* Invert byte order */
		swap_bytes_asm(u_shared_buffers.s_noise.ass_noise_capture, (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE));
		/* Get squared magnitude */
		arm_cmplx_mag_squared_q15(u_shared_buffers.s_noise.ass_noise_capture, u_shared_buffers.s_noise.ass_noise_squared_mag,
				s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
		if (uc_i < (SYMBOLS_8 - 1)) {
			pplc_if_read_buf((BCODE_ZONE4 | (((uc_i + 1) * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_first_carrier - (EXTRA_CARRIERS_TO_READ_NOISE >> 1))),
					(uint8_t *)(u_shared_buffers.s_noise.ass_noise_capture), (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE) << 2,
					false);
		}

		if (uc_working_band == WB_CENELEC_A) {
			/* Shift and average. Shift -4, because the final add will be 8 + 8 symbols */
			arm_shift_q15(u_shared_buffers.s_noise.ass_noise_squared_mag, -4, u_shared_buffers.s_noise.ass_noise_squared_mag,
					s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
			arm_add_q15(u_shared_buffers.s_noise.ass_noise_squared_mag, u_shared_buffers.s_noise.ass_noise_avg,
					u_shared_buffers.s_noise.ass_noise_avg, s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
		} else {
			/* Shift and average. Final add will be 48 symbols, so we do an approximation */

			/*
			 * aux = captured
			 * aux = aux + (captured >> 2)
			 * aux = aux + (captured >> 4)
			 * aux = aux >> 6
			 * avg = avg + aux
			 */
			memcpy(u_shared_buffers.s_noise.ass_noise_aux_sum, u_shared_buffers.s_noise.ass_noise_squared_mag,
					s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
			arm_shift_q15(u_shared_buffers.s_noise.ass_noise_squared_mag, -2, u_shared_buffers.s_noise.ass_noise_aux_shift,
					s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
			arm_add_q15(u_shared_buffers.s_noise.ass_noise_aux_sum, u_shared_buffers.s_noise.ass_noise_aux_shift,
					u_shared_buffers.s_noise.ass_noise_aux_sum, s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
			arm_shift_q15(u_shared_buffers.s_noise.ass_noise_squared_mag, -4, u_shared_buffers.s_noise.ass_noise_aux_shift,
					s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
			arm_add_q15(u_shared_buffers.s_noise.ass_noise_aux_sum, u_shared_buffers.s_noise.ass_noise_aux_shift,
					u_shared_buffers.s_noise.ass_noise_aux_sum, s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);

			/* Instead of shifting 6 here, to save some precision we shift 4. Later a shift of 16 in the other direction
			 * is set to 14 to compensate this */
			arm_shift_q15(u_shared_buffers.s_noise.ass_noise_aux_sum, -4, u_shared_buffers.s_noise.ass_noise_aux_sum,
					s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
			arm_add_q15(u_shared_buffers.s_noise.ass_noise_aux_sum, u_shared_buffers.s_noise.ass_noise_avg,
					u_shared_buffers.s_noise.ass_noise_avg, s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE);
		}

		/* Check Peak1 */
		ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
		if (ul_int_flags & INT_PEAK1_MASK_32) {
			/* Set Rx step */
			s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
			/* As we have aborted noise capture, a noise error has been generated */
			/* Clear sw flag and interrupt flag */
			ul_phy_int_flags &= ~INT_NOISE_ERROR_MASK_32;
			atpl250_clear_noise_error_int();
			/* Exit */
			return;
		}
	}

	if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_FIRST_PASS) {
		/* Set path IOB -> FFT */
		atpl250_set_iobuf_to_fft();
		/* Clear Rx Full */
		atpl250_clear_rx_full();
		/* Set Rx step */
		if (uc_working_band == WB_CENELEC_A) {
			s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_SECOND_PASS;
		} else {
			s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_INTERMEDIATE_PASS;
		}
	} else if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_INTERMEDIATE_PASS) {
		/* Increase pass and change step if necessary */
		uc_curr_intermediate_pass++;
		if (uc_curr_intermediate_pass == NUM_INTERMEDIATE_PASSES) {
			s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_SECOND_PASS;
		}

		/* Set path IOB -> FFT */
		atpl250_set_iobuf_to_fft();
		/* Clear Rx Full */
		atpl250_clear_rx_full();
	} else { /* STEP_RX_NOISE_CAPTURE_SECOND_PASS */
		 /* Transform values to dB */
		ul_agc_factor_squared = (uint32_t)((us_noise_agc_factor * us_noise_agc_factor)) >> 12;
		us_sum_noise_db = 0;
		for (uc_i = 0; uc_i < (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE); uc_i++) {
			ul_aux_squared_module = (uint32_t)(u_shared_buffers.s_noise.ass_noise_avg[uc_i]);

			if (ul_aux_squared_module > 0) {
				ul_rssi_sifted_by_agc = (ul_aux_squared_module << 14) / ul_agc_factor_squared;
				/* Perform a search in table to find dB value */
				uc_idx_min = 0;
				uc_idx_max = RSSI_VALUES_TABLE_SIZE;
				while (uc_idx_min != uc_idx_max) {
					uc_idx = (uc_idx_max + uc_idx_min) / 2;
					if (ul_rssi_sifted_by_agc < caul_rssi_values[uc_idx]) {
						uc_idx_max = uc_idx;
					} else {
						uc_idx_min = uc_idx + 1;
					}
				}
				u_shared_buffers.s_noise.ass_noise_avg[uc_i] = uc_idx;
			} else {
				u_shared_buffers.s_noise.ass_noise_avg[uc_i] = 0;
			}

			/* Sum values for carriers in band to get avg later */
			if ((uc_i > (EXTRA_CARRIERS_TO_READ_NOISE >> 1)) && (uc_i <= (s_band_constants.uc_num_carriers + (EXTRA_CARRIERS_TO_READ_NOISE >> 1)))) {
				us_sum_noise_db += u_shared_buffers.s_noise.ass_noise_avg[uc_i];
			}
		}

		/* Get average value */
		us_avg_noise_db = us_sum_noise_db / s_band_constants.uc_num_carriers;
		atpl250.m_uc_noise_peak_power = us_avg_noise_db + s_band_constants.uc_noise_correction_factor_db;

		/* Calculate noise derivative and look for noised tones */
		/* Derivative 2 */
		puc_noise_per_carrier = (uint16_t *)(&u_shared_buffers.s_noise.ass_noise_avg[2]);
		uc_max_noise = 0;
		uc_i_filter_applied = 0xFF;
		for (uc_i = 0; uc_i < (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE - 4); uc_i++) {
			/* Derivative */
			if ((*(puc_noise_per_carrier + uc_i) > *(puc_noise_per_carrier + uc_i - 2)) &&
					(*(puc_noise_per_carrier + uc_i) > *(puc_noise_per_carrier + uc_i + 2)) &&
					((*(puc_noise_per_carrier + uc_i) - *(puc_noise_per_carrier + uc_i - 2)) > MIN_NOISE_DERIVATIVE_2_DB) &&
					((*(puc_noise_per_carrier + uc_i) - *(puc_noise_per_carrier + uc_i + 2)) > MIN_NOISE_DERIVATIVE_2_DB)) {
				/* Difference with average */
				if ((*(puc_noise_per_carrier + uc_i) > us_avg_noise_db) &&
						((*(puc_noise_per_carrier + uc_i) - us_avg_noise_db) > MIN_NOISE_AVG_DIF_DB)) {
					/* Carrier noise */
					if (*(puc_noise_per_carrier + uc_i) > s_band_constants.uc_min_noise_notch_db) {
						/* Noised carrier, if it has the maximum noise value, select it to be notched */
						if (*(puc_noise_per_carrier + uc_i) > uc_max_noise) {
							uc_max_noise = *(puc_noise_per_carrier + uc_i);
							uc_selected_carrier_to_notch = s_band_constants.uc_first_carrier + uc_i - (EXTRA_CARRIERS_TO_READ_NOISE >> 1) + 2;
							uc_i_filter_applied = uc_i;
							atpl250.m_uc_noise_peak_power = *(puc_noise_per_carrier + uc_i) + s_band_constants.uc_noise_correction_factor_db;
							LOG_PHY(("C2 %u %udB\r\n", uc_selected_carrier_to_notch,
									*(puc_noise_per_carrier + uc_i) + s_band_constants.uc_noise_correction_factor_db));
						}
					}
				}
			}
		}
		if (uc_max_noise) {
			/* Check if noise in next carrier is near the max, to apply filter on that carrier */
			if (*(puc_noise_per_carrier + uc_i_filter_applied + 1) > (uc_max_noise - MIN_DIFF_PREV_CARRIER_DB)) {
				uc_selected_carrier_to_notch = s_band_constants.uc_first_carrier + uc_i_filter_applied + 1 - (EXTRA_CARRIERS_TO_READ_NOISE >> 1) + 2;
				uc_perform_third_derivative = 0;
				LOG_PHY(("C2 LESS %u %udB\r\n", uc_selected_carrier_to_notch,
						*(puc_noise_per_carrier + uc_i) + s_band_constants.uc_noise_correction_factor_db));
			}
		}

		if (uc_perform_third_derivative) {
			/* Derivative 3. Look for noises greater than in derivative 2 */
			puc_noise_per_carrier = (uint16_t *)(&u_shared_buffers.s_noise.ass_noise_avg[3]);
			for (uc_i = 0; uc_i < (s_band_constants.uc_num_carriers + EXTRA_CARRIERS_TO_READ_NOISE - 3); uc_i++) {
				/* Derivative */
				if ((*(puc_noise_per_carrier + uc_i) > *(puc_noise_per_carrier + uc_i - 3)) &&
						(*(puc_noise_per_carrier + uc_i) > *(puc_noise_per_carrier + uc_i + 3)) &&
						((*(puc_noise_per_carrier + uc_i) - *(puc_noise_per_carrier + uc_i - 3)) > MIN_NOISE_DERIVATIVE_3_DB) &&
						((*(puc_noise_per_carrier + uc_i) - *(puc_noise_per_carrier + uc_i + 3)) > MIN_NOISE_DERIVATIVE_3_DB)) {
					/* Difference with average */
					if ((*(puc_noise_per_carrier + uc_i) > us_avg_noise_db) &&
							((*(puc_noise_per_carrier + uc_i) - us_avg_noise_db) > MIN_NOISE_AVG_DIF_DB)) {
						/* Carrier noise */
						if (*(puc_noise_per_carrier + uc_i) > s_band_constants.uc_min_noise_notch_db) {
							/* Noised carrier, if it has the maximum noise value, select it to be notched */
							if (*(puc_noise_per_carrier + uc_i) > uc_max_noise) {
								uc_max_noise = *(puc_noise_per_carrier + uc_i);
								uc_selected_carrier_to_notch = s_band_constants.uc_first_carrier + uc_i - (EXTRA_CARRIERS_TO_READ_NOISE >> 1) + 3;
								atpl250.m_uc_noise_peak_power = *(puc_noise_per_carrier + uc_i) + s_band_constants.uc_noise_correction_factor_db;
								LOG_PHY(("C3 %u %udB\r\n", uc_selected_carrier_to_notch,
										*(puc_noise_per_carrier + uc_i) + s_band_constants.uc_noise_correction_factor_db));
							}
						}
					}
				}
			}
		}

		/* If Peak1 has been detected, calculated filter is not valid */
		ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
		if (!(ul_int_flags & INT_PEAK1_MASK_32)) {
			/* If noise has been detected, apply filter and reset TXRXB */
			if (uc_max_noise) {
				atpl250.m_uc_rrc_notch_active = 1;
				atpl250.m_uc_rrc_notch_index = uc_selected_carrier_to_notch;
				for (uc_i = 0; uc_i <= FILTER_CONFIG_NUM_STEPS; uc_i++) {
					/* <= because last step is for writing to HW */
					atpl250_set_rrc_notch_filter(uc_selected_carrier_to_notch, uc_i);
					ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
					if ((ul_int_flags & INT_PEAK1_MASK_32)) {
						/* Abort */
						s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
						/* As we have aborted noise capture, a noise error has been generated */
						/* Clear sw flag and interrupt flag */
						ul_phy_int_flags &= ~INT_NOISE_ERROR_MASK_32;
						atpl250_clear_noise_error_int();
						return;
					}
				}
				atpl250_sync_pream_txrxb_push_rst();
				ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
				if ((ul_int_flags & INT_PEAK1_MASK_32)) {
					/* Reset FFT IOB Rotator and SyncM */
					atpl250_iob_rotator_and_fft_push_rst();
					atpl250_syncm_detector_push_rst();
					atpl250_syncm_detector_release_rst();
					/* Clear interrupts */
					ul_phy_int_flags &= (~INT_PEAK1_MASK_32);
					ul_phy_int_flags &= (~INT_NOISE_ERROR_MASK_32);
					atpl250_clear_peak1_int();
					atpl250_clear_noise_error_int();
					/* SyncM reset raises a No Peak2 int */
					atpl250_clear_no_peak2_int();
					/* Look for chirp again */
					pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x01);
					/* Release all resets */
					atpl250_iob_rotator_and_fft_release_rst();
				}

				/* Apply filter */
				pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x80);  /* SRST_DEC=1 */
				atpl250_enable_rrc_notch_filter(1);
				atpl250_sync_pream_txrxb_release_rst();
				pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);  /* SRST_DEC=0 */
			} else {
				atpl250.m_uc_rrc_notch_active = 0;
				atpl250.m_uc_rrc_notch_index = 0;
			}
		}

		/* Set Rx step */
		s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;

		/* Reset IOB and FFT, unless peak already detected */
		ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
		if (!(ul_int_flags & INT_PEAK1_MASK_32)) {
			atpl250_iob_push_rst();
			atpl250_iob_release_rst();
		}

		/* As we have aborted noise capture, a noise error has been generated */
		/* Clear sw flag and interrupt flag */
		ul_phy_int_flags &= ~INT_NOISE_ERROR_MASK_32;
		atpl250_clear_noise_error_int();
	}
#endif
}

/**
 * \brief Function to trigger Data Confirm Callback
 *
 */
static void _trigger_data_confirm(void)
{
	if (g_phy_callbacks.m_p_data_confirm != NULL) {
		xPhyMsgTxResult_t tx_result;
		uint8_t uc_current_emit_gain = 1;
		uint16_t us_new_emit_gain;
		us_new_emit_gain = 0;

		if (atpl250.m_uc_plc_disable == 0) {
			tx_result.m_uc_id_buffer = 0;
			tx_result.e_tx_result = s_phy_tx_ctl.e_tx_state;
			tx_result.m_ul_end_tx_time = ul_tx_end_time;
			tx_result.m_ul_rms_calc = 0;

			/*LOG_PHY(("txres %u\r\n", tx_result.e_tx_result));*/

			if (tx_result.e_tx_result == PHY_TX_RESULT_SUCCESS) {
				tx_result.m_ul_rms_calc = pplc_if_read32(REG_ATPL250_RMS_CALC_32);
				LOG_PHY(("RMSCalc %lu\r\n", tx_result.m_ul_rms_calc));
				uc_current_emit_gain = pplc_if_read8(REG_ATPL250_EMIT_GAIN_VL8);
				if ((atpl250.m_uc_auto_detect_impedance != FIXED_STATE_FIXED_GAIN) && (uc_last_tx_power < NUM_TX_LEVELS)) {
					atpl250.m_ul_last_rmscalc_corrected
						= ((p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init *
							(tx_result.m_ul_rms_calc >> 16)) / uc_current_emit_gain) << 16;
					switch (atpl250.m_uc_impedance_state) {
					case HI_STATE:
						if (atpl250.m_uc_auto_detect_impedance == AUTO_STATE_VAR_GAIN) {
							if (atpl250.m_ul_last_rmscalc_corrected < p_caul_th2_hi[uc_last_tx_power]) {
								/* Go to state VLO_STATE */
								atpl250.m_uc_impedance_state = VLO_STATE;
								us_new_emit_gain = (uint16_t)p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init;
							} else if (atpl250.m_ul_last_rmscalc_corrected < p_caul_th1_hi[uc_last_tx_power]) {
								/* Go to state LO_STATE */
								atpl250.m_uc_impedance_state = LO_STATE;
								us_new_emit_gain = (uint16_t)p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init;
							} else {
								/* Continue in state HI_STATE */
								us_new_emit_gain
									= ((p_caul_max_rms_hi[uc_last_tx_power] >>
										16) * uc_current_emit_gain) / (tx_result.m_ul_rms_calc >> 16);
								if (us_new_emit_gain > p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max) {
									us_new_emit_gain = p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max;
								} else if (us_new_emit_gain < p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min) {
									us_new_emit_gain = p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min;
								}
							}
						} else {
							/* FIXED_STATE_VAR_GAIN. Continue in state HI_STATE */
							us_new_emit_gain
								= ((p_caul_max_rms_hi[uc_last_tx_power] >>
									16) * uc_current_emit_gain) / (tx_result.m_ul_rms_calc >> 16);
							if (us_new_emit_gain > p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max) {
								us_new_emit_gain = p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max;
							} else if (us_new_emit_gain < p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min) {
								us_new_emit_gain = p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min;
							}
						}

						break;

					case LO_STATE:
						if (atpl250.m_uc_auto_detect_impedance == AUTO_STATE_VAR_GAIN) {
							if (atpl250.m_ul_last_rmscalc_corrected < p_caul_th2_lo[uc_last_tx_power]) {
								/* Go to state VLO_STATE */
								atpl250.m_uc_impedance_state = VLO_STATE;
							} else if (atpl250.m_ul_last_rmscalc_corrected > p_caul_th1_lo[uc_last_tx_power]) {
								/* Go to state HI_STATE */
								atpl250.m_uc_impedance_state = HI_STATE;
							}
						}

						us_new_emit_gain = p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init;
						break;

					case VLO_STATE:
						if (atpl250.m_uc_auto_detect_impedance == AUTO_STATE_VAR_GAIN) {
							if (atpl250.m_ul_last_rmscalc_corrected < p_caul_th2_vlo[uc_last_tx_power]) {
								/* Continue in state VLO_STATE */
								us_new_emit_gain
									= ((p_caul_max_rms_vlo[uc_last_tx_power] >>
										16) * uc_current_emit_gain) / (tx_result.m_ul_rms_calc >> 16);
								if (us_new_emit_gain > p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max) {
									us_new_emit_gain = p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max;
								} else if (us_new_emit_gain < p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min) {
									us_new_emit_gain = p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min;
								}
							} else if (atpl250.m_ul_last_rmscalc_corrected > p_caul_th1_vlo[uc_last_tx_power]) {
								/* Go to state HI_STATE */
								atpl250.m_uc_impedance_state = HI_STATE;
								us_new_emit_gain = p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init;
							} else {
								/* Go to state LO_STATE */
								atpl250.m_uc_impedance_state = LO_STATE;
								us_new_emit_gain = p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init;
							}
						} else {
							/* FIXED_STATE_VAR_GAIN. Continue in state VLO_STATE */
							us_new_emit_gain
								= ((p_caul_max_rms_vlo[uc_last_tx_power] >>
									16) * uc_current_emit_gain) / (tx_result.m_ul_rms_calc >> 16);
							if (us_new_emit_gain > p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max) {
								us_new_emit_gain = p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max;
							} else if (us_new_emit_gain < p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min) {
								us_new_emit_gain = p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min;
							}
						}

						break;

					default:
						/* Do nothing */
						break;
					}
				} else {
					us_new_emit_gain = p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init;
					atpl250.m_ul_last_rmscalc_corrected
						= ((p_emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init *
							(tx_result.m_ul_rms_calc >> 16)) / uc_current_emit_gain) << 16;
				}

				atpl250_update_branch_cfg(atpl250.m_uc_impedance_state, (uint8_t)us_new_emit_gain);
			}

			g_phy_callbacks.m_p_data_confirm(&tx_result);
		} else {
			/* PLC disabled. Send success confirm even if frame is not transmitted */
			tx_result.m_uc_id_buffer = 0;
			s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_SUCCESS;
			tx_result.e_tx_result = PHY_TX_RESULT_SUCCESS;
			tx_result.m_ul_end_tx_time = ul_tx_end_time;
			tx_result.m_ul_rms_calc = 0;

			g_phy_callbacks.m_p_data_confirm(&tx_result);
		}
	}

	/* Reset Tx Timeout */
	ul_tx_timeout_timer = ul_ms_counter + PHY_TX_TIMEOUT_MS;

	#ifdef ENABLE_SNIFFER
	if ((g_phy_sniffer_callbacks.m_p_sniffer_data_confirm != NULL) && (s_phy_tx_ctl.e_tx_state == PHY_TX_RESULT_SUCCESS)) {
		/* Build xPhyMsgTxResult_t  */
		tx_result_sniffer.m_uc_id_buffer = 0;
		tx_result_sniffer.e_tx_result = s_phy_tx_ctl.e_tx_state;
		tx_result_sniffer.m_ul_end_tx_time = ul_tx_end_time;
		tx_result_sniffer.m_ul_rms_calc = pplc_if_read32(REG_ATPL250_RMS_CALC_32);

		/* Build xPhyMsgTx_t */
		tx_param_sniffer.m_uc_tx_power = uc_phy_tx_full_gain;
		tx_param_sniffer.e_mod_type = s_phy_tx_ctl.e_mod_type;
		tx_param_sniffer.e_mod_scheme = s_phy_tx_ctl.e_mod_scheme;
		tx_param_sniffer.m_uc_pdc = s_phy_tx_ctl.m_uc_pdc;
		memcpy(tx_param_sniffer.m_auc_tone_map, s_phy_tx_ctl.m_auc_tone_map, s_band_constants.uc_tonemap_size);
		tx_param_sniffer.m_uc_2_rs_blocks = s_phy_tx_ctl.e_rs_blocks;
		tx_param_sniffer.e_delimiter_type = s_phy_tx_ctl.e_delimiter_type;
		switch (s_phy_tx_ctl.e_delimiter_type) {
		case DT_SOF_NO_RESP:
		case DT_SOF_RESP:
			tx_param_sniffer.m_puc_data_buf = &s_phy_tx_ctl.auc_tx_buf[s_band_constants.uc_fch_len];
			tx_param_sniffer.m_us_data_len = s_phy_tx_ctl.m_us_payload_len;
			tx_param_sniffer.m_us_data_len = s_phy_tx_ctl.m_us_payload_len;
			break;

		case DT_ACK:
		case DT_NACK:
			tx_param_sniffer.m_puc_data_buf = &s_phy_tx_ctl.auc_tx_buf[0];
			tx_param_sniffer.m_us_data_len = 4;
			break;
		}

		tx_param_sniffer.m_ul_tx_time = ul_last_tx_time;

		uc_phy_generic_flags |= PHY_GENERIC_FLAG_SNIFFER_CONFIRM;
	}
	#endif
}

/**
 * \brief Function to trigger Data Indication Callback
 *
 */
static void _trigger_data_indication(void)
{
	uint8_t uc_i;

	/* Check whether PLC is disabled */
	if (atpl250.m_uc_plc_disable) {
		return;
	}

	if (g_phy_callbacks.m_p_data_indication != NULL) {
		xPhyMsgRx_t rx_msg;
		rx_msg.m_uc_buff_id = 0;
		rx_msg.e_mod_type = s_phy_rx_ctl.e_mod_type;
		rx_msg.m_us_data_len = s_phy_rx_ctl.m_us_rx_len;
		for (uc_i = 0; uc_i < s_band_constants.uc_tonemap_size; uc_i++) {
			rx_msg.m_auc_tone_map[uc_i] = s_phy_rx_ctl.m_auc_tone_map[uc_i];
		}
		rx_msg.e_mod_scheme = s_phy_rx_ctl.e_mod_scheme;
		rx_msg.m_us_evm_header = s_phy_rx_ctl.m_us_evm_header;
		rx_msg.m_us_evm_payload = s_phy_rx_ctl.m_us_evm_payload;
		rx_msg.m_us_rssi = s_phy_rx_ctl.m_us_rssi;
		rx_msg.m_us_agc_factor = s_phy_rx_ctl.m_us_agc_factor;
		_zcd_calc_phase_difference();
		rx_msg.m_uc_zct_diff = s_phy_rx_ctl.m_uc_zct_diff;
		rx_msg.e_delimiter_type = s_phy_rx_ctl.e_delimiter_type;
		rx_msg.m_ul_rx_time = ul_rx_end_time - s_band_constants.ul_rrc_delay_us;
		rx_msg.m_ul_frame_duration = ul_rx_end_time - ul_rx_sync_time + s_band_constants.ul_preamble_duration;
		rx_msg.m_uc_rs_corrected_errors = s_phy_rx_ctl.m_uc_rs_corrected_errors;

		if ((s_phy_rx_ctl.e_delimiter_type == DT_SOF_NO_RESP) || (s_phy_rx_ctl.e_delimiter_type == DT_SOF_RESP)) {
			rx_msg.m_puc_data_buf = &s_phy_rx_ctl.auc_rx_buf[s_band_constants.uc_fch_len];
		} else {
			rx_msg.m_puc_data_buf = &s_phy_rx_ctl.auc_rx_buf[0];
		}

		g_phy_callbacks.m_p_data_indication(&rx_msg);
	}

	#ifdef ENABLE_SNIFFER
	if (g_phy_sniffer_callbacks.m_p_sniffer_data_indication != NULL) {
		rx_msg_sniffer.m_uc_buff_id = 0;
		rx_msg_sniffer.e_mod_type = s_phy_rx_ctl.e_mod_type;
		rx_msg_sniffer.m_us_data_len = s_phy_rx_ctl.m_us_rx_len;
		for (uc_i = 0; uc_i < s_band_constants.uc_tonemap_size; uc_i++) {
			rx_msg_sniffer.m_auc_tone_map[uc_i] = s_phy_rx_ctl.m_auc_tone_map[uc_i];
		}
		rx_msg_sniffer.e_mod_scheme = s_phy_rx_ctl.e_mod_scheme;
		rx_msg_sniffer.m_us_evm_header = s_phy_rx_ctl.m_us_evm_header;
		rx_msg_sniffer.m_us_evm_payload = s_phy_rx_ctl.m_us_evm_payload;
		rx_msg_sniffer.m_us_rssi = s_phy_rx_ctl.m_us_rssi;
		rx_msg_sniffer.m_us_agc_factor = s_phy_rx_ctl.m_us_agc_factor;
		_zcd_calc_phase_difference();
		rx_msg_sniffer.m_uc_zct_diff = s_phy_rx_ctl.m_uc_zct_diff;
		rx_msg_sniffer.e_delimiter_type = s_phy_rx_ctl.e_delimiter_type;
		rx_msg_sniffer.m_ul_rx_time = ul_rx_end_time - s_band_constants.ul_rrc_delay_us;
		rx_msg_sniffer.m_ul_frame_duration = ul_rx_end_time - ul_rx_sync_time + s_band_constants.ul_preamble_duration;
		rx_msg_sniffer.m_uc_rs_corrected_errors = s_phy_rx_ctl.m_uc_rs_corrected_errors;

		if ((s_phy_rx_ctl.e_delimiter_type == DT_SOF_NO_RESP) || (s_phy_rx_ctl.e_delimiter_type == DT_SOF_RESP)) {
			rx_msg_sniffer.m_puc_data_buf = &s_phy_rx_ctl.auc_rx_buf[s_band_constants.uc_fch_len];
		} else {
			rx_msg_sniffer.m_puc_data_buf = &s_phy_rx_ctl.auc_rx_buf[0];
		}

		uc_phy_generic_flags |= PHY_GENERIC_FLAG_SNIFFER_INDICATION;
	}
	#endif

	#ifdef PRINT_PILOTS_DBG
	/* Print Pilots */
	printf("Rx Pilots:\r\n");
	for (uint16_t us_i = 0; us_i < us_symbol_counter_tmp - 1; us_i++) {
		for (uint8_t uc_j = 0; uc_j < CARR_BUFFER_LEN; uc_j++) {
			printf("%02x", auc_pilot_pos_tmp[us_i][uc_j]);
		}
		printf("\r\n");
	}
	#endif
}

/**
 * \brief Handler for Inout buffer interrupt in Tx mode
 *
 */
static void handle_iob_int_tx(void)
{
	uint8_t uc_i;
	uint16_t us_flag_tx_error;
	uint8_t num_intrlv_tx_max;
	uint8_t num_intrlv_tx;
	uint8_t uc_cd_int;
	uint8_t uc_p_symbol_len_max;

	uc_cd_int = atpl250_read_cd_int();
	if (uc_cd_int && ((s_phy_tx_ctl.e_tx_step == STEP_TX_HEADER) || (s_phy_tx_ctl.e_tx_step == STEP_TX_HEADER_FIRST) ||
			(s_phy_tx_ctl.e_tx_step == STEP_TX_PREAMBLE_LAST))) {
		/* CD before start of transmission */
		atpl250_txrxb_push_rst();
		end_tx();
		pplc_if_and8(REG_ATPL250_TXRXB_STATE_VL8, (uint8_t)(~0x02u));
		atpl250_txrxb_release_rst();
		/* Abort Tx */
		pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x01);
		s_phy_tx_ctl.e_tx_step = STEP_TX_CANCELLED_TX;
		uc_set_sym_ready = false;
		/* Enable possible configured notch filters */
		atpl250_enable_rrc_notch_filter(atpl250.m_uc_rrc_notch_active);
		return;
	}

	switch (s_phy_tx_ctl.e_tx_step) {
	case STEP_TX_PREAMBLE_LAST:
		s_phy_tx_ctl.e_tx_step = STEP_TX_HEADER_FIRST;
		_tx_preamble_last();
		uc_set_sym_ready = true;
		break;

	case STEP_TX_HEADER_FIRST:
		s_phy_tx_ctl.e_tx_step = STEP_TX_HEADER;

		if (uc_working_band != WB_CENELEC_A) {
			/* Read from IOB, to let the previous symbols go through FFT. NOTE: the time spent here depends on SPI speed (supposed 15MHz) */
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
			pplc_if_read_buf(BCODE_ZONE4, s_phy_tx_ctl.m_auc_inactive_carriers_pos, CARR_BUFFER_LEN, true);
		}

		if (uc_working_band == WB_FCC) {
			generate_inactive_carriers_fcc(s_phy_tx_ctl.m_auc_tone_map, s_phy_tx_ctl.m_auc_inactive_carriers_pos);
			generate_inactive_carriers_fcc(s_phy_tx_ctl.m_auc_inv_tone_map, s_phy_tx_ctl.m_auc_inv_inactive_carriers_pos);
		} else if (uc_working_band == WB_ARIB) {
			generate_inactive_carriers_arib(s_phy_tx_ctl.m_auc_tone_map, s_phy_tx_ctl.m_auc_inactive_carriers_pos);
			generate_inactive_carriers_arib(s_phy_tx_ctl.m_auc_inv_tone_map, s_phy_tx_ctl.m_auc_inv_inactive_carriers_pos);
		} else {
			generate_inactive_carriers_cenelec_a(s_phy_tx_ctl.m_auc_tone_map[0], s_phy_tx_ctl.m_auc_inactive_carriers_pos);
			generate_inactive_carriers_cenelec_a(s_phy_tx_ctl.m_auc_inv_tone_map[0], s_phy_tx_ctl.m_auc_inv_inactive_carriers_pos);
		}

		/* Generate the combination of inactive and notched carriers */
		for (uc_i = 0; uc_i < CARR_BUFFER_LEN; uc_i++) {
			s_phy_tx_ctl.m_auc_static_and_dynamic_notching_pos[uc_i] = auc_static_notching_pos[uc_i] |
					s_phy_tx_ctl.m_auc_inactive_carriers_pos[uc_i];
			s_phy_tx_ctl.m_auc_static_and_inv_dynamic_notching_pos[uc_i] = auc_static_notching_pos[uc_i] |
					s_phy_tx_ctl.m_auc_inv_inactive_carriers_pos[uc_i];
		}

		s_phy_tx_ctl.m_uc_num_active_carriers = get_active_carriers(s_phy_tx_ctl.m_auc_inactive_carriers_pos, (uint8_t *)auc_static_notching_pos);

		s_phy_tx_ctl.m_uc_num_pilots = s_phy_tx_ctl.m_uc_num_active_carriers / PILOT_FREQ_SPA;
		if (s_phy_tx_ctl.m_uc_num_active_carriers % PILOT_FREQ_SPA) {
			s_phy_tx_ctl.m_uc_num_pilots++;
		}

		if (uc_working_band == WB_CENELEC_A) {
			/* Calculate FCH parameters */
			phy_fch_encode_g3_cenelec_a(&s_phy_tx_ctl, auc_fch);
		} else {
			/* Calculate FCH parameters */
			phy_fch_encode_g3_fcc_arib(&s_phy_tx_ctl, auc_fch);
		}

		/* Copy FCH */
		if (s_phy_tx_ctl.e_rs_blocks == RS_BLOCKS_1_BLOCK) {
			atpl250.m_us_last_tx_msg_payload_symbols = s_phy_tx_ctl.m_us_tx_payload_symbols;
		} else {
			atpl250.m_us_last_tx_msg_payload_symbols = s_phy_tx_ctl.m_us_tx_payload_symbols << 1;
		}

		memcpy(&s_phy_tx_ctl.auc_tx_buf[0], auc_fch, s_band_constants.uc_fch_len);
		/* Write Header in Raw Data */
		write_raw_header(&s_phy_tx_ctl.auc_tx_buf[0]);

		_tx_header_first();
		uc_set_sym_ready = true;
		break;

	case STEP_TX_HEADER:
		if (s_phy_tx_ctl.m_us_tx_pending_symbols > MAX_NUM_SYM_MOD) {
			/* More than one block left to send. Send it, set sym ready, decrease pending symbols and keep in same state */
			feed_modulator_fch(MAX_NUM_SYM_MOD, NO_SYM_OFFSET, CHANGE_MODULATION);
			s_phy_tx_ctl.m_us_tx_pending_symbols -= MAX_NUM_SYM_MOD;
		} else {
			/* Last FCH block */
			if ((s_phy_tx_ctl.e_delimiter_type == DT_ACK) || (s_phy_tx_ctl.e_delimiter_type == DT_NACK)) {
				s_phy_tx_ctl.e_tx_step = STEP_TX_END;
				_tx_header_last();
			} else {
				if (s_phy_tx_ctl.e_mod_scheme == MOD_SCHEME_COHERENT) {
					s_phy_tx_ctl.e_tx_step = STEP_TX_COH_S1S2;
					_tx_header_last();
				} else {
					s_phy_tx_ctl.e_tx_step = STEP_TX_PAYLOAD_FIRST;
					_tx_header_last();
					if (s_phy_tx_ctl.e_rs_blocks != RS_BLOCKS_1_BLOCK) {
						disable_HW_chain_autorst();
						/* Set value to stop scrambler */
						pplc_if_write16(REG_ATPL250_INTERLEAVER_BPSCR_L16, (s_phy_tx_ctl.m_us_payload_len << 3) - 1);
					}

					write_raw_payload(&s_phy_tx_ctl, 0);
				}
			}

			/* Set pending symbols for payload */
			s_phy_tx_ctl.m_us_tx_pending_symbols = s_phy_tx_ctl.m_us_tx_payload_symbols;
		}

		/* Set sym ready */
		uc_set_sym_ready = true;

		break;

	case STEP_TX_COH_S1S2:
		_coh_tx_s1s2();
		uc_set_sym_ready = true;
		s_phy_tx_ctl.e_tx_step = STEP_TX_PAYLOAD_FIRST;
		if (s_phy_tx_ctl.e_rs_blocks != RS_BLOCKS_1_BLOCK) {
			disable_HW_chain_autorst();
			/* Set value to stop scrambler */
			pplc_if_write16(REG_ATPL250_INTERLEAVER_BPSCR_L16, (s_phy_tx_ctl.m_us_payload_len << 3) - 1);
		}

		write_raw_payload(&s_phy_tx_ctl, 0);
		break;

	case STEP_TX_PAYLOAD_FIRST:
		#ifdef PRINT_PILOTS_DBG
		if (s_phy_tx_ctl.e_mod_scheme == MOD_SCHEME_COHERENT) {
			/* Initialize pilot position for debugging */
			memset(auc_pilot_pos_tx_tmp, 0, sizeof(auc_pilot_pos_tmp));
			us_symbol_counter_tmp = 0;
		}
		#endif

		/* Change value to modulator abcd points (minus 3dB) */
		pplc_if_write_buf(REG_ATPL250_INOUTB_PSK_VALUES1_32, (uint8_t *)cauc_abcd_minus3db, ABCD_POINTS_LEN);

		if (s_phy_tx_ctl.e_rs_blocks != RS_BLOCKS_2_BLOCKS_TRANSMITTING_SECOND) {
			/*Jumps are not used because payload is modulated in software and sent to hardware as true point
			 * Static notching is carried out by predistorsion */

			/* Initialize interleaver address to read payload */
			ul_interleaver_read_pointer = us_fch_interleaver_size_with_padding;

			/* Initialize LSFR */
			uc_ini_lsfr_payload_asm = (uc_notched_carriers * uc_num_symbols_fch) % 127;

			/* Prepare array used by PAYLOAD_MODULATION() assembler function */
			if (uc_working_band == WB_CENELEC_A) {
				_fill_dynamic_notching_cenelec_a(&s_phy_tx_ctl.m_auc_tone_map[0]);
			} else {
				_fill_dynamic_notching_fcc_arib(&s_phy_tx_ctl.m_auc_tone_map[0]);
			}

			if (s_phy_tx_ctl.e_mod_scheme == MOD_SCHEME_COHERENT) {
				_pilots_init(s_phy_tx_ctl.m_uc_num_pilots, s_phy_tx_ctl.m_uc_num_active_carriers);
			} else {
				memset(auc_pilots_state_carrier, 0, sizeof(auc_pilots_state_carrier));
			}

			_fill_state_carriers((uint8_t)s_phy_tx_ctl.e_mod_type, (uint8_t)s_phy_tx_ctl.e_mod_scheme);

			if (s_phy_tx_ctl.e_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
				/* Read reference from hardware */
				pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (7 * CFG_IOB_SAMPLES_PER_SYMBOL) + s_band_constants.uc_last_carrier);
				pplc_if_read_buf((BCODE_ZONE3 | (s_band_constants.uc_first_carrier + (CFG_IOB_SAMPLES_PER_SYMBOL * 7))),
						(uint8_t *)aul_reference_payload, s_band_constants.uc_num_carriers * 4, true);
				for (uc_i = 0; uc_i < s_band_constants.uc_num_carriers; uc_i++) {
					auc_reference_payload_asm[s_band_constants.uc_num_carriers - uc_i - 1] = aul_reference_payload[uc_i] >> 24;
				}
			} else {
				/* Set reference SYMCP */
				if (uc_working_band == WB_FCC) {
					uc_p_symbol_len_max = P_SYMBOL_LEN_MAX_FCC;
					puc_full_psymbol = auc_full_psymbol_fcc;
				} else if (uc_working_band == WB_ARIB) {
					uc_p_symbol_len_max = P_SYMBOL_LEN_MAX_ARIB;
					puc_full_psymbol = auc_full_psymbol_arib;
				} else {
					uc_p_symbol_len_max = P_SYMBOL_LEN_MAX_CENELEC_A;
					puc_full_psymbol = auc_full_psymbol_cenelec_a;
				}

				for (uc_i = 0; uc_i < uc_p_symbol_len_max; uc_i++) {
					auc_reference_payload_asm[s_band_constants.uc_num_carriers - uc_i * 2 - 1] = (puc_full_psymbol[uc_i] >> 4) & 0x0F;
					auc_reference_payload_asm[s_band_constants.uc_num_carriers - uc_i * 2 - 2] = puc_full_psymbol[uc_i] & 0x0F;
				}
			}
		} else { /* (s_phy_tx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_TRANSMITTING_SECOND) */
			 /* Reset address to read payload */
			ul_interleaver_read_pointer = 0;
		}

		uc_aux_pointer_asm = 0;

		/* Wait until interleaver is loaded */
		if (uc_legacy_mode) {
			num_intrlv_tx_max = 2;
		} else {
			switch (s_phy_tx_ctl.e_mod_type) {
			case MOD_TYPE_BPSK_ROBO:
			case MOD_TYPE_BPSK:
				num_intrlv_tx_max = 2;
				break;

			case MOD_TYPE_QPSK:
				num_intrlv_tx_max = 3;
				break;

			case MOD_TYPE_8PSK:
				num_intrlv_tx_max = 4;
				break;

			case MOD_TYPE_QAM: /* Not supported */
			default:
				num_intrlv_tx_max = 2;
				break;
			}
		}

		if (s_phy_tx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_TRANSMITTING_SECOND) {
			/* Interleaver is reseted in for second RS block, so only second RS block is stored in interleaver */
			num_intrlv_tx_max -= 1;
		}

		num_intrlv_tx = 0;
		us_flag_tx_error = 0;

		while (num_intrlv_tx < num_intrlv_tx_max) {
			num_intrlv_tx = pplc_if_read8(REG_ATPL250_INTERLEAVER_INFO2_H8);
			if (++us_flag_tx_error > MAX_INTL_ITERATIONS) {
				break;
			}
		}

		if (++us_flag_tx_error > MAX_INTL_ITERATIONS) {
			/* Stop transmitting */
			LOG_PHY(("EndTx Intl Err\r\n"));
			end_tx();
			s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_TIMEOUT;
			s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;
			atpl250.m_ul_tx_timeout++;
			atpl250.m_ul_tx_total_errors++;

			/* Get Tx end time */
			ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);

			/* Set flag to perform Tx end */
			uc_phy_generic_flags |= PHY_GENERIC_FLAG_END_TX;
			/* Exit case */
			break;
		}

		if (s_phy_tx_ctl.m_us_tx_pending_symbols > MAX_NUM_SYM_MOD) {
			if (s_phy_tx_ctl.e_mod_scheme == MOD_SCHEME_COHERENT) {
				s_sym_cfg.m_uc_gain = uc_phy_tx_full_gain;
				s_sym_cfg.m_uc_rep1514 = 0;
				s_sym_cfg.m_uc_rep1312 = 0;
				s_sym_cfg.m_uc_rep1110 = 0;
				s_sym_cfg.m_uc_rep98 = 0;
				s_sym_cfg.m_uc_rep76 = 0;
				s_sym_cfg.m_uc_rep54 = 0;
				s_sym_cfg.m_uc_rep32 = 0;
				s_sym_cfg.m_uc_rep10 = 0;
				s_sym_cfg.m_uc_overlap = 8;
				s_sym_cfg.m_uc_cyclicprefix = 30;
				s_sym_cfg.m_uc_is_last_symbol = 0;
				s_sym_cfg.m_uc_repetitions = 0;
				/* Send payload symbols */
				feed_modulator_payload(&s_phy_tx_ctl, MAX_NUM_SYM_MOD, 0x03, &s_sym_cfg);
			} else {
				/* Send payload symbols */
				feed_modulator_payload(&s_phy_tx_ctl, MAX_NUM_SYM_MOD, 0x00, &s_sym_cfg);
			}

			s_phy_tx_ctl.m_us_tx_pending_symbols -= MAX_NUM_SYM_MOD;
			if (s_phy_tx_ctl.m_us_tx_pending_symbols > MAX_NUM_SYM_MOD) {
				s_phy_tx_ctl.e_tx_step = STEP_TX_PAYLOAD;
			} else {
				s_phy_tx_ctl.e_tx_step = STEP_TX_PAYLOAD_LAST;
			}
		} else {
			if (s_phy_tx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_TRANSMITTING_FIRST) {
				/* First of 2 RS blocks: */
				/* Feed modulator with pending symbols, write next block to HW chain, */
				/* go back to FIRST_PAYLOAD to start second block resetting pending symbols to block size (stored in m_us_tx_payload_symbols) */
				if (s_phy_tx_ctl.e_mod_scheme == MOD_SCHEME_COHERENT) {
					s_sym_cfg.m_uc_gain = uc_phy_tx_full_gain;
					s_sym_cfg.m_uc_rep1514 = 0;
					s_sym_cfg.m_uc_rep1312 = 0;
					s_sym_cfg.m_uc_rep1110 = 0;
					s_sym_cfg.m_uc_rep98 = 0;
					s_sym_cfg.m_uc_rep76 = 0;
					s_sym_cfg.m_uc_rep54 = 0;
					s_sym_cfg.m_uc_rep32 = 0;
					s_sym_cfg.m_uc_rep10 = 0;
					s_sym_cfg.m_uc_overlap = 8;
					s_sym_cfg.m_uc_cyclicprefix = 30;
					s_sym_cfg.m_uc_is_last_symbol = 0;
					s_sym_cfg.m_uc_repetitions = 0;
					/* remove configuration for S1 and S2 */
					s_sym_cfg.m_uc_sym_idx = 0;
					_config_symbol(&s_sym_cfg);
					s_sym_cfg.m_uc_sym_idx = 1;
					_config_symbol(&s_sym_cfg);
				}

				feed_modulator_payload(&s_phy_tx_ctl, s_phy_tx_ctl.m_us_tx_pending_symbols, 0x00,
						&s_sym_cfg);

				/* Reset interleaver for second RS block */
				pplc_if_or8(REG_ATPL250_INTERLEAVER_CTL_H8, 0x01);
				/* Enable HW chain autoreset for second block */
				enable_HW_chain_autorst();
				/* Reset value to stop scrambler */
				pplc_if_write16(REG_ATPL250_INTERLEAVER_BPSCR_L16, 0x7FFF);
				write_raw_payload(&s_phy_tx_ctl, s_phy_tx_ctl.m_us_payload_len);
				s_phy_tx_ctl.m_us_tx_pending_symbols = s_phy_tx_ctl.m_us_tx_payload_symbols;
				s_phy_tx_ctl.e_tx_step = STEP_TX_PAYLOAD_FIRST;
				s_phy_tx_ctl.e_rs_blocks = RS_BLOCKS_2_BLOCKS_TRANSMITTING_SECOND;
			} else {
				if (s_phy_tx_ctl.e_mod_scheme == MOD_SCHEME_COHERENT) {
					s_sym_cfg.m_uc_gain = uc_phy_tx_full_gain;
					s_sym_cfg.m_uc_rep1514 = 0;
					s_sym_cfg.m_uc_rep1312 = 0;
					s_sym_cfg.m_uc_rep1110 = 0;
					s_sym_cfg.m_uc_rep98 = 0;
					s_sym_cfg.m_uc_rep76 = 0;
					s_sym_cfg.m_uc_rep54 = 0;
					s_sym_cfg.m_uc_rep32 = 0;
					s_sym_cfg.m_uc_rep10 = 0;
					s_sym_cfg.m_uc_overlap = 8;
					s_sym_cfg.m_uc_cyclicprefix = 30;
					s_sym_cfg.m_uc_is_last_symbol = 0;
					s_sym_cfg.m_uc_repetitions = 0;
					/* remove configuration for S1 and S2 */
					s_sym_cfg.m_uc_sym_idx = 0;
					_config_symbol(&s_sym_cfg);
					s_sym_cfg.m_uc_sym_idx = 1;
					_config_symbol(&s_sym_cfg);
				}

				s_sym_cfg.m_uc_gain = uc_phy_tx_full_gain;
				s_sym_cfg.m_uc_rep1514 = 0;
				s_sym_cfg.m_uc_rep1312 = 0;
				s_sym_cfg.m_uc_rep1110 = 0;
				s_sym_cfg.m_uc_rep98 = 0;
				s_sym_cfg.m_uc_rep76 = 0;
				s_sym_cfg.m_uc_rep54 = 0;
				s_sym_cfg.m_uc_rep32 = 0;
				s_sym_cfg.m_uc_rep10 = 0;
				s_sym_cfg.m_uc_overlap = 8;
				s_sym_cfg.m_uc_cyclicprefix = 30;
				s_sym_cfg.m_uc_is_last_symbol = 1;
				s_sym_cfg.m_uc_repetitions = 0;
				/* Set symbols to transmit */
				feed_modulator_payload(&s_phy_tx_ctl, s_phy_tx_ctl.m_us_tx_pending_symbols,
						(1 << (s_phy_tx_ctl.m_us_tx_pending_symbols - 1)), &s_sym_cfg);
				s_phy_tx_ctl.m_us_tx_pending_symbols = 0;
				s_phy_tx_ctl.e_tx_step = STEP_TX_END;
			}
		}

		uc_set_sym_ready = true;
		break;

	case STEP_TX_PAYLOAD:
		/* Set symbols to transmit */
		feed_modulator_payload(&s_phy_tx_ctl, MAX_NUM_SYM_MOD, 0x00, &s_sym_cfg);
		s_phy_tx_ctl.m_us_tx_pending_symbols -= MAX_NUM_SYM_MOD;
		if (s_phy_tx_ctl.m_us_tx_pending_symbols > MAX_NUM_SYM_MOD) {
			s_phy_tx_ctl.e_tx_step = STEP_TX_PAYLOAD;
		} else {
			s_phy_tx_ctl.e_tx_step = STEP_TX_PAYLOAD_LAST;
		}

		uc_set_sym_ready = true;
		break;

	case STEP_TX_PAYLOAD_LAST:
		if (s_phy_tx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_TRANSMITTING_FIRST) {
			/* First of 2 RS blocks: */
			/* Feed modulator with pending symbols, write next block to HW chain, */
			/* go back to FIRST_PAYLOAD to start second block resetting pending symbols to block size (stored in m_us_tx_payload_symbols) */
			feed_modulator_payload(&s_phy_tx_ctl, s_phy_tx_ctl.m_us_tx_pending_symbols, 0x00, &s_sym_cfg);
			/* Reset interleaver for second RS block */
			pplc_if_or8(REG_ATPL250_INTERLEAVER_CTL_H8, 0x01);
			/* Enable HW chain autoreset for second block */
			enable_HW_chain_autorst();
			/* Reset value to stop scrambler */
			pplc_if_write16(REG_ATPL250_INTERLEAVER_BPSCR_L16, 0x7FFF);
			write_raw_payload(&s_phy_tx_ctl, s_phy_tx_ctl.m_us_payload_len);
			s_phy_tx_ctl.m_us_tx_pending_symbols = s_phy_tx_ctl.m_us_tx_payload_symbols;
			s_phy_tx_ctl.e_tx_step = STEP_TX_PAYLOAD_FIRST;
			s_phy_tx_ctl.e_rs_blocks = RS_BLOCKS_2_BLOCKS_TRANSMITTING_SECOND;
		} else {
			s_phy_tx_ctl.e_tx_step = STEP_TX_END;
			/* Symbol configuration */
			s_sym_cfg.m_uc_gain = uc_phy_tx_full_gain;
			s_sym_cfg.m_uc_rep1514 = 0;
			s_sym_cfg.m_uc_rep1312 = 0;
			s_sym_cfg.m_uc_rep1110 = 0;
			s_sym_cfg.m_uc_rep98 = 0;
			s_sym_cfg.m_uc_rep76 = 0;
			s_sym_cfg.m_uc_rep54 = 0;
			s_sym_cfg.m_uc_rep32 = 0;
			s_sym_cfg.m_uc_rep10 = 0;
			s_sym_cfg.m_uc_overlap = 8;
			s_sym_cfg.m_uc_cyclicprefix = 30;
			s_sym_cfg.m_uc_is_last_symbol = 1;
			s_sym_cfg.m_uc_repetitions = 0;
			feed_modulator_payload(&s_phy_tx_ctl, s_phy_tx_ctl.m_us_tx_pending_symbols,
					(1 << (s_phy_tx_ctl.m_us_tx_pending_symbols - 1)), &s_sym_cfg);
			s_phy_tx_ctl.m_us_tx_pending_symbols = 0;
		}

		uc_set_sym_ready = true;
		break;

	case STEP_TX_END:
		/* Prepare for next reception */
		atpl250_hold_on_reference(false);
		atpl250_set_mod_bpsk_truepoint();

		disable_HW_chain();     /* Disable HW chain to write in modulator */
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier); /* Avoid overflow to 1st symbol */
		pplc_if_write_jump((BCODE_COH | (s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, CFG_IOB_SAMPLES_PER_SYMBOL + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 2nd symbol */
		pplc_if_write_jump((BCODE_COH | (CFG_IOB_SAMPLES_PER_SYMBOL + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])),
				(uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 2) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 3rd symbol */
		pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 2) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])),
				(uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 3) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 4th symbol */
		pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 3) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])),
				(uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 4) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 5th symbol */
		pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 4) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])),
				(uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 5) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 6th symbol */
		pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 5) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])),
				(uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 6) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 7th symbol */
		pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 6) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])),
				(uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 7) + s_band_constants.uc_last_used_carrier); /* Avoid overflow to 8th symbol */
		pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 7) + s_band_constants.uc_first_carrier + auc_unmasked_carrier_list[0])),
				(uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_2);

		atpl250_set_mod_bpsk();
		break;

	case STEP_TX_NO_TX:
	case STEP_TX_PREAMBLE_FIRST:
	case STEP_TX_CANCELLED_TX:
	/* Not valid in this interrupt */
	default:
		uc_set_sym_ready = phy_extension_handler_iob_tx();
		break;
	}
}

/**
 * \brief Handler for Inout buffer interrupt in Rx mode
 *
 */
static void handle_iob_int_rx(void)
{
	uint8_t uc_change_rx_mod = false;
	uint8_t uc_header_ok;
	uint32_t ul_rs_cfg;
	uint16_t us_bytes_to_read;
	volatile uint16_t us_bits_through_rx_chain;
	uint8_t uc_flag_rx_error;
	int16_t ss_num_sym_fch_unused;
	uint16_t us_flag_tx_error;
	uint8_t num_intrlv_tx_max;
	uint8_t num_intrlv_tx;
	uint16_t us_payload_symbols;
	uint8_t uc_num_blocks;
	uint8_t uc_num_sym_h_est_fch;

	switch (s_phy_rx_ctl.e_rx_step) {
	case STEP_RX_PEAK2:
		/* Config chain to load data to deinterleaver */
		/* 15: Bypass second scrambler, set 6 repetitions */
		pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x150B);

		/* Enable RSSI calculation */
		atpl250_enable_rssi();
		atpl250_enable_evm();

		if (uc_working_band == WB_FCC) {
			uc_num_sym_h_est_fch = NUM_SYM_H_EST_FCH_FCC_ARIB;
			/* Updates the number of FCH symbols received without having updated H */
			ss_num_fch_since_last_H_update -= s_phy_rx_ctl.m_uc_next_demod_symbols;
		} else if (uc_working_band == WB_ARIB) {
			uc_num_sym_h_est_fch = NUM_SYM_H_EST_FCH_FCC_ARIB;
		} else {
			uc_num_sym_h_est_fch = NUM_SYM_H_EST_FCH_CENELEC_A;
		}

		/* Get symbols and go to next step */
		/* This will be the number of pending symbols after the get_demodulator_fch operation */
		s_phy_rx_ctl.m_us_rx_pending_symbols -= s_phy_rx_ctl.m_uc_next_demod_symbols;
		ss_num_sym_fch_unused = s_phy_rx_ctl.m_us_rx_pending_symbols - uc_num_sym_h_est_fch;
		if (ss_num_sym_fch_unused == 0) {
			/* It is assumed that (NUM_FCH_SYM_AFTER_PEAK2 + uc_num_sym_h_est_fch) <= FCH symbols. Hence, ss_num_sym_fch_unused cannot be negative */
			get_demodulator_fch(s_phy_rx_ctl.m_uc_next_demod_symbols, CHANGE_MODULATION, 1); /* Configure interrupt after one symbol*/
			s_phy_rx_ctl.m_uc_next_demod_symbols = 1;
		} else if (ss_num_sym_fch_unused < uc_num_sym_block_demod_fch) {
			get_demodulator_fch(s_phy_rx_ctl.m_uc_next_demod_symbols, CHANGE_MODULATION, (uint8_t)ss_num_sym_fch_unused);
			s_phy_rx_ctl.m_uc_next_demod_symbols = ss_num_sym_fch_unused;
		} else {
			get_demodulator_fch(s_phy_rx_ctl.m_uc_next_demod_symbols, CHANGE_MODULATION, (uint8_t)uc_num_sym_block_demod_fch);
			s_phy_rx_ctl.m_uc_next_demod_symbols = uc_num_sym_block_demod_fch;
		}

		s_phy_rx_ctl.e_rx_step = STEP_RX_HEADER;
#ifdef ENABLE_PYH_PROCESS_RECALL
		/* Force phy process recall when near the end of FCH reception just in case it is an ACK */
		uc_force_phy_process_recall = 1;
#endif

		/* Clear Rx full */
		uc_clear_rx_full = true;
		break;

	case STEP_RX_HEADER:

		if (uc_working_band == WB_FCC) {
			uc_num_sym_h_est_fch = NUM_SYM_H_EST_FCH_FCC_ARIB;
			if (ss_num_fch_since_last_H_update <= 0) {
				/* Update of the channel estimate and 1/H */
				compensate_sfo_in_chan_est(CHANNEL_IN_FCH);

				/* Reset counter */
				ss_num_fch_since_last_H_update = ss_num_sym_fch_for_H_update;
			} else {
				ss_num_fch_since_last_H_update -= s_phy_rx_ctl.m_uc_next_demod_symbols;
			}
		} else if (uc_working_band == WB_ARIB) {
			uc_num_sym_h_est_fch = NUM_SYM_H_EST_FCH_FCC_ARIB;
		} else {
			uc_num_sym_h_est_fch = NUM_SYM_H_EST_FCH_CENELEC_A;
		}

		/* Get next symbols block */
		/*This will be the number of pending symbols after the get_demodulator_fch operation*/
		s_phy_rx_ctl.m_us_rx_pending_symbols -= s_phy_rx_ctl.m_uc_next_demod_symbols;
		ss_num_sym_fch_unused = s_phy_rx_ctl.m_us_rx_pending_symbols - uc_num_sym_h_est_fch;
		if (ss_num_sym_fch_unused == 0) { /* Configure next interrupt for one symbol, but do not read*/
			get_demodulator_fch(s_phy_rx_ctl.m_uc_next_demod_symbols, CHANGE_MODULATION, 1);
			s_phy_rx_ctl.m_uc_next_demod_symbols = 1;
		} else if (ss_num_sym_fch_unused < 0) { /* Configure next interrupt for one symbol and read symbol */
			get_demodulator_fch(s_phy_rx_ctl.m_uc_next_demod_symbols, CHANGE_MODULATION, 1);
			s_phy_rx_ctl.m_uc_next_demod_symbols = 1;

			/* Disable HW chain */
			pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~0x01u));

			/* Move FCH raw symbol to the SAM4*/
			pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier);
			pplc_if_read_jump((BCODE_ZONE4 | (auc_unmasked_carrier_list[0] + s_band_constants.uc_first_carrier)),
					(uint8_t *)(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols +
					2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_fch_sym_read)),
					2 * uc_used_carriers * 2, JUMP_COL_2, false);

			/* Invert byte order of the previous FCH symbol*/
			if (uc_num_fch_sym_read) {
				swap_bytes_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols +
						2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_fch_sym_read - 1)), uc_used_carriers);
			}

			uc_num_fch_sym_read++;

			pplc_if_do_read((uint8_t *)(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols +
					2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_fch_sym_read)), 2 * uc_used_carriers * 2);

			/* Enable HW chain */
			pplc_if_or8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x01);
		} else if (ss_num_sym_fch_unused < uc_num_sym_block_demod_fch) {
			get_demodulator_fch(s_phy_rx_ctl.m_uc_next_demod_symbols, CHANGE_MODULATION, (uint8_t)ss_num_sym_fch_unused);
			s_phy_rx_ctl.m_uc_next_demod_symbols = ss_num_sym_fch_unused;
		} else {
			get_demodulator_fch(s_phy_rx_ctl.m_uc_next_demod_symbols, CHANGE_MODULATION, (uint8_t)uc_num_sym_block_demod_fch);
			s_phy_rx_ctl.m_uc_next_demod_symbols = uc_num_sym_block_demod_fch;
		}

		/* Check whether FCH is complete or not */
		if (s_phy_rx_ctl.m_us_rx_pending_symbols) {
			/* There are still symbols to receive, clear Rx Full and keep in same state */
			uc_clear_rx_full = true;
		} else {
			/* Invert byte order of the last FCH symbol*/
			swap_bytes_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols +
					2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_fch_sym_read - 1)), uc_used_carriers);

			/* Reset number of FCH symbols used for channel/SFO for next frame */
			uc_num_fch_sym_read = 0;

			/* FCH Complete. Rx full will be cleared after BER interrupt */

			/* Set unload mode and send pulse to unload deinterleaver */
			pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x13); /* BPSK for header */

			/* Wait until all bits are stored in raw data */
			uc_flag_rx_error = 0;
			us_bits_through_rx_chain = pplc_if_read16(REG_ATPL250_INTERLEAVER_INFO2_L16);
			while (us_bits_through_rx_chain < (s_band_constants.uc_fch_len_bits - 1)) {
				us_bits_through_rx_chain = pplc_if_read16(REG_ATPL250_INTERLEAVER_INFO2_L16);
				if (++uc_flag_rx_error > 100) {
					break;
				}
			}

			if (++uc_flag_rx_error > 100) {
				/* Clear pending symbols and reset Rx mode */
				s_phy_rx_ctl.m_us_rx_pending_symbols = 0;
				s_phy_rx_ctl.m_us_rx_len = 0;
				atpl250.m_ul_rx_exception++;
				end_rx();
				s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
#ifdef ENABLE_PYH_PROCESS_RECALL
				/* Clear phy process recall in case of error */
				uc_force_phy_process_recall = 0;
#endif

				LOG_PHY(("RxChainErr\r\n"));
				break; /* If error is detected, everything is already reset, so break to exit interrupt */
			}

			/* Disable HW chain to read raw header */
			pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~0x01u));

			/* Read raw data from bit 0 */
			pplc_if_write16(REG_ATPL250_RAW_DATA_H16, 0);
			/* Read Header */
			pplc_if_read_rep(REG_ATPL250_RAW_DATA_VL8, 1, &s_phy_rx_ctl.auc_rx_buf[0], s_band_constants.uc_fch_len, true);

			/* Decode header */
			if (uc_working_band == WB_CENELEC_A) {
				uc_header_ok = phy_fch_decode_g3_cenelec_a(&s_phy_rx_ctl, (uint8_t *)auc_static_notching_pos);
			} else {
				uc_header_ok = phy_fch_decode_g3_fcc_arib(&s_phy_rx_ctl, (uint8_t *)auc_static_notching_pos);
			}

			if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_1_BLOCK) {
				atpl250.m_us_last_rx_msg_payload_symbols = s_phy_rx_ctl.m_us_rx_payload_symbols;
			} else {
				atpl250.m_us_last_rx_msg_payload_symbols = s_phy_rx_ctl.m_us_rx_payload_symbols << 1;
			}

			/* If not correct, end rx and break */
			if (!uc_header_ok) {
				LOG_PHY(("Bad CRC FCH\r\n"));
				s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
				s_phy_rx_ctl.m_us_rx_pending_symbols = 0;
				atpl250.m_ul_rx_bad_crc_fch++;
				end_rx();
				uc_clear_rx_full = false;
#ifdef ENABLE_PYH_PROCESS_RECALL
				/* Clear phy process recall in case of error */
				uc_force_phy_process_recall = 0;
#endif

			#if (LOG_SPI == 1)
				dumpLogSpi();
			#endif

				break;
			} else {
				/* Check if a transmission is already programmed */
				if (s_phy_tx_ctl.e_tx_step != STEP_TX_NO_TX) {
					pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x01);
					s_phy_tx_ctl.e_tx_step = STEP_TX_CANCELLED_TX;
					uc_set_sym_ready = false;
					/* Enable possible configured notch filters */
					atpl250_enable_rrc_notch_filter(atpl250.m_uc_rrc_notch_active);
				}

				/* Correct header, check whether it is a data frame or an ACK/NACK */
				if ((s_phy_rx_ctl.e_delimiter_type == DT_SOF_NO_RESP) || (s_phy_rx_ctl.e_delimiter_type == DT_SOF_RESP)) { /* Data frame */
					/* Simulate tx to run BER */
					pplc_if_write32(REG_ATPL250_INTERLEAVER_CFG3_32, s_band_constants.uc_fch_len_bits - 1); /* Set first and last bit */
					pplc_if_or8(REG_ATPL250_BER_PERIPH_CFG1_VH8, 0x80); /* AUTO_BER='1' */
					e_ber_status = BER_FCH;
					pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x2F);   /* BPSK, Tx mode, Load mode, G3 */
#ifdef ENABLE_PYH_PROCESS_RECALL
					/* Clear phy process recall in case of data frame, it will be set near the end of frame */
					uc_force_phy_process_recall = 0;
#endif

					if (s_phy_rx_ctl.e_mod_scheme == MOD_SCHEME_COHERENT) {
						/* Set interrupt to 1 symbol to receive S1*/
						atpl250_set_num_symbols_cfg(1);
						s_phy_rx_ctl.e_rx_step = STEP_RX_COH_S1;
					} else {
						if (s_phy_rx_ctl.m_us_rx_pending_symbols <= DEFAULT_SYM_NEXT) {
							/* If less or equal than DEFAULT_SYM_NEXT symbols pending, set next demod symbols to pending symbols,
							 * and state to last demod */
							s_phy_rx_ctl.m_uc_next_demod_symbols = (uint8_t)s_phy_rx_ctl.m_us_rx_pending_symbols;
							s_phy_rx_ctl.e_demod_step = STEP_DEMOD_LAST;
#ifdef ENABLE_PYH_PROCESS_RECALL
							/* Force phy process recall when near the end of a reception */
							uc_force_phy_process_recall = 1;
#endif
						} else if (s_phy_rx_ctl.m_us_rx_pending_symbols >= s_band_constants.uc_min_sym_for_offset_correction_dif) {
							/* If more than minimum symbols needed to go through all steps pending, set next demod symbols to First
							 * demod symbols,
							 * and state to First demod */
							s_phy_rx_ctl.m_uc_next_demod_symbols = s_band_constants.uc_pay_sym_first_demod;
							s_phy_rx_ctl.e_demod_step = STEP_DEMOD_FIRST;
						} else {
							/* Otherwise, set first number of symbols to interrupt so there is 3 symbols left at the end for last 2
							 * interrupts, */
							/* and set state to standard demod */
							s_phy_rx_ctl.m_uc_next_demod_symbols
								= ((s_phy_rx_ctl.m_us_rx_pending_symbols % DEFAULT_SYM_NEXT) + DEFAULT_SYM_NEXT -
									(PAY_SYM_ONE_BEFORE_LAST_DEMOD + PAY_SYM_LAST_DEMOD)) % DEFAULT_SYM_NEXT;
							if (s_phy_rx_ctl.m_uc_next_demod_symbols == 0) {
								s_phy_rx_ctl.m_uc_next_demod_symbols = DEFAULT_SYM_NEXT;
							}

							s_phy_rx_ctl.e_demod_step = STEP_DEMOD_STD;
						}

						atpl250_set_num_symbols_cfg(s_phy_rx_ctl.m_uc_next_demod_symbols);
						/* Set state to start receiving payload */
						s_phy_rx_ctl.e_rx_step = STEP_RX_PAYLOAD_FIRST;
					}
				} else {
					/* ACK/NACK. FCH already in buffer, just send indication to upper layer */
					LOG_PHY(("ACK Rx\r\n"));

					/* Get Rx end time */
					ul_rx_end_time = ul_rx_sync_time + (uint32_t)(uc_num_symbols_fch) * s_band_constants.ul_frame_symbol_duration + s_band_constants.ul_half_symbol_duration;

					/* Get evm and rssi */
					s_phy_rx_ctl.m_us_evm_payload = 0;
					s_phy_rx_ctl.m_us_rssi = atpl250_read_rssi();
					/* Reset evm and rssi */
					atpl250_reset_evm();
					atpl250_reset_rssi();
					/* Disable RSSI calculation for later channel estimation */
					atpl250_disable_rssi();
					atpl250_disable_evm();
					/* Clear pending symbols and reset Rx mode */
					s_phy_rx_ctl.m_us_rx_pending_symbols = 0;
					s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
					end_rx();
					uc_clear_rx_full = false;
					uc_phy_generic_flags |= PHY_GENERIC_FLAG_END_RX;
				}
			}
		}

		if (s_phy_rx_ctl.e_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
			/* Indicate that phase shift must be applied to the demodulation of the first symbol of the next block */
			uc_apply_phase_shift = false;
		}

		break;

	case STEP_RX_COH_S1:

		/* Disable HW chain */
		pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~0x01u));

		/* Launch DMA transfer of S1 */
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier);
		pplc_if_read_jump((BCODE_ZONE4 | (auc_unmasked_carrier_list[0] + s_band_constants.uc_first_carrier)),
				(uint8_t *)(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols
				+ 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch)),
				2 * uc_used_carriers * 2, JUMP_COL_2, false);

		/* Obtain modulating symbols for S1. This is needed for the cases when tone mask exists. To extract values from conj_syncp */
		obtain_modulating_conj_complex_values((uint8_t *)auc_ones, uc_used_carriers, PREAMBLE_FCH_S1S2_PART, auc_unmasked_carrier_list,
				ass_modulating_symbols);

		/* We have plenty of time until the S1 is received->compute the demodulating values for the S2 */
		arm_negate_q15(ass_modulating_symbols, ass_symbol_aux, 2 * uc_used_carriers);

		/* Read S1 */
		pplc_if_do_read((uint8_t *)(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols
				+ 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch)), 2 * uc_used_carriers * 2);

		/* Set interrupt to 1 symbol to receive S2*/
		atpl250_set_num_symbols_cfg(1);

		/* Set next step to receive S2 */
		s_phy_rx_ctl.e_rx_step = STEP_RX_COH_S2;

		/* Enable HW chain */
		pplc_if_or8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x01);

		/* Clear INOB interrupt */
		uc_clear_rx_full = true;

		break;

	case STEP_RX_COH_S2:
		/*ul_start = DWT->CYCCNT;*/

		/* Disable HW chain */
		pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~0x01u));

		if (s_phy_rx_ctl.m_us_rx_payload_symbols < s_band_constants.uc_min_sym_for_offset_correction_coh) {
			/* Reset number of symbols after rx is blocked, as there is no SFO correction */
			atpl250_set_symbols_rx_block(0xFFFF);
		}

		/* Launch DMA transfer of S2 */
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, s_band_constants.uc_last_used_carrier);
		pplc_if_read_jump((BCODE_ZONE4 | (auc_unmasked_carrier_list[0] + s_band_constants.uc_first_carrier)),
				(uint8_t *)(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols
				+ 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 1)),
				2 * uc_used_carriers * 2, JUMP_COL_2, false);

		/* Process S1 */
		swap_bytes_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols
				+ 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch)), uc_used_carriers);

		/* Multiply by conjugate of the reference symbol */
		arm_cmplx_mult_cmplx_q15((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols
				+ 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch)), ass_modulating_symbols,
				(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers *
				(uc_num_sym_valid_preamble + uc_num_sym_valid_fch)),
				uc_used_carriers);

		#ifdef SMOOTHING
		smooth_carriers_asm(auc_smooth_control_preamble_fch, uc_num_carr_smooth_preamble_fch,
				(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols + 2 * uc_used_carriers *
				(uc_num_sym_valid_preamble + uc_num_sym_valid_fch)));
		#endif

		/* Adds a scaled version of S1 to the average symbol and scales S1. The scaling factors have to be configured in the assembler code */
		shift_add_shift_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols
				+ 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch)), ass_average_symbol, uc_used_carriers);

		/* Read S2 */
		pplc_if_do_read((uint8_t *)(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols
				+ 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 1)), 2 * uc_used_carriers * 2);

		/* Process S2 */
		swap_bytes_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols
				+ 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 1)), uc_used_carriers);

		/* Multiply by conjugate of the reference symbol */
		arm_cmplx_mult_cmplx_q15((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols
				+ 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 1)), ass_symbol_aux,
				(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols
				+ 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 1)), uc_used_carriers);  /* Q3.12*/

		#ifdef SMOOTHING
		smooth_carriers_asm(auc_smooth_control_preamble_fch, uc_num_carr_smooth_preamble_fch,
				(u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols
				+ 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 1)));
		#endif

		/* Adds a scaled version of S2 to the average symbol and scales S2. The scaling factors have to be configured in the assembler code */
		shift_add_shift_asm((u_shared_preamble_and_payload_symbols.ass_pm_fch_s1s2_symbols
				+ 2 * uc_used_carriers * (uc_num_sym_valid_preamble + uc_num_sym_valid_fch + 1)), ass_average_symbol, uc_used_carriers);

		/* Perform channel estimation */
		chn_estimation_from_s1s2();

		if (uc_working_band == WB_CENELEC_A) {
			/* Estimates the SFO */
			sampling_error_est_from_preamble_fch_s1s2(MOD_SCHEME_COHERENT, SFO_FOR_PAYLOAD);
			/* Compensates the SFO in the estimated channel that will be applied before the FFT window is adjusted */
			compensate_sfo_in_chan_est(CHANNEL_BEFORE_CP_CHANGE);
		} else {
			if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_1_BLOCK) {
				uc_num_blocks = 1;
			} else {
				uc_num_blocks = 2;
			}

			if ((s_phy_rx_ctl.m_us_rx_payload_symbols * uc_num_blocks) < s_band_constants.uc_min_sym_for_offset_correction_coh) {
				/* Compensates the SFO in the estimated channel that will be applied before the FFT window is adjusted */
				compensate_sfo_in_chan_est(CHANNEL_FOR_SHORT_PAYLOADS);
			} else {
				/* Estimates the SFO */
				sampling_error_est_from_preamble_fch_s1s2(MOD_SCHEME_COHERENT, SFO_FOR_PAYLOAD);
				/* Compensates the SFO in the estimated channel that will be applied before the FFT window is adjusted */
				compensate_sfo_in_chan_est(CHANNEL_BEFORE_CP_CHANGE);
			}
		}

		/* Set number of symbols for next interrupt */
		/* Moved here to profit from the time of the DMA transfer */
		if (s_phy_rx_ctl.m_us_rx_pending_symbols <= DEFAULT_SYM_NEXT) {
			/* If less or equal than DEFAULT_SYM_NEXT symbols pending, set next demod symbols to pending symbols, and state to last demod */
			s_phy_rx_ctl.m_uc_next_demod_symbols = (uint8_t)s_phy_rx_ctl.m_us_rx_pending_symbols;
			s_phy_rx_ctl.e_demod_step = STEP_DEMOD_LAST;
#ifdef ENABLE_PYH_PROCESS_RECALL
			/* Force phy process recall when near the end of a reception */
			uc_force_phy_process_recall = 1;
#endif
		} else if (s_phy_rx_ctl.m_us_rx_pending_symbols >= s_band_constants.uc_min_sym_for_offset_correction_coh) {
			/* If more than minimum symbols needed to go through all steps pending, set next demod symbols to First demod symbols,
			 * and state to First demod */
			s_phy_rx_ctl.m_uc_next_demod_symbols = s_band_constants.uc_pay_sym_first_demod;
			s_phy_rx_ctl.e_demod_step = STEP_DEMOD_FIRST;
		} else {
			/* Otherwise, set first number of symbols to interrupt so there is 3 symbols left at the end for last 2 interrupts, */
			/* and set state to standard demod */
			s_phy_rx_ctl.m_uc_next_demod_symbols
				= ((s_phy_rx_ctl.m_us_rx_pending_symbols % DEFAULT_SYM_NEXT) + DEFAULT_SYM_NEXT -
					(PAY_SYM_ONE_BEFORE_LAST_DEMOD + PAY_SYM_LAST_DEMOD)) % DEFAULT_SYM_NEXT;
			if (s_phy_rx_ctl.m_uc_next_demod_symbols == 0) {
				s_phy_rx_ctl.m_uc_next_demod_symbols = DEFAULT_SYM_NEXT;
			}

			s_phy_rx_ctl.e_demod_step = STEP_DEMOD_STD;
		}

		/* Modulation will be changed on first payload demodulation, there is no need to set it here */
		/* Write pilots position */
		/* Moved here to profit from the time of the DMA transfer */
		set_pilot_position(s_phy_rx_ctl.m_uc_num_pilots, s_phy_rx_ctl.m_uc_num_active_carriers, s_phy_rx_ctl.m_auc_reg_pilot_info,
				s_phy_rx_ctl.m_auc_pilot_pos, s_phy_rx_ctl.m_auc_inactive_carriers_pos, (uint8_t *)auc_static_notching_pos);
		/* Keep track of pilot position of first symbol */
		memcpy(s_phy_rx_ctl.m_auc_pilot_pos_first_symbol, s_phy_rx_ctl.m_auc_pilot_pos, CARR_BUFFER_LEN);

		/* Change value to modulator abcd points */
		pplc_if_write_buf(REG_ATPL250_INOUTB_PSK_VALUES1_32, (uint8_t *)cauc_abcd_fullgain, ABCD_POINTS_LEN);

		atpl250_set_num_symbols_cfg(s_phy_rx_ctl.m_uc_next_demod_symbols);

		#ifdef PRINT_PILOTS_DBG
		/* Initialize pilot position for debugging */
		memset(auc_pilot_pos_tmp, 0, sizeof(auc_pilot_pos_tmp));
		us_symbol_counter_tmp = 0;
		/* Copy pilot position for first symbol */
		memcpy(&(auc_pilot_pos_tmp[us_symbol_counter_tmp][0]), &s_phy_rx_ctl.m_auc_pilot_pos[0], CARR_BUFFER_LEN);
		us_symbol_counter_tmp++;
		#endif

		/* Enable RSSI calculation after channel estimation */
		atpl250_enable_rssi();
		atpl250_enable_evm();

		if (uc_working_band == WB_CENELEC_A) {
			ber_config_cenelec_a(&s_phy_rx_ctl);
		} else {
			ber_config_fcc_arib(&s_phy_rx_ctl);
		}

		s_phy_rx_ctl.e_rx_step = STEP_RX_PAYLOAD_FIRST;
		uc_clear_rx_full = true;
		break;

	case STEP_RX_PAYLOAD_FIRST:
		/* Set interleaver size */
		interleaver_config(s_phy_rx_ctl.m_uc_payload_carriers, s_phy_rx_ctl.m_us_rx_pending_symbols, s_phy_rx_ctl.e_mod_type);

		/* Config chain to load data to deinterleaver */
		switch (s_phy_rx_ctl.e_mod_type) {
		case MOD_TYPE_BPSK_ROBO:
			pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x130B);
			break;

		case MOD_TYPE_BPSK:
			pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x100B);
			break;

		case MOD_TYPE_QPSK:
			pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x104B);
			break;

		case MOD_TYPE_8PSK:
			pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x108B);
			break;

		case MOD_TYPE_QAM: /* Not supported */
			pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x10CB);
			break;
		}

		/* HW chain not used because jumps are not used to demodulate (data carriers are extracted by software). */
		disable_HW_chain();

		if (s_phy_rx_ctl.e_rs_blocks != RS_BLOCKS_2_BLOCKS_RECEIVING_SECOND) {
			/* Initialize state carrier array used in demodulation assembly function (read from hardware without jump) */
			if (uc_working_band == WB_CENELEC_A) {
				_fill_dynamic_notching_cenelec_a(&s_phy_rx_ctl.m_auc_tone_map[0]);
			} else {
				_fill_dynamic_notching_fcc_arib(&s_phy_rx_ctl.m_auc_tone_map[0]);
			}

			if (s_phy_rx_ctl.e_mod_scheme == MOD_SCHEME_COHERENT) {
				_pilots_init(s_phy_rx_ctl.m_uc_num_pilots, s_phy_rx_ctl.m_uc_num_active_carriers);
			} else {
				_pilots_init(0, s_phy_rx_ctl.m_uc_num_active_carriers);
			}

			_fill_state_carriers((uint8_t)s_phy_rx_ctl.e_mod_type, (uint8_t)s_phy_rx_ctl.e_mod_scheme);

			/* Get first and last used carriers for payload and PN sequence */
			s_phy_rx_ctl.m_uc_rx_first_carrier = get_first_used_carrier(s_phy_rx_ctl.m_auc_static_and_dynamic_notching_pos);
			s_phy_rx_ctl.m_uc_rx_first_carrier_pn_seq = get_first_used_carrier(s_phy_rx_ctl.m_auc_static_and_inv_dynamic_notching_pos);

			/* Initialize deinterleaver address to write payload */
			ul_deinterleaver_write_pointer = us_fch_interleaver_size_with_padding;
		} else {
			/* Reset deinterleaver address to write payload for second RS block */
			ul_deinterleaver_write_pointer = 0;
		}

		/* Set rx modulation */
		uc_change_rx_mod = true;
	/* No break, next case has to be executed also for first payload */

	case STEP_RX_PAYLOAD:
		switch (s_phy_rx_ctl.e_demod_step) {
		case STEP_DEMOD_FIRST:
			if (s_phy_rx_ctl.e_rs_blocks != RS_BLOCKS_2_BLOCKS_RECEIVING_SECOND) {
				/* Set CP to value derived from SFO estimation (adding + 8 overlapping) */
				atpl250_set_cp_rx((30 - sc_sfo_time_offset));
				/*atpl250_set_cp_rx(30);*/
				/* Set RX block value to symbols set for second demod and unblock rx */
				atpl250_set_symbols_rx_block(PAY_SYM_SECOND_DEMOD);
				atpl250_unblock_rx();
			}

			/* Decrease pending symbols */
			s_phy_rx_ctl.m_us_rx_pending_symbols -= s_band_constants.uc_pay_sym_first_demod;

			/* Get demodulator, set Second for next demod and state */
			get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod, PAY_SYM_SECOND_DEMOD,
					NO_READ_INACTIVE_CARRIERS, uc_apply_phase_shift);
			s_phy_rx_ctl.e_demod_step = STEP_DEMOD_SECOND;
			/* Keep receiving payload */
			s_phy_rx_ctl.e_rx_step = STEP_RX_PAYLOAD;
			/* Clear Rx full */
			uc_clear_rx_full = true;

			break;

		case STEP_DEMOD_SECOND:
			if (s_phy_rx_ctl.e_rs_blocks != RS_BLOCKS_2_BLOCKS_RECEIVING_SECOND) {
				/* Set CP to previous value after correction (30 = 22 cp + 8 overlapping)*/
				atpl250_set_cp_rx(30);
				/* Reset RX block value to receive the entire packet */
				atpl250_set_symbols_rx_block(0xFFFF);
				atpl250_unblock_rx();
			}

			/* Decrease pending symbols */
			s_phy_rx_ctl.m_us_rx_pending_symbols -= PAY_SYM_SECOND_DEMOD;
			if (s_phy_rx_ctl.m_us_rx_pending_symbols <= DEFAULT_SYM_NEXT) {
				/* If less or equal than DEFAULT_SYM_NEXT symbols pending, set next demod symbols to pending symbols, and state to last demod */
				get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod,
						(uint8_t)s_phy_rx_ctl.m_us_rx_pending_symbols,
						NO_READ_INACTIVE_CARRIERS, uc_apply_phase_shift);
				s_phy_rx_ctl.e_demod_step = STEP_DEMOD_LAST;
#ifdef ENABLE_PYH_PROCESS_RECALL
				/* Force phy process recall when near the end of a reception */
				uc_force_phy_process_recall = 1;
#endif
			} else {
				/* Otherwise, set number of symbols to interrupt so there is 3 symbols left at the end for last 2 interrupts, */
				/* and set state to standard demod */

				if (((s_phy_rx_ctl.m_us_rx_pending_symbols % DEFAULT_SYM_NEXT) + DEFAULT_SYM_NEXT - 3) % DEFAULT_SYM_NEXT == 0) {
					get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod, DEFAULT_SYM_NEXT,
							NO_READ_INACTIVE_CARRIERS, uc_apply_phase_shift);
				} else {
					get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod,
							((s_phy_rx_ctl.m_us_rx_pending_symbols % DEFAULT_SYM_NEXT) + DEFAULT_SYM_NEXT - 3) % DEFAULT_SYM_NEXT,
							NO_READ_INACTIVE_CARRIERS, uc_apply_phase_shift);
				}

				s_phy_rx_ctl.e_demod_step = STEP_DEMOD_STD;
			}

			if (s_phy_rx_ctl.e_mod_scheme == MOD_SCHEME_COHERENT) {
				/* Update FEQ to be used after FFT window adjustment */
				compensate_sfo_in_chan_est(CHANNEL_AFTER_CP_CHANGE);
			} else {
				/* Indicate that phase shift must be applied to the demodulation of the first symbol of the next block */
				uc_apply_phase_shift = true;
			}

			/* Keep receiving payload */
			s_phy_rx_ctl.e_rx_step = STEP_RX_PAYLOAD;
			/* Clear Rx full */
			uc_clear_rx_full = true;

			break;

		case STEP_DEMOD_STD:
			/* Decrease pending symbols */
			s_phy_rx_ctl.m_us_rx_pending_symbols -= s_phy_rx_ctl.m_uc_next_demod_symbols;
#ifdef ENABLE_PYH_PROCESS_RECALL
			/* Force phy process recall when near the end of a reception */
			if (s_phy_rx_ctl.m_us_rx_pending_symbols < 10) {
				uc_force_phy_process_recall = 1;
			}
#endif
			/* Check whether next interrupt should be different than DEFAULT_SYM_NEXT symbols */
			if (s_phy_rx_ctl.m_us_rx_pending_symbols > (PAY_SYM_ONE_BEFORE_LAST_DEMOD + PAY_SYM_LAST_DEMOD)) {
				/* Get demodulator and set Default symbols for next demod, keep demod state */
				get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod, DEFAULT_SYM_NEXT,
						NO_READ_INACTIVE_CARRIERS, uc_apply_phase_shift);
			} else {
				/* Get demodulator, set One before last for next demod and state */
				get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod, PAY_SYM_ONE_BEFORE_LAST_DEMOD,
						NO_READ_INACTIVE_CARRIERS, uc_apply_phase_shift);
				s_phy_rx_ctl.e_demod_step = STEP_DEMOD_ONE_BEFORE_LAST;
			}

			if (s_phy_rx_ctl.e_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
				/* Reset phase shift control */
				uc_apply_phase_shift = false;
			}

			/* Keep receiving payload */
			s_phy_rx_ctl.e_rx_step = STEP_RX_PAYLOAD;
			/* Clear Rx full */
			uc_clear_rx_full = true;

			break;

		case STEP_DEMOD_ONE_BEFORE_LAST:
			/* Decrease pending symbols */
			s_phy_rx_ctl.m_us_rx_pending_symbols -= PAY_SYM_ONE_BEFORE_LAST_DEMOD;
			/* Get demodulator, set Last for next demod and state */
			get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod, PAY_SYM_LAST_DEMOD,
					NO_READ_INACTIVE_CARRIERS, uc_apply_phase_shift);
			s_phy_rx_ctl.e_demod_step = STEP_DEMOD_LAST;
#ifdef ENABLE_PYH_PROCESS_RECALL
			/* Force phy process recall when near the end of a reception */
			uc_force_phy_process_recall = 1;
#endif
			/* Keep receiving payload */
			s_phy_rx_ctl.e_rx_step = STEP_RX_PAYLOAD;
			/* Clear Rx full */
			uc_clear_rx_full = true;

			break;

		case STEP_DEMOD_LAST:
			/*Resets the resampling register for the next frame*/
			pplc_if_write32(REG_ATPL250_RESAMP24BITS_1_32, RESAMPLE_STEP);
			pplc_if_write32(REG_ATPL250_RESAMP24BITS_2_32, RESAMPLING_24BITS_2_VALUE);

			if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_RECEIVING_FIRST) {
				/* Last part of payload and configure for second RS block */

				if (s_phy_rx_ctl.m_us_rx_payload_symbols <= DEFAULT_SYM_NEXT) {
					/* If less or equal than DEFAULT_SYM_NEXT symbols of payload, set next demod symbols to payload symbols, and state to
					 * last demod */
					get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod,
							s_phy_rx_ctl.m_us_rx_payload_symbols, NO_READ_INACTIVE_CARRIERS, uc_apply_phase_shift);
					s_phy_rx_ctl.e_demod_step = STEP_DEMOD_LAST;
#ifdef ENABLE_PYH_PROCESS_RECALL
					/* Force phy process recall when near the end of a reception */
					uc_force_phy_process_recall = 1;
#endif
				} else {
					/* Otherwise, set first number of symbols to interrupt so there is 3 symbols left at the end for last 2 interrupts, */
					/* and set state to standard demod */
					if (((s_phy_rx_ctl.m_us_rx_payload_symbols % DEFAULT_SYM_NEXT) + DEFAULT_SYM_NEXT - 3) % DEFAULT_SYM_NEXT == 0) {
						get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod, DEFAULT_SYM_NEXT,
								NO_READ_INACTIVE_CARRIERS, uc_apply_phase_shift);
					} else {
						get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod,
								((s_phy_rx_ctl.m_us_rx_payload_symbols % DEFAULT_SYM_NEXT) + DEFAULT_SYM_NEXT - 3) % DEFAULT_SYM_NEXT,
								NO_READ_INACTIVE_CARRIERS, uc_apply_phase_shift);
					}

					s_phy_rx_ctl.e_demod_step = STEP_DEMOD_STD;
				}
			} else {
				/* Get Rx end time */
				if (s_phy_rx_ctl.e_mod_scheme == MOD_SCHEME_COHERENT) {
					us_payload_symbols = s_phy_rx_ctl.m_us_rx_payload_symbols + NUM_SYMBOLS_S1S2;
				} else {
					us_payload_symbols = s_phy_rx_ctl.m_us_rx_payload_symbols;
				}

				if (uc_working_band == WB_CENELEC_A) {
					ul_rx_end_time = ul_rx_sync_time +
							(uint32_t)(us_payload_symbols + uc_num_symbols_fch) * s_band_constants.ul_frame_symbol_duration + s_band_constants.ul_half_symbol_duration;
				} else {
					if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_RECEIVING_SECOND) {
						ul_rx_end_time = ul_rx_sync_time
								+ (uint32_t)(us_payload_symbols + s_phy_rx_ctl.m_us_rx_payload_symbols +
								uc_num_symbols_fch) * s_band_constants.ul_frame_symbol_duration
								+ s_band_constants.ul_half_symbol_duration;
					} else {
						ul_rx_end_time = ul_rx_sync_time +
								(uint32_t)(us_payload_symbols + uc_num_symbols_fch) * s_band_constants.ul_frame_symbol_duration + s_band_constants.ul_half_symbol_duration;
					}
				}

				/* Reset TXRX buf and FFT */
				end_rx_fft_txrx();

				/* Last part of payload */
				get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod, DEFAULT_SYM_NEXT,
						NO_READ_INACTIVE_CARRIERS, uc_apply_phase_shift);
				s_phy_rx_ctl.e_demod_step = STEP_DEMOD_STD;
			}

			/* Config RS decoder */
			ul_rs_cfg = (uint32_t)0x06 << 24; /* Add 6 zeroes to flush convolutional encoder */
			ul_rs_cfg |= (uint32_t)(s_phy_rx_ctl.m_us_rx_len) << 16; /* Payload length */
			ul_rs_cfg |= (uint32_t)(s_phy_rx_ctl.m_us_rx_len + s_phy_rx_ctl.m_uc_rs_parity + 1) << 8; /* Payload length + Parity for Chien */
			ul_rs_cfg |= (0x20 | s_phy_rx_ctl.m_uc_rs_parity); /* Enable syndrome and set parity */
			pplc_if_write32(REG_ATPL250_RS_CFG_32, ul_rs_cfg);

			/* Reset Raw data memory, so raw payload bits are stored from the beginning */
			/* (this is necessary only in case we have 2 RS blocks, but it is made generic for simplicity) */
			pplc_if_or8(REG_ATPL250_INTERLEAVER_CTL_H8, 0x08);
			if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_RECEIVING_FIRST) {
				disable_HW_chain_autorst();
				/* Set value to stop scrambler */
				pplc_if_write16(REG_ATPL250_INTERLEAVER_BPSCR_L16, (s_phy_rx_ctl.m_us_rx_len << 3) - 1);
			} else if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_RECEIVING_SECOND) {
				enable_HW_chain_autorst();
				/* Reset value to stop scrambler */
				pplc_if_write16(REG_ATPL250_INTERLEAVER_BPSCR_L16, 0x7FFF);
			}

			/* Set unload mode and send pulse to unload deinterleaver */
			switch (s_phy_rx_ctl.e_mod_type) {
			case MOD_TYPE_BPSK_ROBO:
			case MOD_TYPE_BPSK:
				pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x13);
				break;

			case MOD_TYPE_QPSK:
				pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x53);
				break;

			case MOD_TYPE_8PSK:
				pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x93);
				break;

			case MOD_TYPE_QAM: /* Not supported */
				pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0xD3);
				break;
			}

			if (s_phy_rx_ctl.e_rs_blocks != RS_BLOCKS_2_BLOCKS_RECEIVING_FIRST) {
				/* Get evm and rssi */
				switch (s_phy_rx_ctl.e_mod_type) {
				case MOD_TYPE_BPSK_ROBO:
				case MOD_TYPE_BPSK:
					s_phy_rx_ctl.m_us_evm_payload = atpl250_read_evm_bpsk();
					break;

				case MOD_TYPE_QPSK:
					s_phy_rx_ctl.m_us_evm_payload = atpl250_read_evm_qpsk();
					break;

				case MOD_TYPE_8PSK:
					s_phy_rx_ctl.m_us_evm_payload = atpl250_read_evm_8psk();
					break;

				case MOD_TYPE_QAM: /* Not supported */
					s_phy_rx_ctl.m_us_evm_payload = 0;
					break;
				}
				s_phy_rx_ctl.m_us_rssi = atpl250_read_rssi();

				/* Reset IOB */
				end_rx_iob();
				/* Reset evm and rssi */
				atpl250_reset_evm();
				atpl250_reset_rssi();
				/* Disable RSSI calculation for later channel estimation */
				atpl250_disable_rssi();
				atpl250_disable_evm();
			}

			/* Calculate number of bytes to read as payload raw data */
			us_bytes_to_read = s_phy_rx_ctl.m_us_rx_len + ((s_phy_rx_ctl.m_uc_rs_parity + 1) * 2);

			/* Wait until all bits are stored in raw data */
			uc_flag_rx_error = 0;
			us_bits_through_rx_chain = pplc_if_read16(REG_ATPL250_INTERLEAVER_INFO2_L16);
			while (us_bits_through_rx_chain < (us_bytes_to_read * 8 - 1)) {
				us_bits_through_rx_chain = pplc_if_read16(REG_ATPL250_INTERLEAVER_INFO2_L16);
				if (++uc_flag_rx_error > 100) {
					break;
				}
			}

			if (++uc_flag_rx_error > 100) {
				/* Clear pending symbols and reset Rx mode */
				s_phy_rx_ctl.m_us_rx_pending_symbols = 0;
				s_phy_rx_ctl.m_us_rx_len = 0;
				atpl250.m_ul_rx_exception++;
				end_rx();
				s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;

				LOG_PHY(("RxChainErr\r\n"));
				break; /* If error is detected, everything is already reset, so break to exit interrupt */
			} else {
				/* Disable HW chain to read raw payload */
				pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~0x01u));

				if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_RECEIVING_SECOND) {
					/* Read raw data from bit 0 */
					pplc_if_write16(REG_ATPL250_RAW_DATA_H16, 0);
					/* Read Payload + RS + Syndrome */
					pplc_if_read_rep(REG_ATPL250_RAW_DATA_VL8, 1, &s_phy_rx_ctl.auc_rx_buf[s_band_constants.uc_fch_len + us_bytes_to_read], us_bytes_to_read,
							true);
					us_rs_decode_idx = s_band_constants.uc_fch_len + us_bytes_to_read;
				} else {
					/* Read raw data from bit 0 */
					pplc_if_write16(REG_ATPL250_RAW_DATA_H16, 0);
					/* Read Payload + RS + Syndrome */
					pplc_if_read_rep(REG_ATPL250_RAW_DATA_VL8, 1, &s_phy_rx_ctl.auc_rx_buf[s_band_constants.uc_fch_len], us_bytes_to_read, true);
					us_rs_decode_idx = s_band_constants.uc_fch_len;
				}

				pplc_if_and8(REG_ATPL250_RS_CFG_VL8, (uint8_t)(~0x20u)); /* SYNDROM_EN=0 */
				pplc_if_or8(REG_ATPL250_RS_CFG_VL8, 0x40); /* ENCODER_EN=1 */

				/* Disable interrupt, so RS is performed before BER results are collected or before next symbols are processed in case of 2 RS
				 * blocks */
				disable_pplc_interrupt();

				/* BER */
				if (s_phy_rx_ctl.e_rs_blocks != RS_BLOCKS_2_BLOCKS_RECEIVING_FIRST) {
					/* Set Auto Ber for HW to start Ber automatically */
					pplc_if_or8(REG_ATPL250_BER_PERIPH_CFG1_VH8, 0x80); /* AUTO_BER='1' */
					e_ber_status = BER_PAYLOAD;
				}

				/* Clear Valid data */
				s_rx_ber_payload_data.uc_valid_data = 0;
				/* Simulate tx to run BER */
				pplc_if_or8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x2F); /* BPSK, Tx mode, Load mode, G3 (OR instead of write, to keep modulation on
				                                                     * 2 MSBits) */

				if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_RECEIVING_FIRST) {
					/* Clear Rx full */
					uc_clear_rx_full = true;
					/* Wait until interleaver is loaded */
					if (uc_legacy_mode) {
						num_intrlv_tx_max = 2;
					} else {
						switch (s_phy_rx_ctl.e_mod_type) {
						case MOD_TYPE_BPSK_ROBO:
						case MOD_TYPE_BPSK:
							num_intrlv_tx_max = 2;
							break;

						case MOD_TYPE_QPSK:
							num_intrlv_tx_max = 3;
							break;

						case MOD_TYPE_8PSK:
							num_intrlv_tx_max = 4;
							break;

						case MOD_TYPE_QAM: /* Not supported */
						default:
							num_intrlv_tx_max = 2;
							break;
						}
					}

					num_intrlv_tx = 0;
					us_flag_tx_error = 0;

					/* printf("%u\r\n", num_intrlv_tx_max); */

					while (num_intrlv_tx < num_intrlv_tx_max) {
						num_intrlv_tx = pplc_if_read8(REG_ATPL250_INTERLEAVER_INFO2_H8);
						if (++us_flag_tx_error > MAX_INTL_ITERATIONS) {
							break;
						}
					}

					/* printf("%u\r\n", us_flag_tx_error); */

					if (++us_flag_tx_error > MAX_INTL_ITERATIONS) {
						/* Error in simulated Tx for BER of RS first block */
						LOG_PHY(("Intl Err (BER) first RS block\r\n"));
						enable_pplc_interrupt();
						phy_reset_request();
						/* Exit case */
						break;
					}

					/* Reset interleaver for second RS block */
					pplc_if_or8(REG_ATPL250_INTERLEAVER_CTL_H8, 0x01);
				}

				/* Set flag to run RS */
				uc_phy_generic_flags |= PHY_GENERIC_FLAG_CHECK_RS;
			}

			/* End of payload reception, check RS blocks to continue */
			if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_1_BLOCK) {
				/* Clear pending symbols and reset Rx mode */
				s_phy_rx_ctl.m_us_rx_pending_symbols = 0;
				s_phy_rx_ctl.e_rx_step = STEP_RX_WAIT_RS;
			} else if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_RECEIVING_FIRST) {
				/* Clear pending symbols and reset Rx mode */
				s_phy_rx_ctl.m_us_rx_pending_symbols = s_phy_rx_ctl.m_us_rx_payload_symbols;
				s_phy_rx_ctl.e_rx_step = STEP_RX_PAYLOAD_FIRST;
				s_phy_rx_ctl.e_rs_blocks = RS_BLOCKS_2_BLOCKS_RECEIVING_SECOND;
				/* Flag to clear Rx full will be set on BER interrupt */
			} else if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_RECEIVING_SECOND) {
				/* Clear pending symbols and reset Rx mode */
				s_phy_rx_ctl.m_us_rx_pending_symbols = 0;
				s_phy_rx_ctl.e_rx_step = STEP_RX_WAIT_RS;
				s_phy_rx_ctl.e_rs_blocks = RS_BLOCKS_2_BLOCKS_SECOND_RECEIVED;
			}

			break;
		}

		break;

	case STEP_RX_NO_RX:
		/* Not valid in this interrupt */
		/* Just in case, clear Rx full bit and cancel reception */
		uc_clear_rx_full = true;
		end_rx();
		break;

	default:
		break;
	}
}

/**
 * \brief Handler for Inout buffer interrupt
 *
 */
void handle_iob_int(void)
{
	uint16_t us_iob_ctl_h16;

#ifdef ENABLE_SIGNAL_DUMP
	uint8_t b;
	int16_t val;
	uint16_t i;
	uint16_t us_agc_factor;

	if (atpl250.m_uc_trigger_signal_dump > 0) {
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, 0xFFFF);

		pplc_if_read_buf(BCODE_ZONE4 | 0x0000, &auc_dump_signal[uc_dumped_buffers][0], 512, true);
		pplc_if_read_buf(BCODE_ZONE4 | 0x0100, &auc_dump_signal[uc_dumped_buffers][512], 512, true);
		pplc_if_read_buf(BCODE_ZONE4 | 0x0200, &auc_dump_signal[uc_dumped_buffers][1024], 512, true);
		pplc_if_read_buf(BCODE_ZONE4 | 0x0300, &auc_dump_signal[uc_dumped_buffers][1536], 512, true);

		uc_clear_rx_full = true;
		uc_dumped_buffers++;
		if (uc_dumped_buffers == DUMP_BUFFERS) {
			us_agc_factor = pplc_if_read16(REG_ATPL250_RSSI_H16);
			atpl250_txrxb_push_rst();
			printf("AGC Factor %hu\r\n", us_agc_factor);

			for (b = 0; b < DUMP_BUFFERS; b++) {
				for (i = 0; i < 2048; i += 2) {
					val = (auc_dump_signal[b][i] << 8) | (auc_dump_signal[b][i + 1] & 0x00FF);
					printf("%d\r\n", val);
					fflush(stdout);
					for (ul_loop = 0; ul_loop < 3000; ul_loop++) {
					}
				}
			}

			/* Clear real mode */
			atpl250_clear_iob_real_mode();
			/* Set back SPI speed after dumping */
			pplc_if_set_speed(PPLC_CLOCK_24M);
			/* Clear flag to stop dumping */
			atpl250.m_uc_trigger_signal_dump = 0;
			/* Ensure Rx state is idle */
			s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
			/* Set flag to reset Phy layer after dump */
			uc_phy_generic_flags |= PHY_GENERIC_FLAG_INIT_PHY_PARAMS;
		}

		/* Clear interrupt flag */
		atpl250_clear_iob_int();

		/* Do nothing else but dump */
		return;
	}
#endif

	if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_ADAPT) {
		if (s_phy_tx_ctl.e_tx_step != STEP_TX_NO_TX) {
			/* Look for chirp (this will abort noise capture and Noise Error will be generated) */
			pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x01);
			s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_ABORTED;
		} else {
			/* Read AGC Factor*/
			us_noise_agc_factor = pplc_if_read16(REG_ATPL250_RSSI_H16);
			/* Move to next step */
			s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_FIRST_PASS;
		}

		/* Change number of symbols and clear rx full */
		atpl250_set_num_symbols_cfg(NOISE_CAPTURE_SYMBOLS);
		uc_clear_rx_full = true;
	} else if ((s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_FIRST_PASS) || (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_INTERMEDIATE_PASS) ||
			(s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_SECOND_PASS)) {
		/* Handle Noise Capture */
		_analyse_noise_capture();
	} else if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_ABORTED) {
		/* Clear Rx full */
		uc_clear_rx_full = true;
		LOG_PHY(("Noise Capture aborted\r\n"));
		#ifdef NOISE_CAPTURE_USING_FALSE_PEAK
		/* Set Rx step */
		s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
		#endif
	} else {
		/* Read InOut buffer ctl */
		us_iob_ctl_h16 = pplc_if_read16(REG_ATPL250_INOUTB_CTL_H16);

		if ((us_iob_ctl_h16 & 0x0100) && !(us_iob_ctl_h16 & 0x0400)) { /* TX_MODE=1 && SYM_READY=0 */
			handle_iob_int_tx();
		} else if ((us_iob_ctl_h16 & 0x0002) && (us_iob_ctl_h16 & 0x0200)) { /* RX_MODE=1 && RX_FULL=1 */
			handle_iob_int_rx();
		} else {
			LOG_PHY(("IOB No Tx No Rx\r\n"));
		}
	}

	/* Clear interrupt flag */
	atpl250_clear_iob_int();
}

/**
 * \brief Handler for Peak 1 interrupt
 *
 */
void handle_peak1_int(void)
{
	/* Just in case we were capturing noise, set back FFT_SHIFT */
	pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, s_band_constants.uc_rx_fft_shift);

	/* Set number of symbols after rx is blocked for SFO correction */
	atpl250_set_symbols_rx_block(uc_num_symbols_fch + NUM_SYMBOLS_S1S2 + s_band_constants.uc_pay_sym_first_demod);

	/* Set number of symbols for first interrupt */
	s_phy_rx_ctl.m_us_rx_pending_symbols = uc_num_symbols_fch;
	if (uc_working_band == WB_CENELEC_A) {
		s_phy_rx_ctl.m_uc_next_demod_symbols = NUM_FCH_SYM_AFTER_PEAK2_CENELEC_A;
	} else {
		s_phy_rx_ctl.m_uc_next_demod_symbols = NUM_FCH_SYM_AFTER_PEAK2_FCC_ARIB;
	}

	if (uc_working_band == WB_FCC) {
		/* Set number of FCH symbols for H update*/
		ss_num_sym_fch_for_H_update = 255; /* No update in no-legacy mode. In legacy mode, final value will be determined when the SFO is estimated. */
		ss_num_fch_since_last_H_update = 255;
	}

	/* Reset number of FCH symbols */
	uc_num_sym_valid_fch = 0;

	/*
	 *      s_phy_rx_ctl.m_uc_next_demod_symbols = uc_num_symbols_fch % uc_num_sym_block_demod_fch;
	 *      if (s_phy_rx_ctl.m_uc_next_demod_symbols == 0) {
	 *              s_phy_rx_ctl.m_uc_next_demod_symbols = uc_num_sym_block_demod_fch;
	 *      }
	 */

	/* Set IO buffer to first interrupt symbols */
	atpl250_set_num_symbols_cfg(s_phy_rx_ctl.m_uc_next_demod_symbols);

#ifdef ENABLE_SIGNAL_DUMP
	if (atpl250.m_uc_trigger_signal_dump == 0) {
		/* Set path TXRXB -> FFT */
		atpl250_set_iobuf_to_fft();
	}

#else
	/* Set path TXRXB -> FFT */
	atpl250_set_iobuf_to_fft();
#endif

	/* Check Noise Error flag */
	if (ul_phy_int_flags & INT_NOISE_ERROR_MASK_32) {
		/* Clear flag */
		ul_phy_int_flags &= ~INT_NOISE_ERROR_MASK_32;
		/* Clear interrupt flag */
		atpl250_clear_noise_error_int();
		/* Set flag for Noise Capture */
		uc_phy_generic_flags |= PHY_GENERIC_FLAG_NOISE_CAPTURE;
	}

	/* Check Noise Capture Start flag */
	if (ul_phy_int_flags & INT_NOISE_CAPTURE_MASK_32) {
		/* Clear flag */
		ul_phy_int_flags &= ~INT_NOISE_CAPTURE_MASK_32;
		/* Clear interrupt flag */
		atpl250_clear_noise_capture_int();
	}

	us_agc_int =  atpl250_read_agc_int();
	uc_agc_ext =  atpl250_read_agcs_ext();

	s_phy_rx_ctl.m_us_agc_factor = atpl250_read_agc_factor();
	s_phy_rx_ctl.m_uc_agcs_active = atpl250_read_agcs_active();
	s_phy_rx_ctl.m_us_agc_fine = atpl250_read_agc_fine();

	atpl250_force_agc_ext(s_phy_rx_ctl.m_uc_agcs_active);
	atpl250_force_agc_int(s_phy_rx_ctl.m_us_agc_fine);

	LOG_PHY(("P1\r\n"));

	s_phy_rx_ctl.e_rx_step = STEP_RX_PEAK1;
	s_phy_rx_ctl.m_ul_start_rx_watch_ms = oss_get_up_time_ms();

	/* Clear interrupt flag */
	atpl250_clear_peak1_int();
}

/**
 * \brief Handler for Peak 2 interrupt
 *
 */
void handle_peak2_int(void)
{
	LOG_PHY(("P2\r\n"));

	/* Speed up SPI transactions*/
	pplc_if_set_speed(PPLC_CLOCK_24M);

	s_phy_rx_ctl.e_rx_step = STEP_RX_PEAK2;

	_zcd_calc_pdc_rx();

	/* Get sync time */
	ul_rx_sync_time = pplc_if_read32(REG_ATPL250_PEAK2_TIME_32);

	/* Configure Rotator */
	pplc_if_write32(REG_ATPL250_ROTATOR_CONFIG0_32, 0x0008A3AE);
	pplc_if_write32(REG_ATPL250_ROTATOR_CONFIG3_32, 0x0502007F);
	pplc_if_write8(REG_ATPL250_ROTATOR_CONFIG3_H8, 0x01);

	/* Channel estimation from preamble */
	chn_estimation_from_preamble();

	/* BER initialization */
	ber_init();

	/* Clear interrupt flag */
	atpl250_clear_peak2_int();
}

/**
 * \brief Handler for No Peak 2 interrupt
 *
 */
void handle_no_peak2_int(void)
{
	/* If no Peak2 is detected, reset reception to be ready for another one */
	LOG_PHY(("No Peak2\r\n"));
	s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
	atpl250.m_ul_rx_false_positive++;
	end_rx();

	/* Clear interrupt flag */
	atpl250_clear_no_peak2_int();
}

/**
 * \brief Handler for Peak 1 3 interrupt
 *
 */
void handle_peak13_int(void)
{
	/* Not used for G3 */

	/* Clear interrupt flag */
	atpl250_clear_peak13_int();
}

/**
 * \brief Handler for Zero Cross interrupt
 *
 */
void handle_zero_cross_int(void)
{
	/* TODO: Implement */

	/* Clear interrupt flag */
	atpl250_clear_zc_int();
}

/**
 * \brief Handler for Viterbi interrupt
 *
 */
void handle_viterbi_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_vtb_int();
}

/**
 * \brief Handler for Reed Solomon interrupt
 *
 */
void handle_reed_solomon_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_rs_int();
}

/**
 * \brief Handler for Start Tx 1 interrupt
 *
 */
void handle_start_tx_1_int(void)
{
	uint8_t uc_cd_int;

	/* Slow down SPI transactions*/
	pplc_if_set_speed(PPLC_CLOCK_15M);

	atpl250_sync_release_rst(); /* Just in case it was pushed (ACK) */

	enable_CD_interrupt();
	sb_cd_int_enabled = true;
	atpl250_clear_cd_int();
	uc_cd_int = atpl250_read_cd_int();
	if (uc_cd_int) {
		pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x01);
		s_phy_tx_ctl.e_tx_step = STEP_TX_CANCELLED_TX;
		uc_set_sym_ready = false;
		atpl250_clear_start_tx1_int();
		return;
	}

	if (s_phy_rx_ctl.e_rx_step != STEP_RX_NO_RX) {
		pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x0F); /* Cancel any programmed transmission */
		s_phy_tx_ctl.e_tx_step = STEP_TX_CANCELLED_TX;
		uc_set_sym_ready = false;
		/* Clear interrupt flag */
		atpl250_clear_start_tx1_int();
		return;
	}

	if (pplc_if_read8(REG_ATPL250_INTERLEAVER_CTL_VL8) & 0x01) {
		pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x0F); /* Cancel any programmed transmission */
		s_phy_tx_ctl.e_tx_step = STEP_TX_CANCELLED_TX;
		uc_set_sym_ready = false;
		/* Clear interrupt flag */
		atpl250_clear_start_tx1_int();
		return;
	}

#if defined(CALCULATE_TX_PDC)
	_zcd_calc_pdc_tx_start();
#endif
	/* This should only happen on TX Preamble Step */
	if (s_phy_tx_ctl.e_tx_step == STEP_TX_PREAMBLE_FIRST) {
		/* Set path TXRXB -> FFT */
		atpl250_set_iobuf_to_fft();

	#ifndef TX_PREAMBLE_HW
		/* Set next state to send header first segment */
		s_phy_tx_ctl.e_tx_step = STEP_TX_PREAMBLE_LAST;
		/* Set FFT_SHIFT for Tx */
		pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, s_band_constants.uc_tx_fft_shift);
		/* Start transmitting preamble */
		_tx_preamble_first();
	#else
		if (uc_working_band == WB_FCC) {
			generate_inactive_carriers_fcc(s_phy_tx_ctl.m_auc_tone_map, s_phy_tx_ctl.m_auc_inactive_carriers_pos);
			generate_inactive_carriers_fcc(s_phy_tx_ctl.m_auc_inv_tone_map, s_phy_tx_ctl.m_auc_inv_inactive_carriers_pos);
		} else if (uc_working_band == WB_ARIB) {
			generate_inactive_carriers_arib(s_phy_tx_ctl.m_auc_tone_map, s_phy_tx_ctl.m_auc_inactive_carriers_pos);
			generate_inactive_carriers_arib(s_phy_tx_ctl.m_auc_inv_tone_map, s_phy_tx_ctl.m_auc_inv_inactive_carriers_pos);
		} else {
			generate_inactive_carriers_cenelec_a(s_phy_tx_ctl.m_auc_tone_map[0], s_phy_tx_ctl.m_auc_inactive_carriers_pos);
			generate_inactive_carriers_cenelec_a(s_phy_tx_ctl.m_auc_inv_tone_map[0], s_phy_tx_ctl.m_auc_inv_inactive_carriers_pos);
		}

		/* Generate the combination of inactive and notched carriers */
		for (uc_i = 0; uc_i < CARR_BUFFER_LEN; uc_i++) {
			s_phy_tx_ctl.m_auc_static_and_dynamic_notching_pos[uc_i] = auc_static_notching_pos[uc_i] |
					s_phy_tx_ctl.m_auc_inactive_carriers_pos[uc_i];
			s_phy_tx_ctl.m_auc_static_and_inv_dynamic_notching_pos[uc_i] = auc_static_notching_pos[uc_i] |
					s_phy_tx_ctl.m_auc_inv_inactive_carriers_pos[uc_i];
		}

		s_phy_tx_ctl.m_uc_num_active_carriers
			= get_active_carriers(s_phy_tx_ctl.m_auc_inactive_carriers_pos, (uint8_t *)auc_static_notching_pos);

		s_phy_tx_ctl.m_uc_num_pilots = s_phy_tx_ctl.m_uc_num_active_carriers / PILOT_FREQ_SPA;
		if (s_phy_tx_ctl.m_uc_num_active_carriers % PILOT_FREQ_SPA) {
			s_phy_tx_ctl.m_uc_num_pilots++;
		}

		if (uc_working_band == WB_CENELEC_A) {
			/* Calculate FCH parameters */
			phy_fch_encode_g3_cenelec_a(&s_phy_tx_ctl, auc_fch);
		} else {
			/* Calculate FCH parameters */
			phy_fch_encode_g3_fcc_arib(&s_phy_tx_ctl, auc_fch);
		}

		/* Copy FCH */
		if (s_phy_tx_ctl.e_rs_blocks == RS_BLOCKS_1_BLOCK) {
			atpl250.m_us_last_tx_msg_payload_symbols = s_phy_tx_ctl.m_us_tx_payload_symbols;
		} else {
			atpl250.m_us_last_tx_msg_payload_symbols = s_phy_tx_ctl.m_us_tx_payload_symbols << 1;
		}

		memcpy(&s_phy_tx_ctl.auc_tx_buf[0], auc_fch, s_band_constants.uc_fch_len);
		/* Write Header in Raw Data	 */
		write_raw_header(&s_phy_tx_ctl.auc_tx_buf[0]);

		uc_cd_int = atpl250_read_cd_int();
		if (uc_cd_int) {
			pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x01);
			s_phy_tx_ctl.e_tx_step = STEP_TX_CANCELLED_TX;
			uc_set_sym_ready = false;
			atpl250_clear_start_tx1_int();
			return;
		}

		/* Set next state to send header */
		s_phy_tx_ctl.e_tx_step = STEP_TX_HEADER;
		/* Set FFT_SHIFT for Tx */
		pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, s_band_constants.uc_tx_fft_shift);
		/* Set reference preamble and start transmitting header */
		_write_ref_preamble_and_header_first();
	#endif

		uc_set_sym_ready = true;
	}

	if (uc_set_sym_ready == false) {
		uc_set_sym_ready = phy_extension_handler_start_tx();
	}

	/* Disable possible configured notch filters */
	atpl250_enable_rrc_notch_filter(0);

	/* Clear interrupt flag */
	atpl250_clear_start_tx1_int();
}

/**
 * \brief Handler for Start Tx 2 interrupt
 *
 */
void handle_start_tx_2_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_start_tx2_int();
}

/**
 * \brief Handler for Start Tx 3 interrupt
 *
 */
void handle_start_tx_3_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_start_tx3_int();
}

/**
 * \brief Handler for Start Tx 4 interrupt
 *
 */
void handle_start_tx_4_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_start_tx4_int();
}

/**
 * \brief Handler for End Tx 1 interrupt
 *
 */
void handle_end_tx_1_int(void)
{
	/* Get Tx end time */
	ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32) - CFG_TXRX_PLC_OFF1_US - s_band_constants.ul_end_tx_offset_us;
	disable_CD_interrupt();
	sb_cd_int_enabled = false;

	if (s_phy_tx_ctl.e_tx_step == STEP_TX_END) {
		/* Stop transmitting */
		LOG_PHY(("EndT OK\r\n"));
		end_tx();
		atpl250.m_ul_tx_total++;
		atpl250.m_ul_tx_total_bytes += s_phy_tx_ctl.m_us_payload_len;
		s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_SUCCESS;
		s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;
	} else if (s_phy_tx_ctl.e_tx_step == STEP_TX_CANCELLED_TX) {
		LOG_PHY(("EndT Cancelled\r\n"));
		s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_BUSY_RX;
		s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;
		atpl250.m_ul_tx_bad_busy_channel++;
		atpl250.m_ul_tx_total_errors++;
	} else {
		LOG_PHY(("EndTx NO TX_END\r\n"));
		end_tx();
		s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_TIMEOUT;
		s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;
		atpl250.m_ul_tx_bad_busy_channel++;
		atpl250.m_ul_tx_total_errors++;
	}

	/* Enable possible configured notch filters */
	atpl250_enable_rrc_notch_filter(atpl250.m_uc_rrc_notch_active);

	/* Clear interrupt flag */
	atpl250_clear_end_tx1_int();

	/* Set flag to perform end of transmission */
	uc_phy_generic_flags |= PHY_GENERIC_FLAG_END_TX;
}

/**
 * \brief Handler for End Tx 2 interrupt
 *
 */
void handle_end_tx_2_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_end_tx2_int();
}

/**
 * \brief Handler for End Tx 3 interrupt
 *
 */
void handle_end_tx_3_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_end_tx3_int();
}

/**
 * \brief Handler for End Tx 4 interrupt
 *
 */
void handle_end_tx_4_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_end_tx4_int();
}

/**
 * \brief Handler for CD Tx 1 interrupt
 *
 */
void handle_cd_tx_1_int(void)
{
	disable_CD_interrupt();
	sb_cd_int_enabled = false;
	/* Abort transmission */
	pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x01);

	if ((s_phy_tx_ctl.e_tx_step == STEP_TX_HEADER) || (s_phy_tx_ctl.e_tx_step == STEP_TX_HEADER_FIRST) ||
			(s_phy_tx_ctl.e_tx_step == STEP_TX_PREAMBLE_LAST)) {
		/*tx_start has been executed*/
		clear_tx_chain();
		end_tx();
	}

	atpl250.m_ul_tx_bad_busy_channel++;
	atpl250.m_ul_tx_total_errors++;
	s_phy_tx_ctl.e_tx_step = STEP_TX_CANCELLED_TX;
	uc_set_sym_ready = false;

	/* Enable possible configured notch filters */
	atpl250_enable_rrc_notch_filter(atpl250.m_uc_rrc_notch_active);

	/* Clear interrupt flag */
	atpl250_clear_cd_tx1_int();
}

/**
 * \brief Handler for CD Tx 2 interrupt
 *
 */
void handle_cd_tx_2_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_cd_tx2_int();
}

/**
 * \brief Handler for CD Tx 3 interrupt
 *
 */
void handle_cd_tx_3_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_cd_tx3_int();
}

/**
 * \brief Handler for CD Tx 4 interrupt
 *
 */
void handle_cd_tx_4_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_cd_tx4_int();
}

/**
 * \brief Handler for Tx 1 Overlap interrupt
 *
 */
void handle_overlap_tx_1_int(void)
{
	LOG_PHY(("OvTxErr\r\n"));
	atpl250.m_ul_tx_bad_busy_tx++;
	atpl250.m_ul_tx_total_errors++;

	/* Clear interrupt flag */
	atpl250_clear_overlap_tx1_int();
}

/**
 * \brief Handler for Tx 2 Overlap interrupt
 *
 */
void handle_overlap_tx_2_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_overlap_tx2_int();
}

/**
 * \brief Handler for Tx 3 Overlap interrupt
 *
 */
void handle_overlap_tx_3_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_overlap_tx3_int();
}

/**
 * \brief Handler for Tx 4 Overlap interrupt
 *
 */
void handle_overlap_tx_4_int(void)
{
	/* Not used */

	/* Clear interrupt flag */
	atpl250_clear_overlap_tx4_int();
}

/**
 * \brief Handler for Rx Error interrupt
 *
 */
void handle_rx_error_int(void)
{
	/* First, look for chirp, to avoid symbols to keep going to IOB */
	pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x01);

	/* Clear Noise measure */
	pplc_if_and8(REG_ATPL250_TXRXB_STATE_VL8, 0xEF);

	/* Clear pending symbols and reset Rx mode */
	s_phy_rx_ctl.m_us_rx_pending_symbols = 0;
	s_phy_rx_ctl.m_us_rx_len = 0;
	atpl250.m_ul_rx_exception++;

	if (s_phy_rx_ctl.e_rx_step == STEP_RX_PEAK1) {
		atpl250_syncm_detector_push_rst();
		atpl250_syncm_detector_release_rst();
	}

	if ((s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_ADAPT) || (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_FIRST_PASS) ||
			(s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_INTERMEDIATE_PASS) || (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_SECOND_PASS) ||
			(pplc_if_read32(REG_ATPL250_INT_FLAGS_32) & INT_NOISE_CAPTURE_MASK_32)) {
		LOG_PHY(("RxErr NC\r\n"));
		/* Look for chirp */
		pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x01);
		s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_ABORTED;
		#ifndef NOISE_CAPTURE_USING_FALSE_PEAK
		uc_flag_correct_delta = 1;
		#endif
	} else {
		LOG_PHY(("RxErr Step %u\r\n", s_phy_rx_ctl.e_rx_step));
		end_rx();
		s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
	}

	/* Clear interrupt flag */
	atpl250_clear_rx_error_int();
}

/**
 * \brief Handler for Tx Error interrupt
 *
 */
void handle_tx_error_int(void)
{
	disable_CD_interrupt();
	sb_cd_int_enabled = false;
	LOG_PHY(("TxErr\r\n"));
	if ((s_phy_tx_ctl.e_tx_step == STEP_TX_NO_TX) || (s_phy_tx_ctl.e_tx_step == STEP_TX_PREAMBLE_FIRST)) {
		s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_BUSY_RX;
		s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;
		atpl250.m_ul_tx_bad_busy_channel++;
		atpl250.m_ul_tx_total_errors++;
	} else {
		/* Stop transmitting */
		LOG_PHY(("EndTx Err\r\n"));
		end_tx();
		s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_TIMEOUT;
		s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;
		atpl250.m_ul_tx_timeout++;
		atpl250.m_ul_tx_total_errors++;
	}

	/* Clear interrupt flag */
	atpl250_clear_tx_error_int();

	/* Get Tx end time */
	ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);

	/* Set flag to perform Tx end */
	uc_phy_generic_flags |= PHY_GENERIC_FLAG_END_TX;
}

/**
 * \brief Handler for Noise Error interrupt
 *
 */
void handle_noise_error_int(void)
{
	uint32_t ul_int_flags;

	LOG_PHY(("NoiseErr\r\n"));

	/* Check Rx state */
	/* This error can raise after noise capture has started or instead of, and they have to be handled differently */
	if ((s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_ADAPT) || (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_FIRST_PASS) ||
			(s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_INTERMEDIATE_PASS) || (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_SECOND_PASS) ||
			(s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_ABORTED)) {
		/* Set back FFT_SHIFT */
		pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, s_band_constants.uc_rx_fft_shift);

		/* Reset IOB and FFT, unless peak already detected */
		atpl250_clear_iobuf_to_fft();
		ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
		if (!(ul_int_flags & INT_PEAK1_MASK_32)) {
			atpl250_iob_and_fft_push_rst();
			atpl250_iob_and_fft_release_rst();
		}

		/* Just in case, clear Rx full bit */
		uc_clear_rx_full = true;

		/* Set Rx step */
		s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
	}

	/* Set flag for Noise Capture */
	uc_phy_generic_flags |= PHY_GENERIC_FLAG_NOISE_CAPTURE;

	/* Clear interrupt flag */
	atpl250_clear_noise_error_int();
}

/**
 * \brief Handler for Noise Capture Start interrupt.
 *
 */
void handle_noise_capture_start_int(void)
{
	/* Noise capture started */
	if (s_phy_tx_ctl.e_tx_step != STEP_TX_NO_TX) {
		/* Clear Noise measure */
		pplc_if_and8(REG_ATPL250_TXRXB_STATE_VL8, 0xEF);
		/* Look for chirp (this will abort noise capture and Noise Error will be generated) */
		pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x01);
		s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_ABORTED;
	} else {
		/* Change FFT_SHIFT to avoid saturation */
		pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, NOISE_CAPTURE_FFT_SHIFT);
		/* Config IOB to interrupt after NOISE_CAPTURE_SYMBOLS symbols */
		atpl250_set_num_symbols_cfg(s_band_constants.uc_noise_capture_adapt_symbols);
		/* Set Path FFT -> IOB */
		atpl250_set_iobuf_to_fft();
		/* Disable possible configured notch filters */
		atpl250_enable_rrc_notch_filter(0);
		/* Set Rx step */
		s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_ADAPT;
		/* Get reference time */
		s_phy_rx_ctl.m_ul_start_rx_watch_ms = oss_get_up_time_ms();

		/* Clear Noise measure */
		pplc_if_and8(REG_ATPL250_TXRXB_STATE_VL8, 0xEF);
	}

	/* Clear interrupt flag */
	atpl250_clear_noise_capture_int();
}

/**
 * \brief Handler for BER interrupt
 *
 */
void handle_ber_int(void)
{
	uint8_t uc_num_sym_h_est_fch;

	if (uc_working_band == WB_CENELEC_A) {
		uc_num_sym_h_est_fch = NUM_SYM_H_EST_FCH_CENELEC_A;
	} else {
		uc_num_sym_h_est_fch = NUM_SYM_H_EST_FCH_FCC_ARIB;
	}

	if (e_ber_status == BER_FCH) {
		ber_save_fch_info(&s_rx_ber_fch_data);

		if (s_phy_rx_ctl.e_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
			/* Update channel and SFO only if payload is longer than 16 symbols */
			if (s_phy_rx_ctl.m_us_rx_payload_symbols >= s_band_constants.uc_min_sym_for_offset_correction_dif) {
				/* Change number of symbols after rx is blocked, as there is no S1S2 */
				atpl250_set_symbols_rx_block(uc_num_symbols_fch + s_band_constants.uc_pay_sym_first_demod);

				chn_estimation_from_fch(MOD_SCHEME_DIFFERENTIAL, uc_num_sym_h_est_fch);

				sampling_error_est_from_preamble_fch_s1s2(MOD_SCHEME_DIFFERENTIAL, SFO_FOR_PAYLOAD);
			} else {
				/* Reset number of symbols after rx is blocked, as there is no SFO correction */
				atpl250_set_symbols_rx_block(0xFFFF);
			}

			/* Clean reference to demodulate payload in differential */
			pplc_if_write8(REG_ATPL250_INOUTB_CONF2_H8, 0x00); /* H_1H_BYPASS='0'; H_1H='0'; DEST_Y='0'; SOURCE_H='0' */
		} else {
#if (NUM_SYM_H_EST_FCH_COH > 0)
			chn_estimation_from_fch(MOD_SCHEME_COHERENT, NUM_SYM_H_EST_FCH_COH);
#else
			uc_num_sym_valid_fch = 0;
#endif
		}

		if (uc_working_band == WB_CENELEC_A) {
			ber_config_cenelec_a(&s_phy_rx_ctl);
		} else {
			ber_config_fcc_arib(&s_phy_rx_ctl);
		}

		/* Clear Rx full to start receiving Payload */
		uc_clear_rx_full = true;
	} else if (e_ber_status == BER_PAYLOAD) {
		sb_ber_int_received = true;
		/* Interrupt after rx payload. */

		ber_save_payload_info(&s_rx_ber_payload_data);
		atpl250_ber_push_rst();
		atpl250_ber_release_rst();

		/* Prepare for next Rx */
		prepare_next_rx();
	}

	/* Disable Auto BER */
	pplc_if_and8(REG_ATPL250_BER_PERIPH_CFG1_VH8, 0x7F); /* AUTO_BER='0' */

	/* Clear interrupt flag */
	atpl250_clear_ber_int();
}

/**
 * \brief Handler for CD interrupt
 *
 */
void handle_cd_int(void)
{
	if (sb_cd_int_enabled) {
		uint32_t ul_curr_time, ul_txrx_pad_time, ul_start_tx_time;
		uint16_t aus_tx_time_reg[4] = {REG_ATPL250_TX_TIME1_32, REG_ATPL250_TX_TIME2_32, REG_ATPL250_TX_TIME3_32, REG_ATPL250_TX_TIME4_32};
		uint8_t uc_forced =  s_phy_tx_ctl.m_uc_tx_mode  & TX_MODE_FORCED_TX;
		uint8_t uc_correct_interval = 0;
		ul_curr_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
		ul_start_tx_time = pplc_if_read32(aus_tx_time_reg[0]);
		ul_txrx_pad_time = ul_start_tx_time + s_band_constants.ul_txrx_time_us;

		if (ul_txrx_pad_time < (s_band_constants.ul_txrx_time_us)) {
			/* Overflow */
			if ((ul_curr_time < ul_txrx_pad_time) || (ul_curr_time > ul_start_tx_time)) {
				uc_correct_interval = 1;
			}
		} else {
			if ((ul_curr_time < ul_txrx_pad_time) && (ul_curr_time > ul_start_tx_time)) {
				uc_correct_interval = 1;
			}
		}

		if (!uc_forced && s_phy_tx_ctl.e_tx_step != STEP_TX_CANCELLED_TX) {
			if (uc_correct_interval && (s_phy_tx_ctl.e_tx_step != STEP_TX_NO_TX)) {
				/* we are between tx_start interrupt and tx time and start_tx int has been fully executed */
				disable_CD_interrupt();
				sb_cd_int_enabled = false;
				/* Abort Tx */
				pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x01);

				/* Reset interpolator and emit */
				pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x40);
				pplc_if_or8(REG_ATPL250_GLOBAL_SRST_VH8, 0x80);

				if ((s_phy_tx_ctl.e_tx_step == STEP_TX_HEADER) || (s_phy_tx_ctl.e_tx_step == STEP_TX_HEADER_FIRST) ||
						(s_phy_tx_ctl.e_tx_step == STEP_TX_PREAMBLE_LAST)) {
					/* tx_start has been executed */
					clear_tx_chain();
					end_tx();
				}

				/* Release Reset interpolator and emit */
				pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0xBF);
				pplc_if_and8(REG_ATPL250_GLOBAL_SRST_VH8, 0x7F);

				s_phy_tx_ctl.e_tx_step = STEP_TX_CANCELLED_TX;
				uc_set_sym_ready = false;

				/* Enable possible configured notch filters */
				atpl250_enable_rrc_notch_filter(atpl250.m_uc_rrc_notch_active);
			}
		}
	}

	/* Clear interrupt flag */
	atpl250_clear_cd_int();
}

/**
 * \brief Handler for SPI Error interrupt
 *
 */
void handle_spi_error_int(void)
{
	LOG_PHY(("SpiErr\r\n"));

	/* Clear interrupt flag */
	atpl250_clear_spi_err_int();
}

/**
 * \brief Handler for Start from Reset interrupt
 *
 */
void handle_start_int(void)
{
	LOG_PHY(("Start\r\n"));

	/* Initialize PHY layer parameters */
	_init_phy_layer();

	/* Clear interrupt flag */
	atpl250_clear_start_int();

	/* Set flag to indicate interrupt is executed */
	uc_start_interrupt_executed = 1;
}

/**
 * \brief Create PHY tasks, queues and semaphores
 * Initialize physical parameters and configure ATPL250 device
 *
 * \param serialEnable  Enables PHY layer serialization
 * \param u8Band  Selects PHY layer Working Band
 *
 */
void phy_init(uint8_t uc_serial_enable, uint8_t uc_band)
{
	uint32_t ul_timeout;

	/* Set band */
	uc_working_band = uc_band;

	/* Set constants depending on band */
	if (uc_working_band == WB_FCC) {
		atpl250.m_ul_version = ATPL250_VERSION_NUM_FCC;
		p_emit_gain_limits = emit_gain_limits_fcc;
		p_caul_max_rms_hi = caul_max_rms_hi_fcc;
		p_caul_max_rms_vlo = caul_max_rms_vlo_fcc;
		p_caul_th1_hi = caul_th1_hi_fcc;
		p_caul_th2_hi = caul_th2_hi_fcc;
		p_caul_th1_lo = caul_th1_lo_fcc;
		p_caul_th2_lo = caul_th2_lo_fcc;
		p_caul_th1_vlo = caul_th1_vlo_fcc;
		p_caul_th2_vlo = caul_th2_vlo_fcc;
		s_band_constants.uc_fch_len = FCH_LEN_FCC;
		s_band_constants.uc_fch_len_bits = FCH_LEN_BITS_FCC;
		s_band_constants.uc_tonemap_size = TONE_MAP_SIZE_FCC;
		s_band_constants.uc_first_carrier = FIRST_CARRIER_FCC;
		s_band_constants.uc_last_carrier = LAST_CARRIER_FCC;
		s_band_constants.uc_last_used_carrier = LAST_CARRIER_FCC;
		s_band_constants.uc_num_carriers = NUM_CARRIERS_FCC;
		s_band_constants.uc_num_subbands = NUM_SUBBANDS_FCC;
		s_band_constants.us_fch_interleaver_useful_size = FCH_INTERLEAVER_USEFUL_SIZE_FCC;
		s_band_constants.uc_pilot_offset = PILOT_OFFSET_FCC;
		s_band_constants.uc_num_carriers_in_subband = CARRIERS_IN_SUBBAND_FCC;
		s_band_constants.uc_tx_fft_shift = TX_FFT_SHIFT_FCC;
		s_band_constants.uc_rx_fft_shift = RX_FFT_SHIFT_FCC;
		s_band_constants.uc_emit_gain_hi = EMIT_GAIN_HI_FCC;
		s_band_constants.uc_emit_gain_lo = EMIT_GAIN_LO_FCC;
		s_band_constants.uc_emit_gain_vlo = EMIT_GAIN_VLO_FCC;
		s_band_constants.ul_txrx_time = CFG_TXRX_TIME_FCC_ARIB;
		s_band_constants.ul_txrx_time_us = CFG_TXRX_TIME_US_FCC_ARIB;
		s_band_constants.ul_txrx_plc = CFG_TXRX_PLC_FCC_ARIB;
		s_band_constants.ul_txrx_plc_us = CFG_TXRX_PLC_US_FCC_ARIB;
		s_band_constants.ul_end_tx_offset_us = CFG_END_OF_TX_OFFSET_US_FCC_ARIB;
		s_band_constants.ul_rrc_delay_us = RRC_DELAY_US_FCC_ARIB;
		s_band_constants.uc_pay_sym_first_demod = PAY_SYM_FIRST_DEMOD_FCC_ARIB;
		s_band_constants.uc_min_sym_for_offset_correction_dif = MIN_SYMBOLS_FOR_OFFSET_CORRECTION_FCC_ARIB_DIF;
		s_band_constants.uc_min_sym_for_offset_correction_coh = MIN_SYMBOLS_FOR_OFFSET_CORRECTION_FCC_ARIB_COH;
		s_band_constants.ul_frame_symbol_duration = FRAME_SYMBOL_DURATION_FCC_ARIB;
		s_band_constants.ul_half_symbol_duration = HALF_SYMBOL_DURATION_FCC_ARIB;
		s_band_constants.ul_preamble_duration = PREAMBLE_TIME_DURATION_FCC_ARIB;
		s_band_constants.uc_noise_capture_adapt_symbols = NOISE_CAPTURE_ADAPT_SYMBOLS_FCC_ARIB;
		s_band_constants.uc_noise_correction_factor_db = NOISE_CORRECTION_FACTOR_DB_FCC_ARIB;
		s_band_constants.uc_min_noise_notch_db = MIN_NOISE_NOTCH_DB_FCC_ARIB;
	} else if (uc_working_band == WB_ARIB) {
		atpl250.m_ul_version = ATPL250_VERSION_NUM_ARIB;
		p_emit_gain_limits = emit_gain_limits_arib;
		p_caul_max_rms_hi = caul_max_rms_hi_arib;
		p_caul_max_rms_vlo = caul_max_rms_vlo_arib;
		p_caul_th1_hi = caul_th1_hi_arib;
		p_caul_th2_hi = caul_th2_hi_arib;
		p_caul_th1_lo = caul_th1_lo_arib;
		p_caul_th2_lo = caul_th2_lo_arib;
		p_caul_th1_vlo = caul_th1_vlo_arib;
		p_caul_th2_vlo = caul_th2_vlo_arib;
		s_band_constants.uc_fch_len = FCH_LEN_ARIB;
		s_band_constants.uc_fch_len_bits = FCH_LEN_BITS_ARIB;
		s_band_constants.uc_tonemap_size = TONE_MAP_SIZE_ARIB;
		s_band_constants.uc_first_carrier = FIRST_CARRIER_ARIB;
		s_band_constants.uc_last_carrier = LAST_CARRIER_ARIB;
		s_band_constants.uc_last_used_carrier = LAST_CARRIER_ARIB;
		s_band_constants.uc_num_carriers = NUM_CARRIERS_ARIB;
		s_band_constants.uc_num_subbands = NUM_SUBBANDS_ARIB;
		s_band_constants.us_fch_interleaver_useful_size = FCH_INTERLEAVER_USEFUL_SIZE_ARIB;
		s_band_constants.uc_pilot_offset = PILOT_OFFSET_ARIB;
		s_band_constants.uc_num_carriers_in_subband = CARRIERS_IN_SUBBAND_ARIB;
		s_band_constants.uc_tx_fft_shift = TX_FFT_SHIFT_ARIB;
		s_band_constants.uc_rx_fft_shift = RX_FFT_SHIFT_ARIB;
		s_band_constants.uc_emit_gain_hi = EMIT_GAIN_HI_ARIB;
		s_band_constants.uc_emit_gain_lo = EMIT_GAIN_LO_ARIB;
		s_band_constants.uc_emit_gain_vlo = EMIT_GAIN_VLO_ARIB;
		s_band_constants.ul_txrx_time = CFG_TXRX_TIME_FCC_ARIB;
		s_band_constants.ul_txrx_time_us = CFG_TXRX_TIME_US_FCC_ARIB;
		s_band_constants.ul_txrx_plc = CFG_TXRX_PLC_FCC_ARIB;
		s_band_constants.ul_txrx_plc_us = CFG_TXRX_PLC_US_FCC_ARIB;
		s_band_constants.ul_end_tx_offset_us = CFG_END_OF_TX_OFFSET_US_FCC_ARIB;
		s_band_constants.ul_rrc_delay_us = RRC_DELAY_US_FCC_ARIB;
		s_band_constants.uc_pay_sym_first_demod = PAY_SYM_FIRST_DEMOD_FCC_ARIB;
		s_band_constants.uc_min_sym_for_offset_correction_dif = MIN_SYMBOLS_FOR_OFFSET_CORRECTION_FCC_ARIB_DIF;
		s_band_constants.uc_min_sym_for_offset_correction_coh = MIN_SYMBOLS_FOR_OFFSET_CORRECTION_FCC_ARIB_COH;
		s_band_constants.ul_frame_symbol_duration = FRAME_SYMBOL_DURATION_FCC_ARIB;
		s_band_constants.ul_half_symbol_duration = HALF_SYMBOL_DURATION_FCC_ARIB;
		s_band_constants.ul_preamble_duration = PREAMBLE_TIME_DURATION_FCC_ARIB;
		s_band_constants.uc_noise_capture_adapt_symbols = NOISE_CAPTURE_ADAPT_SYMBOLS_FCC_ARIB;
		s_band_constants.uc_noise_correction_factor_db = NOISE_CORRECTION_FACTOR_DB_FCC_ARIB;
		s_band_constants.uc_min_noise_notch_db = MIN_NOISE_NOTCH_DB_FCC_ARIB;
	} else {
		atpl250.m_ul_version = ATPL250_VERSION_NUM_CENELEC_A;
		p_emit_gain_limits = emit_gain_limits_cenelec_a;
		p_caul_max_rms_hi = caul_max_rms_hi_cenelec_a;
		p_caul_max_rms_vlo = caul_max_rms_vlo_cenelec_a;
		p_caul_th1_hi = caul_th1_hi_cenelec_a;
		p_caul_th2_hi = caul_th2_hi_cenelec_a;
		p_caul_th1_lo = caul_th1_lo_cenelec_a;
		p_caul_th2_lo = caul_th2_lo_cenelec_a;
		p_caul_th1_vlo = caul_th1_vlo_cenelec_a;
		p_caul_th2_vlo = caul_th2_vlo_cenelec_a;
		s_band_constants.uc_fch_len = FCH_LEN_CENELEC_A;
		s_band_constants.uc_fch_len_bits = FCH_LEN_BITS_CENELEC_A;
		s_band_constants.uc_tonemap_size = TONE_MAP_SIZE_CENELEC_A;
		s_band_constants.uc_first_carrier = FIRST_CARRIER_CENELEC_A;
		s_band_constants.uc_last_carrier = LAST_CARRIER_CENELEC_A;
		s_band_constants.uc_last_used_carrier = LAST_CARRIER_CENELEC_A;
		s_band_constants.uc_num_carriers = NUM_CARRIERS_CENELEC_A;
		s_band_constants.uc_num_subbands = NUM_SUBBANDS_CENELEC_A;
		s_band_constants.us_fch_interleaver_useful_size = FCH_INTERLEAVER_USEFUL_SIZE_CENELEC_A;
		s_band_constants.uc_pilot_offset = PILOT_OFFSET_CENELEC_A;
		s_band_constants.uc_num_carriers_in_subband = CARRIERS_IN_SUBBAND_CENELEC_A;
		s_band_constants.uc_tx_fft_shift = TX_FFT_SHIFT_CENELEC_A;
		s_band_constants.uc_rx_fft_shift = RX_FFT_SHIFT_CENELEC_A;
		s_band_constants.uc_emit_gain_hi = EMIT_GAIN_HI_CENELEC_A;
		s_band_constants.uc_emit_gain_lo = EMIT_GAIN_LO_CENELEC_A;
		s_band_constants.uc_emit_gain_vlo = EMIT_GAIN_VLO_CENELEC_A;
		s_band_constants.ul_txrx_time = CFG_TXRX_TIME_CENELEC_A;
		s_band_constants.ul_txrx_time_us = CFG_TXRX_TIME_US_CENELEC_A;
		s_band_constants.ul_txrx_plc = CFG_TXRX_PLC_CENELEC_A;
		s_band_constants.ul_txrx_plc_us = CFG_TXRX_PLC_US_CENELEC_A;
		s_band_constants.ul_end_tx_offset_us = CFG_END_OF_TX_OFFSET_US_CENELEC_A;
		s_band_constants.ul_rrc_delay_us = RRC_DELAY_US_CENELEC_A;
		s_band_constants.uc_pay_sym_first_demod = PAY_SYM_FIRST_DEMOD_CENELEC_A;
		s_band_constants.uc_min_sym_for_offset_correction_dif = MIN_SYMBOLS_FOR_OFFSET_CORRECTION_CENELEC_A;
		s_band_constants.uc_min_sym_for_offset_correction_coh = MIN_SYMBOLS_FOR_OFFSET_CORRECTION_CENELEC_A;
		s_band_constants.ul_frame_symbol_duration = FRAME_SYMBOL_DURATION_CENELEC_A;
		s_band_constants.ul_half_symbol_duration = HALF_SYMBOL_DURATION_CENELEC_A;
		s_band_constants.ul_preamble_duration = PREAMBLE_TIME_DURATION_CENELEC_A;
		s_band_constants.uc_noise_capture_adapt_symbols = NOISE_CAPTURE_ADAPT_SYMBOLS_CENELEC_A;
		s_band_constants.uc_noise_correction_factor_db = NOISE_CORRECTION_FACTOR_DB_CENELEC_A;
		s_band_constants.uc_min_noise_notch_db = MIN_NOISE_NOTCH_DB_CENELEC_A;
	}

	/* Set handler */
	pplc_set_handler(phy_interrupt);

	/* Clear flag to detect start interrupt execution */
	uc_start_interrupt_executed = 0;

	/* Initialize PPLC driver */
	pplc_if_init();

	/* Initialize time counters */
	ul_ms_counter = oss_get_up_time_ms();
	ul_noise_capture_timer = ul_ms_counter + atpl250.m_ul_time_between_noise_captures;
	ul_rx_timeout_timer = ul_ms_counter + PHY_RX_TIMEOUT_MS;
	ul_tx_timeout_timer = ul_ms_counter + PHY_TX_TIMEOUT_MS;

	/* Wait for start interrupt execution or timeout */
	ul_timeout = 0;
	while (!uc_start_interrupt_executed) {
		if (++ul_timeout == 20000) {
			break;
		}
	}

	/* Initialize Product Id, Model and Version */
	memcpy(atpl250.m_ac_prod_id, ATPL250_PRODID, 10);
	atpl250.m_us_model = ATPL250_MODEL;

#ifdef ENABLE_PYH_PROCESS_RECALL
	uc_force_phy_process_recall = 0;
#endif

	/* Init PHY serial interface */
	if (uc_serial_enable) {
		serial_if_init();
	}

	disable_CD_interrupt();
	sb_cd_int_enabled = false;

	/* Init sniffer interface */
	#ifdef ENABLE_SNIFFER
	sniffer_if_init();
	#endif
}

/**
 * \brief PHY handles a reset request from upper layers
 *
 */
void phy_reset_request(void)
{
	uint32_t ul_timeout;

	/* Disable interrupts */
	Disable_global_interrupt();

	/* Clear flag to detect start interrupt execution */
	uc_start_interrupt_executed = 0;

	/* Initialize PPLC driver */
	pplc_if_init();

	/* Enable interrupts */
	Enable_global_interrupt();

	/* Initialize time counters */
	ul_ms_counter = oss_get_up_time_ms();
	ul_noise_capture_timer = ul_ms_counter + atpl250.m_ul_time_between_noise_captures;
	ul_rx_timeout_timer = ul_ms_counter + PHY_RX_TIMEOUT_MS;
	ul_tx_timeout_timer = ul_ms_counter + PHY_TX_TIMEOUT_MS;

	/* Wait for start interrupt execution or timeout */
	ul_timeout = 0;
	while (!uc_start_interrupt_executed) {
		if (++ul_timeout == 20000) {
			break;
		}
	}
}

/**
 * \brief PHY interrupt management
 *
 */
void phy_interrupt(void)
{
	uint32_t ul_flags_read_from_isr;
	uint8_t uc_i;

#if (LOG_SPI == 1)
	addIntMarkToLogSpi();
#endif
	/* Check SPI communication integrity */
	if (atpl250_spi_comm_corrupted()) {
		/* Set flag to reset ATPL250 and return */
		uc_phy_generic_flags |= PHY_GENERIC_FLAG_RESET_PHY;
		return;
	}

	platform_led_int_toggle();

	/* Read flags */
	ul_flags_read_from_isr = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
	ul_phy_int_flags |= ul_flags_read_from_isr;

	/* Clear flags before calling function to handle interrupt sources */
	uc_set_sym_ready = false;
	uc_clear_rx_full = false;

	/* Find triggering source */
	uc_i = 0;
	while ((ul_phy_int_flags != 0) && (uc_i < PHY_NUM_INTERRUPT_SOURCES)) {
		if ((ul_phy_int_flags & (1 << uc_i)) != 0) {
			/* Call handler */
			phy_handler[uc_i].handler();
			/* Clear flag */
			ul_phy_int_flags &= (~(1u << uc_i));
		}

		uc_i++;
	}

	/* Check whether set_sym_ready or clear_rx_full have been set */
	if (uc_set_sym_ready) {
		atpl250_set_sym_ready();
	}

	if (uc_clear_rx_full) {
		atpl250_clear_rx_full();
	}

	platform_led_int_toggle();
}

/**
 * \brief Get PHY layer parameter
 *
 * \param us_id   Parameter Identification (see atpl250.h)
 * \param *p_val  Pointer to parameter value
 * \param uc_len  Length of the parameter
 *
 * \return 0 if there is no error, otherwise returns Error Type (see atpl250.h)
 */
uint8_t phy_get_cfg_param(uint16_t us_id, void *p_val, uint16_t us_len)
{
	uint8_t *puc_mem = NULL;
	uint16_t us_payload_symbols;

	if (ATPL250_REG_PARAM(us_id)) {
		if (us_len == 1) {
			*((uint8_t *)p_val) = pplc_if_read8(us_id);
		} else if (us_len == 2) {
			*((uint16_t *)p_val) = pplc_if_read16(us_id);
		} else if (us_len == 4) {
			*((uint32_t *)p_val) = pplc_if_read32(us_id);
		} else {
			pplc_if_read_buf(us_id, p_val, us_len, true);
		}

		return(PHY_CFG_SUCCESS);
	} else if (ATPL250_PARAM(us_id) == ATPL250_PARAM_MSK) {
		if ((us_id < PHY_PIB_LOWEST_VALUE) || (us_id > PHY_PIB_HIGHEST_VALUE)) {
			return(PHY_CFG_INVALID_INPUT);
		}

		if (us_id == PHY_ID_STATIC_NOTCHING) {
			puc_mem = (uint8_t *)&auc_static_notching_pos;
		} else if (us_id == PHY_ID_LEGACY_MODE) {
			puc_mem = (uint8_t *)&uc_legacy_mode;
		} else if (us_id == PHY_ID_TONE_MAP_RSP_ENABLED_MODS) {
			/* Do nothing on ATPL250 */
			return(PHY_CFG_INVALID_INPUT);
		} else if (us_id == PHY_ID_RESET_PHY_STATS) {
			/* No sense in reading this */
			return(PHY_CFG_INVALID_INPUT);
		} else {
			puc_mem = (uint8_t *)&atpl250 + (us_id & ~ATPL250_PARAM_MSK);
		}
	} else if (ATPL250_PARAM(us_id) == ATPL250_BER_PARAM_MSK) {
		if (ATPL250_EXTENDED_PARAM(us_id) == ATPL250_TM_RESP_PARAM_MSK) {
			/* Get values for Tone Map Response */
			us_payload_symbols = s_phy_rx_ctl.m_us_rx_payload_symbols;
			select_modulation_tone_map((uint8_t)s_phy_rx_ctl.e_mod_scheme, s_phy_rx_ctl.m_auc_static_and_dynamic_notching_pos,
					s_phy_rx_ctl.m_auc_pilot_pos_first_symbol, s_phy_rx_ctl.m_auc_inactive_carriers_pos,
					s_tone_map_response_data.m_auc_tone_map, (uint8_t *)&s_tone_map_response_data.e_mod_type, us_payload_symbols,
					s_phy_rx_ctl.m_uc_payload_carriers, s_phy_rx_ctl.e_rs_blocks, atpl250.m_uc_rrc_notch_index);
			if (uc_working_band == WB_ARIB) {
				s_tone_map_response_data.e_mod_scheme = MOD_SCHEME_DIFFERENTIAL;
			} else {
				s_tone_map_response_data.e_mod_scheme = s_phy_rx_ctl.e_mod_scheme;
			}

			puc_mem = (uint8_t *)&s_tone_map_response_data + (us_id & ~ATPL250_TM_RESP_PARAM_MSK);
		} else {
			puc_mem = (uint8_t *)&s_rx_ber_payload_data + (us_id & ~ATPL250_BER_PARAM_MSK);
			if (sb_ber_int_received == true) {
				_process_ber_info();
				atpl250.m_uc_last_msg_lqi = s_rx_ber_payload_data.uc_lqi;
			}
		}
	} else {
		return(PHY_CFG_INVALID_INPUT);
	}

	memcpy(p_val, puc_mem, us_len);

	return(PHY_CFG_SUCCESS);
}

/**
 * \brief Set PHY layer parameter
 *
 * \param us_id   Parameter Identification (see atpl250.h)
 * \param *p_val  Pointer to parameter value
 * \param uc_len  Length of the parameter
 *
 * \return 0 if there is no error, otherwise returns Error Type (see atpl250.h)
 */
uint8_t phy_set_cfg_param(uint16_t us_id, void *p_val, uint16_t us_len)
{
	uint8_t *puc_mem = NULL;
	uint8_t uc_memcpy_enabled = 1;
	uint8_t uc_i;

	if (ATPL250_REG_PARAM(us_id)) {
		if (us_len == 1) {
			pplc_if_write8(us_id, *((uint8_t *)p_val));
		} else if (us_len == 2) {
			pplc_if_write16(us_id, *((uint16_t *)p_val));
		} else if (us_len == 4) {
			pplc_if_write32(us_id, *((uint32_t *)p_val));
		} else {
			pplc_if_write_buf(us_id, p_val, us_len);
		}

		return(PHY_CFG_SUCCESS);
	} else if (ATPL250_PARAM(us_id) == ATPL250_PARAM_MSK) {
		if ((us_id < PHY_PIB_LOWEST_VALUE) || (us_id > PHY_PIB_HIGHEST_VALUE)) {
			return(PHY_CFG_INVALID_INPUT);
		}

		if (us_id == PHY_ID_CFG_AUTODETECT_IMPEDANCE) {
			uc_memcpy_enabled = 0;
			atpl250.m_uc_auto_detect_impedance = *((uint8_t *)p_val);
			puc_mem = (uint8_t *)&atpl250.m_uc_auto_detect_impedance;
		} else if (us_id == PHY_ID_CFG_IMPEDANCE) {
			uc_memcpy_enabled = 0;
			atpl250.m_uc_impedance_state = *((uint8_t *)p_val);
			switch (atpl250.m_uc_impedance_state) {
			case HI_STATE:
				atpl250_update_branch_cfg(atpl250.m_uc_impedance_state, s_band_constants.uc_emit_gain_hi);
				break;

			case LO_STATE:
				atpl250_update_branch_cfg(atpl250.m_uc_impedance_state, s_band_constants.uc_emit_gain_lo);
				break;

			case VLO_STATE:
				atpl250_update_branch_cfg(atpl250.m_uc_impedance_state, s_band_constants.uc_emit_gain_vlo);
				break;

			default:
				atpl250_update_branch_cfg(atpl250.m_uc_impedance_state, s_band_constants.uc_emit_gain_hi);
				break;
			}
		} else if (us_id == PHY_ID_STATIC_NOTCHING) {
			puc_mem = (uint8_t *)&auc_static_notching_pos;
		} else if (us_id == PHY_ID_RRC_NOTCH_ACTIVE) {
			uc_memcpy_enabled = 0;
			atpl250.m_uc_rrc_notch_active = *((uint8_t *)p_val);
			pplc_if_or8(REG_ATPL250_GLOBAL_SRST_H8, 0x80);  /* SRST_DEC=1 */
			atpl250_sync_pream_txrxb_push_rst();
			atpl250_enable_rrc_notch_filter(atpl250.m_uc_rrc_notch_active);
			atpl250_sync_pream_txrxb_release_rst();
			pplc_if_and8(REG_ATPL250_GLOBAL_SRST_H8, 0x7F);  /* SRST_DEC=0 */
		} else if (us_id == PHY_ID_RRC_NOTCH_INDEX) {
			uc_memcpy_enabled = 0;
			atpl250.m_uc_rrc_notch_index = *((uint8_t *)p_val);
			for (uc_i = 0; uc_i <= FILTER_CONFIG_NUM_STEPS; uc_i++) {
				/* <= because last step is for writing to HW */
				atpl250_set_rrc_notch_filter(atpl250.m_uc_rrc_notch_index, uc_i);
			}
		} else if (us_id == PHY_ID_RRC_NOTCH_AUTODETECT) {
			uc_memcpy_enabled = 0;
			/* Set flag for Noise Capture */
			uc_phy_generic_flags |= PHY_GENERIC_FLAG_NOISE_CAPTURE;
		} else if (us_id == PHY_ID_LEGACY_MODE) {
			puc_mem = (uint8_t *)&uc_legacy_mode;
		} else if (us_id == PHY_ID_TONE_MAP_RSP_ENABLED_MODS) {
			uc_memcpy_enabled = 0;
			/* Do nothing on ATPL250 */
		} else if (us_id == PHY_ID_FORCE_NO_OUTPUT_SIGNAL) {
			uc_memcpy_enabled = 0;
			atpl250.m_uc_force_no_output_signal = *((uint8_t *)p_val);
			if (atpl250.m_uc_force_no_output_signal) {
				pplc_if_write8(REG_ATPL250_FFT_CONFIG_VH8, 0x00);
			} else {
				pplc_if_write8(REG_ATPL250_FFT_CONFIG_VH8, 0xFF);
			}
		} else if (us_id == PHY_ID_RESET_PHY_STATS) {
			uc_memcpy_enabled = 0;
			/* Reset all statistics */
			atpl250.m_ul_tx_total = 0;
			atpl250.m_ul_tx_total_bytes = 0;
			atpl250.m_ul_tx_total_errors = 0;
			atpl250.m_ul_tx_bad_busy_tx = 0;
			atpl250.m_ul_tx_bad_busy_channel = 0;
			atpl250.m_ul_tx_bad_len = 0;
			atpl250.m_ul_tx_bad_format = 0;
			atpl250.m_ul_tx_timeout = 0;
			atpl250.m_ul_rx_total = 0;
			atpl250.m_ul_rx_total_bytes = 0;
			atpl250.m_ul_rx_RS_errors = 0;
			atpl250.m_ul_rx_exception = 0;
			atpl250.m_ul_rx_bad_len = 0;
			atpl250.m_ul_rx_bad_crc_fch = 0;
			atpl250.m_ul_rx_false_positive = 0;
			atpl250.m_ul_rx_bad_format = 0;
			atpl250.m_ul_time_freeline = 0;
			atpl250.m_ul_time_busyline = 0;
		} else if (us_id == PHY_ID_TRIGGER_SIGNAL_DUMP) {
#ifdef ENABLE_SIGNAL_DUMP
			uc_memcpy_enabled = 0;
			atpl250.m_uc_trigger_signal_dump = *((uint8_t *)p_val);

			if ((atpl250.m_uc_trigger_signal_dump == IMMEDIATE_DUMP) || (atpl250.m_uc_trigger_signal_dump == DUMP_ON_RECEPTION)) {
				atpl250_iob_push_rst();
				atpl250_txrxb_rotator_and_fft_push_rst();
				/* 1024 samples, 1 symbol */
				atpl250_set_iob_partition(IOB_1_SYMBOL_OF_1024_SAMPLES);
				atpl250_clear_iobuf_to_fft();
				/* Read only real part */
				pplc_if_or8(REG_ATPL250_INOUTB_CTL_H8, 0x01);
				/* Samples transmitted from IOB; samples received to IOB; Frames divided in symbols */
				pplc_if_write8(REG_ATPL250_TXRXB_CTL_H8, 0x03);
				/* CP_EXCLUSIVO=0 */
				pplc_if_write8(REG_ATPL250_TXRXB_CTL_VL8, 0x00);
				/* cyclic_prefix1=0; overlap1=0; cyclic_prefix2=0; overlap2=0 */
				pplc_if_write32(REG_ATPL250_TXRXB_SYM_CFG_32, 0x00000000);
				/* Discard 0 samples after PEAK1. Discard 0 samples after PEAK2 (0.5*SYNCM+0.5*(cyclic_prefix2-overlap2) */
				pplc_if_write32(REG_ATPL250_TXRXB_OFFSETS_32, 0x00000000);
				/* 4 prev symbols; Division by 4; average 4 symbols, enable average and send previuos symbols */
				pplc_if_write32(REG_ATPL250_TXRXB_PRE_ANALYSIS_32, 0x03010302);
				/* Disable SYNCM DETECTOR */
				pplc_if_and8(REG_ATPL250_SYNCM_CTL_L8, 0x7F);
				pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, 0xFFFF);
				atpl250_iob_release_rst();
				atpl250_txrxb_rotator_and_fft_release_rst();

				pplc_if_set_speed(PPLC_CLOCK_30M);
				uc_dumped_buffers = 0;

				/* In case value is IMMEDIATE_DUMP, force peak to dump immediately */
				if (atpl250.m_uc_trigger_signal_dump == IMMEDIATE_DUMP) {
					/* Reset TxRxBuffer */
					atpl250_txrxb_push_rst();
					atpl250_txrxb_release_rst();
					for (ul_loop = 0; ul_loop < 10000; ul_loop++) {
					}
					/* Force peak */
					pplc_if_or8(REG_ATPL250_TXRXB_CFG_VL8, 0x01);
				}
			}
#endif
		} else {
			puc_mem = (uint8_t *)&atpl250 + (us_id & ~ATPL250_PARAM_MSK);
		}
	} else {
		return(PHY_CFG_INVALID_INPUT);
	}

	if (uc_memcpy_enabled) {
		if (puc_mem != NULL) {
			memcpy(puc_mem, p_val, us_len);
		}
	}

	if (us_id == PHY_ID_STATIC_NOTCHING || us_id == PHY_ID_LEGACY_MODE) {
		uc_phy_generic_flags |= PHY_GENERIC_FLAG_INIT_PHY_PARAMS;
	}

	return(PHY_CFG_SUCCESS);
}

/**
 * \brief Set PHY layer parameter
 *
 * \param us_id   Parameter Identification (see atpl250.h)
 * \param uc_cmd  Command to operation (PHY_CMD_CFG_AND, PHY_CMD_CFG_OR, PHY_CMD_CFG_XOR)
 * \param uc_mask Mask Bits
 *
 * \return 0 if there is no error, otherwise returns Error Type (see atpl250.h)
 */
uint8_t phy_cmd_cfg_param(uint16_t us_id, uint8_t uc_cmd, uint8_t uc_mask)
{
	uint8_t uc_result = PHY_CFG_SUCCESS;

	if (ATPL250_REG_PARAM(us_id)) {
		if (uc_cmd == PHY_CMD_CFG_AND) {
			pplc_if_and8(us_id, uc_mask);
		} else if (uc_cmd == PHY_CMD_CFG_OR) {
			pplc_if_or8(us_id, uc_mask);
		} else if (uc_cmd == PHY_CMD_CFG_XOR) {
			pplc_if_xor8(us_id, uc_mask);
		} else {
			uc_result = PHY_CFG_INVALID_INPUT;
		}
	} else {
		uc_result = PHY_CFG_INVALID_INPUT;
	}

	return uc_result;
}

/**
 * \brief Sends a frame using the PHY layer
 *
 * \param px_msg  Pointer to message structure data.
 *
 * \return Tx result value (see atpl250.h)
 */
uint8_t phy_tx_frame(xPhyMsgTx_t *px_msg)
{
	uint8_t uc_is_ack;
	uint8_t uc_i;
	uint32_t ul_curr_time;
	uint32_t ul_delayed_safety_time;
	uint8_t uc_invalid_tone_map;

	/* Check whether PLC is disabled */
	if (atpl250.m_uc_plc_disable) {
		ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
		uc_phy_generic_flags |= PHY_GENERIC_FLAG_END_TX;
		return PHY_TX_RESULT_PROCESS;
	}

	if (uc_working_band == WB_CENELEC_A) {
		ul_delayed_safety_time = DELAYED_TX_SAFETY_TIME_CENELEC_A;
	} else {
		ul_delayed_safety_time = DELAYED_TX_SAFETY_TIME_FCC_ARIB;
	}

	/* Set tx gain */
	if (px_msg->m_uc_tx_power <= 15) {
		uc_phy_tx_full_gain = auc_txgain[px_msg->m_uc_tx_power];
	} else {
		uc_phy_tx_full_gain = auc_txgain[15];
	}

	if (atpl250.m_uc_force_no_output_signal) {
		uc_phy_tx_full_gain = 1;
	}

	if (s_phy_tx_ctl.e_tx_step != STEP_TX_NO_TX) {
		/* Transmission already in progress */
		/* Do not generate confirm, because confirm will raise for the current transmission */
		return PHY_TX_RESULT_BUSY_TX;
	} else if (s_phy_rx_ctl.e_rx_step != STEP_RX_NO_RX) {
		/* Reception in progress */
		s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_BUSY_RX;
		s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;

		/* Get Tx end time */
		ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);

		_trigger_data_confirm();
		atpl250.m_ul_tx_bad_busy_channel++;
		atpl250.m_ul_tx_total_errors++;

		return PHY_TX_RESULT_BUSY_RX;
	}

	LOG_PHY(("TxPower %u\r\n", px_msg->m_uc_tx_power));

	if (uc_working_band == WB_CENELEC_A) {
		/* Check whether emit gain has to change */
		if ((atpl250.m_uc_auto_detect_impedance == AUTO_STATE_VAR_GAIN) && (px_msg->m_uc_tx_power >= NUM_TX_LEVELS)) {
			atpl250_update_branch_cfg(1, s_band_constants.uc_emit_gain_lo);
			atpl250.m_uc_impedance_state = LO_STATE;
			LOG_PHY(("LOWTxPower ImpState: LO EmitGain: 0x%02X\r\n", s_band_constants.uc_emit_gain_lo));
		}
	}

	for (uc_i = 0; uc_i < s_band_constants.uc_tonemap_size; uc_i++) {
		s_phy_tx_ctl.m_auc_tone_map[uc_i] = px_msg->m_auc_tone_map[uc_i];
		s_phy_tx_ctl.m_auc_inv_tone_map[uc_i] = (uint8_t)(~(unsigned int)(px_msg->m_auc_tone_map[uc_i]));
	}

	if (px_msg->e_mod_type == MOD_TYPE_BPSK_ROBO) {
		s_phy_tx_ctl.m_uc_rs_parity = 0x07;
	} else {
		s_phy_tx_ctl.m_uc_rs_parity = 0x0F;
	}

	if (uc_working_band == WB_CENELEC_A) {
		if (px_msg->m_us_data_len > (PHY_MAX_PAYLOAD_SIZE - s_phy_tx_ctl.m_uc_rs_parity)) {
			s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_INV_LENGTH;
			s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;

			/* Get Tx end time */
			ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
			atpl250.m_ul_tx_bad_len++;
			atpl250.m_ul_tx_total_errors++;
			_trigger_data_confirm();

			return PHY_TX_RESULT_INV_LENGTH;
		}

		s_phy_tx_ctl.m_us_payload_len =  px_msg->m_us_data_len;
		s_phy_tx_ctl.e_rs_blocks = RS_BLOCKS_1_BLOCK;
	} else if (uc_working_band == WB_FCC) {
		if (px_msg->m_uc_2_rs_blocks) {
			if (px_msg->m_us_data_len > (2 * (PHY_MAX_PAYLOAD_SIZE - s_phy_tx_ctl.m_uc_rs_parity))) {
				s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_INV_LENGTH;
				s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;

				/* Get Tx end time */
				ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
				atpl250.m_ul_tx_bad_len++;
				atpl250.m_ul_tx_total_errors++;
				_trigger_data_confirm();

				return PHY_TX_RESULT_INV_LENGTH;
			}

			s_phy_tx_ctl.e_rs_blocks = RS_BLOCKS_2_BLOCKS_TRANSMITTING_FIRST;
		} else {
			if (px_msg->m_us_data_len > (PHY_MAX_PAYLOAD_SIZE - s_phy_tx_ctl.m_uc_rs_parity)) {
				s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_INV_LENGTH;
				s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;

				/* Get Tx end time */
				ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
				atpl250.m_ul_tx_bad_len++;
				atpl250.m_ul_tx_total_errors++;
				_trigger_data_confirm();

				return PHY_TX_RESULT_INV_LENGTH;
			}

			s_phy_tx_ctl.e_rs_blocks = RS_BLOCKS_1_BLOCK;
		}

		if (px_msg->m_uc_2_rs_blocks) {
			if (px_msg->m_us_data_len & 0x0001) {
				s_phy_tx_ctl.m_us_payload_len =  (px_msg->m_us_data_len + 1) >> 1;
			} else {
				s_phy_tx_ctl.m_us_payload_len =  (px_msg->m_us_data_len) >> 1;
			}
		} else {
			s_phy_tx_ctl.m_us_payload_len =  px_msg->m_us_data_len;
		}
	} else {
		if (px_msg->m_us_data_len > (PHY_MAX_PAYLOAD_SIZE - s_phy_tx_ctl.m_uc_rs_parity)) {
			s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_INV_LENGTH;
			s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;

			/* Get Tx end time */
			ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
			atpl250.m_ul_tx_bad_len++;
			atpl250.m_ul_tx_total_errors++;
			_trigger_data_confirm();

			return PHY_TX_RESULT_INV_LENGTH;
		}

		s_phy_tx_ctl.e_rs_blocks = RS_BLOCKS_1_BLOCK;

		s_phy_tx_ctl.m_auc_inv_tone_map[2] &= 0x03;

		s_phy_tx_ctl.m_us_payload_len =  px_msg->m_us_data_len;
	}

	s_phy_tx_ctl.e_mod_type = px_msg->e_mod_type;
	s_phy_tx_ctl.e_mod_scheme = px_msg->e_mod_scheme;
	s_phy_tx_ctl.e_delimiter_type = px_msg->e_delimiter_type;
	s_phy_tx_ctl.m_uc_pdc = px_msg->m_uc_pdc;
	s_phy_tx_ctl.m_uc_tx_mode = px_msg->m_uc_tx_mode;

	uc_is_ack = (px_msg->e_delimiter_type == DT_ACK) || (px_msg->e_delimiter_type == DT_NACK);

	if (uc_working_band == WB_CENELEC_A) {
		/* Set preemphasis */
		if (memcmp(auc_prev_preemphasis, px_msg->m_auc_preemphasis, sizeof(auc_prev_preemphasis)) != 0) {
			_set_preemphasis(&px_msg->m_auc_preemphasis[0]);
			memcpy(auc_prev_preemphasis, px_msg->m_auc_preemphasis, sizeof(auc_prev_preemphasis));
		}

		if (uc_is_ack) {
			/* In ack, FCH is filled with data coming from upper layer */
			memcpy(auc_fch, px_msg->m_puc_data_buf, s_band_constants.uc_fch_len);
			s_phy_tx_ctl.m_us_payload_len = 0;
		} else {
			/* Data frame */
			/* Check valid tone map */
			if ((px_msg->m_auc_tone_map[0] > 0x3F) || (px_msg->m_auc_tone_map[0] == 0x00)) {
				s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_INV_TONEMAP;
				s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;

				/* Get Tx end time */
				ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);

				atpl250.m_ul_tx_bad_format++;
				atpl250.m_ul_tx_total_errors++;
				_trigger_data_confirm();

				return PHY_TX_RESULT_INV_TONEMAP;
			}
		}
	} else {
		if (uc_is_ack) {
			/* In ack, FCH is filled with data coming from upper layer */
			memcpy(auc_fch, px_msg->m_puc_data_buf, s_band_constants.uc_fch_len);
			s_phy_tx_ctl.m_us_payload_len = 0;
		} else {
			/* Data frame */
			/* Set preemphasis */
			if (memcmp(auc_prev_preemphasis, px_msg->m_auc_preemphasis, sizeof(auc_prev_preemphasis)) != 0) {
				_set_preemphasis(&px_msg->m_auc_preemphasis[0]);
				memcpy(auc_prev_preemphasis, px_msg->m_auc_preemphasis, sizeof(auc_prev_preemphasis));
			}

			/* Check valid tone map */
			uc_invalid_tone_map = 1;
			for (uc_i = 0; uc_i < s_band_constants.uc_tonemap_size; uc_i++) {
				if ((px_msg->m_auc_tone_map[uc_i] != 0x00)) {
					uc_invalid_tone_map = 0;
					break;
				}
			}

			if (uc_invalid_tone_map) {
				s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_INV_TONEMAP;
				s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;

				/* Get Tx end time */
				ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
				atpl250.m_ul_tx_bad_format++;
				atpl250.m_ul_tx_total_errors++;
				_trigger_data_confirm();

				return PHY_TX_RESULT_INV_TONEMAP;
			}
		}
	}

	if (!uc_is_ack) {
		/* Copy payload */
		memcpy(&s_phy_tx_ctl.auc_tx_buf[s_band_constants.uc_fch_len], px_msg->m_puc_data_buf, px_msg->m_us_data_len);
	}

	#ifdef TX_PREAMBLE_HW
	/* Emit preamble from HW */
	/*	atpl250_txrxb_push_rst(); */
	pplc_if_or8(REG_ATPL250_TX_CONF_VH8, 0x10);         /* EMIT_PREAMBLE0 = 1 */

	/*	atpl250_txrxb_release_rst(); */
	#else
	/* Emit preamble from SW */
	/*	atpl250_txrxb_push_rst(); */
	pplc_if_and8(REG_ATPL250_TX_CONF_VH8, 0xEF);         /* EMIT_PREAMBLE0 = 0 */

	/*	atpl250_txrxb_release_rst(); */
	#endif

	/* Check time on delayed transmissions */
	if ((px_msg->m_uc_tx_mode) & TX_MODE_DELAYED_TX) {
		/* Enter critical section */
		pplc_disable_all_interrupts();
		ul_curr_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
		if (ul_curr_time > px_msg->m_ul_tx_time) {
			if ((ul_curr_time > 0xFFC00000) && (px_msg->m_ul_tx_time < 0x400000)) {
				/* Current time near overflow and prog time past overflow. 4 seconds margin */
				/* This is not a timeout, continue */
			} else {
				/* Exit critical section */
				pplc_enable_all_interrupts();
				/* Too late to program tx */
				s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_TIMEOUT;
				s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;
				atpl250.m_ul_tx_timeout++;
				atpl250.m_ul_tx_total_errors++;

				/* Get Tx end time */
				ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);

				_trigger_data_confirm();

				return PHY_TX_RESULT_TIMEOUT;
			}
		} else if ((px_msg->m_ul_tx_time - ul_curr_time) < ul_delayed_safety_time) {
			/* Too late to program tx */
			/* Exit critical section */
			pplc_enable_all_interrupts();
			s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_TIMEOUT;
			s_phy_tx_ctl.e_tx_step = STEP_TX_NO_TX;
			atpl250.m_ul_tx_timeout++;
			atpl250.m_ul_tx_total_errors++;

			/* Get Tx end time */
			ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);

			_trigger_data_confirm();

			return PHY_TX_RESULT_TIMEOUT;
		}
	}

	if (uc_is_ack) {
		atpl250_disable_CD_peakfull();
		atpl250_sync_push_rst();
	} else {
		atpl250_enable_CD_peakfull();
	}

#ifdef ENABLE_SNIFFER
	ul_last_tx_time = px_msg->m_ul_tx_time;
#endif

	/* Set transmission state machine in process */
	s_phy_tx_ctl.e_tx_state = PHY_TX_RESULT_PROCESS;
	s_phy_tx_ctl.e_tx_step = STEP_TX_PREAMBLE_FIRST;

	/* Clear interrupt flags and errors */
	/* pplc_if_write32(REG_ATPL250_INT_FLAGS_32, 0x00000000); */
	atpl250_clear_cd_int();
	/* Configure tx parameters */
	_config_tx((px_msg->m_uc_tx_mode) & TX_MODE_FORCED_TX, (px_msg->m_uc_tx_mode) & TX_MODE_DELAYED_TX, px_msg->m_ul_tx_time);
	/* Exit critical section */
	pplc_enable_all_interrupts();

	return PHY_TX_RESULT_PROCESS;
}

/**
 * \brief Function to set callbacks to call on upper layer
 *
 * \param p_callbacks  Pointer to array of Callback functions
 *
 */
void phy_set_callbacks(struct TPhyCallbacks *p_callbacks)
{
	g_phy_callbacks = *p_callbacks;
}

/**
 * \brief Function to set callbacks to call on sniffer layer
 *
 * \param p_callbacks  Pointer to array of Callback functions
 *
 */
void phy_set_sniffer_callbacks(struct TPhySnifferCallbacks *p_callbacks)
{
	g_phy_sniffer_callbacks = *p_callbacks;
}

/**
 * \brief Function to clear sniffer callbacks
 *
 */
void phy_clear_sniffer_callbacks(void)
{
	g_phy_sniffer_callbacks.m_p_sniffer_data_confirm = NULL;
	g_phy_sniffer_callbacks.m_p_sniffer_data_indication = NULL;
}

/**
 * \brief Process to attend tasks in micro controller mode
 *
 */
uint8_t phy_process(void)
{
	ul_ms_counter = oss_get_up_time_ms();

	if (s_phy_tx_ctl.e_tx_step >= STEP_TX_PREAMBLE_FIRST && s_phy_tx_ctl.e_tx_step <= STEP_TX_END) {
		if ((ul_ms_counter - s_phy_tx_ctl.m_ul_start_tx_watch_ms) > 1000) {
			atpl250.m_ul_tx_timeout++;
			atpl250.m_ul_tx_total_errors++;
			phy_reset_request();
		}
	}

	if (s_phy_rx_ctl.e_rx_step != STEP_RX_NO_RX) {
		if ((ul_ms_counter - s_phy_rx_ctl.m_ul_start_rx_watch_ms) > 1000) {
			atpl250.m_ul_rx_exception++;
			phy_reset_request();
		}
	}

#ifdef PHY_RX_TIMEOUT_RESET
	if ((ul_rx_timeout_timer - ul_ms_counter) > PHY_RX_TIMEOUT_MS) {
		if ((s_phy_tx_ctl.e_tx_step == STEP_TX_NO_TX) && (s_phy_rx_ctl.e_rx_step == STEP_RX_NO_RX)) {
			/* Set flag to reset ATPL250 */
			uc_phy_generic_flags |= PHY_GENERIC_FLAG_RESET_PHY;
		}
	}
#endif

#ifdef PHY_TX_TIMEOUT_RESET
	if ((ul_tx_timeout_timer - ul_ms_counter) > PHY_TX_TIMEOUT_MS) {
		if ((s_phy_tx_ctl.e_tx_step == STEP_TX_NO_TX) && (s_phy_rx_ctl.e_rx_step == STEP_RX_NO_RX)) {
			/* Set flag to reset ATPL250 */
			uc_phy_generic_flags |= PHY_GENERIC_FLAG_RESET_PHY;
		}
	}
#endif

	if (atpl250.m_uc_enable_auto_noise_capture) {
		if ((ul_noise_capture_timer - ul_ms_counter) > atpl250.m_ul_time_between_noise_captures) {
			/* Restart timer */
			ul_noise_capture_timer = ul_ms_counter + atpl250.m_ul_time_between_noise_captures;
			/* Set flag for Noise Capture */
			uc_phy_generic_flags |= PHY_GENERIC_FLAG_NOISE_CAPTURE;
			/* Check SPI communication integrity */
			if (atpl250_spi_comm_corrupted()) {
				/* Set flag to reset ATPL250 */
				uc_phy_generic_flags |= PHY_GENERIC_FLAG_RESET_PHY;
			}
		}
	}

	if (!uc_phy_generic_flags) {
		/* Nothing to do */
#ifdef ENABLE_PYH_PROCESS_RECALL
		return uc_force_phy_process_recall;

#else
		return 0;
#endif
	}

	/* Check Reset Phy Flag */
	if (uc_phy_generic_flags & PHY_GENERIC_FLAG_RESET_PHY) {
		/* Call platform function to indicate failure, Reset PHY layer and return */
		phy_reset_request();
		uc_phy_generic_flags &= ~PHY_GENERIC_FLAG_RESET_PHY;
		return 0;
	}

	/* Check Init Phy Params Flag */
	if (uc_phy_generic_flags & PHY_GENERIC_FLAG_INIT_PHY_PARAMS) {
		/* Init Phy params, only if Idle in both Tx and Rx */
		if ((s_phy_tx_ctl.e_tx_step == STEP_TX_NO_TX) && (s_phy_rx_ctl.e_rx_step == STEP_RX_NO_RX)) {
			_init_phy_layer();
			/* Clear flag */
			uc_phy_generic_flags &= ~PHY_GENERIC_FLAG_INIT_PHY_PARAMS;
		}
	}

	/* Check Noise Capture Flag */
	if (uc_phy_generic_flags & PHY_GENERIC_FLAG_NOISE_CAPTURE) {
		/* Trigger capture, only if Idle in both Tx and Rx */
		if ((s_phy_tx_ctl.e_tx_step == STEP_TX_NO_TX) && (s_phy_rx_ctl.e_rx_step == STEP_RX_NO_RX)) {
			_trigger_noise_capture();
			/* Clear flag */
			uc_phy_generic_flags &= ~PHY_GENERIC_FLAG_NOISE_CAPTURE;
		}
	}

#ifdef ENABLE_SNIFFER

	/* Check sniffer flags
	 *  It is important the this check is done before the other,
	 *  in order to be executed in the next phy process call, and not in the same one */
	if (uc_phy_generic_flags & PHY_GENERIC_FLAG_SNIFFER_INDICATION) {
		g_phy_sniffer_callbacks.m_p_sniffer_data_indication(&rx_msg_sniffer);
		/* Clear flag */
		uc_phy_generic_flags &= ~PHY_GENERIC_FLAG_SNIFFER_INDICATION;
	}

	if (uc_phy_generic_flags & PHY_GENERIC_FLAG_SNIFFER_CONFIRM) {
		g_phy_sniffer_callbacks.m_p_sniffer_data_confirm(&tx_result_sniffer, &tx_param_sniffer);
		/* Clear flag */
		uc_phy_generic_flags &= ~PHY_GENERIC_FLAG_SNIFFER_CONFIRM;
	}
#endif

	/* Check End Tx Flag */
	if (uc_phy_generic_flags & PHY_GENERIC_FLAG_END_TX) {
		/* Perform end of transmission */
		_trigger_data_confirm();

		/* Clear flag */
		uc_phy_generic_flags &= ~PHY_GENERIC_FLAG_END_TX;

#ifdef ENABLE_PYH_PROCESS_RECALL
		/* Clear phy process recall, just in case it was previously set */
		uc_force_phy_process_recall = 0;
#endif

		#if (LOG_SPI == 1)
		dumpLogSpi();
		#endif
	}

	/* Check End Rx Flag */
	if (uc_phy_generic_flags & PHY_GENERIC_FLAG_END_RX) {
		atpl250.m_ul_rx_total++;
		atpl250.m_ul_rx_total_bytes += s_phy_rx_ctl.m_us_rx_len;
		s_phy_rx_ctl.m_uc_rs_corrected_errors = 0;
		/* Perform end of reception */
		_trigger_data_indication();

		/* Clear flag */
		uc_phy_generic_flags &= ~PHY_GENERIC_FLAG_END_RX;

#ifdef ENABLE_PYH_PROCESS_RECALL
		/* Clear phy process recall, just in case it was previously set */
		uc_force_phy_process_recall = 0;
#endif

		#if (LOG_SPI == 1)
		dumpLogSpi();
		#endif
	}

	/* Check RS Flag */
	if (uc_phy_generic_flags & PHY_GENERIC_FLAG_CHECK_RS) {
		/* Clear flag */
		uc_phy_generic_flags &= ~PHY_GENERIC_FLAG_CHECK_RS;

		/* Disable PHY interrupt */
		disable_pplc_interrupt();

		/* Check Reed Solomon */
		s_phy_rx_ctl.m_uc_rs_corrected_errors
			= rs_correct_errors(&s_phy_rx_ctl.auc_rx_buf[us_rs_decode_idx], s_phy_rx_ctl.m_us_rx_len, (s_phy_rx_ctl.m_uc_rs_parity + 1));

		/* Enable PHY interrupt */
		enable_pplc_interrupt();

		if (s_phy_rx_ctl.m_uc_rs_corrected_errors != 255) {
			/* RS was able to correct errors */
			if ((s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_1_BLOCK) || (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_SECOND_RECEIVED)) {
				if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_SECOND_RECEIVED) {
					/* When 2 RS blocks are received, data in buffer contains payload + parity + syndrome for both blocks */
					/* Move data before passing it to upper layer */
					memmove(&s_phy_rx_ctl.auc_rx_buf[s_band_constants.uc_fch_len + s_phy_rx_ctl.m_us_rx_len],
							&s_phy_rx_ctl.auc_rx_buf[s_band_constants.uc_fch_len + s_phy_rx_ctl.m_us_rx_len + ((s_phy_rx_ctl.m_uc_rs_parity + 1) * 2)],
							s_phy_rx_ctl.m_us_rx_len);
					/* Rx length is double than stored in each block */
					s_phy_rx_ctl.m_us_rx_len <<= 1;
				}

				/* Reset Rx Timeout */
				ul_rx_timeout_timer = ul_ms_counter + PHY_RX_TIMEOUT_MS;

				/* Check if noise capture has to be delayed */
				if (atpl250.m_uc_delay_noise_capture_after_rx) {
					if (s_phy_rx_ctl.m_uc_rs_corrected_errors == 0) {
						/* Reset flag and delay capture */
						uc_phy_generic_flags &= ~PHY_GENERIC_FLAG_NOISE_CAPTURE;
						ul_ms_counter = oss_get_up_time_ms();
						ul_noise_capture_timer = ul_ms_counter + atpl250.m_ul_time_between_noise_captures;
					}
				}

				atpl250.m_ul_rx_total++;
				atpl250.m_ul_rx_total_bytes += s_phy_rx_ctl.m_us_rx_len;
				/* Perform end of reception */
				s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
				_trigger_data_indication();

				#if (LOG_SPI == 1)
				dumpLogSpi();
				#endif
			}
		} else {
			/* Errors could not be corrected */
			/* Clear pending symbols and reset Rx mode */
			s_phy_rx_ctl.m_us_rx_pending_symbols = 0;
			s_phy_rx_ctl.m_us_rx_len = 0;
			atpl250.m_ul_rx_RS_errors++;
			/* Disable Auto BER */
			pplc_if_and8(REG_ATPL250_BER_PERIPH_CFG1_VH8, 0x7F); /* AUTO_BER='0' */
			end_rx();
			s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;

			LOG_PHY(("RxErr RS\r\n"));
		}

#ifdef ENABLE_PYH_PROCESS_RECALL
		/* Clear phy process recall, just in case it was previously set */
		uc_force_phy_process_recall = 0;
#endif
	}

#ifdef ENABLE_PYH_PROCESS_RECALL
	return uc_force_phy_process_recall;

#else
	return 0;
#endif
}

/**
 * \brief Dummy Phy Tester Tool Serialization Addon
 *
 */
void Dummy_serial_if_init(void)
{
}

/**
 * \brief Dummy Sniffer Serialization Addon
 *
 */
void Dummy_sniffer_if_init(void)
{
}

/**
 * \brief Dummy Phy layer extension handler_start_tx
 *
 */
uint8_t Dummy_phy_extension_handler_start_tx(void)
{
	return false;
}

/**
 * \brief Dummy Phy layer extension handler_iob_tx
 *
 */
uint8_t Dummy_phy_extension_handler_iob_tx(void)
{
	return false;
}

void get_phy_tx_ctl( struct phy_tx_ctl *ps_phy_tx_ctl_dst )
{
	memcpy(ps_phy_tx_ctl_dst, &s_phy_tx_ctl, sizeof(struct phy_tx_ctl));
}

void get_sym_cfg( struct sym_cfg *ps_sym_cfg )
{
	memcpy(ps_sym_cfg, &s_sym_cfg, sizeof(struct sym_cfg));
}
