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

/* OSS includes */
#include "oss_if.h"

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

/* Symbol duration for PDC calculation */
#define FRAME_SYMBOL_DURATION                           695
#define PREAMBLE_SYMBOL_DURATION                        640
#define PREAMBLE_TIME_DURATION                          5760   /* 640*9 us */
#define HALF_SYMBOL_DURATION                            320   /* 640/2 us */

/* Number of symbols S1 S2 to configure interrupts */
#define NUM_SYMBOLS_S1S2   2

/* Time needed before start transmitting */
#define DELAYED_TX_SAFETY_TIME   (CFG_TXRX_PLC1_US + RRC_DELAY_US + 200)

/* Tx Gain Values*/
static uint8_t uc_phy_tx_full_gain = 0xFF;
const uint8_t auc_txgain[16] = {0xFF, 0xB4, 0x80, 0x5A, 0x40, 0x2D, 0x20, 0x17, 0x10, 0x0B, 0x08, 0x06, 0x04, 0x03, 0x02, 0x01};
const uint16_t aus_preemphasis_att_values[32] = {0x7FFF, 0x5A82, 0x4000, 0x2D41, 0x2000, 0x16A0, 0x1000, 0x0B50,
						 0x0800, 0x05A8, 0x0400, 0x02D4, 0x0200, 0x016A, 0x0100, 0x00B5,
						 0x0080, 0x005B, 0x0040, 0x002D, 0x0020, 0x0017, 0x0010, 0x000B,
						 0x0008, 0x0006, 0x0004, 0x0003, 0x0002, 0x0001, 0x0001, 0x0001};

const uint16_t aus_ripple_correction[16][PROTOCOL_CARRIERS]
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
const uint16_t aus_ripple_correction_LO[16][PROTOCOL_CARRIERS]
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

const uint16_t aus_ripple_correction_HI[16][PROTOCOL_CARRIERS]
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
const uint8_t auc_full_psymbol[P_SYMBOL_LEN_MAX] = {0x21, 0x0F, 0xEC, 0xA7, 0x3F, 0xB6,
						    0x1B, 0x5E, 0x7F, 0x7F, 0x6D, 0x28,
						    0xD2, 0x6A, 0xD0, 0x23, 0x56, 0x77};

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

uint8_t auc_psymbol[P_SYMBOL_LEN_MAX];

/* Buffer to construct FCH */
static uint8_t auc_fch[FCH_LEN];

/* PHY Parameters structure instance */
static atpl250_t atpl250;

/* BER data structure instances */
static struct s_rx_ber_fch_data_t s_rx_ber_fch_data;
static struct s_rx_ber_payload_data_t s_rx_ber_payload_data;

/* TM Response data structure instance */
static struct s_tone_map_response_data_t s_tone_map_response_data;

/* Default Static Notching Array */
uint8_t auc_static_notching_pos[CARR_BUFFER_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
/* SFSK notching Carriers [39-49] */
/* uint8_t auc_static_notching_pos[CARR_BUFFER_LEN] = {0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; */

/*Legacy Mode Indicator*/
uint8_t uc_legacy_mode = 0x01;

/* Hard reset Flag */
uint8_t uc_hard_reset_flag = 0x01;

/* Noise capture constants and variables */
#define NOISE_CAPTURE_ADAPT_SYMBOLS   2
#define NOISE_CAPTURE_SYMBOLS         8
#define NOISE_CAPTURE_PERIOD          60000 /* ms */
#define NOISE_CAPTURE_DELTA           100 /* us */
#define NOISE_CAPTURE_FFT_SHIFT       3
#define RSSI_VALUES_TABLE_SIZE        80
#define NOISE_CORRECTION_FACTOR_DB    49
#define NOISE_MIN_VALUE_DB            30
#define MIN_NOISE_DERIVATIVE_2_DB     6
#define MIN_NOISE_DERIVATIVE_3_DB     6
#define MIN_DIFF_PREV_CARRIER_DB      4
#define MIN_NOISE_AVG_DIF_DB          12
#define MIN_NOISE_NOTCH_DB            (73 - NOISE_CORRECTION_FACTOR_DB)
/* Table containing values of RSSI / AGC FACTOR to obtain db values depending on index */
const uint32_t caul_rssi_values[RSSI_VALUES_TABLE_SIZE] = {1, 1, 1, 2, 2, 3, 4, 5, 7, 8, 11, 14, 17, 22, 28,
							   35, 44, 56, 70, 89, 112, 141, 177, 223, 281, 354, 446, 562, 707, 891, 1122,
							   1412, 1778, 2238, 2818, 3548, 4466, 5623, 7079, 8912, 11220, 14125, 17782,
							   22387, 28183, 35481, 44668, 56234, 70794, 89125, 112201, 141253, 177827,
							   223872, 281838, 354813, 446683, 562341, 707945, 891250, 1122018, 1412537,
							   1778279, 2238721, 2818382, 3548133, 4466835, 5623413, 7079457, 8912509,
							   11220184, 14125375, 17782794, 22387211, 28183829, 35481338, 44668359,
							   56234132, 70794578, 89125093};

enum ber_status {
	BER_FCH = 0,
	BER_PAYLOAD = 1
};

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

static uint16_t us_noise_agc_factor;
static uint32_t ul_noise_capture_timer;
static uint32_t ul_ms_counter;
static volatile uint8_t uc_start_interrupt_executed;

#ifdef ENABLE_PYH_PROCESS_RECALL
static uint8_t uc_force_phy_process_recall;
#endif

/* PDC ratio for PDC calculation */
static uint8_t uc_pdc_ratio;

enum ber_status e_ber_status;

#ifdef ENABLE_SNIFFER
static xPhyMsgTxResult_t tx_result_sniffer;
static xPhyMsgTx_t tx_param_sniffer;
static xPhyMsgRx_t rx_msg_sniffer;
#endif

#ifdef ENABLE_SIGNAL_DUMP
#define DUMP_BUFFERS 20 /* Each buffer captures 2,56ms */
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
const struct emit_gain_limits_type emit_gain_limits[NUM_IMPEDANCE_STATES] = {
	{EMIT_GAIN_HI, 0x70, 0x1A},
	{EMIT_GAIN_LO, 0, 0},
	{EMIT_GAIN_VLO, 0x86, 0x3A}
};

const uint32_t caul_max_rms_hi[NUM_TX_LEVELS] = {79000118, 55861519, 39500059, 27930760, 19750030, 13965380, 9875015, 6982690};
const uint32_t caul_max_rms_vlo[NUM_TX_LEVELS] = {254295002, 179813720, 127147501, 89906860, 63573750, 44953430, 31786875, 22476715};

/* const uint32_t caul_th1_hi[NUM_TX_LEVELS]= {TXLEVEL00, TXLEVEL01, TXLEVEL02, TXLEVEL03, TXLEVEL04, TXLEVEL05, TXLEVEL06, TXLEVEL07}; */
/*Threshold levels using HI-VLO states*/
const uint32_t caul_th1_hi[NUM_TX_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint32_t caul_th2_hi[NUM_TX_LEVELS] = {67150100, 47482291, 33575050, 23741146, 16787525, 11870573, 8393763, 5935287};
const uint32_t caul_th1_lo[NUM_TX_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint32_t caul_th2_lo[NUM_TX_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint32_t caul_th1_vlo[NUM_TX_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
#ifdef DISABLE_VLO_TO_HI_JUMP
const uint32_t caul_th2_vlo[NUM_TX_LEVELS] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
#else
const uint32_t caul_th2_vlo[NUM_TX_LEVELS] = {333236691, 235633924, 166618345, 117816962, 83309173, 58908481, 41654586, 29454240};
#endif

/* Shared PHY Variables */
union shared_phy_buffers u_shared_buffers;
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
uint8_t auc_unmasked_carrier_list[PROTOCOL_CARRIERS];
uint8_t auc_masked_carrier_list[PROTOCOL_CARRIERS];
uint8_t auc_predistortion[PROTOCOL_CARRIERS * 4];
int32_t asl_freq_index[PROTOCOL_CARRIERS], asl_freq_index_squared[PROTOCOL_CARRIERS]; /*frequency indexes and squared frequency indexes in Q8.24*/
int32_t asl_delay_symbols[(NUM_FULL_SYMBOLS_PREAMBLE + SYMBOLS_8 + 2)] = {0}; /*first sample of the FFT window of each symbol in Q6.26*/
uint8_t uc_agc_ext;
uint16_t us_agc_int;
uint8_t uc_impulsive_noise_detected;

/*--------------------------------------------------------------------------------------*/

/**
 * \brief Inverts the byte order of the int16_t elements of array
 *
 */
static inline void _invert_byte_order(int16_t *pss_input_array, uint16_t us_num_elements)
{
	uint16_t us_i;
	for (us_i = 0; us_i < us_num_elements; us_i++) {
		*(pss_input_array + us_i) = (*(pss_input_array + us_i)  << 8) | ((*(pss_input_array + us_i) >> 8) & 0x00FF);
	}
}

/**
 * \brief Function set tx_preemphasis
 *
 */
static void _set_preemphasis(uint8_t *puc_preemphasis)
{
	uint8_t uc_j;
	uint8_t uc_preemphasis_value;
	uint8_t uc_notched_carrier;
	uint16_t us_i;
	uint16_t us_pe_value;

	uint16_t us_ripple_correction;
	uint16_t us_pe_value_ref;
	uint8_t uc_k;
	uc_k = 0;

	/* Clear predistortion buffer, as it is in the sahred phy buffers area */
	memset(auc_predistortion, 0, sizeof(auc_predistortion));

	for (us_i = 0; us_i < NUM_SUBBANDS; us_i++) {
		uc_preemphasis_value = puc_preemphasis[us_i];

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

		auc_predistortion[us_i * NUM_CARRIERS_IN_SUBBAND * 4 + 0] = (us_pe_value & 0xFF00) >> 8;
		auc_predistortion[us_i * NUM_CARRIERS_IN_SUBBAND * 4 + 1] = us_pe_value & 0x00FF;

		if (atpl250.m_uc_impedance_state == VLO_STATE) {
			us_ripple_correction = aus_ripple_correction[uc_preemphasis_value][uc_k++];
		} else if (atpl250.m_uc_impedance_state == LO_STATE) {
			us_ripple_correction = aus_ripple_correction_LO[uc_preemphasis_value][uc_k++];
		} else {
			us_ripple_correction = aus_ripple_correction_HI[uc_preemphasis_value][uc_k++];
		}

		us_pe_value = us_pe_value_ref - us_ripple_correction;

		auc_predistortion[us_i * NUM_CARRIERS_IN_SUBBAND * 4 + 4] = (us_pe_value & 0xFF00) >> 8;
		auc_predistortion[us_i * NUM_CARRIERS_IN_SUBBAND * 4 + 5] = us_pe_value & 0x00FF;

		if (atpl250.m_uc_impedance_state == VLO_STATE) {
			us_ripple_correction = aus_ripple_correction[uc_preemphasis_value][uc_k++];
		} else if (atpl250.m_uc_impedance_state == LO_STATE) {
			us_ripple_correction = aus_ripple_correction_LO[uc_preemphasis_value][uc_k++];
		} else {
			us_ripple_correction = aus_ripple_correction_HI[uc_preemphasis_value][uc_k++];
		}

		us_pe_value = us_pe_value_ref - us_ripple_correction;

		auc_predistortion[us_i * NUM_CARRIERS_IN_SUBBAND * 4 + 8] = (us_pe_value & 0xFF00) >> 8;
		auc_predistortion[us_i * NUM_CARRIERS_IN_SUBBAND * 4 + 9] = us_pe_value & 0x00FF;

		if (atpl250.m_uc_impedance_state == VLO_STATE) {
			us_ripple_correction = aus_ripple_correction[uc_preemphasis_value][uc_k++];
		} else if (atpl250.m_uc_impedance_state == LO_STATE) {
			us_ripple_correction = aus_ripple_correction_LO[uc_preemphasis_value][uc_k++];
		} else {
			us_ripple_correction = aus_ripple_correction_HI[uc_preemphasis_value][uc_k++];
		}

		us_pe_value = us_pe_value_ref - us_ripple_correction;
		auc_predistortion[us_i * NUM_CARRIERS_IN_SUBBAND * 4 + 12] = (us_pe_value & 0xFF00) >> 8;
		auc_predistortion[us_i * NUM_CARRIERS_IN_SUBBAND * 4 + 13] = us_pe_value & 0x00FF;

		if (atpl250.m_uc_impedance_state == VLO_STATE) {
			us_ripple_correction = aus_ripple_correction[uc_preemphasis_value][uc_k++];
		} else if (atpl250.m_uc_impedance_state == LO_STATE) {
			us_ripple_correction = aus_ripple_correction_LO[uc_preemphasis_value][uc_k++];
		} else {
			us_ripple_correction = aus_ripple_correction_HI[uc_preemphasis_value][uc_k++];
		}

		us_pe_value = us_pe_value_ref - us_ripple_correction;
		auc_predistortion[us_i * NUM_CARRIERS_IN_SUBBAND * 4 + 16] = (us_pe_value & 0xFF00) >> 8;
		auc_predistortion[us_i * NUM_CARRIERS_IN_SUBBAND * 4 + 17] = us_pe_value & 0x00FF;

		if (atpl250.m_uc_impedance_state == VLO_STATE) {
			us_ripple_correction = aus_ripple_correction[uc_preemphasis_value][uc_k++];
		} else if (atpl250.m_uc_impedance_state == LO_STATE) {
			us_ripple_correction = aus_ripple_correction_LO[uc_preemphasis_value][uc_k++];
		} else {
			us_ripple_correction = aus_ripple_correction_HI[uc_preemphasis_value][uc_k++];
		}

		us_pe_value = us_pe_value_ref - us_ripple_correction;
		auc_predistortion[us_i * NUM_CARRIERS_IN_SUBBAND * 4 + 20] = (us_pe_value & 0xFF00) >> 8;
		auc_predistortion[us_i * NUM_CARRIERS_IN_SUBBAND * 4 + 21] = us_pe_value & 0x00FF;
	}

	/* Set 0s in notched carriers */
	for (uc_j = 0; uc_j < uc_notched_carriers; uc_j++) {
		uc_notched_carrier = (auc_masked_carrier_list[uc_j] << 2);
		auc_predistortion[uc_notched_carrier + 0] = 0x00;
		auc_predistortion[uc_notched_carrier + 1] = 0x00;
		auc_predistortion[uc_notched_carrier + 2] = 0x00;
		auc_predistortion[uc_notched_carrier + 3] = 0x00;
	}

	/* Write to Zone1 */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 0) + LAST_CARRIER); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((0 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 1) + LAST_CARRIER); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((1 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 2) + LAST_CARRIER); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((2 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 3) + LAST_CARRIER); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((3 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 4) + LAST_CARRIER); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((4 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 5) + LAST_CARRIER); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((5 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 6) + LAST_CARRIER); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((6 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 7) + LAST_CARRIER); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_buf((BCODE_ZONE1 | ((7 * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER)), auc_predistortion, PROTOCOL_CARRIERS * 4);
}

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
	uint16_t us_time_error = HALF_SYMBOL_DURATION;

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
		ul_zctx = (timer_ref - ul_zc_time) + CFG_TXRX_PLC1_US + PREAMBLE_TIME_DURATION;
	} else {
		ul_zctx = (timer_ref - ul_zc_time + 0xFFFFFFFF) + CFG_TXRX_PLC1_US + PREAMBLE_TIME_DURATION;
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
 * \param uc_buf_id   Buffer Id
 * \param uc_forced   Forced tx (no carrier sense)
 * \param uc_delayed  Delayed tx (not immediate)
 * \param ul_tx_time  Tx time (only valid if uc_delayed=1)
 *
 */
static void _config_tx(uint8_t uc_buf_id, uint8_t uc_forced, uint8_t uc_delayed, uint32_t ul_tx_time)
{
	uint8_t uc_mask;
	uint8_t uc_mask_immediate;
	uint32_t ul_reg_addr;
	uint32_t ul_plc_time;

	/* Set variables depending on buffer id */
	switch (uc_buf_id) {
	case 0:
		uc_mask = 0x01;
		uc_mask_immediate = 0x11;
		ul_reg_addr = REG_ATPL250_TX_TIME1_32;
		ul_plc_time = CFG_TXRX_PLC1_US + RRC_DELAY_US;
		break;

	case 1:
		uc_mask = 0x02;
		uc_mask_immediate = 0x22;
		ul_reg_addr = REG_ATPL250_TX_TIME2_32;
		ul_plc_time = CFG_TXRX_PLC2_US + RRC_DELAY_US;
		break;

	case 2:
		uc_mask = 0x04;
		uc_mask_immediate = 0x44;
		ul_reg_addr = REG_ATPL250_TX_TIME3_32;
		ul_plc_time = CFG_TXRX_PLC3_US + RRC_DELAY_US;
		break;

	case 3:
		uc_mask = 0x08;
		uc_mask_immediate = 0x88;
		ul_reg_addr = REG_ATPL250_TX_TIME3_32;
		ul_plc_time = CFG_TXRX_PLC4_US + RRC_DELAY_US;
		break;

	default:
		uc_mask = 0x01;
		uc_mask_immediate = 0x11;
		ul_reg_addr = REG_ATPL250_TX_TIME1_32;
		ul_plc_time = CFG_TXRX_PLC1_US + RRC_DELAY_US;
		break;
	}

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
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 7) + LAST_CARRIER); /* Avoid overflow to n-th symbol when writing */
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
	pplc_if_write_jump((BCODE_COH | (FIRST_CARRIER + auc_unmasked_carrier_list[0]
			+ (7 * CFG_IOB_SAMPLES_PER_SYMBOL))), (uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_1);

	atpl250_hold_on_reference(false);

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
		pplc_if_write_jump((BCODE_DIFT | (FIRST_CARRIER + auc_masked_carrier_list[0])), auc_predistortion, uc_notched_carriers, JUMP_COL_1);
	}

	/* Set number of symbols */
	atpl250_set_num_symbols_cfg(1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, LAST_CARRIER); /* Avoid overflow to n-th symbol when writing */
	atpl250_set_mod_bpsk_truepoint();
	pplc_if_write_jump((BCODE_COH | (FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_1);
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
	pplc_if_write_jump((BCODE_COH | (FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_1);
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

	atpl250_hold_on_reference(false);

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
		feed_modulator_fch(s_phy_tx_ctl.m_us_tx_pending_symbols, NO_SYM_OFFSET, NO_CHANGE_MODULATION);
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
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, LAST_CARRIER); /* Avoid overflow to 1st symbol when writing */
	pplc_if_write_jump((BCODE_COH | (FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, CFG_IOB_SAMPLES_PER_SYMBOL + LAST_CARRIER); /* Avoid overflow to 2nd symbol when writing */
	pplc_if_write_jump((BCODE_COH | (CFG_IOB_SAMPLES_PER_SYMBOL + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 2) + LAST_CARRIER); /* Avoid overflow to 3rd symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 2) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 3) + LAST_CARRIER); /* Avoid overflow to 4th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 3) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 4) + LAST_CARRIER); /* Avoid overflow to 5th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 4) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 5) + LAST_CARRIER); /* Avoid overflow to 6th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 5) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 6) + LAST_CARRIER); /* Avoid overflow to 7th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 6) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 7) + LAST_CARRIER); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 7) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
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
	uc_used_carriers = PROTOCOL_CARRIERS - uc_notched_carriers;

	/* Calculate number of FCH symbols = (([FCH bits] + 6)*2*6 + [used carriers] - 1) / ([used carriers]) */
	uc_num_symbols_fch = ((FCH_LEN_BITS + 6) * 2 * 6 + uc_used_carriers - 1) / (uc_used_carriers);

	/* Fill P symbol */
	uc_p_index = 0;
	uc_p_index_new = 0;
	uc_masked_index = 0;
	memset(auc_psymbol, 0, P_SYMBOL_LEN_MAX);
	for (uc_i = FIRST_CARRIER; uc_i <= LAST_CARRIER; uc_i++) {
		/* Calculate byte and bit position for index */
		uc_byte_index = uc_i >> 3;
		uc_bit_index = uc_i & 0x07;

		if (!(auc_static_notching_pos[uc_byte_index] & (1 << uc_bit_index))) { /* Used carrier */
			/* List with the indexes of active carriers */
			auc_unmasked_carrier_list[uc_p_index_new] = uc_i - FIRST_CARRIER;

			/* Get carrier value from full P symbol */
			if (uc_p_index % 2) {
				/* Low nibble */
				uc_carrier_value = auc_full_psymbol[uc_p_index / 2] & 0x0F;
			} else {
				/* High nibble */
				uc_carrier_value = (auc_full_psymbol[uc_p_index / 2] & 0xF0) >> 4;
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
			auc_masked_carrier_list[uc_masked_index] = uc_i - FIRST_CARRIER;
			uc_masked_index++;
		}

		/* Advance index for full p symbol (in any case) */
		uc_p_index++;
	}

	/* Set P symbol length */
	uc_psymbol_len = (uc_used_carriers + 1) / 2;

	/* Calculate variables needed for channel estimation depending on notching */
	/* ceil(FCH_INTERLEAVER_USEFUL_SIZE / carriers) * carriers */
	us_fch_interleaver_size_with_padding = ((FCH_INTERLEAVER_USEFUL_SIZE + uc_used_carriers - 1) / uc_used_carriers) * uc_used_carriers;
	/* ceil(FCH_INTERLEAVER_SIZE_WITH_PADDING / 8 bits in byte) */
	uc_fch_interleaver_byte_size = (us_fch_interleaver_size_with_padding + 7) / 8;
	/* ceil(carriers / 8 bits in byte) */
	uc_fch_symbol_byte_size = (uc_used_carriers + 7) / 8;
	uc_psymbol_no_truepoint_byte_len = uc_fch_symbol_byte_size;

	/* Calculate clock resample variable */
	ul_resample_var = 2275L + ((uc_num_symbols_fch - 8) * 278L);
	ull_resample_const2_inv = (RESAMPLE_CONST_2 * (uint64_t)ul_resample_var) / RESAMPLE_CONST_1;
	ul_resample_const2_inv = (uint32_t)ull_resample_const2_inv;

	/*Calculate vectors used for SFO estimation*/
	/*Vector of frequency indexes and of the squared frequency indexes in Q1.31 scaled by 1/2^SCALING_FREQ_VALUES*/
	for (uc_i = 0; uc_i < uc_used_carriers; uc_i++) {
		asl_freq_index[uc_i] = (VALUE_1_2_Q_1_31 >> (SCALING_FREQ_VALUES - 1)) * ((int32_t)(auc_unmasked_carrier_list[uc_i] + FIRST_CARRIER));
		asl_freq_index_squared[uc_i]
			= (((VALUE_1_2_Q_1_31 >>
				(SCALING_FREQ_VALUES -
				1)) *
				((int32_t)(auc_unmasked_carrier_list[uc_i] +
				FIRST_CARRIER))) >> SCALING_FREQ_VALUES) * ((int32_t)(auc_unmasked_carrier_list[uc_i] + FIRST_CARRIER));
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
}

/**
 * \brief Initialize PHY layer variables and structures
 *
 */
static void _init_phy_layer(void)
{
	#if defined(SMOOTHING)
	uint8_t auc_empty_map[CARR_BUFFER_LEN] = {0};
	uint8_t auc_pilot_empty[PROTOCOL_CARRIERS], uc_empty = 0;
	#endif

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
		atpl250.m_uc_enable_auto_noise_capture = 1;
		uc_phy_generic_flags |= PHY_GENERIC_FLAG_NOISE_CAPTURE;
#endif
		atpl250.m_uc_delay_noise_capture_after_rx = 1;
		atpl250.m_ul_last_rmscalc_corrected = 0x00000000;
		uc_hard_reset_flag = 0x00;
	}

	uc_pdc_ratio = 0; /* Invalid value, it will be used in order to detect if it has been initialized */

	/* HW Registers Initialization */
	atpl250_hw_init(atpl250.m_uc_impedance_state);

	#if defined(SMOOTHING)
	/* Calculates the list of carriers to be smoothed */
	control_smooth(auc_static_notching_pos, auc_empty_map, auc_smooth_control_preamble_fch, uc_num_carr_smooth_preamble_fch, auc_pilot_empty, &uc_empty);
	#endif

	/* Configures the resampling register */
	pplc_if_write32(REG_ATPL250_RESAMP24BITS_1_32, RESAMPLE_STEP);
	pplc_if_write32(REG_ATPL250_RESAMP24BITS_2_32, RESAMPLING_24BITS_2_VALUE);

	/* Configure Jump RAM */
	init_jump_ram((uint8_t *)auc_static_notching_pos); /* Static notching as parameter is equivalent to configure jumps for a full tone map */

	/* Set P symbol phases as reference for later coherent demodulation */
	set_p_symbol_phases(auc_psymbol);

	/* Prepare for Rx */
	prepare_next_rx();

	atpl250_set_mod_bpsk_truepoint();

	disable_HW_chain();     /* Disable HW chain to write in modulator */
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, LAST_CARRIER); /* Avoid overflow to 1st symbol when writing */
	pplc_if_write_jump((BCODE_COH | (FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, CFG_IOB_SAMPLES_PER_SYMBOL + LAST_CARRIER); /* Avoid overflow to 2nd symbol when writing */
	pplc_if_write_jump((BCODE_COH | (CFG_IOB_SAMPLES_PER_SYMBOL + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 2) + LAST_CARRIER); /* Avoid overflow to 3rd symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 2) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 3) + LAST_CARRIER); /* Avoid overflow to 4th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 3) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 4) + LAST_CARRIER); /* Avoid overflow to 5th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 4) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 5) + LAST_CARRIER); /* Avoid overflow to 6th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 5) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 6) + LAST_CARRIER); /* Avoid overflow to 7th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 6) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
	pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 7) + LAST_CARRIER); /* Avoid overflow to 8th symbol when writing */
	pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 7) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
			uc_psymbol_len, JUMP_COL_1);
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
	uint32_t ul_curr_time;

	/* Read current time */
	ul_curr_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
	/* Set capture time */
	pplc_if_write32(REG_ATPL250_TX_NOISE_TIME_32, ul_curr_time + NOISE_CAPTURE_DELTA);
	/* Activate Noise measure */
	pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x10);
}

/**
 * \brief Function to handle Noise Capture
 *
 */
static void _analyse_noise_capture(void)
{
	uint8_t uc_i, uc_i_filter_applied;
	uint8_t uc_idx, uc_idx_max, uc_idx_min;
	uint8_t uc_max_noise, uc_max_noise_derivative2, uc_selected_carrier_to_notch = 0;
	uint8_t uc_perform_third_derivative = 1;
	uint16_t us_sum_noise_db, us_avg_noise_db;
	uint16_t *puc_noise_per_carrier;
	uint32_t ul_int_flags;
	uint32_t ul_aux_squared_module;
	uint32_t ul_agc_factor_squared;
	uint32_t ul_rssi_sifted_by_agc;

	if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_FIRST_PASS) {
		atpl250_clear_iobuf_to_fft();
		atpl250_clear_rx_full();
		/* Check Peak1 */
		ul_int_flags = pplc_if_read32(REG_ATPL250_INT_FLAGS_32);
		if (ul_int_flags & INT_PEAK1_MASK_32) {
			/* Restore FFT_SHIFT value */
			pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, RX_FFT_SHIFT);
			/* Clear Rx Full */
			atpl250_clear_rx_full();
			/* Set Rx step */
			s_phy_rx_ctl.e_rx_step = STEP_RX_NO_RX;
			/* Return to receive frame */
			return;
		}
	} else { /* STEP_RX_NOISE_CAPTURE_SECOND_PASS */
		atpl250_rotator_and_fft_push_rst();
		atpl250_clear_iobuf_to_fft();
		atpl250_clear_rx_full();

		/* Restore FFT_SHIFT value */
		pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, RX_FFT_SHIFT);

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
		pplc_if_read_buf((BCODE_ZONE4 | ((uc_i * CFG_IOB_SAMPLES_PER_SYMBOL) + FIRST_CARRIER - (EXTRA_CARRIERS_TO_READ_NOISE >> 1))),
				(uint8_t *)(u_shared_buffers.s_noise.ass_noise_capture), (PROTOCOL_CARRIERS + EXTRA_CARRIERS_TO_READ_NOISE) << 2);
		/* Invert byte order */
		_invert_byte_order(u_shared_buffers.s_noise.ass_noise_capture, (PROTOCOL_CARRIERS + EXTRA_CARRIERS_TO_READ_NOISE) << 1);
		/* Get squared magnitude */
		arm_cmplx_mag_squared_q15(u_shared_buffers.s_noise.ass_noise_capture, u_shared_buffers.s_noise.ass_noise_squared_mag,
				PROTOCOL_CARRIERS + EXTRA_CARRIERS_TO_READ_NOISE);
		/* Shift and average. Shift -4, because the final add will be 8 + 8 symbols */
		arm_shift_q15(u_shared_buffers.s_noise.ass_noise_squared_mag, -4, u_shared_buffers.s_noise.ass_noise_squared_mag,
				PROTOCOL_CARRIERS + EXTRA_CARRIERS_TO_READ_NOISE);
		arm_add_q15(u_shared_buffers.s_noise.ass_noise_squared_mag, u_shared_buffers.s_noise.ass_noise_avg,
				u_shared_buffers.s_noise.ass_noise_avg, PROTOCOL_CARRIERS + EXTRA_CARRIERS_TO_READ_NOISE);

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
		s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_SECOND_PASS;
	} else { /* STEP_RX_NOISE_CAPTURE_SECOND_PASS */
		 /* Transform values to dB */
		ul_agc_factor_squared = (uint32_t)((us_noise_agc_factor * us_noise_agc_factor)) >> 12;
		us_sum_noise_db = 0;
		for (uc_i = 0; uc_i < (PROTOCOL_CARRIERS + EXTRA_CARRIERS_TO_READ_NOISE); uc_i++) {
			ul_aux_squared_module = (uint32_t)(u_shared_buffers.s_noise.ass_noise_avg[uc_i]);

			if (ul_aux_squared_module > 0) {
				ul_rssi_sifted_by_agc = (ul_aux_squared_module << 16) / ul_agc_factor_squared;
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
				u_shared_buffers.s_noise.ass_noise_avg[uc_i] = uc_idx_min;
			} else {
				u_shared_buffers.s_noise.ass_noise_avg[uc_i] = 0;
			}

			/* Sum values for carriers in band to get avg later */
			if ((uc_i > (EXTRA_CARRIERS_TO_READ_NOISE >> 1)) && (uc_i <= (PROTOCOL_CARRIERS + (EXTRA_CARRIERS_TO_READ_NOISE >> 1)))) {
				us_sum_noise_db += u_shared_buffers.s_noise.ass_noise_avg[uc_i];
			}
		}

		/* Get average value */
		us_avg_noise_db = us_sum_noise_db / PROTOCOL_CARRIERS;
		atpl250.m_uc_noise_peak_power = us_avg_noise_db + NOISE_CORRECTION_FACTOR_DB;

		LOG_PHY(("Analyse Noise Capture\r\n"));

		/* Calculate noise derivative and look for noised tones */
		/* Derivative 2 */
		puc_noise_per_carrier = (uint16_t *)(&u_shared_buffers.s_noise.ass_noise_avg[2]);
		uc_max_noise = 0;
		uc_i_filter_applied = 0xFF;
		for (uc_i = 0; uc_i < (PROTOCOL_CARRIERS + EXTRA_CARRIERS_TO_READ_NOISE - 4); uc_i++) {
			/* Derivative */
			if ((*(puc_noise_per_carrier + uc_i) > *(puc_noise_per_carrier + uc_i - 2)) &&
					(*(puc_noise_per_carrier + uc_i) > *(puc_noise_per_carrier + uc_i + 2)) &&
					((*(puc_noise_per_carrier + uc_i) - *(puc_noise_per_carrier + uc_i - 2)) > MIN_NOISE_DERIVATIVE_2_DB) &&
					((*(puc_noise_per_carrier + uc_i) - *(puc_noise_per_carrier + uc_i + 2)) > MIN_NOISE_DERIVATIVE_2_DB)) {
				/* Difference with average */
				if ((*(puc_noise_per_carrier + uc_i) > us_avg_noise_db) &&
						((*(puc_noise_per_carrier + uc_i) - us_avg_noise_db) > MIN_NOISE_AVG_DIF_DB)) {
					/* Carrier noise */
					if (*(puc_noise_per_carrier + uc_i) > MIN_NOISE_NOTCH_DB) {
						/* Noised carrier, if it has the maximum noise value, select it to be notched */
						if (*(puc_noise_per_carrier + uc_i) > uc_max_noise) {
							uc_max_noise = *(puc_noise_per_carrier + uc_i);
							uc_selected_carrier_to_notch = FIRST_CARRIER + uc_i - (EXTRA_CARRIERS_TO_READ_NOISE >> 1) + 2;
							uc_i_filter_applied = uc_i;
							atpl250.m_uc_noise_peak_power = *(puc_noise_per_carrier + uc_i) + NOISE_CORRECTION_FACTOR_DB;
							LOG_PHY(("C2 %u %udB\r\n", uc_selected_carrier_to_notch,
									*(puc_noise_per_carrier + uc_i) + NOISE_CORRECTION_FACTOR_DB));
						}
					}
				}
			}
		}
		uc_max_noise_derivative2 = uc_max_noise;
		if (uc_max_noise) {
			/* Check if noise in next carrier is near the max, to apply filter on that carrier */
			if (*(puc_noise_per_carrier + uc_i_filter_applied + 1) > (uc_max_noise - MIN_DIFF_PREV_CARRIER_DB)) {
				uc_selected_carrier_to_notch = FIRST_CARRIER + uc_i_filter_applied + 1 - (EXTRA_CARRIERS_TO_READ_NOISE >> 1) + 2;
				uc_perform_third_derivative = 0;
				LOG_PHY(("C2 LESS %u %udB\r\n", uc_selected_carrier_to_notch,
						*(puc_noise_per_carrier + uc_i_filter_applied + 1) + NOISE_CORRECTION_FACTOR_DB));
			}
		}

		if (uc_perform_third_derivative) {
			/* Derivative 3. Look for noises greater than in derivative 2 */
			puc_noise_per_carrier = (uint16_t *)(&u_shared_buffers.s_noise.ass_noise_avg[3]);
			for (uc_i = 0; uc_i < (PROTOCOL_CARRIERS + EXTRA_CARRIERS_TO_READ_NOISE - 3); uc_i++) {
				/* Derivative */
				if ((*(puc_noise_per_carrier + uc_i) > *(puc_noise_per_carrier + uc_i - 3)) &&
						(*(puc_noise_per_carrier + uc_i) > *(puc_noise_per_carrier + uc_i + 3)) &&
						((*(puc_noise_per_carrier + uc_i) - *(puc_noise_per_carrier + uc_i - 3)) > MIN_NOISE_DERIVATIVE_3_DB) &&
						((*(puc_noise_per_carrier + uc_i) - *(puc_noise_per_carrier + uc_i + 3)) > MIN_NOISE_DERIVATIVE_3_DB)) {
					/* Difference with average */
					if ((*(puc_noise_per_carrier + uc_i) > us_avg_noise_db) &&
							((*(puc_noise_per_carrier + uc_i) - us_avg_noise_db) > MIN_NOISE_AVG_DIF_DB)) {
						/* Carrier noise */
						if (*(puc_noise_per_carrier + uc_i) > MIN_NOISE_NOTCH_DB) {
							/* Noised carrier, if it has the maximum noise value, select it to be notched */
							if (*(puc_noise_per_carrier + uc_i) > uc_max_noise) {
								uc_max_noise = *(puc_noise_per_carrier + uc_i);
								uc_selected_carrier_to_notch = FIRST_CARRIER + uc_i - (EXTRA_CARRIERS_TO_READ_NOISE >> 1) + 3;
								uc_i_filter_applied = uc_i;
								atpl250.m_uc_noise_peak_power = *(puc_noise_per_carrier + uc_i) + NOISE_CORRECTION_FACTOR_DB;
								LOG_PHY(("C3 %u %udB\r\n", uc_selected_carrier_to_notch,
										*(puc_noise_per_carrier + uc_i) + NOISE_CORRECTION_FACTOR_DB));
							}
						}
					}
				}
			}
		}

		if (uc_max_noise > uc_max_noise_derivative2) {
			/* Check if noise in next carrier is near the max, to apply filter on that carrier */
			if (*(puc_noise_per_carrier + uc_i_filter_applied + 1) > (uc_max_noise - MIN_DIFF_PREV_CARRIER_DB)) {
				uc_selected_carrier_to_notch = FIRST_CARRIER + uc_i_filter_applied + 1 - (EXTRA_CARRIERS_TO_READ_NOISE >> 1) + 3;
				LOG_PHY(("C3 LESS %u %udB\r\n", uc_selected_carrier_to_notch,
						*(puc_noise_per_carrier + uc_i_filter_applied + 1) + NOISE_CORRECTION_FACTOR_DB));
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
						= ((emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init *
							(tx_result.m_ul_rms_calc >> 16)) / uc_current_emit_gain) << 16;
					switch (atpl250.m_uc_impedance_state) {
					case HI_STATE:
						if (atpl250.m_uc_auto_detect_impedance == AUTO_STATE_VAR_GAIN) {
							if (atpl250.m_ul_last_rmscalc_corrected < caul_th2_hi[uc_last_tx_power]) {
								/* Go to state VLO_STATE */
								atpl250.m_uc_impedance_state = VLO_STATE;
								us_new_emit_gain = (uint16_t)emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init;
							} else if (atpl250.m_ul_last_rmscalc_corrected < caul_th1_hi[uc_last_tx_power]) {
								/* Go to state LO_STATE */
								atpl250.m_uc_impedance_state = LO_STATE;
								us_new_emit_gain = (uint16_t)emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init;
							} else {
								/* Continue in state HI_STATE */
								us_new_emit_gain
									= ((caul_max_rms_hi[uc_last_tx_power] >>
										16) * uc_current_emit_gain) / (tx_result.m_ul_rms_calc >> 16);
								if (us_new_emit_gain > emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max) {
									us_new_emit_gain = emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max;
								} else if (us_new_emit_gain < emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min) {
									us_new_emit_gain = emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min;
								}
							}
						} else {
							/* FIXED_STATE_VAR_GAIN. Continue in state HI_STATE */
							us_new_emit_gain
								= ((caul_max_rms_hi[uc_last_tx_power] >>
									16) * uc_current_emit_gain) / (tx_result.m_ul_rms_calc >> 16);
							if (us_new_emit_gain > emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max) {
								us_new_emit_gain = emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max;
							} else if (us_new_emit_gain < emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min) {
								us_new_emit_gain = emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min;
							}
						}

						break;

					case LO_STATE:
						if (atpl250.m_uc_auto_detect_impedance == AUTO_STATE_VAR_GAIN) {
							if (atpl250.m_ul_last_rmscalc_corrected < caul_th2_lo[uc_last_tx_power]) {
								/* Go to state VLO_STATE */
								atpl250.m_uc_impedance_state = VLO_STATE;
							} else if (atpl250.m_ul_last_rmscalc_corrected > caul_th1_lo[uc_last_tx_power]) {
								/* Go to state HI_STATE */
								atpl250.m_uc_impedance_state = HI_STATE;
							}
						}

						us_new_emit_gain = emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init;
						break;

					case VLO_STATE:
						if (atpl250.m_uc_auto_detect_impedance == AUTO_STATE_VAR_GAIN) {
							if (atpl250.m_ul_last_rmscalc_corrected < caul_th2_vlo[uc_last_tx_power]) {
								/* Continue in state VLO_STATE */
								us_new_emit_gain
									= ((caul_max_rms_vlo[uc_last_tx_power] >>
										16) * uc_current_emit_gain) / (tx_result.m_ul_rms_calc >> 16);
								if (us_new_emit_gain > emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max) {
									us_new_emit_gain = emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max;
								} else if (us_new_emit_gain < emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min) {
									us_new_emit_gain = emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min;
								}
							} else if (atpl250.m_ul_last_rmscalc_corrected > caul_th1_vlo[uc_last_tx_power]) {
								/* Go to state HI_STATE */
								atpl250.m_uc_impedance_state = HI_STATE;
								us_new_emit_gain = emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init;
							} else {
								/* Go to state LO_STATE */
								atpl250.m_uc_impedance_state = LO_STATE;
								us_new_emit_gain = emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init;
							}
						} else {
							/* FIXED_STATE_VAR_GAIN. Continue in state VLO_STATE */
							us_new_emit_gain
								= ((caul_max_rms_vlo[uc_last_tx_power] >>
									16) * uc_current_emit_gain) / (tx_result.m_ul_rms_calc >> 16);
							if (us_new_emit_gain > emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max) {
								us_new_emit_gain = emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_max;
							} else if (us_new_emit_gain < emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min) {
								us_new_emit_gain = emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_min;
							}
						}

						break;

					default:
						/* Do nothing */
						break;
					}
				} else {
					us_new_emit_gain = emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init;
					atpl250.m_ul_last_rmscalc_corrected
						= ((emit_gain_limits[atpl250.m_uc_impedance_state].uc_emit_gain_init *
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
		memcpy(tx_param_sniffer.m_auc_tone_map, s_phy_tx_ctl.m_auc_tone_map, TONE_MAP_SIZE);
		tx_param_sniffer.m_uc_2_rs_blocks = s_phy_tx_ctl.e_rs_blocks;
		tx_param_sniffer.e_delimiter_type = s_phy_tx_ctl.e_delimiter_type;
		switch (s_phy_tx_ctl.e_delimiter_type) {
		case DT_SOF_NO_RESP:
		case DT_SOF_RESP:
			tx_param_sniffer.m_puc_data_buf = &s_phy_tx_ctl.auc_tx_buf[FCH_LEN];
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
		for (uc_i = 0; uc_i < TONE_MAP_SIZE; uc_i++) {
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
		rx_msg.m_ul_rx_time = ul_rx_end_time - RRC_DELAY_US;
		rx_msg.m_ul_frame_duration = ul_rx_end_time - ul_rx_sync_time + PREAMBLE_TIME_DURATION;
		rx_msg.m_uc_rs_corrected_errors = s_phy_rx_ctl.m_uc_rs_corrected_errors;

		if ((s_phy_rx_ctl.e_delimiter_type == DT_SOF_NO_RESP) || (s_phy_rx_ctl.e_delimiter_type == DT_SOF_RESP)) {
			rx_msg.m_puc_data_buf = &s_phy_rx_ctl.auc_rx_buf[FCH_LEN];
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
		for (uc_i = 0; uc_i < TONE_MAP_SIZE; uc_i++) {
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
		rx_msg_sniffer.m_ul_rx_time = ul_rx_end_time - RRC_DELAY_US;
		rx_msg_sniffer.m_ul_frame_duration = ul_rx_end_time - ul_rx_sync_time + PREAMBLE_TIME_DURATION;
		rx_msg_sniffer.m_uc_rs_corrected_errors = s_phy_rx_ctl.m_uc_rs_corrected_errors;

		if ((s_phy_rx_ctl.e_delimiter_type == DT_SOF_NO_RESP) || (s_phy_rx_ctl.e_delimiter_type == DT_SOF_RESP)) {
			rx_msg_sniffer.m_puc_data_buf = &s_phy_rx_ctl.auc_rx_buf[FCH_LEN];
		} else {
			rx_msg_sniffer.m_puc_data_buf = &s_phy_rx_ctl.auc_rx_buf[0];
		}

		uc_phy_generic_flags |= PHY_GENERIC_FLAG_SNIFFER_INDICATION;
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
	uint8_t uc_flag_tx_error;
	uint8_t num_intrlv_tx_max;
	uint8_t num_intrlv_tx;
	uint8_t uc_cd_int;

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

		/* Set Tx mode. TXRXBUF_TX_MODE = 1 */
		pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x02);

		generate_inactive_carriers_cenelec_a(s_phy_tx_ctl.m_auc_tone_map[0], s_phy_tx_ctl.m_auc_inactive_carriers_pos);
		generate_inactive_carriers_cenelec_a(s_phy_tx_ctl.m_auc_inv_tone_map[0], s_phy_tx_ctl.m_auc_inv_inactive_carriers_pos);

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

		/* Calculate FCH parameters */
		phy_fch_encode_g3_cenelec_a(&s_phy_tx_ctl, auc_fch);

		/* Copy FCH */
		if (s_phy_tx_ctl.e_rs_blocks == RS_BLOCKS_1_BLOCK) {
			atpl250.m_us_last_tx_msg_payload_symbols = s_phy_tx_ctl.m_us_tx_payload_symbols;
		} else {
			atpl250.m_us_last_tx_msg_payload_symbols = s_phy_tx_ctl.m_us_tx_payload_symbols << 1;
		}

		memcpy(&s_phy_tx_ctl.auc_tx_buf[0], auc_fch, FCH_LEN);
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
		/* Set jumps */
		generate_jumps((uint8_t *)auc_static_notching_pos, s_phy_tx_ctl.m_auc_inactive_carriers_pos);
		/* Get first and last used carriers for payload and PN sequence */
		s_phy_tx_ctl.m_uc_tx_first_carrier = get_first_used_carrier(
				s_phy_tx_ctl.m_auc_static_and_dynamic_notching_pos);
		s_phy_tx_ctl.m_uc_tx_first_carrier_pn_seq = get_first_used_carrier(
				s_phy_tx_ctl.m_auc_static_and_inv_dynamic_notching_pos);

		/* Change value to modulator abcd points (minus 3dB) */
		pplc_if_write_buf(REG_ATPL250_INOUTB_PSK_VALUES1_32, (uint8_t *)cauc_abcd_minus3db, ABCD_POINTS_LEN);

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

			case MOD_TYPE_QAM:
				num_intrlv_tx_max = 5;
				break;

			default:
				num_intrlv_tx_max = 2;
				break;
			}
		}

		if (s_phy_tx_ctl.e_rs_blocks == RS_BLOCKS_2_BLOCKS_TRANSMITTING_SECOND) {
			/* Number of interleavers: 1 for FCH, double in payload */
			num_intrlv_tx_max = (num_intrlv_tx_max << 1) - 1;
		}

		num_intrlv_tx = 0;

		uc_flag_tx_error = 0;
		while (num_intrlv_tx < num_intrlv_tx_max) {
			num_intrlv_tx = pplc_if_read8(REG_ATPL250_INTERLEAVER_INFO2_H8);
			if (++uc_flag_tx_error > 100) {
				break;
			}
		}

		if (++uc_flag_tx_error > 100) {
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

		/* Set interleaver to unload mode, so data can be written to modulator after interleaver is filled with payload */
		switch (s_phy_tx_ctl.e_mod_type) {
		case MOD_TYPE_BPSK_ROBO:
		case MOD_TYPE_BPSK:
			pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x07); /* 07: BPSK unload mode, tx mode, G3 mode, enable HW chain */
			break;

		case MOD_TYPE_QPSK:
			pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x47); /* 47: QPSK unload mode, tx mode, G3 mode, enable HW chain */
			break;

		case MOD_TYPE_8PSK:
			pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x87); /* 87: 8PSK unload mode, tx mode, G3 mode, enable HW chain */
			break;

		case MOD_TYPE_QAM:
			pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0xC7); /* C7: QAM unload mode, tx mode, G3 mode, enable HW chain */
			break;
		}

		if (s_phy_tx_ctl.e_mod_scheme == MOD_SCHEME_COHERENT) {
			if (s_phy_tx_ctl.e_rs_blocks != RS_BLOCKS_2_BLOCKS_TRANSMITTING_SECOND) {
				set_pilot_position(s_phy_tx_ctl.m_uc_num_pilots, s_phy_tx_ctl.m_uc_num_active_carriers,
						s_phy_tx_ctl.m_auc_pilot_pos,
						s_phy_tx_ctl.m_auc_inactive_carriers_pos, (uint8_t *)auc_static_notching_pos);
			}
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
				feed_modulator_payload(&s_phy_tx_ctl, MAX_NUM_SYM_MOD, 0x03, &s_sym_cfg, CHANGE_MODULATION);
			} else {
				/* Send payload symbols */
				feed_modulator_payload(&s_phy_tx_ctl, MAX_NUM_SYM_MOD, 0x00, &s_sym_cfg, CHANGE_MODULATION);
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
						&s_sym_cfg, CHANGE_MODULATION);

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
						(1 << (s_phy_tx_ctl.m_us_tx_pending_symbols - 1)), &s_sym_cfg, CHANGE_MODULATION);
				s_phy_tx_ctl.m_us_tx_pending_symbols = 0;
				s_phy_tx_ctl.e_tx_step = STEP_TX_END;
			}
		}

		uc_set_sym_ready = true;
		break;

	case STEP_TX_PAYLOAD:
		/* Set symbols to transmit */
		feed_modulator_payload(&s_phy_tx_ctl, MAX_NUM_SYM_MOD, 0x00, &s_sym_cfg, NO_CHANGE_MODULATION);
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
			feed_modulator_payload(&s_phy_tx_ctl, s_phy_tx_ctl.m_us_tx_pending_symbols, 0x00, &s_sym_cfg,
					NO_CHANGE_MODULATION);
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
					(1 << (s_phy_tx_ctl.m_us_tx_pending_symbols - 1)), &s_sym_cfg, NO_CHANGE_MODULATION);
			s_phy_tx_ctl.m_us_tx_pending_symbols = 0;
		}

		uc_set_sym_ready = true;
		break;

	case STEP_TX_END:
		/* Prepare for next reception */
		atpl250_hold_on_reference(false);
		atpl250_set_mod_bpsk_truepoint();

		disable_HW_chain();     /* Disable HW chain to write in modulator */
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, LAST_CARRIER); /* Avoid overflow to 1st symbol */
		pplc_if_write_jump((BCODE_COH | (FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol, uc_psymbol_len, JUMP_COL_1);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, CFG_IOB_SAMPLES_PER_SYMBOL + LAST_CARRIER); /* Avoid overflow to 2nd symbol */
		pplc_if_write_jump((BCODE_COH | (CFG_IOB_SAMPLES_PER_SYMBOL + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
				uc_psymbol_len, JUMP_COL_1);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 2) + LAST_CARRIER); /* Avoid overflow to 3rd symbol */
		pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 2) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
				uc_psymbol_len, JUMP_COL_1);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 3) + LAST_CARRIER); /* Avoid overflow to 4th symbol */
		pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 3) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
				uc_psymbol_len, JUMP_COL_1);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 4) + LAST_CARRIER); /* Avoid overflow to 5th symbol */
		pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 4) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
				uc_psymbol_len, JUMP_COL_1);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 5) + LAST_CARRIER); /* Avoid overflow to 6th symbol */
		pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 5) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
				uc_psymbol_len, JUMP_COL_1);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 6) + LAST_CARRIER); /* Avoid overflow to 7th symbol */
		pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 6) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
				uc_psymbol_len, JUMP_COL_1);
		pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, (CFG_IOB_SAMPLES_PER_SYMBOL * 7) + LAST_CARRIER); /* Avoid overflow to 8th symbol */
		pplc_if_write_jump((BCODE_COH | ((CFG_IOB_SAMPLES_PER_SYMBOL * 7) + FIRST_CARRIER + auc_unmasked_carrier_list[0])), (uint8_t *)auc_psymbol,
				uc_psymbol_len, JUMP_COL_1);

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

	switch (s_phy_rx_ctl.e_rx_step) {
	case STEP_RX_SYNCM:
		/* Configure number of symbols for next interrupt */
		s_phy_rx_ctl.m_us_rx_pending_symbols = uc_num_symbols_fch;
		s_phy_rx_ctl.m_uc_next_demod_symbols = uc_num_symbols_fch % uc_num_sym_block_demod_fch;
		if (s_phy_rx_ctl.m_uc_next_demod_symbols == 0) {
			s_phy_rx_ctl.m_uc_next_demod_symbols = uc_num_sym_block_demod_fch;
		}

		/* Set IO buffer to first interrupt after the SYNCM with FFT window displaced */
		atpl250_set_num_symbols_cfg(s_phy_rx_ctl.m_uc_next_demod_symbols);

		/* Estimate channel and go to next step */
		chn_estimation_from_preamble();
		s_phy_rx_ctl.e_rx_step = STEP_RX_PEAK2;

		/* Clear Rx full */
		uc_clear_rx_full = true;
		break;

	case STEP_RX_PEAK2:
		/* Config chain to load data to deinterleaver */
		/* 15: Bypass second scrambler, set 6 repetitions */
		pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x150B);

		/* Enable RSSI calculation */
		atpl250_enable_rssi();
		atpl250_enable_evm();

		/* Get symbols and go to next step */
		/* Avoid using displaced M symbol as reference for first demodulation */
		atpl250_set_reference_dist_sym(2);
		get_demodulator_fch(1, 0, CHANGE_MODULATION, uc_num_sym_block_demod_fch);
		/* Reset reference to previous symbol */
		atpl250_set_reference_dist_sym(1);
		get_demodulator_fch(s_phy_rx_ctl.m_uc_next_demod_symbols - 1, 1, CHANGE_MODULATION, uc_num_sym_block_demod_fch);

		s_phy_rx_ctl.m_us_rx_pending_symbols -= s_phy_rx_ctl.m_uc_next_demod_symbols;
		s_phy_rx_ctl.m_uc_next_demod_symbols = uc_num_sym_block_demod_fch;
		s_phy_rx_ctl.e_rx_step = STEP_RX_HEADER;
#ifdef ENABLE_PYH_PROCESS_RECALL
		/* Force phy process recall when near the end of FCH reception just in case it is an ACK */
		uc_force_phy_process_recall = 1;
#endif

		/* Clear Rx full */
		uc_clear_rx_full = true;
		break;

	case STEP_RX_HEADER:
		/* Get next symbols block */
		get_demodulator_fch(s_phy_rx_ctl.m_uc_next_demod_symbols, 0, CHANGE_MODULATION, uc_num_sym_block_demod_fch);
		s_phy_rx_ctl.m_us_rx_pending_symbols -= s_phy_rx_ctl.m_uc_next_demod_symbols;

		/* Check whether FCH is complete or not */
		if (s_phy_rx_ctl.m_us_rx_pending_symbols) {
			/* There are still symbols to receive, clear Rx Full and keep in same state */
			uc_clear_rx_full = true;
		} else {
			/* FCH Complete. Rx full will be cleared after BER interrupt */

			/* Set unload mode and send pulse to unload deinterleaver */
			pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x13); /* BPSK for header */

			/* Wait until all bits are stored in raw data */
			uc_flag_rx_error = 0;
			us_bits_through_rx_chain = pplc_if_read16(REG_ATPL250_INTERLEAVER_INFO2_L16);
			while (us_bits_through_rx_chain < (FCH_LEN_BITS - 1)) {
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
			pplc_if_read_rep(REG_ATPL250_RAW_DATA_VL8, 1, &s_phy_rx_ctl.auc_rx_buf[0], FCH_LEN);

			/* Decode header */
			uc_header_ok = phy_fch_decode_g3_cenelec_a(&s_phy_rx_ctl, (uint8_t *)auc_static_notching_pos);

			if (s_phy_rx_ctl.e_rs_blocks == RS_BLOCKS_1_BLOCK) {
				atpl250.m_us_last_rx_msg_payload_symbols = s_phy_rx_ctl.m_us_rx_payload_symbols;
			} else {
				atpl250.m_us_last_rx_msg_payload_symbols = s_phy_rx_ctl.m_us_rx_payload_symbols << 1;
			}

			s_phy_rx_ctl.m_ul_pilot_idx = 0;

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
					pplc_if_write32(REG_ATPL250_INTERLEAVER_CFG3_32, FCH_LEN_BITS - 1); /* Set first and last bit */
					pplc_if_or8(REG_ATPL250_BER_PERIPH_CFG1_VH8, 0x80); /* AUTO_BER='1' */
					e_ber_status = BER_FCH;
					pplc_if_write8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x2F);   /* BPSK, Tx mode, Load mode, G3 */
#ifdef ENABLE_PYH_PROCESS_RECALL
					/* Clear phy process recall in case of data frame, it will be set near the end of frame */
					uc_force_phy_process_recall = 0;
#endif

					if (s_phy_rx_ctl.e_mod_scheme == MOD_SCHEME_COHERENT) {
						/* Set interrupt to 2 symbols to receive S1 and S2 */
						atpl250_set_num_symbols_cfg(NUM_SYMBOLS_S1S2);
						s_phy_rx_ctl.e_rx_step = STEP_RX_COH_S1S2;
					} else {
						if (s_phy_rx_ctl.m_us_rx_pending_symbols <= DEFAULT_SYM_NEXT) {
							/* If less or equal than DEFAULT_SYM_NEXT symbols pending, set next demod symbols to pending symbols,
							 * and state to last demod */
							s_phy_rx_ctl.m_uc_next_demod_symbols = (uint8_t)s_phy_rx_ctl.m_us_rx_pending_symbols;
							s_phy_rx_ctl.e_demod_step = STEP_DEMOD_LAST;
#ifdef ENABLE_PYH_PROCESS_RECALL
							uc_force_phy_process_recall = 1;
#endif
						} else {
							/* Otherwise, set first number of symbols to interrupt so there is 3 symbols left at the end for last 2
							 * interrupts, */
							/* and set state to standard demod */
							s_phy_rx_ctl.m_uc_next_demod_symbols
								= ((s_phy_rx_ctl.m_us_rx_pending_symbols %
									DEFAULT_SYM_NEXT) + DEFAULT_SYM_NEXT - 3) % DEFAULT_SYM_NEXT;
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
					ul_rx_end_time = ul_rx_sync_time + (uint32_t)(uc_num_symbols_fch) * FRAME_SYMBOL_DURATION + HALF_SYMBOL_DURATION;

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

		break;

	case STEP_RX_COH_S1S2:
		/* Disable RSSI calculation for channel estimation */
		atpl250_disable_rssi();
		atpl250_disable_evm();

		/* Change value to modulator abcd points */
		pplc_if_write_buf(REG_ATPL250_INOUTB_PSK_VALUES1_32, (uint8_t *)cauc_abcd_fullgain, ABCD_POINTS_LEN);

		/* Disable HW chain */
		pplc_if_and8(REG_ATPL250_INTERLEAVER_CTL_VL8, (uint8_t)(~0x01u));

		/* Perform channel estimation */
		chn_estimation_from_s1s2();

		/* Estimates the SFO */
		sampling_error_est_from_preamble_fch_s1s2(MOD_SCHEME_COHERENT, s_phy_rx_ctl.m_us_rx_payload_symbols);

		/* Compensates the SFO in the estimated channel */
		compensate_sfo_in_chan_est();

		/* Change value to modulator abcd points */
		pplc_if_write_buf(REG_ATPL250_INOUTB_PSK_VALUES1_32, (uint8_t *)cauc_abcd_fullgain, ABCD_POINTS_LEN);

		/* Set number of symbols for next interrupt */
		if (s_phy_rx_ctl.m_us_rx_pending_symbols <= DEFAULT_SYM_NEXT) {
			/* If less or equal than DEFAULT_SYM_NEXT symbols pending, set next demod symbols to pending symbols, and state to last demod */
			s_phy_rx_ctl.m_uc_next_demod_symbols = (uint8_t)s_phy_rx_ctl.m_us_rx_pending_symbols;
			s_phy_rx_ctl.e_demod_step = STEP_DEMOD_LAST;
#ifdef ENABLE_PYH_PROCESS_RECALL
			/* Force phy process recall when near the end of a reception */
			uc_force_phy_process_recall = 1;
#endif
		} else {
			/* Otherwise, set first number of symbols to interrupt so there is 3 symbols left at the end for last 2 interrupts, */
			/* and set state to standard demod */
			s_phy_rx_ctl.m_uc_next_demod_symbols
				= ((s_phy_rx_ctl.m_us_rx_pending_symbols % DEFAULT_SYM_NEXT) + DEFAULT_SYM_NEXT - 3) % DEFAULT_SYM_NEXT;
			if (s_phy_rx_ctl.m_uc_next_demod_symbols == 0) {
				s_phy_rx_ctl.m_uc_next_demod_symbols = DEFAULT_SYM_NEXT;
			}

			s_phy_rx_ctl.e_demod_step = STEP_DEMOD_STD;
		}

		atpl250_set_num_symbols_cfg(s_phy_rx_ctl.m_uc_next_demod_symbols);

		/* Modulation will be changed on first payload demodulation, there is no need to set it here */
		/* Write pilots position */
		set_pilot_position(s_phy_rx_ctl.m_uc_num_pilots, s_phy_rx_ctl.m_uc_num_active_carriers,
				s_phy_rx_ctl.m_auc_pilot_pos, s_phy_rx_ctl.m_auc_inactive_carriers_pos, (uint8_t *)auc_static_notching_pos);
		/* Keep track of pilot position of first symbol */
		memcpy(s_phy_rx_ctl.m_auc_pilot_pos_first_symbol, s_phy_rx_ctl.m_auc_pilot_pos, CARR_BUFFER_LEN);

		ber_config_cenelec_a(&s_phy_rx_ctl);

		/* Enable HW chain */
		pplc_if_or8(REG_ATPL250_INTERLEAVER_CTL_VL8, 0x01);

		/* Enable RSSI calculation from channel estimation */
		atpl250_enable_rssi();
		atpl250_enable_evm();

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

		case MOD_TYPE_QAM:
			pplc_if_write16(REG_ATPL250_INTERLEAVER_CTL_L16, ((uint16_t)(uc_legacy_mode) << 15) | 0x10CB);
			break;
		}

		/* Set jumps */
		generate_jumps((uint8_t *)auc_static_notching_pos, s_phy_rx_ctl.m_auc_inactive_carriers_pos);
		/* Get first and last used carriers for payload and PN sequence */
		s_phy_rx_ctl.m_uc_rx_first_carrier = get_first_used_carrier(s_phy_rx_ctl.m_auc_static_and_dynamic_notching_pos);
		s_phy_rx_ctl.m_uc_rx_first_carrier_pn_seq = get_first_used_carrier(s_phy_rx_ctl.m_auc_static_and_inv_dynamic_notching_pos);

		/* Set rx modulation */
		uc_change_rx_mod = true;
	/* No break, next case has to be executed also for first payload */

	case STEP_RX_PAYLOAD:
		switch (s_phy_rx_ctl.e_demod_step) {
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
						READ_INACTIVE_CARRIERS);
			} else {
				/* Get demodulator, set One before last for next demod and state */
				get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod, PAY_SYM_ONE_BEFORE_LAST_DEMOD,
						READ_INACTIVE_CARRIERS);
				s_phy_rx_ctl.e_demod_step = STEP_DEMOD_ONE_BEFORE_LAST;
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
					NO_READ_INACTIVE_CARRIERS);
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
							s_phy_rx_ctl.m_us_rx_payload_symbols, NO_READ_INACTIVE_CARRIERS);
					s_phy_rx_ctl.e_demod_step = STEP_DEMOD_LAST;
#ifdef ENABLE_PYH_PROCESS_RECALL
					/* Force phy process recall when near the end of a reception */
					uc_force_phy_process_recall = 1;
#endif
				} else if (s_phy_rx_ctl.m_us_rx_payload_symbols <= (DEFAULT_SYM_NEXT << 1)) {
					/* If number of symbols between Dafault and 2*Default, set default symbols and state to one before last */
					get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod, DEFAULT_SYM_NEXT,
							NO_READ_INACTIVE_CARRIERS);
					s_phy_rx_ctl.e_demod_step = STEP_DEMOD_ONE_BEFORE_LAST;
				} else {
					/* Otherwise, set first number of symbols to interrupt so there is 3 symbols left at the end for last 2 interrupts, */
					/* and set state to standard demod */
					if (((s_phy_rx_ctl.m_us_rx_payload_symbols % DEFAULT_SYM_NEXT) + DEFAULT_SYM_NEXT - 3) % DEFAULT_SYM_NEXT == 0) {
						get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod, DEFAULT_SYM_NEXT,
								NO_READ_INACTIVE_CARRIERS);
					} else {
						get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod,
								((s_phy_rx_ctl.m_us_rx_payload_symbols % DEFAULT_SYM_NEXT) + DEFAULT_SYM_NEXT - 3) % DEFAULT_SYM_NEXT,
								NO_READ_INACTIVE_CARRIERS);
					}

					s_phy_rx_ctl.e_demod_step = STEP_DEMOD_STD;
				}
			} else {
				/* Get Rx end time */
				ul_rx_end_time = ul_rx_sync_time
						+ (uint32_t)(s_phy_rx_ctl.m_us_rx_payload_symbols +
						uc_num_symbols_fch) * FRAME_SYMBOL_DURATION + HALF_SYMBOL_DURATION;

				if (s_phy_rx_ctl.e_mod_scheme == MOD_SCHEME_COHERENT) {
					ul_rx_end_time += NUM_SYMBOLS_S1S2 * FRAME_SYMBOL_DURATION;
				}

				/* Reset TXRX buf and FFT */
				end_rx_fft_txrx();

				/* Last part of payload */
				get_demodulator_payload(&s_phy_rx_ctl, s_phy_rx_ctl.m_uc_next_demod_symbols, uc_change_rx_mod, DEFAULT_SYM_NEXT,
						NO_READ_INACTIVE_CARRIERS);
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

			case MOD_TYPE_QAM:
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

				case MOD_TYPE_QAM:
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
					pplc_if_read_rep(REG_ATPL250_RAW_DATA_VL8, 1, &s_phy_rx_ctl.auc_rx_buf[FCH_LEN + us_bytes_to_read], us_bytes_to_read);
					us_rs_decode_idx = FCH_LEN + us_bytes_to_read;
				} else {
					/* Read raw data from bit 0 */
					pplc_if_write16(REG_ATPL250_RAW_DATA_H16, 0);
					/* Read Payload + RS + Syndrome */
					pplc_if_read_rep(REG_ATPL250_RAW_DATA_VL8, 1, &s_phy_rx_ctl.auc_rx_buf[FCH_LEN], us_bytes_to_read);
					us_rs_decode_idx = FCH_LEN;
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

		pplc_if_read_buf(BCODE_ZONE4 | 0x0000, &auc_dump_signal[uc_dumped_buffers][0], 256);
		pplc_if_read_buf(BCODE_ZONE4 | 0x0080, &auc_dump_signal[uc_dumped_buffers][256], 256);
		pplc_if_read_buf(BCODE_ZONE4 | 0x0100, &auc_dump_signal[uc_dumped_buffers][512], 256);
		pplc_if_read_buf(BCODE_ZONE4 | 0x0180, &auc_dump_signal[uc_dumped_buffers][768], 256);
		pplc_if_read_buf(BCODE_ZONE4 | 0x0200, &auc_dump_signal[uc_dumped_buffers][1024], 256);
		pplc_if_read_buf(BCODE_ZONE4 | 0x0280, &auc_dump_signal[uc_dumped_buffers][1280], 256);
		pplc_if_read_buf(BCODE_ZONE4 | 0x0300, &auc_dump_signal[uc_dumped_buffers][1536], 256);
		pplc_if_read_buf(BCODE_ZONE4 | 0x0380, &auc_dump_signal[uc_dumped_buffers][1792], 256);

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
			pplc_if_set_high_speed();
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
	} else if ((s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_FIRST_PASS) || (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_SECOND_PASS)) {
		/* Handle Noise Capture */
		_analyse_noise_capture();
	} else if (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_ABORTED) {
		/* Clear Rx full */
		uc_clear_rx_full = true;
		LOG_PHY(("Noise Capture aborted\r\n"));
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
	pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, RX_FFT_SHIFT);

	/* Set IO buffer to first interrupt after PEAK2 (the SYNCM with FFT window displaced) */
	atpl250_set_num_symbols_cfg(1);

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

	s_phy_rx_ctl.e_rx_step = STEP_RX_SYNCM;

	_zcd_calc_pdc_rx();

	/* Get sync time */
	ul_rx_sync_time = pplc_if_read32(REG_ATPL250_PEAK2_TIME_32);

	/* Configure Rotator */
	pplc_if_write32(REG_ATPL250_ROTATOR_CONFIG0_32, 0x0008A3AE);
	pplc_if_write32(REG_ATPL250_ROTATOR_CONFIG3_32, 0x0502007F);
	pplc_if_write8(REG_ATPL250_ROTATOR_CONFIG3_H8, 0x01);

	/* Channel estimation from preamble */
	chn_estimation_from_syncp();

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

	atpl250_sync_release_rst(); /* Just in case it was pushed (ACK) */

	enable_CD_interrupt();
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
		pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, TX_FFT_SHIFT);
		/* Start transmitting preamble */
		_tx_preamble_first();
	#else
		generate_inactive_carriers_cenelec_a(s_phy_tx_ctl.m_auc_tone_map[0],
				s_phy_tx_ctl.m_auc_inactive_carriers_pos);
		generate_inactive_carriers_cenelec_a(s_phy_tx_ctl.m_auc_inv_tone_map[0],
				s_phy_tx_ctl.m_auc_inv_inactive_carriers_pos);

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

		/* Calculate FCH parameters */
		phy_fch_encode_g3_cenelec_a(&s_phy_tx_ctl, auc_fch);

		/* Copy FCH */
		if (s_phy_tx_ctl.e_rs_blocks == RS_BLOCKS_1_BLOCK) {
			atpl250.m_us_last_tx_msg_payload_symbols = s_phy_tx_ctl.m_us_tx_payload_symbols;
		} else {
			atpl250.m_us_last_tx_msg_payload_symbols = s_phy_tx_ctl.m_us_tx_payload_symbols << 1;
		}

		memcpy(&s_phy_tx_ctl.auc_tx_buf[0], auc_fch, FCH_LEN);
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
		pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, TX_FFT_SHIFT);
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
	ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32) - CFG_TXRX_PLC_OFF1_US - CFG_END_OF_TX_OFFSET_US;
	disable_CD_interrupt();

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
			(s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_SECOND_PASS) ||
			(pplc_if_read32(REG_ATPL250_INT_FLAGS_32) & INT_NOISE_CAPTURE_MASK_32)) {
		LOG_PHY(("RxErr NC\r\n"));
		/* Look for chirp */
		pplc_if_or8(REG_ATPL250_TXRXB_STATE_VL8, 0x01);
		s_phy_rx_ctl.e_rx_step = STEP_RX_NOISE_CAPTURE_ABORTED;
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
			(s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_SECOND_PASS) || (s_phy_rx_ctl.e_rx_step == STEP_RX_NOISE_CAPTURE_ABORTED)) {
		/* Set back FFT_SHIFT */
		pplc_if_write8(REG_ATPL250_FFT_CONFIG_L8, RX_FFT_SHIFT);

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
		atpl250_set_num_symbols_cfg(NOISE_CAPTURE_ADAPT_SYMBOLS);
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
	uint16_t us_payload_symbols;
	int8_t sc_snr_background_db, sc_snr_impulsive_db;
	uint16_t us_total_symbols, us_noised_symbols;
	uint32_t ul_min_noised_symbols, ul_max_noised_symbols;

	us_payload_symbols = s_phy_rx_ctl.m_us_rx_payload_symbols;

	if (e_ber_status == BER_FCH) {
		ber_save_fch_info(&s_rx_ber_fch_data);

		if (s_phy_rx_ctl.e_mod_scheme == MOD_SCHEME_DIFFERENTIAL) {
			chn_estimation_from_fch(MOD_SCHEME_DIFFERENTIAL, NUM_SYM_H_EST_FCH);
			sampling_error_est_from_preamble_fch_s1s2(MOD_SCHEME_DIFFERENTIAL, s_phy_rx_ctl.m_us_rx_payload_symbols);

			/* Clean reference to demodulate payload in differential */
			pplc_if_write8(REG_ATPL250_INOUTB_CONF2_H8, 0x00); /* H_1H_BYPASS='0'; H_1H='0'; DEST_Y='0'; SOURCE_H='0' */
		} else {
#if (NUM_SYM_H_EST_FCH_COH > 0)
			chn_estimation_from_fch(MOD_SCHEME_COHERENT, NUM_SYM_H_EST_FCH_COH);
#else
			uc_num_sym_valid_fch = 0;
#endif
		}

		ber_config_cenelec_a(&s_phy_rx_ctl);

		/* Clear Rx full to start receiving Payload */
		uc_clear_rx_full = true;
	} else if (e_ber_status == BER_PAYLOAD) {
		/* Interrupt after rx payload. */
		ber_save_payload_info(&s_rx_ber_payload_data);
		s_rx_ber_payload_data.uc_lqi = get_lqi_and_per_carrier_snr((uint8_t)s_phy_rx_ctl.e_mod_scheme,
				s_phy_rx_ctl.m_auc_static_and_dynamic_notching_pos,
				s_phy_rx_ctl.m_auc_pilot_pos_first_symbol, s_rx_ber_payload_data.auc_carrier_snr,
				us_payload_symbols);

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
	uint32_t ul_curr_time, ul_txrx_pad_time, ul_start_tx_time;
	uint16_t aus_tx_time_reg[4] = {REG_ATPL250_TX_TIME1_32, REG_ATPL250_TX_TIME2_32, REG_ATPL250_TX_TIME3_32, REG_ATPL250_TX_TIME4_32};
	uint8_t uc_forced =  s_phy_tx_ctl.m_uc_tx_mode  & TX_MODE_FORCED_TX;
	uint8_t uc_correct_interval = 0;
	ul_curr_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
	ul_start_tx_time = pplc_if_read32(aus_tx_time_reg[0]);
	ul_txrx_pad_time = ul_start_tx_time + CFG_TXRX_TIME1_US;

	if (ul_txrx_pad_time < (CFG_TXRX_TIME1_US)) {
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
 * \param uc_band  Selects PHY layer Working Band (not used in this PHY layer implementation)
 *
 */
void phy_init(uint8_t uc_serial_enable, uint8_t uc_band)
{
	uint32_t ul_timeout;

	UNUSED(uc_band);
	uc_working_band = WB_CENELEC_A;

	/* Set handler */
	pplc_set_handler(phy_interrupt);

	/* Clear flag to detect start interrupt execution */
	uc_start_interrupt_executed = 0;

	/* Initialize PPLC driver */
	pplc_if_init();

	/* Initialize time counters */
	ul_ms_counter = oss_get_up_time_ms();
	ul_noise_capture_timer = ul_ms_counter + atpl250.m_ul_time_between_noise_captures;

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
	atpl250.m_ul_version = ATPL250_VERSION_NUM;

#ifdef ENABLE_PYH_PROCESS_RECALL
	uc_force_phy_process_recall = 0;
#endif

	/* Init PHY serial interface */
	if (uc_serial_enable) {
		serial_if_init();
	}

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
			pplc_if_read_buf(us_id, p_val, us_len);
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
					s_phy_rx_ctl.m_uc_payload_carriers, atpl250.m_uc_rrc_notch_index);
			s_tone_map_response_data.e_mod_scheme = MOD_SCHEME_DIFFERENTIAL;
			puc_mem = (uint8_t *)&s_tone_map_response_data + (us_id & ~ATPL250_TM_RESP_PARAM_MSK);
		} else {
			puc_mem = (uint8_t *)&s_rx_ber_payload_data + (us_id & ~ATPL250_BER_PARAM_MSK);
			if (s_rx_ber_payload_data.uc_valid_data == 1) {
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
				atpl250_update_branch_cfg(atpl250.m_uc_impedance_state, EMIT_GAIN_HI);
				break;

			case LO_STATE:
				atpl250_update_branch_cfg(atpl250.m_uc_impedance_state, EMIT_GAIN_LO);
				break;

			case VLO_STATE:
				atpl250_update_branch_cfg(atpl250.m_uc_impedance_state, EMIT_GAIN_VLO);
				break;

			default:
				atpl250_update_branch_cfg(atpl250.m_uc_impedance_state, EMIT_GAIN_HI);
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
			_trigger_noise_capture();
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
				/* only real part */
				atpl250_set_iob_real_mode();
				/* Samples transmitted from IOB; samples received to IOB; Frames divided in symbols */
				pplc_if_write8(REG_ATPL250_TXRXB_CTL_H8, 0x03);
				/* CP_EXCLUSIVO=0 */
				pplc_if_write8(REG_ATPL250_TXRXB_CTL_VL8, 0x00);
				/* cyclic_prefix1=0; overlap1=0; cyclic_prefix2=0; overlap2=0 */
				pplc_if_write32(REG_ATPL250_TXRXB_SYM_CFG_32, 0x00000000);
				/* Discard 0 samples after PEAK1. Discard 0 samples after PEAK2 (0.5*SYNCM+0.5*(cyclic_prefix2-overlap2) */
				pplc_if_write32(REG_ATPL250_TXRXB_OFFSETS_32, 0x00000000);
				if (atpl250.m_uc_trigger_signal_dump == DUMP_ON_RECEPTION) {
					/* 4 prev symbols; Division by 4; average 4 symbols, enable average and send previuos symbols */
					pplc_if_write32(REG_ATPL250_TXRXB_PRE_ANALYSIS_32, 0x03010302);
				} else {
					/* 4 prev symbols; Division by 4; average 4 symbols, enable average and send previuos symbols */
					pplc_if_write32(REG_ATPL250_TXRXB_PRE_ANALYSIS_32, 0x00010000);
				}

				/* Disable SYNCM DETECTOR */
				pplc_if_and8(REG_ATPL250_SYNCM_CTL_L8, 0x7F);
				pplc_if_write16(REG_ATPL250_INOUTB_OVERFLOW_H16, 0xFFFF);
				atpl250_iob_release_rst();
				atpl250_txrxb_rotator_and_fft_release_rst();

				pplc_if_set_dump_speed();
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
	uint8_t uc_bufid;
	uint8_t uc_is_ack;
	uint8_t uc_i;
	uint32_t ul_curr_time;

	/* Check whether PLC is disabled */
	if (atpl250.m_uc_plc_disable) {
		ul_tx_end_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
		uc_phy_generic_flags |= PHY_GENERIC_FLAG_END_TX;
		return PHY_TX_RESULT_PROCESS;
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

	uc_last_tx_power = px_msg->m_uc_tx_power;

	/* Update local tx data */
	uc_bufid = 0; /* Forced to 0 */

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

	/* Check whether emit gain has to change */
	LOG_PHY(("TxPower %u\r\n", px_msg->m_uc_tx_power));
	if ((atpl250.m_uc_auto_detect_impedance == AUTO_STATE_VAR_GAIN) && (px_msg->m_uc_tx_power >= NUM_TX_LEVELS)) {
		atpl250_update_branch_cfg(1, EMIT_GAIN_LO);
		atpl250.m_uc_impedance_state = LO_STATE;
		LOG_PHY(("LOWTxPower ImpState: LO EmitGain: 0x%02X\r\n", EMIT_GAIN_LO));
	}

	/* Set preemphasis */
	_set_preemphasis(&px_msg->m_auc_preemphasis[0]);

	for (uc_i = 0; uc_i < TONE_MAP_SIZE; uc_i++) {
		s_phy_tx_ctl.m_auc_tone_map[uc_i] = px_msg->m_auc_tone_map[uc_i];
		s_phy_tx_ctl.m_auc_inv_tone_map[uc_i] = (uint8_t)(~(unsigned int)(px_msg->m_auc_tone_map[uc_i]));
	}

	if (px_msg->e_mod_type == MOD_TYPE_BPSK_ROBO) {
		s_phy_tx_ctl.m_uc_rs_parity = 0x07;
	} else {
		s_phy_tx_ctl.m_uc_rs_parity = 0x0F;
	}

	if (px_msg->m_us_data_len > (PHY_MAX_PAYLOAD_WITH_RS_SIZE - s_phy_tx_ctl.m_uc_rs_parity)) {
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

	s_phy_tx_ctl.e_mod_type = px_msg->e_mod_type;
	s_phy_tx_ctl.e_mod_scheme = px_msg->e_mod_scheme;
	s_phy_tx_ctl.e_delimiter_type = px_msg->e_delimiter_type;
	s_phy_tx_ctl.m_uc_pdc = px_msg->m_uc_pdc;
	s_phy_tx_ctl.m_uc_tx_mode = px_msg->m_uc_tx_mode;

	uc_is_ack = (px_msg->e_delimiter_type == DT_ACK) || (px_msg->e_delimiter_type == DT_NACK);

	if (uc_is_ack) {
		/* In ack, FCH is filled with data coming from upper layer */
		memcpy(auc_fch, px_msg->m_puc_data_buf, FCH_LEN);
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

	if (!uc_is_ack) {
		/* Copy payload */
		memcpy(&s_phy_tx_ctl.auc_tx_buf[FCH_LEN], px_msg->m_puc_data_buf, px_msg->m_us_data_len);
	}

	#ifdef TX_PREAMBLE_HW
	/* Emit preamble from HW */
	/*	atpl250_txrxb_push_rst(); */
	switch (uc_bufid) {
	case 0:
		pplc_if_or8(REG_ATPL250_TX_CONF_VH8, 0x10);         /* EMIT_PREAMBLE0 = 1 */
		break;

	case 1:
		pplc_if_or8(REG_ATPL250_TX_CONF_VH8, 0x20);         /* EMIT_PREAMBLE1 = 1 */
		break;

	case 2:
		pplc_if_or8(REG_ATPL250_TX_CONF_VH8, 0x40);         /* EMIT_PREAMBLE2 = 1 */
		break;

	case 3:
		pplc_if_or8(REG_ATPL250_TX_CONF_VH8, 0x80);         /* EMIT_PREAMBLE3 = 1 */
		break;
	}
	/*	atpl250_txrxb_release_rst(); */
	#else
	/* Emit preamble from SW */
	/*	atpl250_txrxb_push_rst(); */
	switch (uc_bufid) {
	case 0:
		pplc_if_and8(REG_ATPL250_TX_CONF_VH8, 0xEF);         /* EMIT_PREAMBLE0 = 0 */
		break;

	case 1:
		pplc_if_and8(REG_ATPL250_TX_CONF_VH8, 0xDF);         /* EMIT_PREAMBLE1 = 0 */
		break;

	case 2:
		pplc_if_and8(REG_ATPL250_TX_CONF_VH8, 0xBF);         /* EMIT_PREAMBLE2 = 0 */
		break;

	case 3:
		pplc_if_and8(REG_ATPL250_TX_CONF_VH8, 0x7F);         /* EMIT_PREAMBLE3 = 0 */
		break;
	}
	/*	atpl250_txrxb_release_rst(); */
	#endif

	/* Check time on delayed transmissions */
	if ((px_msg->m_uc_tx_mode) & TX_MODE_DELAYED_TX) {
		ul_curr_time = pplc_if_read32(REG_ATPL250_TX_TIMER_REF_32);
		if (ul_curr_time > px_msg->m_ul_tx_time) {
			if ((ul_curr_time > 0xFFC00000) && (px_msg->m_ul_tx_time < 0x400000)) {
				/* Current time near overflow and prog time past overflow. 4 seconds margin */
				/* This is not a timeout, continue */
			} else {
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
		} else if ((px_msg->m_ul_tx_time - ul_curr_time) < DELAYED_TX_SAFETY_TIME) {
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
	_config_tx(uc_bufid, (px_msg->m_uc_tx_mode) & TX_MODE_FORCED_TX, (px_msg->m_uc_tx_mode) & TX_MODE_DELAYED_TX, px_msg->m_ul_tx_time);

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
					memmove(&s_phy_rx_ctl.auc_rx_buf[FCH_LEN + s_phy_rx_ctl.m_us_rx_len],
							&s_phy_rx_ctl.auc_rx_buf[FCH_LEN + s_phy_rx_ctl.m_us_rx_len + ((s_phy_rx_ctl.m_uc_rs_parity + 1) * 2)],
							s_phy_rx_ctl.m_us_rx_len);
					/* Rx length is double than stored in each block */
					s_phy_rx_ctl.m_us_rx_len <<= 1;
				}

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

		/* Clear flag */
		uc_phy_generic_flags &= ~PHY_GENERIC_FLAG_CHECK_RS;

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
