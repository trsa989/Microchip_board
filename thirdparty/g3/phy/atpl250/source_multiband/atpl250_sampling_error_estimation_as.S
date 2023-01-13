/**
 * \brief Assembler file with functions for SFO estimation
 *
 */
 #include "atpl250_channel_and_sfo_estimation_params.h"

 #if defined(__GNUC__)
 .syntax unified
 .text
 .eabi_attribute Tag_ABI_align8_preserved,1  /* Equivalent to PRESERVE8 used in KEIL and RealView */
#elif defined(__IAR_SYSTEMS_ASM__)
 SECTION atpl250_sampling_error_estimation_as:CODE:NOROOT(8)
 PRESERVE8
 THUMB
#elif defined(__CC_ARM)
 PRESERVE8
 AREA phy_plc_tx_chain_g3_modulator, CODE, READONLY
#else
	#error "Unsupported assembler!"
#endif


#if defined(__GNUC__)
 #define ASM_TAG(tag) tag:
#elif defined(__IAR_SYSTEMS_ASM__)
 #define ASM_TAG(tag) tag
#elif defined(__CC_ARM)
 #define ASM_TAG(tag) tag
#else
	#error "Unsupported assembler!"
#endif

/**
 * \brief Compute partial terms of the numerator of denominator of the SFO estimation from preamble, FCH and S1S2
 *		  Prototype: void compute_num_dem_sfo_asm(q15_t *pss_input_symbol);
 */
#if defined(__GNUC__)
 .global compute_partial_num_dem_sfo_cen_a_asm
 .type compute_partial_num_dem_sfo_cen_a_asm, %function /*Define compute_partial_num_dem_sfo_cen_a_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC compute_partial_num_dem_sfo_cen_a_asm
 EXTERN uc_used_carriers
 EXTERN asl_freq_index_squared
 EXTERN pss_average_symbol
 EXTERN sl_partial_den
 EXTERN asl_freq_index
 EXTERN sl_partial_num
#elif defined(__CC_ARM)
 EXPORT compute_partial_num_dem_sfo_cen_a_asm
 IMPORT uc_used_carriers
 IMPORT asl_freq_index_squared
 IMPORT pss_average_symbol
 IMPORT sl_partial_den
 IMPORT asl_freq_index
 IMPORT sl_partial_num
#endif

ASM_TAG(compute_partial_num_dem_sfo_cen_a_asm) /* Entry point for function */
	PUSH {R4, R5, R6, R7, R8, R9, R10, R11, R12}
	LDR R1, =uc_used_carriers
	LDRB R1, [R1]
	MOV R2, R0 /* Copy the actual symbol address. Will be used in the dot product of the Im */
	LDR R3, =asl_freq_index_squared
	LDR R4, =pss_average_symbol	/* Complex conjugate of the average symbol */
	LDR R4, [R4]
	MOV R11, #0 /* R12, R11 are the 64 bits accumulator for the dot products */
	MOV R12, #0

ASM_TAG(loop_Re_compute_partial_num_dem_sfo_cen_a_asm)
	LDR R5, [R0], #4 /* Reads Im, Re of the symbol */
	LDR R6, [R4], #4 /* Reads Im, Re of the *(average symbol) */
	SMULBB R7, R5, R6
	ASR R7, R7, #1 /* Re (symbol) x Re (average symbol) in Q3.29 */
	SMULTT R8, R5, R6
	RSB R8, R8, #0 /* -[Im (symbol) x Im (average symbol)] in Q2.30 */
	ADD R8, R7, R8, ASR #1 /* Re (equalized symbol) = Re (symbol) x Re (average symbol) -[Im (symbol) x Im (average symbol)] in Q3.29 */
	LDR R7, [R3], #4
	SMULL R9, R10, R8, R7 /* Re (equalized symbol)*asl_freq_index_squared in Q4.60 */
	LSR R9, R9, #Q4_60_TO_Q16_48    /* Q4.60->Q16.48. NO ROUNDING APPLIED */
	BFI R9, R10, #(32-Q4_60_TO_Q16_48), #Q4_60_TO_Q16_48 /* Q4.60->Q16.48 */
	ADDS R11, R11, R9 /* Accumulates to previous dot product partial value */
	ADC R12, R12, R10, ASR #Q4_60_TO_Q16_48
	SUBS R1, #1
        BNE loop_Re_compute_partial_num_dem_sfo_cen_a_asm
	LSR R11, R11, #(Q16_48_TO_Q1_31 + SCALING_TIME_DIMENSION + SCALING_FREQ_DIMENSION_CENELEC_A + 2) /* +2 is to get the same result as in with the arm code, to avoid modifying the rest of the C code */
	BFI R11, R12, #(32 - Q16_48_TO_Q1_31 - SCALING_TIME_DIMENSION - SCALING_FREQ_DIMENSION_CENELEC_A - 2), #(Q16_48_TO_Q1_31 + SCALING_TIME_DIMENSION + SCALING_FREQ_DIMENSION_CENELEC_A + 2)
	LDR R4, =sl_partial_den
	STR R11, [R4]

	LDR R1, =uc_used_carriers
	LDRB R1, [R1]
	MOV R0, R2 /* Actual symbol address in R0 */
	LDR R3, =asl_freq_index
	LDR R4, =pss_average_symbol
	LDR R4, [R4]
	MOV R11, #0 /* R12, R11 are the 64 bits accumulator for the dot products */
	MOV R12, #0
ASM_TAG(loop_Im_compute_partial_num_dem_sfo_cen_a_asm)
	LDR R5, [R0], #4 /* Reads Im, Re of the symbol */
	LDR R6, [R4], #4 /* Reads Im, Re of the *(average symbol) */
	SMULTB R7, R5, R6
	ASR R7, R7, #1 /* Im (symbol) x Re (average symbol) in Q3.29 */
	SMULBT R8, R5, R6 /* Re (symbol) x Im (average symbol) in Q2.30 */
	ADD R8, R7, R8, ASR #1 /* Im (equalized symbol) = Im (symbol) x Re (average symbol) + Re (symbol) x Im (average symbol) in Q3.29 */
	LDR R7, [R3], #4
	SMULL R9, R10, R8, R7 /* Im (equalized symbol)*asl_freq_index in Q4.60 */
	LSR R9, R9, #Q4_60_TO_Q16_48    /* Q4.60->Q16.48. NO ROUNDING APPLIED */
	BFI R9, R10, #(32-Q4_60_TO_Q16_48), #Q4_60_TO_Q16_48 /* Q4.60->Q16.48 */
	ADDS R11, R11, R9 /* Accumulates to previous dot product partial value */
	ADC R12, R12, R10, ASR #Q4_60_TO_Q16_48
	SUBS R1, #1
        BNE loop_Im_compute_partial_num_dem_sfo_cen_a_asm
	LSR R11, R11, #(Q16_48_TO_Q1_31+SCALING_TIME_DIMENSION+SCALING_FREQ_DIMENSION_CENELEC_A + 2)   /* +2 is to get the same result as in with the arm code, to avoid modifying the rest of the C code */
	BFI R11, R12, #(32-Q16_48_TO_Q1_31-SCALING_TIME_DIMENSION-SCALING_FREQ_DIMENSION_CENELEC_A - 2), #(Q16_48_TO_Q1_31+SCALING_TIME_DIMENSION+SCALING_FREQ_DIMENSION_CENELEC_A + 2)
	LDR R4, =sl_partial_num
	STR R11, [R4]

	POP {R4, R5, R6, R7, R8, R9, R10, R11, R12}
	BX LR
#if defined(__IAR_SYSTEMS_ASM__) || defined(__CC_ARM)
        LTORG
#endif

/**
 * \brief Compute partial terms of the numerator of denominator of the SFO estimation from preamble, FCH and S1S2
 *		  Prototype: void compute_num_dem_sfo_asm(q15_t *pss_input_symbol);
 */
#if defined(__GNUC__)
 .global compute_partial_num_dem_sfo_fcc_asm
 .type compute_partial_num_dem_sfo_fcc_asm, %function /*Define compute_partial_num_dem_sfo_fcc_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC compute_partial_num_dem_sfo_fcc_asm
 EXTERN uc_used_carriers
 EXTERN asl_freq_index_squared
 EXTERN pss_average_symbol
 EXTERN sl_partial_den
 EXTERN asl_freq_index
 EXTERN sl_partial_num
#elif defined(__CC_ARM)
 EXPORT compute_partial_num_dem_sfo_fcc_asm
 IMPORT uc_used_carriers
 IMPORT asl_freq_index_squared
 IMPORT pss_average_symbol
 IMPORT sl_partial_den
 IMPORT asl_freq_index
 IMPORT sl_partial_num
#endif

ASM_TAG(compute_partial_num_dem_sfo_fcc_asm) /* Entry point for function */
	PUSH {R4, R5, R6, R7, R8, R9, R10, R11, R12}
	LDR R1, =uc_used_carriers
	LDRB R1, [R1]
	MOV R2, R0 /* Copy the actual symbol address. Will be used in the dot product of the Im */
	LDR R3, =asl_freq_index_squared
	LDR R4, =pss_average_symbol	/* Complex conjugate of the average symbol */
	LDR R4, [R4]
	MOV R11, #0 /* R12, R11 are the 64 bits accumulator for the dot products */
	MOV R12, #0

ASM_TAG(loop_Re_compute_partial_num_dem_sfo_fcc_asm)
	LDR R5, [R0], #4 /* Reads Im, Re of the symbol */
	LDR R6, [R4], #4 /* Reads Im, Re of the *(average symbol) */
	SMULBB R7, R5, R6
	ASR R7, R7, #1 /* Re (symbol) x Re (average symbol) in Q3.29 */
	SMULTT R8, R5, R6
	RSB R8, R8, #0 /* -[Im (symbol) x Im (average symbol)] in Q2.30 */
	ADD R8, R7, R8, ASR #1 /* Re (equalized symbol) = Re (symbol) x Re (average symbol) -[Im (symbol) x Im (average symbol)] in Q3.29 */
	LDR R7, [R3], #4
	SMULL R9, R10, R8, R7 /* Re (equalized symbol)*asl_freq_index_squared in Q4.60 */
	LSR R9, R9, #Q4_60_TO_Q16_48    /* Q4.60->Q16.48. NO ROUNDING APPLIED */
	BFI R9, R10, #(32-Q4_60_TO_Q16_48), #Q4_60_TO_Q16_48 /* Q4.60->Q16.48 */
	ADDS R11, R11, R9 /* Accumulates to previous dot product partial value */
	ADC R12, R12, R10, ASR #Q4_60_TO_Q16_48
	SUBS R1, #1
        BNE loop_Re_compute_partial_num_dem_sfo_fcc_asm
	LSR R11, R11, #(Q16_48_TO_Q1_31 + SCALING_TIME_DIMENSION + SCALING_FREQ_DIMENSION_FCC + 2) /* +2 is to get the same result as in with the arm code, to avoid modifying the rest of the C code */
	BFI R11, R12, #(32 - Q16_48_TO_Q1_31 - SCALING_TIME_DIMENSION - SCALING_FREQ_DIMENSION_FCC - 2), #(Q16_48_TO_Q1_31 + SCALING_TIME_DIMENSION + SCALING_FREQ_DIMENSION_FCC + 2)
	LDR R4, =sl_partial_den
	STR R11, [R4]

	LDR R1, =uc_used_carriers
	LDRB R1, [R1]
	MOV R0, R2 /* Actual symbol address in R0 */
	LDR R3, =asl_freq_index
	LDR R4, =pss_average_symbol
	LDR R4, [R4]
	MOV R11, #0 /* R12, R11 are the 64 bits accumulator for the dot products */
	MOV R12, #0
ASM_TAG(loop_Im_compute_partial_num_dem_sfo_fcc_asm)
	LDR R5, [R0], #4 /* Reads Im, Re of the symbol */
	LDR R6, [R4], #4 /* Reads Im, Re of the *(average symbol) */
	SMULTB R7, R5, R6
	ASR R7, R7, #1 /* Im (symbol) x Re (average symbol) in Q3.29 */
	SMULBT R8, R5, R6 /* Re (symbol) x Im (average symbol) in Q2.30 */
	ADD R8, R7, R8, ASR #1 /* Im (equalized symbol) = Im (symbol) x Re (average symbol) + Re (symbol) x Im (average symbol) in Q3.29 */
	LDR R7, [R3], #4
	SMULL R9, R10, R8, R7 /* Im (equalized symbol)*asl_freq_index in Q4.60 */
	LSR R9, R9, #Q4_60_TO_Q16_48    /* Q4.60->Q16.48. NO ROUNDING APPLIED */
	BFI R9, R10, #(32-Q4_60_TO_Q16_48), #Q4_60_TO_Q16_48 /* Q4.60->Q16.48 */
	ADDS R11, R11, R9 /* Accumulates to previous dot product partial value */
	ADC R12, R12, R10, ASR #Q4_60_TO_Q16_48
	SUBS R1, #1
        BNE loop_Im_compute_partial_num_dem_sfo_fcc_asm
	LSR R11, R11, #(Q16_48_TO_Q1_31+SCALING_TIME_DIMENSION+SCALING_FREQ_DIMENSION_FCC + 2)   /* +2 is to get the same result as in with the arm code, to avoid modifying the rest of the C code */
	BFI R11, R12, #(32-Q16_48_TO_Q1_31-SCALING_TIME_DIMENSION-SCALING_FREQ_DIMENSION_FCC - 2), #(Q16_48_TO_Q1_31+SCALING_TIME_DIMENSION+SCALING_FREQ_DIMENSION_FCC + 2)
	LDR R4, =sl_partial_num
	STR R11, [R4]

	POP {R4, R5, R6, R7, R8, R9, R10, R11, R12}
	BX LR
#if defined(__IAR_SYSTEMS_ASM__) || defined(__CC_ARM)
        LTORG
#endif


/**
 * \brief Compute partial terms of the numerator of denominator of the SFO estimation from preamble, FCH and S1S2
 *		  Prototype: void compute_num_dem_sfo_asm(q15_t *pss_input_symbol);
 */
#if defined(__GNUC__)
 .global compute_partial_num_dem_sfo_arib_asm
 .type compute_partial_num_dem_sfo_arib_asm, %function /*Define compute_partial_num_dem_sfo_arib_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC compute_partial_num_dem_sfo_arib_asm
 EXTERN uc_used_carriers
 EXTERN asl_freq_index_squared
 EXTERN pss_average_symbol
 EXTERN sl_partial_den
 EXTERN asl_freq_index
 EXTERN sl_partial_num
#elif defined(__CC_ARM)
 EXPORT compute_partial_num_dem_sfo_arib_asm
 IMPORT uc_used_carriers
 IMPORT asl_freq_index_squared
 IMPORT pss_average_symbol
 IMPORT sl_partial_den
 IMPORT asl_freq_index
 IMPORT sl_partial_num
#endif

ASM_TAG(compute_partial_num_dem_sfo_arib_asm) /* Entry point for function */
	PUSH {R4, R5, R6, R7, R8, R9, R10, R11, R12}
	LDR R1, =uc_used_carriers
	LDRB R1, [R1]	
	MOV R2, R0 /* Copy the actual symbol address. Will be used in the dot product of the Im */
	LDR R3, =asl_freq_index_squared
	LDR R4, =pss_average_symbol	/* Complex conjugate of the average symbol */
	LDR R4, [R4]
	MOV R11, #0 /* R12, R11 are the 64 bits accumulator for the dot products */
	MOV R12, #0
	
ASM_TAG(loop_Re_compute_partial_num_dem_sfo_arib_asm)
	LDR R5, [R0], #4 /* Reads Im, Re of the symbol */
	LDR R6, [R4], #4 /* Reads Im, Re of the *(average symbol) */
	SMULBB R7, R5, R6
	ASR R7, R7, #1 /* Re (symbol) x Re (average symbol) in Q3.29 */
	SMULTT R8, R5, R6
	RSB R8, R8, #0 /* -[Im (symbol) x Im (average symbol)] in Q2.30 */
	ADD R8, R7, R8, ASR #1 /* Re (equalized symbol) = Re (symbol) x Re (average symbol) -[Im (symbol) x Im (average symbol)] in Q3.29 */
	LDR R7, [R3], #4
	SMULL R9, R10, R8, R7 /* Re (equalized symbol)*asl_freq_index_squared in Q4.60 */
	LSR R9, R9, #Q4_60_TO_Q16_48    /* Q4.60->Q16.48. NO ROUNDING APPLIED */
	BFI R9, R10, #(32-Q4_60_TO_Q16_48), #Q4_60_TO_Q16_48 /* Q4.60->Q16.48 */
	ADDS R11, R11, R9 /* Accumulates to previous dot product partial value */
	ADC R12, R12, R10, ASR #Q4_60_TO_Q16_48
	SUBS R1, #1
        BNE loop_Re_compute_partial_num_dem_sfo_arib_asm
	LSR R11, R11, #(Q16_48_TO_Q1_31 + SCALING_TIME_DIMENSION + SCALING_FREQ_DIMENSION_ARIB + 2) /* +2 is to get the same result as in with the arm code, to avoid modifying the rest of the C code */
	BFI R11, R12, #(32 - Q16_48_TO_Q1_31 - SCALING_TIME_DIMENSION - SCALING_FREQ_DIMENSION_ARIB - 2), #(Q16_48_TO_Q1_31 + SCALING_TIME_DIMENSION + SCALING_FREQ_DIMENSION_ARIB + 2)
	LDR R4, =sl_partial_den
	STR R11, [R4]

	LDR R1, =uc_used_carriers
	LDRB R1, [R1]
	MOV R0, R2 /* Actual symbol address in R0 */
	LDR R3, =asl_freq_index
	LDR R4, =pss_average_symbol
	LDR R4, [R4]
	MOV R11, #0 /* R12, R11 are the 64 bits accumulator for the dot products */
	MOV R12, #0
ASM_TAG(loop_Im_compute_partial_num_dem_sfo_arib_asm)
	LDR R5, [R0], #4 /* Reads Im, Re of the symbol */
	LDR R6, [R4], #4 /* Reads Im, Re of the *(average symbol) */
	SMULTB R7, R5, R6
	ASR R7, R7, #1 /* Im (symbol) x Re (average symbol) in Q3.29 */
	SMULBT R8, R5, R6 /* Re (symbol) x Im (average symbol) in Q2.30 */
	ADD R8, R7, R8, ASR #1 /* Im (equalized symbol) = Im (symbol) x Re (average symbol) + Re (symbol) x Im (average symbol) in Q3.29 */
	LDR R7, [R3], #4
	SMULL R9, R10, R8, R7 /* Im (equalized symbol)*asl_freq_index in Q4.60 */
	LSR R9, R9, #Q4_60_TO_Q16_48    /* Q4.60->Q16.48. NO ROUNDING APPLIED */
	BFI R9, R10, #(32-Q4_60_TO_Q16_48), #Q4_60_TO_Q16_48 /* Q4.60->Q16.48 */
	ADDS R11, R11, R9 /* Accumulates to previous dot product partial value */
	ADC R12, R12, R10, ASR #Q4_60_TO_Q16_48
	SUBS R1, #1
        BNE loop_Im_compute_partial_num_dem_sfo_arib_asm
	LSR R11, R11, #(Q16_48_TO_Q1_31+SCALING_TIME_DIMENSION+SCALING_FREQ_DIMENSION_ARIB + 2)   /* +2 is to get the same result as in with the arm code, to avoid modifying the rest of the C code */
	BFI R11, R12, #(32-Q16_48_TO_Q1_31-SCALING_TIME_DIMENSION-SCALING_FREQ_DIMENSION_ARIB - 2), #(Q16_48_TO_Q1_31+SCALING_TIME_DIMENSION+SCALING_FREQ_DIMENSION_ARIB + 2)
	LDR R4, =sl_partial_num
	STR R11, [R4]

	POP {R4, R5, R6, R7, R8, R9, R10, R11, R12}
	BX LR
#if defined(__IAR_SYSTEMS_ASM__) || defined(__CC_ARM)
        LTORG
#endif


/*
 * \brief Compute the rotation vectors ass_rotation_sfo_first and ass_rotation_sfo_second, to be applied to ass_H. 
 *		  Prototype: void compute_ass_rotation_sfo_asm(int16_t *pss_Wi, int32_t *psl_distance, uint8_t uc_num_symbols, int32_t sl_delta_t);
*/
#if defined(__GNUC__)
 .global compute_ass_rotation_sfo_asm
 .type compute_ass_rotation_sfo_asm, %function /*Define compute_ass_rotation_sfo_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC compute_ass_rotation_sfo_asm
 EXTERN uc_num_s1s2
 EXTERN uc_half_est_delay_sfo_est
 EXTERN asl_delay_symbols
 EXTERN asl_freq_index
 EXTERN pss_rotation_sfo_first
 EXTERN asl_cos_n
 EXTERN asl_sin_n
 EXTERN sl_sfo_time_offset
 EXTERN pss_rotation_sfo_second
#elif defined(__CC_ARM)
 EXPORT compute_ass_rotation_sfo_asm
 IMPORT uc_num_s1s2
 IMPORT uc_half_est_delay_sfo_est
 IMPORT asl_delay_symbols
 IMPORT asl_freq_index
 IMPORT pss_rotation_sfo_first
 IMPORT asl_cos_n
 IMPORT asl_sin_n
 IMPORT sl_sfo_time_offset
 IMPORT pss_rotation_sfo_second
#endif

#if defined(__GNUC__)
 #define ASM_TAG(tag) tag:
#elif defined(__IAR_SYSTEMS_ASM__)
 #define ASM_TAG(tag) tag
#elif defined(__CC_ARM)
 #define ASM_TAG(tag) tag
#else
	#error "Unsupported assembler!"
#endif

ASM_TAG(compute_ass_rotation_sfo_asm)
	PUSH {R4, R5, R6, R7, R8, R9, R10, R11, R12}

	/* Compute the average distance of the estimated channel */
	MOV R8, #0  /* Accumulator for dot product */
	MOV R9, #0
ASM_TAG(loop_average_distance)
	LDRSH R4, [R0], #2 /* ass_Wi */
	LSL R4, R4, #16 /* ass_Wi in Q1.31 */
	LDR R5, [R1], #4 /* asl_distance */
	SMULL R6, R7, R4, R5 /* Q2.62 */
	ASR R6, R6, #Q2_62_TO_Q16_48
	BFI R6, R7, #(32-Q2_62_TO_Q16_48), #Q2_62_TO_Q16_48
	ASR R7, R7, #Q2_62_TO_Q16_48 /* Q16.48 */
	ADDS R8, R8, R6
	ADC	R9, R9, R7 
	SUBS R2, #1
        BNE loop_average_distance
	MOVW R6, #0x8000 /* for half-up rounding*/
	ADD R6, R8
	ASR R6, R6, #Q16_48_TO_Q1_31 
	BFI R6, R9, #(32-Q16_48_TO_Q1_31), #Q16_48_TO_Q1_31 /* R6=sl_average_distance */

	/* sl_aux_2=mult_real_q(sl_delta_t, (*(asl_delay_symbols + NUM_FULL_SYMBOLS_PREAMBLE + SYMBOLS_8 + uc_num_s1s2 -1) 
	+ (SYMBOL_LENGTH_SAMPLES >> (SCALING_DELAY_VALUES - 1)) * (HALF_EST_DELAY_SFO_EST + 1) - sl_average_distance), FRAC_PART_Q31);*/
	LDR R4, =uc_num_s1s2
	LDRB R4, [R4]
	MOV R7, #(NUM_FULL_SYMBOLS_PREAMBLE + NUM_SYMBOLS_8 -1)
	ADD R4, R7  
	LDR R7, =asl_delay_symbols
	ADD R4, R7, R4, LSL #2
	LDR R4, [R4] /* *(asl_delay_symbols + NUM_FULL_SYMBOLS_PREAMBLE + SYMBOLS_8 + uc_num_s1s2 -1) */
	MOV R7, #SYMBOL_LENGTH_SAMPLES
	ASR R7, R7, #(SCALING_DELAY_VALUES - 1)
	LDR R8, =uc_half_est_delay_sfo_est
	LDRB R8, [R8]
	SMULL R7, R8, R7, R8 
	ADD R4, R7 
	SUB R4, R4, R6 /* Second term of the product */	
	SMULL R7, R8, R4, R3 
	ADDS R7, R7, #0x40000000 /* for half-up rounding */
	ADC R8, R8, #0
	ASR R7, R7, #FRAC_PART_Q31
	BFI R7, R8, #(32-FRAC_PART_Q31), #FRAC_PART_Q31 /* R7=sl_aux_2 */
	
	
	/* sl_aux = mult_real_q(R7, VALUE_PI_DIV_2_Q_2_30, FRAC_PART_Q30); */
	MOVW R4, #VALUE_PI_DIV_2_Q_2_30_LOW
	MOVT R4, #VALUE_PI_DIV_2_Q_2_30_TOP
	SMULL R4, R5, R7, R4 
	ADDS R4, R4, #0x20000000 /* for half-up rounding */
	ADC R5, R5, #0
	ASR R4, R4, #FRAC_PART_Q30
	BFI R4, R5, #(32-FRAC_PART_Q30), #FRAC_PART_Q30 /* R4=sl_aux */
		
	LDR R2, =uc_used_carriers /* loop initialization */
	LDRB R2, [R2]
	LDR R6, =asl_freq_index
	MOV R9, #1
	LSL R9, R9, #(FRAC_PART_Q31 - (SCALING_FREQ_VALUES + SCALING_DELAY_VALUES + 1)) /* for half-up rounding */
	LDR R1, =pss_rotation_sfo_first /* loop initialization */
	LDR R1, [R1]
			
ASM_TAG(loop_ass_rotation_first)
	LDR R8, [R6], #4
	SMULL R5, R12, R8, R4 /* asl_phase_first in Q2.62 */
	ADD R5, R9
	ASR R5, R5, #(FRAC_PART_Q31 - (SCALING_FREQ_VALUES + SCALING_DELAY_VALUES + 1 - 3)) 
	BFI R5, R12, #(32 - FRAC_PART_Q31 + (SCALING_FREQ_VALUES + SCALING_DELAY_VALUES + 1 - 3)), #(FRAC_PART_Q31 - (SCALING_FREQ_VALUES + SCALING_DELAY_VALUES + 1 - 3)) /* R5=asl_phase_first in Q5.27 */
	CMP R3, #0 /* checks if sl_delta_t>0 */
	ITE MI /* if sl_delta_t<0*/
		RSBMI R11, R5, #0 /*-asl_phase_shift[uc_i] */
		MOVPL R11, R5
	AND R11, R11, #0xFC000000  /* sl_fix_theta_1 */
	ITE MI 
		ADDMI R5, R11  /* if sl_delta_t<0*/
		SUBPL R5, R5, R11  /* if sl_delta_t>0*/
	LSL R5, R5, #3  /* R5=sl_aprox_theta1_imag in Q2.30 */
	SMULL R10, R12, R5, R5 /* sl_aprox_theta1_imag x sl_aprox_theta1_imag in Q4.60 */
	ADDS R10, R10, #0x40000000 /* for half-up rounding Q4.60 to Q2.30 */
	ADC R12, R12, #0
	ASR R10, R10, #(FRAC_PART_Q30 + 1)
	BFI R10, R12, #(32-FRAC_PART_Q30 - 1), #(FRAC_PART_Q30 + 1)
	MOV R8, #VALUE_1_Q2_30
	SUB R8, R8, R10			/* R8: sl_aprox_theta_1_real in Q2.30 */
	LSR R11, R11, #(26 - 2) /* 4 x uc_index_vector_n */
	LDR R0, =asl_cos_n
	LDR R12, [R0, R11]
	SMULL R10, R12, R8, R12 /* sl_aprox_theta_1_real x cos_n [uc_index_vector_n] */
	LDR R0, =asl_sin_n
	LDR R0, [R0, R11]
	CMP R3,#0
	IT PL
		RSBPL R0, R0, #0 /* if sl_delta_t>0 */
	SMLAL R10, R12, R5, R0 /* sl_aprox_theta_1_real x cos_n [uc_index_vector_n] - (sl_aprox_theta_1_imag x sin_n [uc_index_vector_n] ) */		
	ADD R12, R12, #0x00002000 /* for half-up rounding Q4.60 to Q2.14 */
	ASR R12, R12, #14	
	STRH R12, [R1], #2 /* R10=Re[asl_rotation_first] in Q2.14 */
	RSB R0, R0, #0 /* -(-asl_sin_n[uc_index_vector_n]) or (-asl_sin_n[uc_index_vector_n]) depending on the preceeding operation on this value */
	SMULL R10, R12, R8, R0 /* sl_aprox_theta_1_real x sin_n [uc_index_vector_n] */
	LDR R0, =asl_cos_n
	LDR R0, [R0, R11]
	SMLAL R10, R12, R5, R0 /* sl_aprox_theta_1_real x sin_n [uc_index_vector_n] +- sl_aprox_theta_1_imag x cos_n [uc_index_vector_n] */	
	ADD R12, R12, #0x00002000 /* for half-up rounding Q4.60 to Q2.14 */
	ASR R12, R12, #14	
	STRH R12, [R1], #2 /* R10=Re[asl_rotation_first] in Q2.14 */
	SUBS R2, #1
        BNE loop_ass_rotation_first
	LDR R2, =sl_sfo_time_offset
	LDR R2, [R2]
	CMP R3,#0 /* checks if sl_delta_t>0 */
	IT MI
		RSBMI R2, R2, #0
	AND R2, R2, #0xF8000000
	ITE PL
		SUBPL R4, R7, R2, ASR #(LOG2_FFT_POINTS + SCALING_DELAY_VALUES -4) /* R4=sl_aux*/
		ADDMI R4, R7, R2, ASR #(LOG2_FFT_POINTS + SCALING_DELAY_VALUES -4) /* R4=sl_aux*/
		
ASM_TAG(compute_asl_phase_second)
	LDR R3, =pss_rotation_sfo_second /* loop initialization */
	LDR R3, [R3]
	LDR R2, =uc_used_carriers /* loop initialization */
	LDRB R2, [R2]
	LDR R6, =asl_freq_index

	MOVW R0, #VALUE_PI_DIV_2_Q_2_30_LOW
	MOVT R0, #VALUE_PI_DIV_2_Q_2_30_TOP
	SMULL R7, R8, R0, R4 
	ADDS R7, R7, #0x20000000 /* for half-up rounding */
	ADC R8, R8, #0
	ASR R7, R7, #FRAC_PART_Q30
	BFI R7, R8, #(32-FRAC_PART_Q30), #FRAC_PART_Q30 /* R7= sl_aux_2 = mult_real_q(sl_aux, VALUE_PI_DIV_2_Q_2_30, FRAC_PART_Q30);*/

ASM_TAG(loop_asl_phase_second)
	LDR R8, [R6], #4
	SMULL R5, R12, R8, R7 /* asl_phase_second in Q2.62 */
	ADD R5, R9
	ASR R5, R5, #(FRAC_PART_Q31 - (SCALING_FREQ_VALUES + SCALING_DELAY_VALUES + 1 - 3)) 
	BFI R5, R12, #(32 - FRAC_PART_Q31 + (SCALING_FREQ_VALUES + SCALING_DELAY_VALUES + 1 - 3)), #(FRAC_PART_Q31 - (SCALING_FREQ_VALUES + SCALING_DELAY_VALUES + 1 - 3)) /* R5=asl_phase_second in Q5.27 */
	CMP R5, #0
	ITE MI 
		RSBMI R0, R5, #0 /* if asl_phase_second[uc_i]<0 */
		MOVPL R0, R5
	AND R11, R0, #0xFC000000  /* sl_fix_theta_2 */
	ITE MI
		ADDMI R0, R5, R11
		SUBPL R0, R5, R11
	LSL R0, R0, #3  /* R0=sl_aprox_theta2_imag in Q1.31 */
	SMULL R10, R12, R0, R0 /* sl_aprox_theta2_imag x sl_aprox_theta2_imag in Q4.60 */
	ADDS R10, R10, #0x20000000 /* for half-up rounding Q4.60 to Q2.30 */
	ADC R12, R12, #0
	ASR R10, R10, #(FRAC_PART_Q30 + 1)
	BFI R10, R12, #(32-FRAC_PART_Q30 - 1), #(FRAC_PART_Q30 + 1)
	MOV R8, #VALUE_1_Q2_30
	SUB R8, R8, R10			/* R8: sl_aprox_theta_2_real in Q2.30 */
	LSR R11, R11, #(26 - 2) /* 4 x uc_index_vector_n */
	LDR R1, =asl_cos_n
	LDR R12, [R1, R11]
	SMULL R10, R12, R8, R12 /* sl_aprox_theta_2_real x cos_n [uc_index_vector_n] */
	LDR R1, =asl_sin_n
	LDR R1, [R1, R11]
	CMP R5, #0
	IT PL /* if asl_phase_second[uc_i]>0 */
		RSBPL R1, R1, #0
	SMLAL R10, R12, R0, R1 /* sl_aprox_theta_2_real x cos_n [uc_index_vector_n] +- (sl_aprox_theta_2_imag x sin_n [uc_index_vector_n] ) */	
	ADD R12, R12, #0x00002000 /* for half-up rounding Q4.60 to Q2.14 */
	ASR R12, R12, #14	
	STRH R12, [R3], #2 /* R10=Re[asl_rotation_second] in Q1.15 */
	RSB R1, R1, #0 /* -(-asl_sin_n[uc_index_vector_n]) or -(asl_sin_n[uc_index_vector_n]) depending on the preceeding operation on R1*/
	SMULL R10, R12, R8, R1 /* sl_aprox_theta_1_real x sin_n [uc_index_vector_n] */
	LDR R1, =asl_cos_n
	LDR R1, [R1, R11]
	SMLAL R10, R12, R0, R1 /* sl_aprox_theta_1_real x sin_n [uc_index_vector_n] +- sl_aprox_theta_2_imag x cos_n [uc_index_vector_n] */	
	ADD R12, R12, #0x00002000 /* for half-up rounding Q4.60 to Q2.14 */
	ASR R12, R12, #14
	STRH R12, [R3], #2 /* R10=Re[asl_rotation_second] in Q2.14 */
	SUBS R2, #1
        BNE loop_asl_phase_second
	
	POP {R4, R5, R6, R7, R8, R9, R10, R11, R12}
	BX LR

	
#ifdef UPDATE_CHN_SFO_EST_PAYLOAD
/*
 * \brief Sum the phase error in the last blocks of pilots to the cumulated one and computes the numerator and denominator of the per-carrier SNR
 *		  Prototype: void cum_phase_error_and_carrier_noise_asm(int16_t *pss_Ypilot_current_set, int16_t *pss_Ypilot_previous_set, int32_t *psl_cum_complex_error, int16_t *pss_H);
*/
#if defined(__GNUC__)
 .global cum_phase_error_and_carrier_SNR_num_dem_asm
.type cum_phase_error_and_carrier_SNR_num_dem_asm, %function /* Define cum_phase_error_and_carrier_SNR_num_dem_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC cum_phase_error_and_carrier_SNR_num_dem_asm
 EXTERN psl_Nk
 EXTERN sl_modulus
#elif defined(__CC_ARM)
 EXPORT cum_phase_error_and_carrier_SNR_num_dem_asm
 IMPORT psl_Nk
 IMPORT sl_modulus
#endif

ASM_TAG(cum_phase_error_and_carrier_SNR_num_dem_asm)
	PUSH {R4, R5, R6, R7, R8, R9, R10, R11, R12}

	LDR R11, [R0], #4 /* Reads Im, Re of the current set of pilots in Q1.15 */
	LDR R12, [R1], #4 /* Reads Im, Re of the previous set of pilots in Q1.15 */
	LDM R2, {R5, R6} /* Reads Im, Re of the cum phase in Q8.24 but scaled by 1/2^7*/
	
ASM_TAG(Re_cum_phase) 	/* The operation ASR R7, #1 can be eliminated by doing the sum in Q8.24 scaled by 1/2^7 but, surprisingly, it takes more time */
	SMULBB R7, R11, R12
	ASR R7, R7, #1 /* Re x Re in Q3.29 */
	SMULTT R8, R11, R12
	ADD R8, R7, R8, ASR #1 /* Re = Re x Re + Im x Im in Q3.29 */
	ADD R5, R5, R8, ASR #12 /* Q8.24 scaled by 1/2^7*/

ASM_TAG(Im_cum_phase) 
	SMULTB R7, R11, R12
	ASR R7, R7, #1 /* Im x Re in Q3.29 */
	SMULBT R8, R11, R12 /* Re x Im in Q3.29 */
	RSB R8, R8, #0
	ADD R8, R7, R8, ASR #1 /* Im = Im x Re - Re x Im in Q3.29 */
	ADD R6, R6, R8, ASR #12 /* Q8.24 scaled by 1/2^7*/
	STM R2, {R5,R6}

ASM_TAG(per_carrier_noise_estimation)
	LDR R4, [R3] /* Im, Re of ass_H */

	MOVW R10, #0
	MOVT R10, #0xFFFF /* Used to select the higher bits of a register */
	AND R6, R4, R10 /* Im[ass_H] in Q1.31 */
	AND R7, R11, R10 /* Im[current_pilot] in Q1.31 */
	ASR R6, R6, #2 /* Im[ass_H] in Q3.29 */
	SUB R5, R6, R7, ASR #2 /* Im[ass_H]-Im[current_pilot] in Q3.29 */
	SMULL R8, R9, R5, R5  /* (Im[ass_H]-Im[current_pilot])^2 in Q6.58 */
	SXTH R5, R4 /* Re[ass_H] in Q17.15 */
	SXTH R7, R11 /* Re[current_pilot] in Q17.15 */
	LSL R5, R5, #14 /* Re[ass_H] in Q3.29 */
	SUB R7, R5, R7, LSL #14 /* Re[ass_H]-Re[current_pilot] in Q3.29 */
	SMLAL R8, R9, R7, R7  /* (Im[ass_H]-Im[current_pilot])^2 + (Re[ass_H]-Re[current_pilot])^2 in Q6.58 */

	AND R7, R12, R10 /* Im[previous_pilot] in Q1.31 */
	SUB R4, R6, R7, ASR #2 /* Im[ass_H]-Im[previous_pilot] in Q3.29 */
	SMLAL R8, R9, R4, R4 /* (Im[ass_H]-Im[current_pilot])^2 + (Re[ass_H]-Re[current_pilot])^2 + (Im[ass_H]-Im[previous_pilot])^2 in Q6.58 */
	SXTH R7, R12 /* Re[previous_pilot] in Q17.15 */
	SUB R10, R5, R7, LSL #14 /* Re[ass_H]-Re[previous_pilot] in Q3.29 */
	SMLAL R8, R9, R10, R10  /* (Im[ass_H]-Im[current_pilot])^2 + (Re[ass_H]-Re[current_pilot])^2 + (Im[ass_H]-Im[previous_pilot])^2 +(Re[ass_H]-Re[previous_pilot])^2 in Q6.58 */
	MOVW R8, #1 /* For half-up rounding */
	ADD R9, R9, R8, LSL #(SCALING_NK + 2 -1) 
	ASR R9, R9, #(SCALING_NK + 2) /* Q8.24 scaled by 1/2^SCALING_NK */


	LDR R10, =psl_Nk
	LDR R10, [R10]
	LDR R7, [R10] /* Cumulated value of Nk */
	ADD R9, R7, R9
	STR R9, [R10]

	/*numerator of the SNR: sl_modulus_aux2 = modulus_complex_q(sl_real, sl_imag, FRAC_PART_Q24); */
	SMULL R7, R9, R6, R6 /* Im[ass_H]xIm[ass_H] in Q6.58 */
	SMLAL R7, R9, R5, R5 /* Re[ass_H]xRe[ass_H] + Im[ass_H]xIm[ass_H] in Q6.58 */
	ADD R9, R9, #0x00000002 /* For half-up rounding to Q8.24 */
	ASR R9, R9, #2

	LDR R10, =sl_modulus
	STR R9, [R10]
	
	POP {R4, R5, R6, R7, R8, R9, R10, R11, R12}
	BX LR

/*
 * \brief Compute the numerator and denominator of the SFO computation based on pilots. 
 *		  Prototype: void compute_partial_num_dem_sfo_pil_asm(int32_t *psl_cum_complex_error, int32_t *psl_SNRk, int32_t *psl_freq_index, int16_t *pus_distance);
*/
#if defined(__GNUC__)
 .global compute_partial_num_dem_sfo_pil_asm
 .type compute_partial_num_dem_sfo_pil_asm, %function /*Define compute_partial_num_dem_sfo_pil_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC compute_partial_num_dem_sfo_pil_asm
 EXTERN uc_pilot_used_twice_cum
 EXTERN sl_numerator_pil
 EXTERN sl_denominator_pil
#elif defined(__CC_ARM)
 EXPORT compute_partial_num_dem_sfo_pil_asm
 IMPORT uc_pilot_used_twice_cum
 IMPORT sl_numerator_pil
 IMPORT sl_denominator_pil
#endif

ASM_TAG(compute_partial_num_dem_sfo_pil_asm)
	PUSH {R4, R5, R6, R7, R8}

	LDM R0!, {R4, R5} /* Re, Im of asl_cum_complex_error */

	/* Compute sl_SNRk_by_freq= mult_real_q(asl_SNRk[uc_i], asl_freq_index[uc_i], FRAC_PART_Q31); Result is Q16.16 scaled by 1/2^(SCALING_FREQ_VALUES+SCALING_SNR) */
	LDR R1, [R1] /* asl_SNRk */
	LDR R2, [R2] /* asl_freq_index */
	SMULL R1, R7, R1, R2 
	ADDS R1, R1, #0x40000000 /* for half-up rounding */
	ADC R7, R7, #0
	ASR R1, R1, #FRAC_PART_Q31
	BFI R1, R7, #1, #FRAC_PART_Q31 /* sl_SNRk_by_freq */

	/* Compute sl_SNRk_by_freq_squared = mult_real_q(sl_SNRk_by_freq, asl_freq_index[uc_i], FRAC_PART_Q31); Result in Q16.16 scaled by 1/2^(2*SCALING_FREQ_VALUES+SCALING_SNR) */
	SMULL R2, R7, R1, R2 
	ADDS R2, R2, #0x40000000 /* for half-up rounding */
	ADC R7, R7, #0
	ASR R2, R2, #FRAC_PART_Q31
	BFI R2, R7, #1, #FRAC_PART_Q31 /* sl_SNRk_by_freq_squared */

	/* Compute div_real_q(aus_distance[uc_i], auc_pilot_used_twice_cum[uc_i], FRAC_PART_Q24, &sl_distance_div_pst_sum); */
	/* It is assumed that aus_distance[uc_i]<2^8. Result is Q8.24*/
	LDR R6, =uc_pilot_used_twice_cum
	LDRB R6, [R6]
	LDRSH R3, [R3] /* aus_distance[uc_i] */
	LSL R3, R3, #FRAC_PART_Q24
	ADD R3, R3, R6, ASR #1 /* for half-up rounding the division*/
	SDIV R3, R3, R6
	
	/* Compute sl_numerator += mult_real_q(mult_real_q(asl_cum_complex_error[2 * uc_i + 1], sl_SNRk_by_freq,
	FRAC_PART_Q16), sl_distance_div_pst_sum, FRAC_PART_Q24); Result is Q8.24 scaled by 1/2^(7+SCALING_FREQ_VALUES+SCALING_DELAY_VALUES) */
	SMULL R7, R8, R5, R1 /* Q24.40 */
	ADDS R7, R7, #0x00008000 /* for half-up rounding Q24.40 to Q8.24 */
	ADC R8, R8, #0
	LSR R7, R7, #16
	BFI R7, R8, #16, #16
	SMULL R7, R8, R7, R3 /* Q16.48 */
	ADDS R7, R7, #0x00800000 /* for half-up rounding Q16.48 to Q8.24 */
	ADC R8, R8, #0
	LSR R7, R7, #24
	BFI R7, R8, #8, #24
	LDR R8, =sl_numerator_pil
	LDR R1, [R8]
	ADD R1, R1, R7
	STR R1, [R8]
	
	/* Compute sl_denominator+= mult_real_q(mult_real_q(mult_real_q(asl_cum_complex_error[2 * uc_i], sl_SNRk_by_freq_squared,
		FRAC_PART_Q16), sl_distance_div_pst_sum, FRAC_PART_Q24), sl_distance_div_pst_sum, FRAC_PART_Q24);
		Result is Q8.24 scaled by 1/2^(7+2*SCALING_FREQ_VALUES+SCALING_DELAY_VALUES) */
	SMULL R7, R8, R4, R2 /* Q24.40 */
	ADDS R7, R7, #0x00008000 /* for half-up rounding Q24.40 to Q8.24 */
	ADC R8, R8, #0
	LSR R7, R7, #16
	BFI R7, R8, #16, #16
	SMULL R7,R8, R7, R3 /* Q16.48 */
	ADDS R7, R7, #0x00800000 /* for half-up rounding Q16.48 to Q8.24 */
	ADC R8, R8, #0
	LSR R7, R7, #24
	BFI R7, R8, #8, #24
	SMULL R7, R8, R7, R3 /* Q16.48 */
	ADDS R7, R7, #0x00800000 /* for half-up rounding Q16.48 to Q8.24 */
	ADC R8, R8, #0
	LSR R7, R7, #24
	BFI R7, R8, #8, #24
	LDR R8, =sl_denominator_pil
	LDR R2, [R8]
	ADD R2, R2, R7
	STR R2, [R8]
	
	POP {R4, R5, R6, R7, R8}
	BX LR

#endif

#if defined(__GNUC__)
 .end
#elif defined(__IAR_SYSTEMS_ASM__) || defined(__CC_ARM)
 END
#endif
