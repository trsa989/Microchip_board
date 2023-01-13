/**
 * \brief Assembler file with useful functions used in multiple places
 *
 */
#if defined(__GNUC__)
 .syntax unified
 .text
 .eabi_attribute Tag_ABI_align8_preserved,1  /* Equivalent to PRESERVE8 used in KEIL and RealView */
#elif defined(__IAR_SYSTEMS_ASM__)
 SECTION atpl250_aux_func_as:CODE:NOROOT(8)
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
 * \brief Reverse byte order in each of the two halfwords of a 32 byte word. The number of 32 byte words is us_num_elements. 
 *        Prototype: void swap_bytes_asm(int16_t *pss_input_array, uint16_t us_num_elements);
 */

#if defined(__GNUC__)
 .global swap_bytes_asm
 .type swap_bytes_asm, %function /*Define swap_bytes_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC swap_bytes_asm 
#elif defined(__CC_ARM)
 EXPORT swap_bytes_asm 
#else
	#error "Unsupported assembler!"
#endif

ASM_TAG(swap_bytes_asm)	 
	LDR	R2, [R0]		/* R0 contains the address of the 1st byte */
	REV16	R3, R2
	STR	R3, [R0], #4  /* Steps of 4 because memory is byte-addressable */
	SUBS R1, #1 
	BNE	swap_bytes_asm
	BX	LR

/**
 * \brief Set to zero a vector of Q1.15/Q1.31 complex values. 
 *		  Prototype: void zero_complex_vector_q_asm(uint8_t *psc_address, uint16_t us_num_bytes_in_vector);
 */
#if defined(__GNUC__)
 .global zero_complex_vector_q_asm
 .type zero_complex_vector_q_asm, %function /* Define zero_complex_vector_q_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC zero_complex_vector_q_asm
#elif defined(__CC_ARM)
 EXPORT zero_complex_vector_q_asm
#endif

ASM_TAG(zero_complex_vector_q_asm) /* Entry point for function */
	MOV R2, #0
ASM_TAG(loop_zero_complex_vector_q_asm)
	STR R2, [R0], #4
	SUBS R1, #4
	BNE loop_zero_complex_vector_q_asm

	BX LR

/**
 * \brief Convert Q1.15 to Q1.31 a vector of complex values stored as Re, Im
 *
*/
;.global q1_15_to_q1_31
;.type q1_15_to_q1_31, %function /*Define q1_15_to_q1_3 as a function name */
;
;q1_15_to_q1_31:	  /* Entry point for function */		
;	PUSH {R4}
;
;loop_q1_15_to_q1_31:
;	LDR R4, [R0], #4  /* Load the 2-bytes of Real and Imag part and increment address by 2-bytes */
;	LSL R3, R4, #16     /* Q1.15 to Q1.31 of Re*/
;	BFC R4, #0, #16	 /* Im in Q1.31 */
;	STM R1!, {R3, R4} /* Stores the result and increment address by 8-bytes */
;	SUBS R2, #1
;	BNE loop_q1_15_to_q1_31
;
;	POP {R4}
;	BX	LR

/**
 * \brief Difference of two vectors in Q1.31
 */
;.global sub_q31_asm
;.type sub_q31_asm, %function /* Define sub_q31_asm as a function name */
;
;
;sub_q31_asm: /* Entry point for function */
;	PUSH {R4, R5}  /* Save context */
;	
;loop_sub_q31_asm:
;	LDR R4, [R0], #4 /* X in Q1.31 */
;	LDR R5, [R1], #4 /* X in Q1.31 */
;	QSUB R5, R4, R5  /*  X-Y in Q1.31 */
;	STR R5, [R2], #4
;	SUBS R3, #1
;	BNE loop_sub_q31_asm
;
;	POP {R4, R5} /* Restore context*/
;	BX	LR

/**
 * \brief Scale correlation reference to have maximum range (2^15-1)
 *
 */
#if defined(__GNUC__)
 .global SYNC_SCALE_XCORR_REF
 .type SYNC_SCALE_XCORR_REF, %function /* Define zero_complex_vector_q_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC SYNC_SCALE_XCORR_REF
#elif defined(__CC_ARM)
 EXPORT SYNC_SCALE_XCORR_REF
#endif

ASM_TAG(SYNC_SCALE_XCORR_REF) /* Entry point for function */
	/* R0=pss_ifft_out */
	/* R1=pss_xcorr_ref */
	/* R2=us_num_samples */

	PUSH {R4-R5}

	PUSH {R0, R2}
	MOV R4, #0 /* R4=us_max_abs*/
	/* Search maximum absolute value */
ASM_TAG(sync_scale_xcorr_ref_max_loop)
	LDRSH R5, [R0], #2 /* R5=pss_ifft_out[k] (sQ0.15) */
	REV16 R5, R5
	SXTH R5, R5

	CMP R5, #0
	IT MI
		RSBMI R5, R5, #0 /* R5=abs(pss_ifft_out[k]) (uQ0.15) */

	CMP R5, R4
	IT HI
		MOVHI R4, R5 /* R4=us_max_abs (uQ0.15) */

	SUBS R2, R2, #1
	BNE sync_scale_xcorr_ref_max_loop

	/* Compute factor to multiply by and scale to maximum range */
	MOV R5, #0x7FFF /* R5=1 [uQ0.15] */
	LSL R5, R5, #16 /* R5=1 [uQ0.31] */

	ADD R5, R5, R4, LSR #1
	UDIV R4, R5, R4 /* R4=factor [uQ15.16] */
	POP {R0, R2}
	MOV R3, #0 /* R3=us_xcorr_abs_accum */

ASM_TAG(sync_scale_xcorr_ref_scale_loop)
	LDRSH R5, [R0], #2 /* R5=pss_ifft_out[k] (sQ0.15) */
	REV16 R5, R5
	SXTH R5, R5
	MUL R5, R5, R4 /* R5=pss_ifft_out[k]*factor [sQ0.31] */
	ASRS R5, R5, #16 /* R5=IFFT_OUT(k)*factor [sQ0.15] */
	ADC R5, R5, #0 /* Rounding */

	/* Accumulate absolute values in R3 */
	CMP R5, #0
	ITE PL
		ADDPL R3, R3, R5
		SUBMI R3, R3, R5

	REV16 R5, R5
	SUBS R2, R2, #1
	STRH R5, [R1], #2 /* Store value (1 byte) */

	BNE sync_scale_xcorr_ref_scale_loop

	/* Convert maximum correlation value to ATPL250 correlation (k2=1.64676*k1/128; k2=k1*0.0128653125) */
	/* 0.0128653125 * 2^32 = 55256096 (0x034B2420) */
	MOVW R0, #0x2420
	MOVT R0, #0x034B

	UMULL R1, R0, R3, R0 /* R0(Hi)&R1(Lo) = ul_xcorr_abs_accum * 0.0128653125 (uQ32.32); R0 = ul_xcorr_abs_accum * 0.0128653125 (uQ32.0) */

	POP {R4-R5}
	BX LR

#if defined(__GNUC__)
 .end
#elif defined(__IAR_SYSTEMS_ASM__) || defined(__CC_ARM)
 END
#endif
