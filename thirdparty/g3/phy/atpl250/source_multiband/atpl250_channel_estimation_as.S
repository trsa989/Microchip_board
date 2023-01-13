/**
 * \brief Assembler file with functions used for channel estimation
 *
 */


#include "atpl250_channel_and_sfo_estimation_params.h"

#if defined(__GNUC__)
 .syntax unified
 .text
 .eabi_attribute Tag_ABI_align8_preserved,1  /* Equivalent to PRESERVE8 used in KEIL and RealView */
#elif defined(__IAR_SYSTEMS_ASM__)
 SECTION atpl250_channel_estimation_as:CODE:NOROOT(8)
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
 * \brief Invert channel response given in Q1.15. Result is 16 bits and format is Q(16-[R0]).[R0]
 *	      Assumest that DIV_0_TRP=0 in configuration register SCB_CCR. 
 *		  Prototype: void invert_channel_asm(uint8_t uc_length_fraccional);	
 */
#if defined(__GNUC__)
 .global invert_channel_asm
 .type invert_channel_asm, %function /*Define swap_bytes_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC invert_channel_asm
 EXTERN ass_inv_H
 EXTERN auc_control_avg_invert_chan
#elif defined(__CC_ARM)
 EXPORT invert_channel_asm
 IMPORT ass_inv_H
 IMPORT auc_control_avg_invert_chan
#endif

ASM_TAG(invert_channel_asm)	 /* Entry point for function.  R0 contains uc_length_fraccional*/
	PUSH {R4, R5, R6, R7, R8, R9, R10, R11, R12} /* Save context to stack */
	LDR R3, =ass_inv_H
	LDR R12, =auc_control_avg_invert_chan
	BFC R5, #0, #32
	MOVT R5, #0xFFFF /* Used to select the higher bits of a register. R5 CANNOT BE REUSED */
	SUB R0, R0, #1 	

ASM_TAG(loop_invert_channel_asm) 
	LDR R4, [R2], #4  /* Load the 2-bytes of Real part and the 2-bytes of the Im part and increment address by 4-bytes */
	LDRB R6, [R12], #1  
	CMP R4, #0    /* If Re=0 and Im=0->avg_carrier */
	BNE invert_carrier /* If Re!=0 or Im!=0, invert the complex*/	
	
ASM_TAG(avg_carrier)  /* If Re=Im=0, the actual value is computed as a function of the previous and following carriers. R10 CANNOT BE REUSED INSIDE THE LOOP */		
	CMP R6, #0
	BEQ unnotched_pred
	CMP R6, #2
	BEQ unnotched_suc

	/* Actual carrier is computed as an average of the previous and next carrier */
	LDR R4, [R2, #-8]  /* Load the 2-bytes of Real part and the 2-bytes of Im part of the previous carrier  */
	CMP R4, #0
	IT EQ     /* Previous carrier was zero: its averaged value is in R10 (Im, Re) */
		MOVEQ R4, R10
	AND R7, R5, R4, ASR #1
	SXTH R4, R4    
	ORR R4, R7, R4, ASR #1  /* Contains Im/2, Re/2 */
	LDR R6, [R2]		/* Load the 2-bytes of Real part and 2-bytes of Imag part of the next carrier  */		
	AND R7, R5, R6, ASR #1
	SXTH R8, R6    
	ORR R6, R7, R8, ASR #1  /* Contains Im/2, Re/2 */
	SADD16 R4, R4, R6 /* (Im[previous]+Im[next])/2, (Re[previous]+Re[next])/2 */
	CMP R6, #0
	IT EQ    /* Next carrier is zero: store the averaged value of the actual one in R11 (Im,Re) to compute the average of the next one */
		MOVEQ R10, R4
	CBNZ R4, invert_carrier  /* If Re=!0 or Im!=0, invert the complex */	

ASM_TAG(fix_to_max_carrier) /* Re=0 and Im=0 are still zero, fix the inverse of Re and Im to MAX_INT16 */	
	/* Fix inverse of both Re and Im to the maximum int16*/
	MOV R6, #0x7FFF7FFF
	STR R6, [R3], #2	
	B scale_inverted_chan


ASM_TAG(unnotched_pred)  /* Previous carrier is notched: Actual carrier is fixed to the next one */
	LDR R4, [R2]  /* Load the 2-bytes of Real part and 2-bytes of Imag part of the next carrier */
	CBNZ R4, invert_carrier  /* If Re!=0 or Im!=0, invert the complex*/	
	B fix_to_max_carrier

ASM_TAG(unnotched_suc)  /* Next carrier is notched: Actual carrier is fixed to the previous one */
	LDR R4, [R2, #-6]  /* Load the 2-bytes of Real part and the 2-bytes of Imag part of the previous carrier */
	CBNZ R4, invert_carrier  /* If Re=0, fix the inverse of Re and Im to MAX_INT16*/	
	B fix_to_max_carrier

ASM_TAG(invert_carrier) /* Inverts a complex value whose stored in R4 as Im, Re. Result is given in R6 (Re) and R7 (Im) */		
	AND R7, R5, R4
	ASR R7, R7, #1 /* Im in Q2.30*/
	RSB R7, R7, #0  /* Negates the Imag part */	
	SXTH R6, R4    
	LSL R6, R6, #15 /* Re in Q2.30 */
	MOV R8, #0
	MOV R9, #VALUE_2_TO_30	/* For half-up rounding */
	SMLAL R9, R8, R6, R6 /* Re x Re */
	SMLAL R9, R8, R7, R7 /* Im x Im */		
	ASR R7, R7, #1 /* Im in Q3.29 */
	ASR R6, R6, #1 /* Re in Q3.29 */
	ASRS R8, R8, R0 /* Q11.21 */
	ITT NE		
		SDIVNE R6, R6, R8  /* Result in Q24.8. If R8=0 division is not executed */
		SDIVNE R7, R7, R8  /* Result in Q24.8. If R8=0 division is not executed */
	
	/* Checks and saturates overflow in the Re (in R6) and Im (in R7) part of the inverse */		
	ITE EQ  /* ¿R8=0?*/
		MOVEQ R9, #0
		MOVNE R9, #VALUE_2_TO_15
	/* Check overflow in Re */
	CMP R6, R9
	IT PL
		MOVWPL R6, #MAX_INT16  /* R6-VALUE_2_TO_15>0 */
	CMN R6, R9
	ITT MI
		MOVWMI R6, #MAX_INT16 /* R6+VALUE_2_TO_15<0 */
		RSBMI R6, R6, #0		 /* Needed because inmediate operand in MOVWMI must be 0-65535*/
	/* Check overflow in Im */
	CMP R7, R9	
	IT PL
		MOVWPL R7, #MAX_INT16  
	CMN R7, R9
	ITT MI
		MOVWMI R7, #MAX_INT16
		RSBMI R7, R7, #0	
		
ASM_TAG(scale_inverted_chan)
		MOVW R8, #GAIN_SCALE_INV_H_FEQ
		MOV R9, #VALUE_2_TO_14 /* For half-up rounding */
		SMLABB R6, R6, R8, R9 /* Q9.23 */
		ASR R6, R6, #15 /* Q8.8 */						
		MOVW R8, #GAIN_SCALE_INV_H_FEQ
		MOV R9, #VALUE_2_TO_14 /* For half-up rounding */
		SMLABB R7, R7, R8, R9 /* Q9.23 */
		ASR R7, R7, #15 /* Q8.8 */	
	
ASM_TAG(swap_bytes_inverted_chan)
		PKHBT R6, R6, R7, LSL #16	 
		REV16	R6, R6
				
		/* Stores inverted channel in ass_inv_H */
		STR R6, [R3], #4 /* Stores the Real and Im part and increments address by 4-bytes */

ASM_TAG(loop_invert_channel_asm_end)
	SUBS R1, #1
	BNE loop_invert_channel_asm

ASM_TAG(end)	
	POP {R4, R5, R6, R7, R8, R9, R10, R11, R12} /* Restore context*/
	BX	LR

/**
 * \brief Energy of a vector of values in Q1.31. Result in Q16.48
 *	      Prototype: void energy_vector_q31(int32_t *pss_input_array, uint8_t uc_num_complex_values, int64_t *psll_output);
 */
#if defined(__GNUC__)
 .global energy_vector_q31
 .type energy_vector_q31, %function /*Define energy_vector_q31 as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC energy_vector_q31
#elif defined(__CC_ARM)
 EXPORT energy_vector_q31
#endif

ASM_TAG(energy_vector_q31) /* Entry point for function */
	PUSH {R4, R5, R6, R7}  /* Save context */ 
	MOV R6, #0 /* Clear R6 and R7, they will contain the lowest and highest bits of the energy */
	MOV R7, #0


ASM_TAG(loop_energy_vector_q31)
	LDR R3, [R0], #4  /* A in Q1.31 */
	SMULL R4, R5, R3, R3 /*  A x A in Q2.62 */
	LSR R4, R4, #Q2_62_TO_Q16_48
	BFI R4, R5, #18, #Q2_62_TO_Q16_48  /* R5 contains the lowest bits of AxA in Q16.48 */
	ADDS R6, R6, R4				 /* R6 contains the lowest bits of the cumulative sum in Q16.48 */
	ADC R7, R7, R5, ASR #Q2_62_TO_Q16_48 /* R7 contains the highest bits of the cumulative sum in Q16.48 */
	SUBS R1, #1
	BNE loop_energy_vector_q31
	STM R2!, {R6, R7} 

	POP {R4, R5, R6, R7} /* Restore context*/
	BX	LR

/**
 * \brief Computes the weighting coefficients Wi=Ni/sum(Nj)
 *		  Prototype: void compute_weights_asm(uint8_t uc_num_values);
 */
#if defined(__GNUC__)
 .global compute_weights_asm
 .type compute_weights_asm, %function /*Define compute_weights_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC compute_weights_asm
 EXTERN asl_Ni
 EXTERN asl_inv_Ni
 EXTERN ass_Wi
 EXTERN sl_Ni_min
#elif defined(__CC_ARM)
 EXPORT compute_weights_asm
 IMPORT asl_Ni
 IMPORT asl_inv_Ni
 IMPORT ass_Wi
 IMPORT sl_Ni_min
#endif

ASM_TAG(compute_weights_asm) /* Entry point for function*/
	PUSH {R4, R5, R6, R7, R8}
	LDR R2, =asl_Ni
	LDR R3, =asl_inv_Ni
	LDR R4, =ass_Wi
	MOV R5, R0 /* Replicate the number of Ni values for second loop control */
	LDR R6, =sl_Ni_min
	LDR R6, [R6]

	/* Computes the inverse of all Ni as (min(Ni/2))/Ni */
	MOV R8, #0 /* Accumulator for the sum(1/Ni) */
	LSR R6, R6, #1 /* min(Ni/2) */
ASM_TAG(loop_1_compute_weights_asm)	
	LDR R7, [R2], #4
	CMP R7, #1
	ITTEE MI
		MOVWMI R7, #0xFFFF
		MOVTMI R7, #0x7FFF  /* Fix R6 to MAXINT32*/	  	
		LSRPL R7, R7, #16 /* sl_Ni in Q17.15 */
		SDIVPL R7, R6, R7 /* Since, R6 is in Q2.30, result is Q17.15 */
	ADD R8, R8, R7 /*  Q17.15 */
	LSL R7, R7, #15 /* Q2.30 */
	STR R7, [R3], #4  
	SUBS R0, #1 
	BNE loop_1_compute_weights_asm

	/* Computes (min(Ni/2)/Ni) /sum((min(Ni/2)/Ni)/Ni) */
	LDR R3, =asl_inv_Ni
ASM_TAG(loop_2_compute_weights_asm)
	LDR R6, [R3], #4
	SDIV R6, R6, R8
	STRH R6, [R4], #2
	SUBS R5, #1
	BNE loop_2_compute_weights_asm

	POP {R4, R5, R6, R7, R8}
	BX LR	

/**
 * \brief Equivalent to arm_shift (vector_input, vector_ouput1,...) + arm_add(vector_output1, vector_other,...) + arm_shift(vector_input, vector_output2,...)
 *        It takes into account that vectors are complex and then, the number of elements is even, packed as Im, Re. 
 *		  Prototype: void shift_add_shift_asm(int16_t *pss_input_array1, int16_t *pss_input_array2, uint8_t uc_num_complex_elements);
 */
#if defined(__GNUC__)
 .global shift_add_shift_asm
 .type shift_add_shift_asm, %function /*Define shift_add_shift_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC shift_add_shift_asm
#elif defined(__CC_ARM)
 EXPORT shift_add_shift_asm
#endif

ASM_TAG(shift_add_shift_asm) /* Entry point for function */
	PUSH {R4, R5, R6, R7, R8, R9}
	BFC R7, #0, #32
	SUB R8, R7, #1 /* R8=0xFFFFFFFF */	
	BFC R8, #16, #SCALING_VECTOR_INPUT_BEFORE_STORE 
	MOVT R7, #0xFFFF
	MOVW R9, #0xFFFF

ASM_TAG(loop_shift_add_shift_asm)
	LDR R3, [R0]
	LDR R4, [R1]
	AND R5, R7, R3, ASR #SCALING_VECTOR_INPUT_BEFORE_SUM /* Top halfword contains Im scaled by SCALING_VECTOR_INPUT_BEFORE_SUM */
	SXTH R6, R3    
	AND R6, R9, R6, ASR #SCALING_VECTOR_INPUT_BEFORE_SUM 
	ORR R5, R5, R6  /* Top halfword contains Im scaled by SCALING_VECTOR_INPUT_BEFORE_SUM and bottom halfword contains Re scaled by the same factor	*/
	SADD16 R4, R4, R5
	AND R3, R8, R3, LSL #SCALING_VECTOR_INPUT_BEFORE_STORE  
	STR R4, [R1], #4
	STR R3, [R0], #4
	SUBS R2, #1
	BNE  loop_shift_add_shift_asm

	POP {R4, R5, R6, R7, R8, R9}
	BX LR

/**
 * \brief Equivalent to arm_scale (vector_input, vector_ouput1,...) + arm_add(vector_output1, vector_other,...) + arm_shift(vector_input, vector_output2,...)
 *        It takes into account that vectors are complex and then, the number of elements is even, packed as Im, Re. 
 *		  Prototype: void scale_shift_add_shift_asm(int16_t *pss_input_array1, int16_t *pss_input_array2, uint8_t uc_num_complex_elements);
 */
#if defined(__GNUC__)
 .global scale_shift_add_shift_asm
 .type scale_shift_add_shift_asm, %function /*Define scale_shift_add_shift_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC scale_shift_add_shift_asm
#elif defined(__CC_ARM)
 EXPORT scale_shift_add_shift_asm
#endif

ASM_TAG(scale_shift_add_shift_asm) /* Entry point for function */
	PUSH {R4, R5, R7, R9, R10, R11}
	MOVW R9, #VALUE_SQRT_2_Q13	

ASM_TAG(loop_scale_shift_add_shift_asm)
	LDR R3, [R0]
	LDR R4, [R1]
	SMULTB R10, R3, R9  /* Im * sqrt(2) in Q6.26 */   
	LSL R7, R10, #1 /* R7[31:16]=Im * sqrt(2) in Q5.11 ->There is no need to scale because no overflow will occur if the number of symbols is<16 */
	SMULBB R11, R3, R9  /* Re * sqrt(2)  in Q6.26 */   
	PKHTB R5, R7, R11, ASR #15
	SADD16 R4, R4, R5
	LSL R7, R7, #4 /* R7[31:16]=Im * sqrt(2) in Q1.15 */   
	PKHTB R5, R7, R11, ASR #11
	STR R4, [R1], #4 /* Average symbol */
	STR R5, [R0], #4
	SUBS R2, #1
	BNE  loop_scale_shift_add_shift_asm 

	POP {R4, R5, R7, R9, R10, R11}
	BX LR
	

	/**
 * \brief Computes the squared magnitude of each component of a complex vector in Q1.15, with result in Q3.29
 *		  Result might differ in the the one obtained with ARM functions but the one implemented here has higher precision. 
 *		  Prototype: void cmplx_mag_squared_q15_result_q31_asm(q15_t *pss_input_symbol, q31_t *psl_output_symbol, uint8_t num_complex_elem). 		  
 */
#if defined(__GNUC__)
 .global cmplx_mag_squared_q15_result_q31_asm
 .type cmplx_mag_squared_q15_result_q31_asm, %function /*Define cmplx_mag_squared_q15_result_q31_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC cmplx_mag_squared_q15_result_q31_asm
#elif defined(__CC_ARM)
 EXPORT cmplx_mag_squared_q15_result_q31_asm
#endif

ASM_TAG(cmplx_mag_squared_q15_result_q31_asm) /* Entry point for function */
	PUSH {R4, R5, R6}
	MOV R6, #0 

ASM_TAG(loop_cmplx_mag_squared_q15_result_q31_asm)	
	LDR R3, [R0], #4 /* Re and Im part of the input vector*/
	UMULL R4, R5, R6, R6 /* Clear accumulator */
	SMLALD R4, R5, R3, R3 /* Re x Re +  Im x Im in Q34.30 */
	LSR R4, R4, #1  /* NO ROUNDING APPLIED */
	BFI R4, R5, #31, #1
	STR R4, [R1], #4
	SUBS R2, #1
	BNE loop_cmplx_mag_squared_q15_result_q31_asm

	POP {R4, R5, R6}
	BX LR

/**
 * \brief Applies smoothing to a channel response estimate at the specified carriers.
 *		  The value in each carrier is obtained by averaging the previous estimates in the carrier 
 *		  with the information from the adjacent ones.	  
 *	      Prototype: void smooth_carriers_asm(uint8_t *puc_data_carriers_list, uint8_t uc_num_data_carriers, q15_t *pss_input_symbol);
 */
#if defined(__GNUC__)
 .global smooth_carriers_asm
 .type smooth_carriers_asm, %function /* Define smooth_carriers_asm as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC smooth_carriers_asm
#elif defined(__CC_ARM)
 EXPORT smooth_carriers_asm
#endif

ASM_TAG(smooth_carriers_asm) /* Entry point for function */
	PUSH {R4, R5, R6, R7, R8, R10, R11, R12}
	MOVW R11, #0xAAAB
	MOVT R11, #0x2AAA  /* Value of 1/3 in Q1.31 */

ASM_TAG(first_iteration_smooth_carriers_asm) 
	LDRB R12, [R0], #1 /* R12: index of the carrier: 0...N-1 */
	ADD R12, R2, R12, LSL #2  /* R12: address of the carrier to be smoothed */
	MOV R3, R12 /* Copy address of the actual carrier */
	LDR R4, [R12] /* Actual carrier */
	ADD R12, R12, #4
	LDR R5, [R12] /* Next carrier */
	SUB R12, R12, #8
	LDR R6, [R12] /* Previous carrier */
	
	/* Computes Im of the average */
	MOV R7, R4, ASR #16 
	ADD R7, R7, R5, ASR #16
	ADD R7, R7, R6, ASR #16
	SMULL R7, R8, R7, R11 /* R8: Im in Q18.46 */
	
	/* Computes Re of the average */
	SXTH R4, R4
	SXTH R5, R5
	SXTH R6, R6
	ADD R7, R4, R5
	ADD R7, R7, R6
	SMULL R4, R7, R7, R11 
	ASR R4, R4, #31   /* NO ROUNDING APPLIED */
	BFI R4, R7, #1, #15 /* R4: Re in 1.15 */
	
	PKHBT R4, R4, R8, LSL #17
	MOV R10, R4
	SUBS R1, #1

ASM_TAG(loop_smooth_carriers_asm)
	BEQ last_iteration_smooth_carriers_asm
	LDRB R12, [R0], #1 /* R12: index of the carrier: 0...N-1 */
	ADD R12, R2, R12, LSL #2  /* R12: address of the carrier to be smoothed */
	LDR R4, [R12] /* Actual carrier */
	ADD R12, R12, #4
	LDR R5, [R12] /* Next carrier */
	SUB R12, R12, #8
	LDR R6, [R12] /* Previous carrier */
	
	/* Computes Im of the average */
	MOV R7, R4, ASR #16 
	ADD R7, R7, R5, ASR #16
	ADD R7, R7, R6, ASR #16
	SMULL R7, R8, R7, R11 /* R8: Im in Q18.46 */
	
	/* Computes Re of the average */
	SXTH R4, R4
	SXTH R5, R5
	SXTH R6, R6
	ADD R7, R4, R5
	ADD R7, R7, R6
	SMULL R4, R7, R7, R11 
	ASR R4, R4, #31  /* NO ROUNDING APPLIED */
	BFI R4, R7, #1, #15 /* R8: Re in 1.15 */
	
	PKHBT R4, R4, R8, LSL #17
	STR R10, [R3] /* Store average value of the previous carrier */
	ADD R3, R12, #4 /* Update address of the actual carrier for next iteration */
	MOV R10, R4 /* Store averaged value for next iteration */
	SUBS R1, #1
	B loop_smooth_carriers_asm

ASM_TAG(last_iteration_smooth_carriers_asm) 
	STR R4, [R3] /* Load average value of the previous carrier */

ASM_TAG(end_smooth_carriers_asm)
	POP {R4, R5, R6, R7, R8, R10,  R11, R12}
	BX LR


#ifdef UPDATE_CHN_SFO_EST_PAYLOAD
/*
 * \brief Swap bytes of the pilots, demodulate pilots and update the channel estimate on the frequency of the pilot carriers
 *		  Prototype: void dem_pilots_and_update_chan_as(uint8_t *puc_pilots_per_symbol, q15_t *pss_input_symbol, q15_t *pss_modulating_symbols, uint8_t uc_num_pilots_per_block);
*/
#if defined(__GNUC__)
 .global dem_pilots_and_update_chan_as
 .type dem_pilots_and_update_chan_as, %function /* Define dem_pilots_and_update_chan_as as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC dem_pilots_and_update_chan_as
 EXTERN pass_Ypilots_as
 EXTERN ass_H
#elif defined(__CC_ARM)
 EXPORT dem_pilots_and_update_chan_as
 IMPORT pass_Ypilots_as
 IMPORT ass_H
#endif

ASM_TAG(dem_pilots_and_update_chan_as) /* Entry point for function */
	PUSH {R4, R5, R6, R7, R8, R9, R10, R11, R12}
	LDR R12, =pass_Ypilots_as /* R12 contains the address of the pointer pasl_Ypilots */
	LDR R12, [R12] /* R12 contains the address pointed by pasl_Ypilots */
	LDR R11, =ass_H
	
	/* Computes: 
	for (uc_i = 0; uc_i < uc_num_pilots_per_block; uc_i++) {
		ass_Ypilots[2 *	puc_pilots_per_symbol[uc_i]]= mult_real_q(ass_symbol_aux[2 * uc_i], ass_modulating_symbols[2 * uc_i],
				FRAC_PART_Q17) - mult_real_q(ass_symbol_aux[2 * uc_i + 1], ass_modulating_symbols[2 * uc_i + 1], FRAC_PART_Q17);
		ass_Ypilots[2 * puc_pilots_per_symbol[uc_i] +1] = mult_real_q(ass_symbol_aux[2 * uc_i + 1], ass_modulating_symbols[2 * uc_i],
				FRAC_PART_Q17) + mult_real_q(ass_symbol_aux[2 * uc_i], ass_modulating_symbols[2 * uc_i + 1], FRAC_PART_Q17);				
	} */
ASM_TAG(demodulate_pilots)
	LDR R5, [R1], #4 /* Reads Im, Re of the input symbol in Q1.15 */
	REV16	R5, R5 /* Swap bytes of the read pilots */
	LDR R6, [R2], #4 /* Reads Im, Re of the complex conjugate of the modulating symbol in Q1.15 */

ASM_TAG(Re_Ypilot) 	
	SMULBB R7, R5, R6
	ASR R7, R7, #1 /* Re (input symbol) x Re (conjugate of the modulating symbol) in Q3.29 */
	SMULTT R8, R5, R6
	RSB R8, R8, #0 /* -[Im (input symbol) x Im (conjugate of the modulating symbol) ] in Q2.30 */
	ADD R8, R7, R8, ASR #1 /* Re Ypilot = Re (input symbol) x Re (conjugate of the modulating symbol)  -[Im (input symbol) x Im (conjugate of the modulating symbol) ] in Q3.29 */
	MOVW R4, #VALUE_2_TO_15
	ADD R4, R8, R4 /* half-up rounding */
	MOVW R7, #VALUE_SQRT_2_Q13
	SMULTB R8, R4, R7 /* Q6.26 */

ASM_TAG(Im_Ypilot) 
	SMULTB R7, R5, R6
	ASR R7, R7, #1 /* Im (input symbol) x Re (conjugate of the modulating symbol) in Q3.29 */
	SMULBT R9, R5, R6 /* Re (input symbol) x Im (conjugate of the modulating symbol) in Q2.30 */
	ADD R9, R7, R9, ASR #1 /* Im Ypilot = Im x Re + Re x Im  in Q3.29 */
	MOVW R4, #VALUE_2_TO_15
	ADD R4, R9, R4 /* half-up rounding */
	MOVW R7, #VALUE_SQRT_2_Q13
	SMULTB R9, R4, R7 /* Q6.26 */

ASM_TAG(store_demodulated_pilots)
	/* Store Re and Im in Q1.15 */
	LDRB R10, [R0], #1 
	ADD R7, R12, R10, LSL #2 /* Address of the pilot carrier */
	ADD R5, R8, #VALUE_2_TO_10 /* for half-up rounding */
	ASR R5, R8, #11
	ADD R6, R9, #VALUE_2_TO_10 /* for half-up rounding */
	PKHBT R5, R5, R9, LSL #5
	STR R5, [R7] 


	/* Update channel estimation */
	/* Computes: 
	for (uc_i = 0; uc_i < uc_num_pilots_per_block; uc_i++) {
		ass_H[ 2 * auc_pilot_carriers_list[uc_i] ]	= mult_real_q(ass_H[ 2 * auc_pilot_carriers_list[uc_i] ], COMP_ALPHA_CHANNEL_EST_Q15,
				FRAC_PART_Q15) + mult_real_q(ass_Ypilots[ 2 * auc_pilot_carriers_list[uc_i] ], ALPHA_CHANNEL_EST_Q15, FRAC_PART_Q15);
		ass_H[ 2 * auc_pilot_carriers_list[uc_i] + 1 ] = mult_real_q(ass_H[ 2 * auc_pilot_carriers_list[uc_i] + 1 ], COMP_ALPHA_CHANNEL_EST_Q15,
				FRAC_PART_Q15) + mult_real_q(ass_Ypilots[ 2 * auc_pilot_carriers_list[uc_i] + 1], ALPHA_CHANNEL_EST_Q15, FRAC_PART_Q15);
	}*/
ASM_TAG(update_channel_estimate)
	MOVW R5, #ALPHA_CHANNEL_EST_Q15
	MOVW R6, #COMP_ALPHA_CHANNEL_EST_Q15
	ADD R4, R11, R10, LSL #2 /* Address of the channel estimate (Re, Im) at the frequency of the pilot carrier */
	LDR R7, [R4] /* Im, Re of ass_H at the frequency of the pilot carrier */
	SMULBB R10, R7, R6  /* Q2.30. */
	LSL R8, R8, #5 /* Re_Ypilot in Q1.31 */
	SMLABT R10, R5, R8, R10 /* COMP_ALPHA_CHANNEL_EST_Q15*Re[ass_H] + ALPHA_CHANNEL_EST_Q15*Re_Ypilot Q2.30. NO ROUNDING OF Re_Ypilot have been done before product */
	MOV R8, #VALUE_2_TO_14  /* For half-up rounding */
	ADD R10, R8, R10
	ASR R10, R10, #15 /* Q1.15 */
	SMULTB R8, R7, R6 
	LSL R9, R9, #5 /* Im_Ypilot in Q1.31 */
	SMLABT R8, R5, R9, R8 /* Q2.30*/
	MOV R9, #VALUE_2_TO_14  /* For half-up rounding */
	ADD R8, R9, R8
	PKHBT R8, R10, R8, LSL #1
	STR R8, [R4]

	SUBS R3, #1
	BNE demodulate_pilots
	
	POP {R4, R5, R6, R7, R8, R9, R10, R11, R12}
	BX LR
#if defined(__IAR_SYSTEMS_ASM__) || defined(__CC_ARM)
	LTORG
#endif


/**
 * \brief Compute the position of the pilots in a set of symbols
 *		  Prototype: void compute_pilot_position_cenelec_a_as(uint8_t uc_num_symbols, uint8_t uc_num_pilots, uint8_t uc_index_first_symbol);
 */
#if defined(__GNUC__)
.global compute_pilot_position_cenelec_a_as
.type compute_pilot_position_cenelec_a_as, %function /* Define compute_pilot_position_cenelec_a_as as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC compute_pilot_position_cenelec_a_as
 EXTERN uc_num_active_carriers_as
 EXTERN auc_carrier_state_pilot
 EXTERN auc_active_carrier_state
#elif defined(__CC_ARM)
 EXPORT compute_pilot_position_cenelec_a_as
 IMPORT uc_num_active_carriers_as
 IMPORT auc_carrier_state_pilot
 IMPORT auc_active_carrier_state
#endif

ASM_TAG(compute_pilot_position_cenelec_a_as)
	PUSH {R4-R12}

	SUB R1, R1, #1 /* uc_num_pilots - 1 */
	MOVW R4, #PILOT_FREQ_SPA_AS
	MOVW R5, #0 /* Pilot index */
	LDR R6, =uc_num_active_carriers_as
	LDRB R6, [R6]
	LDR R7, =auc_carrier_state_pilot
	LDR R11, =auc_active_carrier_state
	MOV R10, #1

ASM_TAG(loop_compute_pilot_position_cenelec_a)
	MOVW R8, #PILOT_OFFSET_CENELEC_A_AS
	ADD R8, R8, R2, LSL #1
	SMLABB R8, R4, R5, R8 /* OFFSET + (Freq_Spacing x i) + 2xsymbol_index */
	SDIV R9, R8, R6
	SMULBB R9, R9, R6
	SUB R8, R8, R9
	/*STRB R8, [R1], #1  R8=pilot index from 0...Mactive-1. Pilots are not stored in ascending order */

	LDRB R8, [R11, R8]  /* R8=pilot index from 0...num_carriers-1 */
	LDRB R12, [R7, R8]
	ORR R12, R12, R10
	STRB R12, [R7, R8] /* bit in auc_carrier_state_pilot updated */

	ADD R5, R5, #1 /* pilot index + 1*/
	CMP R1, R5
	BPL loop_compute_pilot_position_cenelec_a

	/* The pilot for one symbol have been already computed */
	SUBS R0, #1
	ITTT NE
		MOVNE R5, #0 /* Pilot index for next iteration*/
		ADDNE R2, R2, #1
		LSLNE R10, R10, #1 /* left shift to place a 1 in the bit index = index symbol */
	BNE loop_compute_pilot_position_cenelec_a

	POP {R4-R12}
	BX LR


/**
 * \brief Compute the position of the pilots in a set of symbols
 *		  Prototype: void compute_pilot_position_fcc_arib_as(uint8_t uc_num_symbols, uint8_t uc_num_pilots, uint8_t uc_index_first_symbol);
 */
#if defined(__GNUC__)
.global compute_pilot_position_fcc_arib_as
.type compute_pilot_position_fcc_arib_as, %function /* Define compute_pilot_position_fcc_arib_as as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC compute_pilot_position_fcc_arib_as
 EXTERN uc_num_active_carriers_as
 EXTERN auc_carrier_state_pilot
 EXTERN auc_active_carrier_state
#elif defined(__CC_ARM)
 EXPORT compute_pilot_position_fcc_arib_as
 IMPORT uc_num_active_carriers_as
 IMPORT auc_carrier_state_pilot
 IMPORT auc_active_carrier_state
#endif

ASM_TAG(compute_pilot_position_fcc_arib_as)
	PUSH {R4-R12}

	SUB R1, R1, #1 /* uc_num_pilots - 1 */
	MOVW R4, #PILOT_FREQ_SPA_AS
	MOVW R5, #0 /* Pilot index */
	LDR R6, =uc_num_active_carriers_as
	LDRB R6, [R6]
	LDR R7, =auc_carrier_state_pilot
	LDR R11, =auc_active_carrier_state
	MOV R10, #1
	
ASM_TAG(loop_compute_pilot_position_fcc_arib)
	MOVW R8, #PILOT_OFFSET_FCC_ARIB_AS
	ADD R8, R8, R2, LSL #1
	SMLABB R8, R4, R5, R8 /* OFFSET + (Freq_Spacing x i) + 2xsymbol_index */
	SDIV R9, R8, R6
	SMULBB R9, R9, R6
	SUB R8, R8, R9	 
	/*STRB R8, [R1], #1  R8=pilot index from 0...Mactive-1. Pilots are not stored in ascending order */

	LDRB R8, [R11, R8]  /* R8=pilot index from 0...num_carriers-1 */
	LDRB R12, [R7, R8]
	ORR R12, R12, R10
	STRB R12, [R7, R8] /* bit in auc_carrier_state_pilot updated */
	
	ADD R5, R5, #1 /* pilot index + 1*/
	CMP R1, R5
	BPL loop_compute_pilot_position_fcc_arib

	/* The pilot for one symbol have been already computed */
	SUBS R0, #1
	ITTT NE
		MOVNE R5, #0 /* Pilot index for next iteration*/
		ADDNE R2, R2, #1
		LSLNE R10, R10, #1 /* left shift to place a 1 in the bit index = index symbol */
	BNE loop_compute_pilot_position_fcc_arib
	
	POP {R4-R12}
	BX LR


/**
 * \brief Compute the sequence of the LFSR that must be used to demodulate the pilots 
 *		  Prototype: void compute_lfsr_sequence_pilots_as(uint8_t *auc_state_carrier_asm, uint8_t *auc_pn_seq_pilots, uint8_t uc_num_symbols, uint8_t uc_protocol_carriers);	
 */
#if defined(__GNUC__)
.global compute_lfsr_sequence_pilots_as
.type compute_lfsr_sequence_pilots_as, %function /* Define compute_lfsr_sequence_pilots_as as a function name */
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC compute_lfsr_sequence_pilots_as
 EXTERN auc_lfsr_pilots_as
 EXTERN pul_index_to_lfsr_as
 EXTERN uc_pointer_byte_lfsr_pilots_as
 EXTERN uc_pointer_bitpair_lfsr_pilots_as
 EXTERN uc_index_symbol_in_block
 EXTERN auc_carrier_state_pilot
#elif defined(__CC_ARM)
 EXPORT compute_lfsr_sequence_pilots_as
 IMPORT auc_lfsr_pilots_as
 IMPORT pul_index_to_lfsr_as
 IMPORT uc_pointer_byte_lfsr_pilots_as
 IMPORT uc_pointer_bitpair_lfsr_pilots_as
 IMPORT uc_index_symbol_in_block
 IMPORT auc_carrier_state_pilot
#endif

ASM_TAG(compute_lfsr_sequence_pilots_as) /* entry point for function */
	PUSH {R4-R12}
	
	LDR R12, =auc_lfsr_pilots_as
	LDR R5, =pul_index_to_lfsr_as
	LDR R5, [R5]
	LDR R5, [R5]
	LDR R10, =uc_pointer_byte_lfsr_pilots_as
	LDRB R10, [R10]
	LDR R11, =uc_pointer_bitpair_lfsr_pilots_as
	LDRB R11, [R11]
	/*MOV R10, #0  pointer to actual byte in auc_pn_seq_pilots */
	/*MOV R11, #0  pointer to pair of bits within byte */
	LDR R4, =uc_index_symbol_in_block
	LDRB R4, [R4]
	MOV R6, #1 /* to select the bit in auc_carrier_state_pilot corresponding to the actual symbol */	
	LSL R6, R6, R4
	MOVW R4, #0 /* Carrier index */	

ASM_TAG(loop_compute_lfsr_sequence_pilots)
	MOV R7, R3 /* R7= num_carriers */
	SUB R7, R7, R4
	SUB R7, R7, #1
	LDRB R7, [R0, R7] /* R7= auc_state_carrier_asm[num_carriers - 1 - uc_i] */
	AND R8, R7, #0x0003 /* R8=number of bits per modulation symbol */
	ANDS R7, R7, #0xC0 /* Check if actual carrier is inactive or notched */
	BNE	unused_carrier

	/* Check if is a data/notched or pilot carrier */
	LDR R8, =auc_carrier_state_pilot
	LDRB R9, [R8, R4]
	ANDS R8, R9, R6
	BEQ end_loop_compute_lfsr_sequence_pilots
	CMP R9, R8, LSL #1 /* If this pilot is used later in the same block, increment input LFSR index but do not take the bits */
	BPL increase_LFSR_index
		
ASM_TAG(pilot_carrier) 
		LDRB R8, [R1, R10] /* read actual byte of auc_pn_seq_pilots */
		LDRB R9, [R12, R5] /* read actual pair of bits of the LFSR */
		LSL R9, R9, R11		
		ORR R8, R8, R9 /* update the actual byte of auc_pn_seq_pilots */
		STRB R8, [R1, R10] /* update auc_pn_seq_pilots */
		ADD R11, R11, #2
		ANDS R11, R11, #0x0007 /* R11 mod 8 */
		IT EQ
		ADDEQ R10, R10, #1 /* increase pointer to auc_pn_seq_pilots */
ASM_TAG(increase_LFSR_index)
	ADD R5, R5, #2  /* increase index of LFSR and take modulo 127 */
	CMP R5, #127
	IT PL
		SUBPL R5, R5, #127
	
	B end_loop_compute_lfsr_sequence_pilots

ASM_TAG(unused_carrier) /* If carrier is inactive or notched, increase index of the LFSR pointer and take modulo 127 */
	ADD R5, R5, R8
	CMP R5, #127
	IT PL
		SUBPL R5, R5, #127

ASM_TAG(end_loop_compute_lfsr_sequence_pilots)
	ADD R4, R4, #1
	CMP R4, R3
	BNE loop_compute_lfsr_sequence_pilots

	SUBS R2, #1
	ITT NE
		MOVWNE R4, #0 /* Reset carrier index for next iteration */
		LSLNE R6, R6, #1 /* to select the bit in auc_carrier_state_pilot corresponding to the actual symbol */
	BNE loop_compute_lfsr_sequence_pilots
	/* Update pointer to LFSR */
	LDR R6, =pul_index_to_lfsr_as
	LDR R6, [R6]
	STR R5, [R6]
	/* Update pointers to auc_pn_seq_pilots*/
	LDR R6, =uc_pointer_byte_lfsr_pilots_as;
	STRB R10, [R6]
	LDR R6, =uc_pointer_bitpair_lfsr_pilots_as
	STRB R11, [R6]

	POP {R4-R12}
	BX LR

#endif 

#if defined(__GNUC__)
 .end
#elif defined(__IAR_SYSTEMS_ASM__) || defined(__CC_ARM)
	END
#endif
