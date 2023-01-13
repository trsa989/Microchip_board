#if defined(__GNUC__)
 .syntax unified	//switch to a nicer syntax
#elif defined(__IAR_SYSTEMS_ASM__)
 SECTION atpl250_demodulator:CODE:NOROOT(8)
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

#if defined(__GNUC__)
 .global DEMOD_GET_DATA_CARRIERS
 .type DEMOD_GET_DATA_CARRIERS STT_FUNC
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC DEMOD_GET_DATA_CARRIERS
#elif defined(__CC_ARM)
 EXPORT DEMOD_GET_DATA_CARRIERS
#endif

ASM_TAG(DEMOD_GET_DATA_CARRIERS)
	PUSH {R4-R12}

	// R0 = puc_demodulator_output (input for this function)
	// R1 = puc_demodulator_output_data_carriers (output for this function)
	// R2 = puc_state_carrier
	// R3 = uc_protocol_carriers

	SUB R8, R3, #1 // R8 = uc_protocol_carriers - 1
	MOV R4, #0 // R4 = uc_word_index_input (variable to indicate the 4-bit word index in output byte)
	PUSH {R2, R8}
	MOV R5, #0 // R5 = uc_word_index_output (variable to indicate the 4-bit word index in output byte)
	MOV R6, #1 // R6 = uc_load_byte (variable to indicate if it is needed to load one byte from demodulator output)
	MOV R7, #0 // R7 = uc_pilot_rotation = 0

	ADD R2, R2, R8 // R2 = puc_state_carrier + uc_protocol_carriers - 1

ASM_TAG(demod_get_data_carriers_loop)
	LDRB R8, [R2], #-1 // R8 = uc_state_carrier
	TST R8, #0x80 // Check static notching
	BNE demod_get_data_carriers_next_carrier // Carriers with tonemask are already removed by HW (read with jump)

	TST R8, #0x60 // Check dynamic notching or pilot
	AND R9, R8, #3 // R9 = uc_modulation
	BNE demod_get_data_carriers_remove_carrier

	// Data carrier -> copy data from puc_demodulator_output to puc_demodulator_output_data_carriers
ASM_TAG(demod_get_data_carriers_copy_data_carrier)
	// Check load byte from demodulator output
	CMP R6, #1
	ITT EQ
		// Load byte
		LSREQ R12, R4, #1
		LDRBEQ R12, [R0, R12]

	// Get corresponding 4-bit word
	TST R4, #1
	ADD R4, R4, #1 // uc_word_index_input++
	ITTEE EQ
		LSREQ R10, R12, #4 // R10=R12[7:4]
		MOVEQ R6, #0 // Next time no load byte
		MOVNE R10, R12 // R10=R12[3:0]
		MOVNE R6, #1 // Next time load byte

	// Insert 4-bit word to corresponding position
	TST R5, #1
	ADD R5, R5, #1 // uc_word_index_output++
	ITEE EQ
		BFIEQ R11, R10, #4, #4 // R11[7:4] = R10[3:0]
		BFINE R11, R10, #0, #4 // R11[3:0] = R10[3:0]
		STRBNE R11, [R1], #1

	SUBS R9, R9, #1 // uc_modulation--
	BNE demod_get_data_carriers_copy_data_carrier

	// Rotate pilots
	CMP R7, #1
	BLO demod_get_data_carriers_next_carrier // uc_rotation==0

	ITTT EQ // if (uc_pilot_rotation == 1) {
		ORREQ R8, R8, #0x20
		STRBEQ R8, [R2, #1] // puc_state_carrier_rx[uc_n] |= 0x20// 
		BEQ demod_get_data_carriers_rotate_pilots_end
	// }

	//  *** CONSECUTIVE PILOTS ***
	CMP R7, #4
	ITTT EQ // if (uc_pilot_rotation == 4) {
		ORREQ R8, R8, #0x20
		STRBEQ R8, [R2, #1] // puc_state_carrier_rx(uc_n) |= 0x20// 
		MOVEQ R7, #2 // uc_pilot_rotation = 2
	// }

ASM_TAG(demod_get_data_carriers_rotate_pilots_end)
	SUB R7, R7, #1 // uc_rotation -= 1

	B demod_get_data_carriers_next_carrier

ASM_TAG(demod_get_data_carriers_remove_carrier)
	// Dynamic notching or pilot -> Increment pointer to demodulator output (carrier not removed by HW)
	ADD R4, R4, R9 // uc_word_index_input += uc_modulation

	MOV R6, #1 // Next time load byte

	// Check pilot
	TST R8, #0x20
	BEQ demod_get_data_carriers_next_carrier

	// Pilot carrier
	CMP R7, #1
	ITEE EQ // if (uc_rotation == 1) {
		// CONSECUTIVE PILOTS
		SUBEQ R7, R7, #1 // uc_pilot_rotation -= 1
	// } else {
		ANDNE R8, R8, #0xDF // Remove pilot indicator
		STRBNE R8, [R2, #1] // puc_state_carrier_rx[uc_n] &= 0xDF

	ADD R7, R7, #2 // uc_pilot_rotation += 2

ASM_TAG(demod_get_data_carriers_next_carrier)
	SUBS R3, R3, #1 // uc_protocol_carriers--
	BNE demod_get_data_carriers_loop

	// Pilots rotation between symbols
	CMP R7, #0
	POP {R8, R9}
	BEQ demod_get_data_carriers_end

ASM_TAG(demod_get_data_carriers_pilot_rotation_loop)
	LDRB R4, [R8, R9] // R4 = puc_state_carrier_rx[uc_n]
	CMP R4, #32
	ITT PL // if (puc_state_carrier_rx[uc_n] >= 32) {
		SUBPL R9, R9, #1 // uc_n--
		BPL demod_get_data_carriers_pilot_rotation_loop
	// }

	CMP R7, #1
	ITTT EQ // if (uc_pilot_rotation == 1) {
		ORREQ R4, R4, #0x20
		STRBEQ R4, [R8, R9] // puc_state_carrier_rx[uc_n] |= 0x20// 
		BEQ demod_get_data_carriers_end // break
	// }

	//  *** CONSECUTIVE PILOTS ***
	CMP R7, #4
	ITTT EQ // if (uc_pilot_rotation == 4) {
		ORREQ R4, R4, #0x20
		STRBEQ R4, [R8, R9] // puc_state_carrier_rx(uc_n) |= 0x20// 
		SUBEQ R7, R7, #2
	// }

	SUB R7, R7, #1 // uc_rotation--
	SUB R9, R9, #1 // uc_n--
	B demod_get_data_carriers_pilot_rotation_loop


ASM_TAG(demod_get_data_carriers_end)
	// Check if there is a 4-bit word pending to write
	TST R5, #1
	IT NE
		STRBNE R11, [R1]

	POP {R4-R12}
	BX LR

#if defined(__GNUC__)
 .end
#elif defined(__IAR_SYSTEMS_ASM__) || defined(__CC_ARM)
 END
#endif
