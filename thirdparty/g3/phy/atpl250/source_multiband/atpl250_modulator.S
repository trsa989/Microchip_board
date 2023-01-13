#if defined(__GNUC__)
 .syntax unified	//switch to a nicer syntax
#elif defined(__IAR_SYSTEMS_ASM__)
 SECTION atpl250_modulator:CODE:NOROOT(8)
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
/*
 * PAYLOAD_MODULATION
 *
 * Created: 17/02/2015 14:15:28
 * Author: miguel.arribas
 */ 
 
 #if defined(__GNUC__)
 .global PAYLOAD_MODULATION	
 .type PAYLOAD_MODULATION STT_FUNC
#elif defined(__IAR_SYSTEMS_ASM__)
 PUBLIC PAYLOAD_MODULATION
 EXTERN ul_mod_vector_aux_asm
 EXTERN uc_aux_pointer_asm
 EXTERN ul_mod_input_pointer_asm
 EXTERN uc_ini_lsfr_payload_asm
 EXTERN auc_state_carrier_asm
 EXTERN auc_reference_payload_asm
 EXTERN ul_mod_output_pointer_asm
 EXTERN auc_lsfr_asm
 EXTERN uc_protocol_carriers_asm
#elif defined(__CC_ARM)
 EXPORT PAYLOAD_MODULATION
 IMPORT ul_mod_vector_aux_asm
 IMPORT uc_aux_pointer_asm
 IMPORT ul_mod_input_pointer_asm
 IMPORT uc_ini_lsfr_payload_asm
 IMPORT auc_state_carrier_asm
 IMPORT auc_reference_payload_asm
 IMPORT ul_mod_output_pointer_asm
 IMPORT auc_lsfr_asm
 IMPORT uc_protocol_carriers_asm
#endif

ASM_TAG(PAYLOAD_MODULATION)

	//PUSH {r0-r12,r14}
	STMFD SP!,{r4-r11,lr};

	LDR R0, =ul_mod_vector_aux_asm
	LDR R0, [R0, #0]
	LDR R1, =uc_aux_pointer_asm		//"puntero" a la palabra dentro de ul_vector_aux
	LDRB R1, [R1, #0]
	LDR R2, =ul_mod_input_pointer_asm		//puntero para recorrer auc_modulator_in_pay
	LDR R2, [R2, #0]
	LDR R3, =uc_ini_lsfr_payload_asm
	LDRB R3, [R3, #0]
	LDR R4, =auc_state_carrier_asm
	LDR R6, =auc_reference_payload_asm
	LDR R7, =ul_mod_output_pointer_asm		//puntero para recorrer auc_modulator_out_pay_v2
	LDR R7, [R7, #0]
	LDR R8, =auc_lsfr_asm
	LDR R9, =uc_protocol_carriers_asm		
	LDRB R9, [R9, #0]
	MOV R10, #0					//uc_rotation = 0

ASM_TAG(start_payload_modulation)

	LDRB R11, [R4, R9]					//R11 = auc_state_carrier[uc_n]
	CMP R11, #32
	BPL static_notching_payload_modulation  //Salta si (R11-32)>=0

	//****************************************************************************************
	//***                                DATA CARRIER                                      ***
	//****************************************************************************************
	CMP R1, #0
	BNE not_loading_payload_modulation
	AND R11, R11, #3					//R11 = auc_state_carrier[uc_n] & 0x03 = uc_modulation_payload
	//Se cargan bytes de auc_modulator_in_pay en ul_vector_aux (3 si 8PSK, 2 si QPSK, 1 si BPSK)
	LDRB R12, [R2], #1
	EOR R0, R12, R0, LSL #8
	CMP R11, #2
	ITTE PL //if (R11 >= 2) {
		LDRBPL R12, [R2], #1
		EORPL R0, R12, R0, LSL #8
	// } else {
		LSLMI R0, R0, #8
	//}
	CMP R11, #3
	ITTE PL //if (R11 >= 3) {
		LDRBPL R12, [R2], #1
		EORPL R0, R12, R0, LSL #8
	// } else {
		LSLMI R0, R0, #8
	//}
	LSL R0, R0, #8
	RBIT R0, R0								//Ahora el primer bit esta en el LSB

ASM_TAG(not_loading_payload_modulation)
	CMP R1, #7
	ITE EQ //if (uc_p == 7) {
		MOVEQ R1, #0
	//} else {
		ADDNE R1, R1, #1
	//}

	//Se calcula el punto de la modulación
	AND R11, R11, #3					//R11 = auc_state_carrier[uc_n] & 0x03 = uc_modulation_payload
	CMP R11, #2
	IT MI //if (R11 < 2) {
		BMI point_bpsk_modulation
	//}
	CMP R11, #3
	IT PL //if (R11 >= 3) {
		BPL point_8psk_modulation
	//}

//point_qpsk_modulation:
	AND R11, R0, #3								//R11 = R0[1:0]
	//Se pasa a codificación Grey
	LSR R12, R11, #1
	EOR R11, R11, R12
	//Se multiplica por 4
	LSL R11, R11, #2
	//Se desplaza ul_vector_aux para quitar los 2 LSB ya procesados
	LSR R0, R0, #2			
	B end_point_modulation

ASM_TAG(point_bpsk_modulation)
	AND R11, R0, #1								//R11 = R0[0]
	LSL R11, R11, #3							//R11 = R11<<3
	//Se desplaza ul_vector_aux para quitar el LSB ya procesado
	LSR R0, R0, #1					
	B end_point_modulation
	
ASM_TAG(point_8psk_modulation)
	AND R11, R0, #7							//R11 = R0[2:0]
	AND R12, R0, #4							//R12 = R0[2] & "00"
	EOR R11, R11, R12, LSR #2				//R11 = R11 xor (R12>>2)
	AND R12, R11, #6						//R12 = R11[2:1] & "0"
	EOR R11, R12, R11, LSL #1				//punto = R11 = (R11<<1) xor R12
	//Se desplaza ul_vector_aux para quitar los 3 LSB ya procesados
	LSR R0, R0, #3							
		
ASM_TAG(end_point_modulation)
	
	//Se calcula uc_modulation
	LDRB R12, [R6, R9]						//R12 = auc_reference_payload[uc_n]
	ADD R11, R11, R12						//R11 = punto + auc_reference_payload[uc_n]
	AND R11, R11, #15						//uc_modulation = R11 = mod(punto + auc_reference_payload(uc_n), 16);

	//Se actualiza uc_rotation
	LDRB R12, [R4, R9]						//R12 = auc_state_carrier[uc_n]
	CMP R10, #1
	ITTT EQ //if (uc_rotation == 1) {
		ORREQ R12, R12, #32
		STRBEQ R12, [R4, R9]				//auc_state_carrier(uc_n) += 32;
		ANDEQ R12, R12, #223
	//}

	// *** START AÑADIDO ESTO PARA LOS PILOTOS CONSECUTIVOS ***
	CMP R10, #4
	ITTTT EQ //if (uc_rotation == 4) {
		ORREQ R12, R12, #32
		STRBEQ R12, [R4, R9]				//auc_state_carrier(uc_n) += 32;
		ANDEQ R12, R12, #223
		SUBEQ R10, R10, #2
	//}
	CMP R10, #1
	// *** FIN AÑADIDO ESTO PARA LOS PILOTOS CONSECUTIVOS ***

	IT PL //if (uc_rotation > 0) {
		SUBPL R10, R10, #1
	//}

	//Si es diferencial, se actualiza la referencia
	ANDS R12, R12, #4
	IT EQ  //if (uc_payload_coherent == 0) {
		STRBEQ R11, [R6, R9]				//auc_reference_payload[uc_n] = uc_modulation
	//}
	EOR R5, R11, R5, LSL #4
	ANDS R12, R9, #1
	IT EQ   //si R9 es par
		STRBEQ R5, [R7], #1                 //asc_modulator_out_pay_v2[us_aux] = R5  ;  us_aux+=1

/*	//Se actualiza auc_modulator_out_pay_v2
	//LDRH R11, [R5, R11, LSL #1]				//R11 = modulation_table[uc_modulation]
	STRH R11, [R7], #2						//asc_modulator_out_pay_v2[us_aux] = R11  ;  us_aux+=2*/


	B end_payload_modulation

	//****************************************************************************************
	//***                                STATIC NOTCHING                                   ***
	//****************************************************************************************
ASM_TAG(static_notching_payload_modulation)
	CMP R11, #128
	BMI dynamic_notching_payload_modulation  //Salta si (R11-128)<0
	
	//STATIC NOTCHING//
	AND R11, R11, #3					//R11 = auc_state_carrier[uc_n] & 0x03 = uc_modulation_payload
	CMP R11, #2
	IT MI //if (R11 < 2) {
		BMI static_bpsk_modulation
	//}
	CMP R11, #3
	IT PL //if (R11 >= 3) {
		BPL static_8psk_modulation
	//}

ASM_TAG(static_qpsk_modulation)
	CMP R3, #126
	ITE PL //if (uc_ini_LFSR_payload >= 126) {
		SUBPL R3, R3, #125	
	//} else {
		ADDMI R3, R3, #2
	//}
	B end_static_modulation
	
ASM_TAG(static_bpsk_modulation)
	CMP R3, #127
	ITE PL //if (uc_ini_LFSR_payload >= 127) {
		SUBPL R3, R3, #126		
	//} else {
		ADDMI R3, R3, #1
	//}
	B end_static_modulation
	
ASM_TAG(static_8psk_modulation)
	CMP R3, #125
	ITE PL //if (uc_ini_LFSR_payload >= 125) {
		SUBPL R3, R3, #124		
	//} else {
		ADDMI R3, R3, #3
	//}

ASM_TAG(end_static_modulation)
	LSL R5, R5, #4
	ANDS R12, R9, #1
	IT EQ   //si R9 es par
		STRBEQ R5, [R7], #1                 //asc_modulator_out_pay_v2[us_aux] = R5  ;  us_aux+=1
	/*STRH R11, [R7], #2			//asc_modulator_out_pay_v2[us_aux] = 0  &  us_aux+=2*/
	B end_payload_modulation


	//****************************************************************************************
	//***                                DINAMYC NOTCHING                                  ***
	//****************************************************************************************
ASM_TAG(dynamic_notching_payload_modulation)
	CMP R11, #64
	BMI pilots_payload_modulation  //Salta si (R11-64)<0
	//DYNAMIC NOTCHING

	LDRB R12, [R8, R3]					//R12 = LFSR[uc_ini_LFSR_payload]
	AND R11, R11, #3					//R11 = auc_state_carrier[uc_n] & 0x03 = uc_modulation_payload

	CMP R11, #2
	IT MI //if (R11 < 2) {
		BMI dynamic_bpsk_modulation
	//}
	CMP R11, #3
	IT PL //if (R11 >= 3) {
		BPL dynamic_8psk_modulation
	//}

//dynamic_qpsk_modulation
	LSR R12, R12, #1					//in QPSK, R12 = (LFSR[uc_ini_LFSR_payload]/2^3) & 0x03
	AND R12, R12, #15
	CMP R3, #126
	ITE PL //if (uc_ini_LFSR_payload >= 126) {
		SUBPL R3, R3, #125	
	//} else {
		ADDMI R3, R3, #2
	//}
	B end_dynamic_modulation
	
ASM_TAG(dynamic_bpsk_modulation)
	AND R12, R12, #1					//in BPSK, R11 = LFSR[uc_ini_LFSR_payload] & 0x01
	LSL R12, R12, #3					//R12 = 8*R12
	CMP R3, #127
	ITE PL //if (uc_ini_LFSR_payload >= 127) {
		SUBPL R3, R3, #126		
	//} else {
		ADDMI R3, R3, #1
	//}
	B end_dynamic_modulation
	
ASM_TAG(dynamic_8psk_modulation)
	LSR R12, R12, #5					//in 8PSK, R11 = floor(LFSR[uc_ini_LFSR_payload]/2^5)
	LSL R12, R12, #1					//R12 = 2*R12
	CMP R3, #125
	ITE PL //if (uc_ini_LFSR_payload >= 125) {
		SUBPL R3, R3, #124		
	//} else {
		ADDMI R3, R3, #3
	//}
	
ASM_TAG(end_dynamic_modulation)

	//Se calcula uc_modulation
	LDRB R11, [R6, R9]						//R11 = auc_reference_payload[uc_n]
	ADD R11, R11, R12						//R11 = R12 + auc_reference_payload[uc_n]
	AND R11, R11, #15						//uc_modulation = R11 = mod(punto + auc_reference_payload(uc_n), 16);

	//Si es diferencial, se actualiza la referencia
	LDRB R12, [R4, R9]						//R12 = auc_state_carrier[uc_n]
	ANDS R12, R12, #4
	IT EQ  //if (uc_payload_coherent == 0) {
		STRBEQ R11, [R6, R9]				//auc_reference_payload[uc_n] = uc_modulation
	//}


	EOR R5, R11, R5, LSL #4
	ANDS R12, R9, #1
	IT EQ   //si R9 es par
		STRBEQ R5, [R7], #1                 //asc_modulator_out_pay_v2[us_aux] = R5  ;  us_aux+=1
/*	//Se actualiza auc_modulator_out_pay_v2
	//LDRH R11, [R5, R11, LSL #1]				//R11 = modulation_table[uc_modulation]
	STRH R11, [R7], #2						//asc_modulator_out_pay_v2[us_aux] = R11  ;  us_aux+=2*/

	B end_payload_modulation

	//****************************************************************************************
	//***                                    PILOTS                                        ***
	//****************************************************************************************
ASM_TAG(pilots_payload_modulation)

	AND R11, R11, #223
	STRB R11, [R4, R9]						//auc_state_carrier[uc_n] &= 0xDF

	LDRB R11, [R8, R3]			//R11 = LFSR[uc_ini_LFSR_payload]
	LSR R11, R11, #1			//R11 = floor(LFSR[uc_ini_LFSR_payload]/2)
	AND R11, R11, #15			//uc_point = R11 = mod(floor(LFSR[uc_ini_LFSR_payload]/2), 2^4);

	//Se calcula uc_modulation
	LDRB R12, [R6, R9]						//R12 = auc_reference_payload[uc_n]
	ADD R11, R11, R12						//R11 = uc_point + auc_reference_payload[uc_n]
	AND R11, R11, #15						//uc_modulation = R11 = mod(punto + auc_reference_payload(uc_n), 16);
	EOR R5, R11, R5, LSL #4
	ANDS R12, R9, #1
	IT EQ   //si R9 es par
		STRBEQ R5, [R7], #1                 //asc_modulator_out_pay_v2[us_aux] = R5  ;  us_aux+=1

/*	//LDRH R11, [R5, R11, LSL #1]				//R11 = modulation_table[uc_modulation]
	//Se actualiza auc_modulator_out_pay_v2
	STRH R11, [R7], #2						//asc_modulator_out_pay_v2[us_aux] = R11  &  us_aux+=2*/

	CMP R3, #126
	ITE PL //if (uc_ini_LFSR_payload >= 126) {
		SUBPL R3, R3, #125		
	//} else {
		ADDMI R3, R3, #2
	//}

	// *** START AÑADIDO ESTO PARA LOS PILOTOS CONSECUTIVOS ***
	CMP R10, #1
	ITTTT EQ //if (uc_rotation == 1) {
		LDRBEQ R12, [R4, R9]					//R12 = auc_state_carrier(uc_n)
		ORREQ R12, R12, #32				
		STRBEQ R12, [R4, R9]				//auc_state_carrier(uc_n) += 32;
		SUBEQ R10, R10, #1
	//}
	ADD R10, R10, #2							//uc_rotation += 2
	// *** FIN AÑADIDO ESTO PARA LOS PILOTOS CONSECUTIVOS ***

	//MOV R10, #2								//uc_rotation = 2





ASM_TAG(end_payload_modulation)

	SUBS R9, R9, #1						//uc_n--
	BPL start_payload_modulation


	//****************************************************************************************
	//***                      ROTACION DE PILOTOS ENTRE SIMBOLOS                          ***
	//****************************************************************************************
	
	LDR R9, =uc_protocol_carriers_asm		
	LDRB R9, [R9, #0]
	//MOV R9, #35					//En CENELEC, uc_n = 35 (Inicializa el indice del bucle al número de portadoras-1)
	//MOV R9, #71					//En FCC, uc_n = 71 (Inicializa el indice del bucle al número de portadoras-1)
	//MOV R9, #53					//En ARIB, uc_n = 53 (Inicializa el indice del bucle al número de portadoras-1)
	CMP R10, #0
	BEQ end_rotation_payload_modulation

ASM_TAG(start_rotation_payload_modulation)
	LDRB R11, [R4, R9]							//R11 = auc_state_carrier[uc_n]
	CMP R11, #32
	ITT PL //if (auc_state_carrier[uc_n] < 32) {
		SUBPL R9, R9, #1						//uc_n--
		BPL start_rotation_payload_modulation
	//}

	CMP R10, #1
	ITTT EQ //if (uc_rotation == 1) {
		ORREQ R11, R11, #32
		STRBEQ R11, [R4, R9]					//auc_state_carrier[uc_n] += 32;
		BEQ end_rotation_payload_modulation		//break
	//}

	// *** START AÑADIDO ESTO PARA LOS PILOTOS CONSECUTIVOS ***
	CMP R10, #4
	ITTTT EQ //if (uc_rotation == 4) {
		LDRBEQ R12, [R4, R9]					//R12 = auc_state_carrier(uc_n)
		ORREQ R12, R12, #32				
		STRBEQ R12, [R4, R9]				//auc_state_carrier(uc_n) += 32;
		SUBEQ R10, R10, #2
	//}
	CMP R10, #1
	// *** FIN AÑADIDO ESTO PARA LOS PILOTOS CONSECUTIVOS ***

	IT PL //if (uc_rotation > 0) {
		SUBPL R10, R10, #1						//uc_rotation--
	//}

	SUB R9, R9, #1								//uc_n--
	B start_rotation_payload_modulation


ASM_TAG(end_rotation_payload_modulation)

	//Actualizar variables
	LDR R11, =ul_mod_vector_aux_asm
	STR R0, [R11, #0]
	LDR R11, =uc_aux_pointer_asm
	STRB R1, [R11, #0]
	LDR R11, =ul_mod_input_pointer_asm
	STR R2, [R11, #0]
	LDR R11, =uc_ini_lsfr_payload_asm
	STRB R3, [R11, #0]
	LDR R11, =ul_mod_output_pointer_asm
	STR R7, [R11, #0]


	 LDMFD SP!, {r4-r11,pc}

        
#if defined(__GNUC__)
 .end
#elif defined(__IAR_SYSTEMS_ASM__) || defined(__CC_ARM)
 END
#endif
