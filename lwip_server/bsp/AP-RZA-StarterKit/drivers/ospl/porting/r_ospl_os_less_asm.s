;/*******************************************************************************
;* DISCLAIMER
;* This software is supplied by Renesas Electronics Corporation and is only
;* intended for use with Renesas products. No other uses are authorized. This
;* software is owned by Renesas Electronics Corporation and is protected under
;* all applicable laws, including copyright laws.
;* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
;* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
;* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
;* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
;* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
;* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
;* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
;* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
;* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
;* Renesas reserves the right, without notice, to make changes to this software
;* and to discontinue the availability of this software. By using this software,
;* you agree to the additional terms and conditions found by accessing the
;* following link:
;* http://www.renesas.com/disclaimer
;* Copyright (C) 2012 - 2016 Renesas Electronics Corporation. All rights reserved.
;*******************************************************************************/
;/*******************************************************************************
;* $FileName: r_ospl_os_less_asm.s $
;* $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
;* $Rev: 35 $
;* $Date:: 2014-04-15 21:38:18 +0900#$
;* Description : OS Porting Layer API
;******************************************************************************/


	AREA    |.text|, CODE, READONLY


;/******************************************************************************
;* Function Name: [R_OSPL_MEMORY_GetCacheLineSize]
;* Description  :
;* Arguments    : None
;* Return Value : None
;******************************************************************************/

	EXPORT  R_OSPL_MEMORY_GetCacheLineSize

R_OSPL_MEMORY_GetCacheLineSize    FUNCTION

	ARM

	MRC     p15, 1, r0, c0, c0, 0   ;// r0 = p15(CCSIDR);    // CCSIDR: Cache Size ID Registers
	AND     r0, r0, #7              ;// r0 = r0 & 7;         // CCSIDR.LineSize: log2(cache_line_byte)-4
	ADD     r0, r0, #4              ;// r0 += 4;             //
	MOV     r1, #1                  ;// r0 = 1 << r0;        // [r0] cache_line_byte
	MOV     r0, r1, LSL r0          ;    :

	BX     lr

	ENDFUNC


;/******************************************************************************
;* Function Name: [R_OSPL_MEMORY_RangeFlush_Sub]
;* Description  :
;* Arguments    : r0: Start address
;*              : r1: Over address (Last + 1)
;*              : r2: Cache line size
;* Return Value : None
;******************************************************************************/

	EXPORT  R_OSPL_MEMORY_RangeFlush_Sub

R_OSPL_MEMORY_RangeFlush_Sub    FUNCTION

	ARM

Loop
	CMP     r0, r1                  ;// for ( r0 = r0;  r0 < r1;  r0 += r2 ) {
	BGE     Fin2                    ;//  :
	MCR     p15, 0, r0, c7, c6, 1   ;//     p15(DCIMVAC) = r0;  // Invalidate
	ADD     r0, r0, r2              ;// }
	B       Loop                    ;//  :
Fin2
	DSB
	BX     lr

	ENDFUNC

	END



