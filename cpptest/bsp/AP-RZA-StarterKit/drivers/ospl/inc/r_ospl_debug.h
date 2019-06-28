/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
* Copyright (C) 2012 - 2016 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/***********************************************************************
* File: r_ospl_debug.h
*    Debug tools provided by OSPL.
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/

#ifndef OSPL_DEBUG_H
#define OSPL_DEBUG_H


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "r_typedefs.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************
Macro definitions
******************************************************************************/

/***********************************************************************
* Define: R_OSPL_DEBUG_TOOL
*    Whether debug tools function is defined
*
* Description:
*    The value can be changed.
*    The value is 1 or 0.
************************************************************************/
#ifndef  R_OSPL_DEBUG_TOOL
#ifndef R_OSPL_NDEBUG
#define  R_OSPL_DEBUG_TOOL  1
#else
#define  R_OSPL_DEBUG_TOOL  0
#endif
#endif


/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/

/* Group: Watch */

/***********************************************************************
* Function: R_D_Add
*    Registers watching integer variable or pointer variable
*
* Arguments:
*    IndexNum   - Watch Number, 0 or more
*    Address    - Address of watching integer variable or pointer variable
*    BreakValue - Breaking value of variable when "R_D_Watch" was called
*    IsPrintf   - Whether "printf" is called, when "R_D_Watch" was called
*
* Return Value:
*    None
*
* Description:
*    <R_D_Add> and <R_D_Watch> are APIs related to watch function.
*    This debug tool is available, if <R_OSPL_DEBUG_TOOL> was set to 1.
*
*    There is not this function, if <R_OSPL_DEBUG_TOOL> macro was defined
*    to be 0. This function is available, if <R_OSPL_DEBUG_TOOL> macro was
*    defined to be 1.
*
* Example:
*    > R_D_Add( 0, &var, 0xB0, true );  // 0xB0 may be not hit
************************************************************************/
#if R_OSPL_DEBUG_TOOL
void  R_D_Add( int_fast32_t IndexNum, volatile const void* Address, uint32_t BreakValue, bool_t IsPrintf );
#endif


/***********************************************************************
* Function: R_D_Watch
*    Show and Check watching variable's value
*
* Arguments:
*    IndexNum - Watch Number, 0 or more
*
* Return Value:
*    None
*
* Description:
*    <R_D_Add> and <R_D_Watch> are APIs related to watch function.
*    This debug tool is available, if "R_OSPL_DEBUG_TOOL" was set to 1.
*    This function calls "printf" with the value of variable registered
*    by "R_D_Add" and breaks when watching variable becomes registered value.
*    This function can be called from out of scope of registered variable.
*    Then this function can be written at many places without care of the scope.
*
*    There is not this function, if <R_OSPL_DEBUG_TOOL> macro was defined
*    to be 0. This function is available, if <R_OSPL_DEBUG_TOOL> macro was
*    defined to be 1.
*
*    If internal data of the debug tool was broken, the global variable in
*    "r_ospl_debug.o" file should be move to safe address (memory map).
*
* Example:
*    > printf( 41 );  // Show the value indicated this place
*    > R_D_Watch( 0 );
************************************************************************/
#if R_OSPL_DEBUG_TOOL
void  R_D_Watch( int_fast32_t IndexNum );
#endif


/* Group: Int Log */

/***********************************************************************
* Function: R_D_AddToIntLog
*    Records to the log fast
*
* Arguments:
*    Value - Recording value
*
* Return Value:
*    None
*
* Description:
*    <R_D_AddToIntLog>, <g_IntLog> and <g_IntLogLength> are APIs related
*    to Int Log function.
*    This debug tool is available, if <R_OSPL_DEBUG_TOOL> was set to 1.
*    This function overwrites from the first of the log, if max element
*    value of "g_IntLog" was over.
*    It is recommended to record not only the showing value of variable,
*    but also the value of the identifier of the place and the value of
*    current time.
*
*    There is not this function, if <R_OSPL_DEBUG_TOOL> macro was defined
*    to be 0. This function is available, if <R_OSPL_DEBUG_TOOL> macro was
*    defined to be 1.
*
*    If internal data of the debug tool was broken, the global variable in
*    "r_ospl_debug.o" file should be move to safe address (memory map).
************************************************************************/
#if R_OSPL_DEBUG_TOOL
void  R_D_AddToIntLog( int_fast32_t Value );
#endif


/***********************************************************************
* Constant: g_IntLogCount
*    Max count of int log
************************************************************************/
#if R_OSPL_DEBUG_TOOL
enum { g_IntLogCount = 100 };
#endif


/***********************************************************************
* Variable: g_IntLog
*    Memory area of int log
************************************************************************/
#if R_OSPL_DEBUG_TOOL
extern volatile int_fast32_t   g_IntLog[ g_IntLogCount ];
#endif


/***********************************************************************
* Variable: g_IntLogLength
*    Length of recorded in <g_IntLog>
************************************************************************/
#if R_OSPL_DEBUG_TOOL
extern volatile int_fast32_t   g_IntLogLength;
#endif


/* Group: Debug Variable */

/***********************************************************************
* Constant: g_DebugVarCount
*    Count of debug variable.
************************************************************************/
#if R_OSPL_DEBUG_TOOL
enum { g_DebugVarCount = 10 };
#endif


/***********************************************************************
* Variable: g_DebugVar
*    Debug variables.
************************************************************************/
#if R_OSPL_DEBUG_TOOL
extern volatile uint_fast32_t  g_DebugVar[ g_DebugVarCount ];
#endif


/* Group: Through Counter */

/***********************************************************************
* Function: R_D_Counter
*    Count the through count
*
* Arguments:
*    in_out_Counter - Input/Output: The through counter
*    TargetCount    - The value comparing with the through counter
*    Label          - The label for "printf", NULL=printf : no output
*
* Return Value:
*    None
*
* Description:
*    This debug tool is available, if <R_OSPL_DEBUG_TOOL> was set to 1.
*    If this function was called with "TargetCount = 0", the count of
*    through is output by "printf" for each calling. If "TargetCount"
*    argument was set to the through count and restart the program,
*    when the counter was counted up to "TargetCount", this function
*    returns "true". If there were many "printf" output, set "Label = NULL".
*    At first calling, the address of the counter is output by "printf".
*    The counter can be look by the debugger.
*
* Example:
*    > { static int tc;  if ( R_D_Counter( &tc, 0, "A" ) ) {
*    > R_DEBUG_BREAK(); }}
************************************************************************/
#if R_OSPL_DEBUG_TOOL
bool_t  R_D_Counter( int_fast32_t* in_out_Counter, int_fast32_t TargetCount, char_t* Label );
#endif


/***********************************************************************
* End of File:
************************************************************************/
#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

#endif /* _OSPL_DEBUG_H */
