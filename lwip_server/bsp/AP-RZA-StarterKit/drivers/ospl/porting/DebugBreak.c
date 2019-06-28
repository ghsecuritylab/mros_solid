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
* File: DebugBreak.c
*    General Break Point
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "r_ospl.h"
#ifndef  R_OSPL_NDEBUG
#include  <stdio.h>
#endif

#ifdef  _SH
#pragma section     _OSPL
#endif


/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/

/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/

/***********************************************************************
* Implement: R_DebugBreak
************************************************************************/
void  R_DebugBreak( const char_t *const  File, int_fast32_t const  Line )
{
#if  R_OSPL_PRINTF == R_OSPL_PRINTF_ENABLED
    /* This code uses many stack memory */
    if ( File == NULL )
    {
        printf( "in R_DebugBreak errnum_t:0x%X\n", Line );
    }
    else
    {
        printf( "in R_DebugBreak (%d) %s\n", Line, File );
    }
#elif  R_OSPL_PRINTF == R_OSPL_PRINTF_TO_INT_LOG
    /* This code uses little stack memory */
    R_D_AddToIntLog( 0xBBBBBBBB );
    R_D_AddToIntLog( (uintptr_t) File );
    R_D_AddToIntLog( Line );
#else
    /* Do Nothing */
    R_UNREFERENCED_VARIABLE_2( File, Line );
#endif
}


