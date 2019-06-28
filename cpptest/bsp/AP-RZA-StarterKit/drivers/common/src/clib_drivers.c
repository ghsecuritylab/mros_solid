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
* Copyright (C) 2012 - 2015 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/***********************************************************************
* File: clib_drivers.c
* - $Module: CLibCommon $ $PublicVersion: 0.90 $ (=CLIB_VERSION)
* - $Rev: 30 $
* - $Date:: 2014-02-13 21:21:47 +0900#$
* - Description: Common code.
************************************************************************/


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "r_ospl.h"
#include  "clib_drivers.h"


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
* Implement: R_int32_t_to_int8_t
*    Cast with range check
*
* Arguments:
*    input  - Input value
*    output - Output value
*
* Return Value:
*    Error Code. 0=No Error.
************************************************************************/
errnum_t  R_int32_t_to_int8_t( int32_t const  input, int8_t* const  output )
{
    errnum_t  e;

    IF ( (input < INT8_MIN)  ||  (input > INT8_MAX) )
    {
        e=E_OTHERS;
        goto fin;
    }
    IF_DQ( output == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }

    *output = (int8_t) input;

    e=0;
fin:
    return  e;
}



