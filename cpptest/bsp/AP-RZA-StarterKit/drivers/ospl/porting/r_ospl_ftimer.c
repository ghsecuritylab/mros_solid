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
/******************************************************************************
* File: r_ospl_ftimer.c
*    Free-run Timer. For RZ/A1.
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: $
* - $Date::                           $
******************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "r_typedefs.h"
#include  "iodefine.h"
#include  "iobitmasks/cpg_iobitmask.h"
#include  "r_ospl.h"


/******************************************************************************
Macro definitions
******************************************************************************/

/***********************************************************************
* Define: R_OSPL_EXTENSION_TIME_SECOND
************************************************************************/
#define  R_OSPL_EXTENSION_TIME_SECOND  15


/******************************************************************************
Typedef definitions
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
* Implement: R_OSPL_FTIMER_InitializeIfNot
************************************************************************/
errnum_t  R_OSPL_FTIMER_InitializeIfNot( r_ospl_ftimer_spec_t* const  out_Specification )
{
    /* ->SEC M1.11.1 : Can not const "is_initialized" because timing */
    bool_t  is_initialized;
    /* <-SEC M1.11.1 */
#if  R_OSPL_FTIMER_IS == R_OSPL_FTIMER_IS_MTU2_1_2
    enum { bits_CST1_CST2 = 0x06 };
#endif

    /* ->QAC 0306 */
#if  R_OSPL_FTIMER_IS == R_OSPL_FTIMER_IS_OSTM0
    struct st_ostm* const  reg_OSTM = &OSTM0;
#elif  R_OSPL_FTIMER_IS == R_OSPL_FTIMER_IS_OSTM1
    struct st_ostm* const  reg_OSTM = &OSTM1;
#endif
    /* <-QAC 0306 */

    /* ->QAC 0306 */
#if  R_OSPL_FTIMER_IS == R_OSPL_FTIMER_IS_OSTM0
    R_OSPL_SET_TO_8_BIT_REGISTER( &CPG.STBCR5, CPG_STBCR5_MSTP51,
                                  CPG_STBCR5_MSTP51_SHIFT, 0 );

#elif  R_OSPL_FTIMER_IS == R_OSPL_FTIMER_IS_OSTM1
    R_OSPL_SET_TO_8_BIT_REGISTER( &CPG.STBCR5, CPG_STBCR5_MSTP50,
                                  CPG_STBCR5_MSTP50_SHIFT, 0 );

#elif  R_OSPL_FTIMER_IS == R_OSPL_FTIMER_IS_MTU2_1_2
    R_OSPL_SET_TO_8_BIT_REGISTER( &CPG.STBCR3, CPG_STBCR3_MSTP33,
                                  CPG_STBCR3_MSTP33_SHIFT, 0 );
#endif

#if  R_OSPL_FTIMER_IS == R_OSPL_FTIMER_IS_MTU2_1_2
    is_initialized = IS_ALL_BITS_NOT_SET( MTU2.TSTR, bits_CST1_CST2 );
#else
    is_initialized = ( (int_fast32_t) reg_OSTM->OSTMnTE == 0 );
#endif
    /* <-QAC 0306 */

    if ( IS( is_initialized ) )    /* Integer Promotions */
    {
#if  R_OSPL_FTIMER_IS != R_OSPL_FTIMER_IS_MTU2_1_2
        enum {  free_running_no_interrupt = 2 };
#endif
        bool_t  was_all_enabled; /* = false; */ /* QAC 3197 */

        was_all_enabled = R_OSPL_DisableAllInterrupt();

#if  R_OSPL_FTIMER_IS == R_OSPL_FTIMER_IS_MTU2_1_2
        {
            enum { external_clock_TCLKD = 0x07u };

            /* ->QAC 0306 */

            /* Channel 1 */
            MTU2.TCR_1  = (uint8_t) external_clock_TCLKD;  /* overflow of the timer2 */
            MTU2.TMDR_1 = 0;
            MTU2.TIOR_1 = 0;
            MTU2.TIER_1 = 0;
            MTU2.TSR_1  = 0;
            MTU2.TCNT_1 = 0;

            /* Channel 2 */
            MTU2.TCR_2  = 0;  /* 33MHz */
            MTU2.TMDR_2 = 0;
            MTU2.TIOR_2 = 0;
            MTU2.TIER_2 = 0;
            MTU2.TSR_2  = 0;
            MTU2.TCNT_2 = 0;

            /* Timer start */
            /* MTU2.TSTR |= bits_CST1_CST2; */
            {
                uint8_t const  value = MTU2.TSTR;
                MTU2.TSTR = (uint8_t)( (uint_fast32_t) value | bits_CST1_CST2 );
            } /* QAC 2100 */

            /* <-QAC 0306 */
        }
#else
        if ( (int_fast32_t) reg_OSTM->OSTMnTE == 0 )    /* Integer Promotions */
        {
            reg_OSTM->OSTMnCTL = free_running_no_interrupt;
            reg_OSTM->OSTMnTS = 1;
        }
#endif

        if ( IS( was_all_enabled ) )
        {
            R_OSPL_EnableAllInterrupt();
        }
    }

    R_OSPL_FTIMER_GetSpecification( out_Specification );

    return  0;
}


/***********************************************************************
* Implement: R_OSPL_FTIMER_GetSpecification
************************************************************************/
void  R_OSPL_FTIMER_GetSpecification( r_ospl_ftimer_spec_t* const  out_Specification )
{
    if ( out_Specification != NULL )
    {
        enum { msec_numerator   = 1 };      /* SEC M1.10.1, QAC-3132 */
        enum { msec_denominator = 33333 };  /* SEC M1.10.1, QAC-3132 */
        static const uint32_t  max_count = 0xFFFFFFFFu;  /* SEC M1.10.1, QAC-3132 */
        static const uint32_t  extension_of_count =
            (R_OSPL_EXTENSION_TIME_SECOND * 1000 * msec_denominator) / msec_numerator;
        /* SEC M1.10.1, QAC-3132 */

        out_Specification->msec_Numerator   = msec_numerator;
        out_Specification->msec_Denominator = msec_denominator;
        out_Specification->MaxCount = max_count;
        out_Specification->ExtensionOfCount = extension_of_count;
    }
}


/***********************************************************************
* Implement: R_OSPL_FTIMER_Get
************************************************************************/
uint32_t  R_OSPL_FTIMER_Get(void)
{
#if  R_OSPL_FTIMER_IS == R_OSPL_FTIMER_IS_MTU2_1_2
    enum { num_16bit = 16 }; /* SEC M1.10.1, QAC-3132 */
    /* ->QAC 0306 */

    uint32_t  now_high = (uint32_t)(MTU2.TCNT_1);
    uint32_t  not_low  = (uint32_t)(MTU2.TCNT_2);

    while( now_high != (uint32_t)(MTU2.TCNT_1) )
    {
        /* If higher byte was changed while reading lower byte, re-read. */
        now_high = (uint32_t)(MTU2.TCNT_1);
        not_low  = (uint32_t)(MTU2.TCNT_2);
    }

    return  ( now_high << num_16bit ) | not_low;

    /* <-QAC 0306 */
#else

    /* ->QAC 0306 */
#if  R_OSPL_FTIMER_IS == R_OSPL_FTIMER_IS_OSTM0
    struct st_ostm* const  reg_OSTM = &OSTM0;
#elif  R_OSPL_FTIMER_IS == R_OSPL_FTIMER_IS_OSTM1
    struct st_ostm* const  reg_OSTM = &OSTM1;
#else
#error
#endif
    /* <-QAC 0306 */

    ASSERT_R( reg_OSTM->OSTMnCMP == 0x0000000u,  R_NOOP() );
    /* If "OSTMnCMP != 0", OSTMnCNT is set from 0 to OSTMnCMP-1 */

    return  reg_OSTM->OSTMnCNT;

#endif
}


