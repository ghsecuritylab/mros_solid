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
* File: r_ospl_memory.c
*    Memory map. For RZ/A1 RTX BSP.
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: $
* - $Date::                           $
******************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "r_ospl.h"
#include  "r_ospl_private.h"
#include "solid_type.h" //tokuyama@kmg 2016.09.15
#include "solid_mem.h"


/******************************************************************************
Macro definitions
******************************************************************************/

/* ->MISRA 17.4 : These are addresses */

#define  GS_0x00000000_EXTERNAL_START               0x00000000u
#define  GS_0x08000000_EXTERNAL_SDRAM_START         0x08000000u
#define  GS_0x20000000_INTERNAL_RAM_START           0x20000000u
#define  GS_0x40000000_EXTERNAL_MIRROR_START        0x40000000u
#define  GS_0x60000000_INTERNAL_RAM_MIRROR_START    0x60000000u
#define  GS_0x80000000_INTERNAL_REGISTERS_START     0x80000000u

/* <-MISRA 17.4 */

/* From "scatter.scat" file */
#if defined( RZ_A1L )
#define  GS_LRAM_SIZE  0x00300000u
#else  /* RZ/A1H */
#define  GS_LRAM_SIZE  0x00A00000u
#endif

#if defined( RZ_A1L_EV_BOARD )
#define  GS_SDRAM_SIZE  0x04000000u
#else
#define  GS_SDRAM_SIZE  0x08000000u
#endif


/***********************************************************************
* Macro: GS_2ND_CACHE_ATTRIBUTE_IN_L1_UNCACHED
*    R_OSPL_AXI_STRONGLY  or  R_OSPL_AXI_WRITE_BACK
*
* Description:
*    See "[area07] CS2, CS3 area (for SDRAM) (mirror)" in "ttb_init.s".
*    R_OSPL_AXI_STRONGLY   - for TTB_PARA_NORMAL_NOT_CACHE_NS (TEX=100).
*    R_OSPL_AXI_WRITE_BACK - for TTB_PARA_NORMAL_NOT_CACHE_L2_CACHE_NS (TEX=111).
************************************************************************/
#define  GS_2ND_CACHE_ATTRIBUTE_IN_L1_UNCACHED  R_OSPL_AXI_STRONGLY


/******************************************************************************
Typedef definitions
******************************************************************************/

/***********************************************************************
* Constants: gs_operation_t
*
*    : GS_STRAIGHT
*    : GS_ERROR
*    : GS_SLIDE
************************************************************************/
typedef enum { GS_STRAIGHT,  GS_ERROR,  GS_SLIDE }  gs_operation_t;


/***********************************************************************
* Structure: gs_table_line_m_t
*    Line in Table. Value is <gs_operation_t> type.
************************************************************************/
typedef struct st_gs_table_line_m_t  gs_table_line_m_t;
struct st_gs_table_line_m_t
{

    /* Variable: Address */
    uintptr_t  Address;

    /* Variable: Value */
    gs_operation_t  Value;
};


/***********************************************************************
* Structure: gs_table_line_i_t
*    Line in Table. Value is integer type.
************************************************************************/
typedef struct st_gs_table_line_i_t  gs_table_line_i_t;
struct st_gs_table_line_i_t
{

    /* Variable: Address */
    uintptr_t  Address;

    /* Variable: Value */
    int_fast32_t  Value;
};


/***********************************************************************
* Structure: gs_table_line_c_t
*    Line in Table. Value is <r_ospl_axi_cache_attribute_t> type.
************************************************************************/
typedef struct st_gs_table_line_c_t  gs_table_line_c_t;
struct st_gs_table_line_c_t
{

    /* Variable: Address */
    uintptr_t  Address;

    /* Variable: Value */
    r_ospl_axi_cache_attribute_t  Value;
};


/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/

/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/

/******************************************************************************
Private global variables and functions
******************************************************************************/

/******************************************************************************
* Implement: R_OSPL_ToPhysicalAddress
******************************************************************************/
// SOLID_MEM_VA2PA() で置き換える tokuyama@kmg 2016.09.14
errnum_t  R_OSPL_ToPhysicalAddress( const volatile void* in_Address, uintptr_t* out_PhysicalAddress )
{
    errnum_t  e;
    int ercd;
#if 0 // tokuyama@kmg 2016.09.14
    /* ->MISRA 11.3 */ /* ->SEC R2.7.1 */
    uintptr_t const  address = (uintptr_t) in_Address;
    /* <-MISRA 11.3 */ /* <-SEC R2.7.1 */

    IF_DQ( out_PhysicalAddress == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }

    *out_PhysicalAddress = address;
#else
    ercd = SOLID_MEM_VA2PA ((SOLID_ADDRESS)in_Address, (SOLID_ADDRESS*)out_PhysicalAddress);
    if (ercd != SOLID_ERR_OK)
    {
        e=E_OTHERS;
        goto fin;
    } 
#endif /* 0 */
    e=0;
fin:
    return  e;
}


/******************************************************************************
* Implement: R_OSPL_ToCachedAddress
******************************************************************************/
errnum_t  R_OSPL_ToCachedAddress( const volatile void* in_Address, void* out_CachedAddress )
{
    /* ->MISRA 11.3 */ /* ->SEC R2.7.1 */
    /* ->QAC 0289 */ /* ->QAC 1002 : Image$$BEGIN_OF_NOCACHE_RAM_BARRIER$$Base */
#if 0 //メモリリージョンが Cached か Uncached かは、SOLID_MEM_Init() での設定なので、途中変更不可 tokuyama@kmg 2016.09.15
#if  IS_MBED_USED
    errnum_t   e;
    uintptr_t  address = (uintptr_t) in_Address;

    const uintptr_t  uncached_from_cached =
        GS_0x60000000_INTERNAL_RAM_MIRROR_START - GS_0x20000000_INTERNAL_RAM_START;

    static gs_table_line_m_t  table[6];
    gs_operation_t            operation;

    IF_DQ( out_CachedAddress == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }


    if ( table[1].Address == NULL )
    {
        uintptr_t  address = (uintptr_t) in_Address;
        uintptr_t  ad_0x20X00000_internal_RAM_2nd_half_start =
            (uintptr_t) &Image$$RW_DATA_NC$$Base - uncached_from_cached;
        uintptr_t  ad_0x60X00000_internal_RAM_2nd_half_start =
            (uintptr_t) &Image$$RW_DATA_NC$$Base;

        gs_table_line_m_t*  t = table;

        t[0].Address = GS_0x00000000_EXTERNAL_START;
        t[0].Value = GS_STRAIGHT;
        t[1].Address = ad_0x20X00000_internal_RAM_2nd_half_start;
        t[1].Value = GS_ERROR;
        t[2].Address = GS_0x20000000_INTERNAL_RAM_START + GS_LRAM_SIZE;
        t[2].Value = GS_STRAIGHT;
        t[3].Address = GS_0x40000000_EXTERNAL_MIRROR_half_START;
        t[3].Value = GS_ERROR;
        t[4].Address = GS_0x60000000_INTERNAL_RAM_MIRROR_START;
        t[4].Value = GS_SLIDE;
        t[5].Address = ad_0x60X00000_internal_RAM_2nd_start;
        t[5].Value = GS_ERROR;
    }


    if ( address < table[1].Address )
    {
        operation = table[0].Value;
    }
    else if ( address < table[2].Address )
    {
        operation = table[1].Value;
    }
    else if ( address < table[3].Address )
    {
        operation = table[2].Value;
    }
    else if ( address < table[4].Address )
    {
        operation = table[3].Value;
    }
    else if ( address < table[5].Address )
    {
        operation = table[4].Value;
    }
    else
    {
        operation = table[5].Value;
    }
    R_STATIC_ASSERT( R_COUNT_OF( table ) == 6, "" );


    if ( operation == GS_STRAIGHT )
    {
        e = 0;
    }
    else if ( operation == GS_SLIDE )
    {
        address -= uncached_from_cached;
        e = 0;
    }
    else
    {
        e = E_ACCESS_DENIED;
    }
#else
    errnum_t  e;
    uintptr_t const  address = (uintptr_t) in_Address;

    static gs_table_line_m_t  table[4];
    gs_operation_t            operation;

    IF_DQ( out_CachedAddress == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }


    if ( table[1].Address == NULL )
    {
        uintptr_t  ad_0x20X00000_internal_RAM_2nd_half_start =
            (uintptr_t)&Image$$BEGIN_OF_NOCACHE_RAM_BARRIER$$ZI$$Limit;

        gs_table_line_m_t*  t = table;

        t[0].Address = GS_0x00000000_EXTERNAL_START;
        t[0].Value = GS_STRAIGHT;
        t[1].Address = ad_0x20X00000_internal_RAM_2nd_half_start;
        t[1].Value = GS_ERROR;
        t[2].Address = GS_0x20000000_INTERNAL_RAM_START + GS_LRAM_SIZE;
        t[2].Value = GS_STRAIGHT;
        t[3].Address = GS_0x40000000_EXTERNAL_MIRROR_START;
        t[3].Value = GS_ERROR;
    }


    if ( address < table[1].Address )
    {
        operation = table[0].Value;
    }
    else if ( address < table[2].Address )
    {
        operation = table[1].Value;
    }
    else if ( address < table[3].Address )
    {
        operation = table[2].Value;
    }
    else
    {
        operation = table[3].Value;
    }
    R_STATIC_ASSERT( R_COUNT_OF( table ) == 4, "" );


    if ( operation == GS_STRAIGHT )
    {
        e = 0;
    }
    else
    {
        e = E_ACCESS_DENIED;
    }
#endif

    *(void**) out_CachedAddress = (void*) address;

fin:
#else
	errnum_t e;
	e = 0;

#endif /* 0 */
    return  e;
    /* <-QAC 0289 *//* <-QAC 1002 */
    /* <-MISRA 11.3 */ /* <-SEC R2.7.1 */
}


/******************************************************************************
* Implement: R_OSPL_ToUncachedAddress
******************************************************************************/
errnum_t  R_OSPL_ToUncachedAddress( const volatile void* in_Address, void* out_UncachedAddress )
{

	errnum_t e;
	e = 0;
#if 0 // メモリリージョンが Cached か Uncached かは、SOLID_MEM_Init() での設定なので、途中変更不可 tokuyama@kmg
    /* ->MISRA 11.3 */ /* ->SEC R2.7.1 */
    /* ->QAC 0289 */ /* ->QAC 1002 : Image$$BEGIN_OF_CACHED_RAM_BARRIER$$Base */
#if  IS_MBED_USED
    errnum_t  e;
    const uintptr_t  uncached_from_cached =
        GS_0x60000000_INTERNAL_RAM_MIRROR_START - GS_0x20000000_INTERNAL_RAM_START;

    static gs_table_line_m_t  table[8];
    gs_operation_t            operation;

    IF_DQ( out_UncachedAddress == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }

    if ( table[1].Address == NULL )
    {
        uintptr_t  address = (uintptr_t) in_Address;
        uintptr_t  ad_0x20X00000_internal_RAM_2nd_half_start =
            (uintptr_t) &Image$$RW_DATA_NC$$Base - uncached_from_cached;
        uintptr_t  ad_0x60X00000_internal_RAM_2nd_half_start =
            (uintptr_t) &Image$$RW_DATA_NC$$Base;

        gs_table_line_m_t*  t = table;

        t[0].Address = GS_0x00000000_EXTERNAL_START;
        t[0].Value = GS_ERROR;
        t[1].Address = ad_0x20X00000_internal_RAM_2nd_half_start;
        t[1].Value = GS_SLIDE;
        t[2].Address = GS_0x20000000_INTERNAL_RAM_START + GS_LRAM_SIZE;
        t[2].Value = GS_ERROR;
        t[3].Address = GS_0x40000000_EXTERNAL_MIRROR_half_START;
        t[3].Value = GS_STRAIGHT;
        t[4].Address = GS_0x60000000_INTERNAL_RAM_MIRROR_START;
        t[4].Value = GS_ERROR;
        t[5].Address = ad_0x60X00000_internal_RAM_2nd_start;
        t[5].Value = GS_STRAIGHT;
        t[6].Address = GS_0x60000000_INTERNAL_RAM_MIRROR_START + GS_LRAM_SIZE;
        t[6].Value = GS_ERROR;
        t[7].Address = GS_0x80000000_INTERNAL_REGISTERS_START;
        t[7].Value = GS_STRAIGHT;
    }


    if ( address < table[1].Address )
    {
        operation = table[0].Value;
    }
    else if ( address < table[2].Address )
    {
        operation = table[1].Value;
    }
    else if ( address < table[3].Address )
    {
        operation = table[2].Value;
    }
    else if ( address < table[4].Address )
    {
        operation = table[3].Value;
    }
    else if ( address < table[5].Address )
    {
        operation = table[4].Value;
    }
    else if ( address < table[6].Address )
    {
        operation = table[5].Value;
    }
    else if ( address < table[7].Address )
    {
        operation = table[6].Value;
    }
    else
    {
        operation = table[7].Value;
    }
    R_STATIC_ASSERT( R_COUNT_OF( table ) == 8, "" );


    if ( operation == GS_STRAIGHT )
    {
        e = 0;
    }
    else if ( operation == GS_SLIDE )
    {
        address += uncached_from_cached;
        e = 0;
    }
    else
    {
        e = E_ACCESS_DENIED;
    }
#else
    errnum_t  e;
    uintptr_t const  address = (uintptr_t) in_Address;
    const uintptr_t  uncached_from_cached =
        GS_0x60000000_INTERNAL_RAM_MIRROR_START - GS_0x20000000_INTERNAL_RAM_START;

    static gs_table_line_m_t  table[8];
    gs_operation_t            operation;

    IF_DQ( out_UncachedAddress == NULL )
    {
        e=E_OTHERS;
        goto fin;
    }


    if ( table[1].Address == NULL )
    {
        uintptr_t  ad_0x20X00000_internal_RAM_2nd_half_start =
            (uintptr_t)&Image$$BEGIN_OF_NOCACHE_RAM_BARRIER$$ZI$$Limit;
        uintptr_t  ad_0x60X00000_internal_RAM_2nd_half_start =
            ad_0x20X00000_internal_RAM_2nd_half_start + uncached_from_cached;

        gs_table_line_m_t*  t = table;

        t[0].Address = GS_0x00000000_EXTERNAL_START;
        t[0].Value = GS_ERROR;
        t[1].Address = ad_0x20X00000_internal_RAM_2nd_half_start;
        t[1].Value = GS_STRAIGHT;
        t[2].Address = GS_0x20000000_INTERNAL_RAM_START + GS_LRAM_SIZE;
        t[2].Value = GS_ERROR;
        t[3].Address = GS_0x40000000_EXTERNAL_MIRROR_START;
        t[3].Value = GS_STRAIGHT;
        t[4].Address = GS_0x60000000_INTERNAL_RAM_MIRROR_START;
        t[4].Value = GS_ERROR;
        t[5].Address = ad_0x60X00000_internal_RAM_2nd_half_start;
        t[5].Value = GS_STRAIGHT;
        t[6].Address = GS_0x60000000_INTERNAL_RAM_MIRROR_START + GS_LRAM_SIZE;
        t[6].Value = GS_ERROR;
        t[7].Address = GS_0x80000000_INTERNAL_REGISTERS_START;
        t[7].Value = GS_STRAIGHT;
    }


    if ( address < table[1].Address )
    {
        operation = table[0].Value;
    }
    else if ( address < table[2].Address )
    {
        operation = table[1].Value;
    }
    else if ( address < table[3].Address )
    {
        operation = table[2].Value;
    }
    else if ( address < table[4].Address )
    {
        operation = table[3].Value;
    }
    else if ( address < table[5].Address )
    {
        operation = table[4].Value;
    }
    else if ( address < table[6].Address )
    {
        operation = table[5].Value;
    }
    else if ( address < table[7].Address )
    {
        operation = table[6].Value;
    }
    else
    {
        operation = table[7].Value;
    }
    R_STATIC_ASSERT( R_COUNT_OF( table ) == 8, "" );


    if ( operation == GS_STRAIGHT )
    {
        e = 0;
    }
    else
    {
        e = E_ACCESS_DENIED;
    }
#endif

    *(void**) out_UncachedAddress = (void*) address;

fin:
#endif /* 0 */
	return e;
    /* <-QAC 0289 *//* <-QAC 1002 */
    /* <-MISRA 11.3 */ /* <-SEC R2.7.1 */
}


/******************************************************************************
* Implement: R_OSPL_MEMORY_GetLevelOfFlush
******************************************************************************/
errnum_t  R_OSPL_MEMORY_GetLevelOfFlush( const void* in_Address, int_fast32_t* out_Level )
{
    static const gs_table_line_i_t  table[] =
    {
        /* .Address                                       .Value */
        { GS_0x00000000_EXTERNAL_START,                        0 },
        { GS_0x08000000_EXTERNAL_SDRAM_START,                  1 },
        { GS_0x08000000_EXTERNAL_SDRAM_START + GS_SDRAM_SIZE,  0 },
        { GS_0x20000000_INTERNAL_RAM_START,                    1 },
        { GS_0x20000000_INTERNAL_RAM_START + GS_LRAM_SIZE,     0 }
    };

    uintptr_t  address = (uintptr_t) in_Address;


    if ( address < table[1].Address )
    {
        *out_Level = table[0].Value;
    }
    else if ( address < table[2].Address )
    {
        *out_Level = table[1].Value;
    }
    else if ( address < table[3].Address )
    {
        *out_Level = table[2].Value;
    }
    else if ( address < table[4].Address )
    {
        *out_Level = table[3].Value;
    }
    else
    {
        *out_Level = table[4].Value;
    }
    R_STATIC_ASSERT( R_COUNT_OF( table ) == 5, "" );

    return  0;
}


/******************************************************************************
* Implement: R_OSPL_MEMORY_GetMaxLevelOfFlush
******************************************************************************/
int_fast32_t  R_OSPL_MEMORY_GetMaxLevelOfFlush()
{
    return  1;
}


/******************************************************************************
* Implement: R_OSPL_AXI_Get2ndCacheAttribute
******************************************************************************/
errnum_t  R_OSPL_AXI_Get2ndCacheAttribute( uintptr_t const  in_PhysicalAddress,
        r_ospl_axi_cache_attribute_t* const  out_CacheAttribute )
{
    uintptr_t  address = in_PhysicalAddress;

    static const gs_table_line_c_t  table[] =
    {
        /* .Address                                  .Value */
        { GS_0x00000000_EXTERNAL_START,      R_OSPL_AXI_WRITE_BACK },
        { GS_0x20000000_INTERNAL_RAM_START,  R_OSPL_AXI_CACHE_ZERO },
        /* If external I/O existed, "*out_CacheAttribute = R_AXI_DEVICE;" */
    };


    if ( address < table[1].Address )
    {
        *out_CacheAttribute = table[0].Value;
    }
    else
    {
        *out_CacheAttribute = table[1].Value;
    }
    R_STATIC_ASSERT( R_COUNT_OF( table ) == 2, "" );

    return  0;
}


/***********************************************************************
* Implement: R_OSPL_AXI_GetProtection
************************************************************************/
errnum_t  R_OSPL_AXI_GetProtection( uintptr_t const  in_PhysicalAddress,
                                    r_ospl_axi_protection_t* const  out_Protection )
{
    *out_Protection = R_OSPL_AXI_NON_SECURE;  /* Same as CPU(TTB) NS bit */

    return  0;
}


