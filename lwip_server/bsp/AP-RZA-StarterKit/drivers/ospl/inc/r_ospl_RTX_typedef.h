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
* File: r_ospl_RTX_typedef.h
*    OS Porting Layer API for RTX
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/

#ifndef OSPL_RTX_TYPEDEF_H
#define OSPL_RTX_TYPEDEF_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************
Macro definitions
******************************************************************************/

/******************************************************************************
Typedef definitions
******************************************************************************/

/***********************************************************************
* Title: Copy_part_of_CMSIS
*    The following code is copied from "cmsys_os.h" file.
*    Because OSPL application is to prevent from depending on one OS functions.
************************************************************************/


/***********************************************************************
* Class: RTX_MailQ
************************************************************************/

/***********************************************************************
* Type: osMailQId
*
* Description:
*    See <Copy_part_of_CMSIS>.
************************************************************************/
#if IS_RTX_USED
typedef struct os_mailQ_cb *osMailQId;
#endif


/***********************************************************************
* Structure: r_ospl_rtx_osMailQDef_t
*    RTX:osMailQDef_t
*
* Description:
*    See <Copy_part_of_CMSIS>.
************************************************************************/
#if IS_RTX_USED
typedef struct r_ospl_rtx_os_mailQ_def
{
    uint32_t                queue_sz;    ///< number of elements in the queue
    uint32_t                 item_sz;    ///< size of an item
    void                       *pool;    ///< memory array for mail
} r_ospl_rtx_osMailQDef_t;  // osMailQDef_t
#endif


/* Section: Global */
/***********************************************************************
* Macro: r_ospl_rtx_osMailQDef
*    RTX:osMailQDef
*
* Description:
*    See <Copy_part_of_CMSIS>.
************************************************************************/
#if IS_RTX_USED
#if defined (osObjectsExternal)  // object is external
#define r_ospl_rtx_osMailQDef(name, queue_sz, type) \
extern const osMailQDef_t os_mailQ_def_##name
#else                            // define the object
#define r_ospl_rtx_osMailQDef(name, queue_sz, type) \
uint32_t os_mailQ_q_##name[4+(queue_sz)] = { 0 }; \
uint32_t os_mailQ_m_##name[3+((sizeof(type)+3)/4)*(queue_sz)]; \
void *   os_mailQ_p_##name[2] = { (os_mailQ_q_##name), os_mailQ_m_##name }; \
const r_ospl_rtx_osMailQDef_t os_mailQ_def_##name =  \
{ (queue_sz), sizeof(type), (os_mailQ_p_##name) }
#endif
#endif


/***********************************************************************
* Macro: osMailQ
*
* Description:
*    See <Copy_part_of_CMSIS>.
************************************************************************/
#if IS_RTX_USED
#define osMailQ(name)  \
&os_mailQ_def_##name
#endif


/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/

#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

#endif /* OSPL_RTX_PRIVATE_H */

