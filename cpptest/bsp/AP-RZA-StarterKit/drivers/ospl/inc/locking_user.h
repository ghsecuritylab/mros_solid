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
* File: locking_user.h
*    Lock related FIT BSP. User defined.
*
* - $Module: OSPL $ $PublicVersion: 0.96 $ (=R_OSPL_VERSION)
* - $Rev: 35 $
* - $Date:: 2014-04-15 21:38:18 +0900#$
************************************************************************/

#ifndef LOCKING_USER_H
#define LOCKING_USER_H


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include  "r_typedefs.h"
#include  "locking_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************
Typedef definitions
******************************************************************************/

/***********************************************************************
* Structure: r_ospl_user_lock_t
*    Example of user defined lock type
************************************************************************/
typedef struct st_r_ospl_user_lock_t  r_ospl_user_lock_t;
struct st_r_ospl_user_lock_t
{
    int  UserLockMember;
};


/******************************************************************************
Macro definitions
******************************************************************************/

/***********************************************************************
* Macro: BSP_CFG_USER_LOCKING_TYPE
*    C-lock (FIT BSP portable)
*
* Description:
*    The value can be changed.
*    This is enabled, when "BSP_CFG_USER_LOCKING_ENABLED == 1".
************************************************************************/
#define  BSP_CFG_USER_LOCKING_TYPE  r_ospl_user_lock_t


/***********************************************************************
* Macro: BSP_CFG_USER_LOCKING_SW_LOCK_FUNCTION
*    Start C-lock for software module
*
* Description:
*    The value can be changed.
*    This is enabled, when "BSP_CFG_USER_LOCKING_ENABLED == 1".
************************************************************************/
#define  BSP_CFG_USER_LOCKING_SW_LOCK_FUNCTION    R_OSPL_USER_LOCK_SoftwareLock


/***********************************************************************
* Macro: BSP_CFG_USER_LOCKING_SW_UNLOCK_FUNCTION
*    End C-lock for software module
*
* Description:
*    The value can be changed.
*    This is enabled, when "BSP_CFG_USER_LOCKING_ENABLED == 1".
************************************************************************/
#define  BSP_CFG_USER_LOCKING_SW_UNLOCK_FUNCTION  R_OSPL_USER_LOCK_SoftwareUnlock


/***********************************************************************
* Macro: BSP_CFG_USER_LOCKING_HW_LOCK_FUNCTION
*    Start C-lock for hardware module
*
* Description:
*    The value can be changed.
*    This is enabled, when "BSP_CFG_USER_LOCKING_ENABLED == 1".
************************************************************************/
#define  BSP_CFG_USER_LOCKING_HW_LOCK_FUNCTION    R_OSPL_USER_LOCK_HardwareLock


/***********************************************************************
* Macro: BSP_CFG_USER_LOCKING_HW_UNLOCK_FUNCTION
*    End C-lock for hardware module
*
* Description:
*    The value can be changed.
*    This is enabled, when "BSP_CFG_USER_LOCKING_ENABLED == 1".
************************************************************************/
#define  BSP_CFG_USER_LOCKING_HW_UNLOCK_FUNCTION  R_OSPL_USER_LOCK_HardwareUnlock


/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/

/***********************************************************************
* Function: R_OSPL_USER_LOCK_SoftwareLock
*    Example of BSP_CFG_USER_LOCKING_SW_LOCK_FUNCTION
************************************************************************/
#if BSP_CFG_USER_LOCKING_ENABLED == 1
bool_t  R_OSPL_USER_LOCK_SoftwareLock( r_ospl_user_lock_t* LockObject );
#endif


/***********************************************************************
* Function: R_OSPL_USER_LOCK_SoftwareUnlock
*    Example of BSP_CFG_USER_LOCKING_SW_UNLOCK_FUNCTION
************************************************************************/
#if BSP_CFG_USER_LOCKING_ENABLED == 1
bool_t  R_OSPL_USER_LOCK_SoftwareUnlock( r_ospl_user_lock_t* LockObject );
#endif


/***********************************************************************
* Function: R_OSPL_USER_LOCK_HardwareLock
*    Example of BSP_CFG_USER_LOCKING_HW_LOCK_FUNCTION
************************************************************************/
#if BSP_CFG_USER_LOCKING_ENABLED == 1
bool_t  R_OSPL_USER_LOCK_HardwareLock( mcu_lock_t  HardwareIndex );
#endif


/***********************************************************************
* Function: R_OSPL_USER_LOCK_HardwareUnlock
*    Example of BSP_CFG_USER_LOCKING_HW_UNLOCK_FUNCTION
************************************************************************/
#if BSP_CFG_USER_LOCKING_ENABLED == 1
bool_t  R_OSPL_USER_LOCK_HardwareUnlock( mcu_lock_t  HardwareIndex );
#endif


/***********************************************************************
* Function: R_OSPL_GetHardwareLockObjectForTest
*    For Test
************************************************************************/
#if BSP_CFG_USER_LOCKING_ENABLED == 1
r_ospl_user_lock_t*  R_OSPL_GetHardwareLockObjectForTest( mcu_lock_t  HardwareIndex );
#endif


/***********************************************************************
* End of File:
************************************************************************/
#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

#endif /* LOCKING_USER_H */

