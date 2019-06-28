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
* $FileName: Project_Config_Sub.h $
* $Rev: 40 $
* $Date:: 2013-12-18 20:02:45 +0900#$
* Description  : This file is for drivers and applications
******************************************************************************/

#ifndef PROJECT_CONFIG_SUB_H
#define PROJECT_CONFIG_SUB_H


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************
Typedef definitions
******************************************************************************/

/******************************************************************************
Macro definitions
******************************************************************************/

/***********************************************************************
* Define: IS_VDC_SAMPLE_VRAM_USED
*    0 or 1.
************************************************************************/
#define  IS_VDC_SAMPLE_VRAM_USED  1


/***********************************************************************
* Define: IS_VDC_USED
*    0 or 1.
************************************************************************/
#define  IS_VDC_USED  1


/***********************************************************************
* Define: IS_EXRAM_USED
*    0 or 1.
************************************************************************/
#define  IS_EXRAM_USED  1


/***********************************************************************
* Define: R_VDC5_VERSION
*    100 = version 1.00. TODO: Is it referenced?
************************************************************************/
#define  R_VDC5_VERSION  100


/***********************************************************************
* Define: IS_CMSIS_USED
*    0 or 1.
************************************************************************/
#define  IS_CMSIS_USED  0


/***********************************************************************
* Define: IS_RTX_USED
*    0 or 1. TODO: Is it referenced?
************************************************************************/
#define  IS_RTX_USED  0


/***********************************************************************
* Define: IS_RZ_A1_BSP_USED
*    0 or 1. TODO: Is it referenced?
************************************************************************/
#define  IS_RZ_A1_BSP_USED  0


/******************************************************************************
Variable Externs
******************************************************************************/

/******************************************************************************
Functions Prototypes
******************************************************************************/

#ifdef __cplusplus
}  /* extern "C" */
#endif /* __cplusplus */

#endif /* PROJECT_CONFIG_SUB_H */
